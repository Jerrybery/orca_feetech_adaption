# Copyright 2019 The ROBEL Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Communication using the FeetechSDK."""

import atexit
import logging
import time
from typing import Optional, Sequence, Union, Tuple
import numpy as np

PROTOCOL_VERSION = 0 # FeetechSDK protocol for Little-Endian

# The following addresses assume XH motors.
# see https://emanual.robotis.com/docs/en/ft/x/xc330-t288/ for control table
# TOFIX: Adapt to Feetech Address and Data Byte Length
ADDR_OPERATING_MODE = 0x21
ADDR_TORQUE_ENABLE = 0x28
ADDR_GOAL_POSITION = 0x2A
ADDR_GOAL_PWM = 100
ADDR_GOAL_CURRENT = 0x1C
ADDR_PROFILE_VELOCITY = 0x2E
ADDR_PRESENT_POSITION = 0x38
ADDR_PRESENT_VELOCITY = 0x3A
ADDR_PRESENT_CURRENT = 0x45
ADDR_PRESENT_POS_VEL_CUR = 126
ADDR_MOVING_STATUS = 0x42
ADDR_PRESENT_TEMPERATURE = 0x3F
ADDR_MOVING_VELOCITY = 0X2E
ADDR_MOVING_ACCELERATION = 0X29

# Data Byte Length
LEN_OPERATING_MODE = 1
LEN_PRESENT_POSITION = 2
LEN_PRESENT_VELOCITY = 2
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_POS_VEL_CUR = 10
LEN_GOAL_POSITION = 2
LEN_GOAL_PWM = 2
LEN_GOAL_CURRENT = 2
LEN_PROFILE_VELOCITY = 2
LEN_MOVING_STATUS = 1
LEN_PRESENT_TEMPERATURE = 1
LEN_MOVING_VELOCITY = 2
LEN_MOVING_ACCELERATION = 1
LEN_TORQUE_ENABLE = 1

DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 0.088 degrees
DEFAULT_VEL_SCALE = 0.229 * 2.0 * np.pi / 60.0  # 0.229 rpm
DEFAULT_CUR_SCALE = 1.34


def Feetech_cleanup_handler():
    """Cleanup function to ensure Feetechs are disconnected properly."""
    open_clients = list(FeetechClient.OPEN_CLIENTS)
    for open_client in open_clients:
        if open_client.port_handler.is_using:
            logging.warning('Forcing client to close.')
        open_client.port_handler.is_using = False
        open_client.disconnect()


def signed_to_unsigned(value: int, size: int) -> int:
    """Converts the given value to its unsigned representation."""
    if value < 0:
        bit_size = 8 * size
        max_value = (1 << bit_size) - 1
        value = max_value + value
    return value


def unsigned_to_signed(value: int, size: int) -> int:
    """Converts the given value from its unsigned representation."""
    bit_size = 8 * size
    if (value & (1 << (bit_size - 1))) != 0:
        value = -((1 << bit_size) - value)
    return value


class FeetechClient:
    """Client for communicating with Feetech motors.

    NOTE: This only supports Protocol 2. # TODO: clarify what is the protocol 2
    """

    # The currently open clients.
    OPEN_CLIENTS = set()

    def __init__(self,
                 motor_ids: Sequence[int],
                 port: str = '/dev/ttyUSB0',
                 baudrate: int = 1000000,
                 lazy_connect: bool = False,
                 pos_scale: Optional[float] = None,
                 vel_scale: Optional[float] = None,
                 cur_scale: Optional[float] = None):
        """Initializes a new client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The Feetech device to talk to. e.g.
                - Linux: /dev/ttyUSB0
                - Mac: /dev/tty.usbserial-*
                - Windows: COM1
            baudrate: The Feetech baudrate to communicate with.
            lazy_connect: If True, automatically connects when calling a method
                that requires a connection, if not already connected.
            pos_scale: The scaling factor for the positions. This is
                motor-dependent. If not provided, uses the default scale.
            vel_scale: The scaling factor for the velocities. This is
                motor-dependent. If not provided uses the default scale.
            cur_scale: The scaling factor for the currents. This is
                motor-dependent. If not provided uses the default scale.
        """
        import scservo_sdk
        self.ft = scservo_sdk

        self._pos_scale = pos_scale or DEFAULT_POS_SCALE
        self._vel_scale = vel_scale or DEFAULT_VEL_SCALE
        self._cur_scale = cur_scale or DEFAULT_CUR_SCALE

        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        self.port_handler = self.ft.PortHandler(port)
        self.packet_handler = self.ft.PacketHandler(PROTOCOL_VERSION)

        
        self._temp_reader = FeetechTempReader(
            self,
            self.motor_ids,
            address=ADDR_PRESENT_TEMPERATURE,
            size=LEN_PRESENT_TEMPERATURE,
        )
        self._position_reader = FeetechPosReader(
            self,
            self.motor_ids,
            address=ADDR_PRESENT_POSITION,
            size=LEN_PRESENT_POSITION,
        )
        self._current_reader = FeetechCurrentReader(
            self,
            self.motor_ids,
            address=ADDR_PRESENT_CURRENT,
            size=LEN_PRESENT_CURRENT,
        )
        
        self._moving_status_reader = FeetechReader(self, self.motor_ids, ADDR_MOVING_STATUS, LEN_MOVING_STATUS)
        self._sync_writers = {}

        self.OPEN_CLIENTS.add(self)

    @property
    def is_connected(self) -> bool:
        return self.port_handler.is_open

    def connect(self):
        """Connects to the Feetech motors.

        NOTE: This should be called after all FeetechClients on the same
            process are created.
        """
        assert not self.is_connected, 'Client is already connected.'

        if self.port_handler.openPort():
            logging.info('Succeeded to open port: %s', self.port_name)
        else:
            raise OSError(
                ('Failed to open port at {} (Check that the device is powered '
                 'on and connected to your computer).').format(self.port_name))

        if self.port_handler.setBaudRate(self.baudrate):
            logging.info('Succeeded to set baudrate to %d', self.baudrate)
        else:
            raise OSError(
                ('Failed to set the baudrate to {} (Ensure that the device was '
                 'configured for this baudrate).').format(self.baudrate))

        # Start with all motors enabled.
        self.set_torque_enabled(self.motor_ids, True)

    def disconnect(self):
        """Disconnects from the Feetech device."""
        if not self.is_connected:
            return
        if self.port_handler.is_using:
            logging.error('Port handler in use; cannot disconnect.')
            return
        # Ensure motors are disabled at the end.
        self.set_torque_enabled(self.motor_ids, False, retries=0)
        self.port_handler.closePort()
        if self in self.OPEN_CLIENTS:
            self.OPEN_CLIENTS.remove(self)

    def set_torque_enabled(self,
                           motor_ids: Sequence[int],
                           enabled: bool = True,
                           retries: int = -1,
                           retry_interval: float = 0.25):
        """Sets whether torque is enabled for the motors.

        Args:
            motor_ids: The motor IDs to configure.
            enabled: Whether to engage or disengage the motors.
            retries: The number of times to retry. If this is <0, will retry
                forever.
            retry_interval: The number of seconds to wait between retries.
        """
        remaining_ids = list(motor_ids)
        if enabled:
            self.sync_write(remaining_ids, [1]*len(remaining_ids), ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE) # Torque for feetech
        else:
            self.sync_write(remaining_ids, [0]*len(remaining_ids), ADDR_TORQUE_ENABLE, LEN_TORQUE_ENABLE) # Torque for feetech

    def set_operating_mode(self, motor_ids: Sequence[int], mode_value: int):
        """
        see https://emanual.robotis.com/docs/en/ft/x/xc330-t288/#operating-mode11
        0: current control mode
        1: velocity control mode
        3: position control mode
        4: multi-turn position control mode
        5: current-based position control mode
        """
        # data in EEPROM area can only be written when torque is disabled
        self.set_torque_enabled(motor_ids, False)
        self.sync_write(motor_ids, [mode_value]*len(motor_ids), ADDR_OPERATING_MODE, LEN_OPERATING_MODE) # ADDR_OPERATING_MODE is the address of operating mode in EEPROM area
        self.set_torque_enabled(motor_ids, True)

    def read_status_is_done_moving(self) -> bool:
        """Returns the last bit of moving status"""
        moving_status = self._moving_status_reader.read().astype(np.int8)
        return np.bitwise_and(moving_status, np.array([0x01] * len(moving_status)).astype(np.int8))

    def read_temperature(self) -> np.ndarray:
        """Reads and returns the present temperature for each motor (in deg C)."""
        return self._temp_reader.read()
    
    def read_current(self) -> np.ndarray:
        """Reads and returns the present current for each motor (in mA)."""
        return self._current_reader.read()
    
    def read_position(self) -> np.ndarray:
        """Reads and returns the present position for each motor (in radians)."""
        return self._position_reader.read()



    def write_desired_pos(self, motor_ids: Sequence[int],
                          positions: np.ndarray):
        """Writes the given desired positions.

        Args:
            motor_ids: The motor IDs to write to.
            positions: The joint angles in radians to write.
        """
        assert len(motor_ids) == len(positions)

        # Convert to Feetech position space.
        positions = positions / self._pos_scale
        self.sync_write(motor_ids, [150]*len(motor_ids), ADDR_MOVING_VELOCITY, LEN_MOVING_VELOCITY) # Vel for feetech
        self.sync_write(motor_ids, [150]*len(motor_ids), ADDR_MOVING_ACCELERATION, LEN_MOVING_ACCELERATION) # Acc for feetech


        times = self.sync_write(motor_ids, positions, ADDR_GOAL_POSITION, LEN_GOAL_POSITION) 
        return times

    def write_desired_current(self, motor_ids: Sequence[int], current: np.ndarray):
        assert len(motor_ids) == len(current)
        self.sync_write(motor_ids, current, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)

    def write_profile_velocity(self, motor_ids: Sequence[int], profile_velocity: np.ndarray):
            assert len(motor_ids) == len(profile_velocity)

            self.sync_write(motor_ids, profile_velocity, ADDR_PROFILE_VELOCITY, LEN_PROFILE_VELOCITY)

    def write_byte(
            self,
            motor_ids: Sequence[int],
            value: int,
            address: int,
    ) -> Sequence[int]:
        """Writes a value to the motors.

        Args:
            motor_ids: The motor IDs to write to.
            value: The value to write to the control table.
            address: The control table address to write to.

        Returns:
            A list of IDs that were unsuccessful.
        """
        self.check_connected()
        errored_ids = []
        for motor_id in motor_ids:
            comm_result, ft_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, address, value)
            success = self.handle_packet_result(
                comm_result, ft_error, motor_id, context='write_byte')
            if not success:
                errored_ids.append(motor_id)
        return errored_ids

    def sync_write(self, motor_ids: Sequence[int],
                   values: Sequence[Union[int, float]], address: int,
                   size: int):
        """Writes values to a group of motors.

        Args:
            motor_ids: The motor IDs to write to.
            values: The values to write.
            address: The control table address to write to.
            size: The size of the control table value being written to.
        """
        times = [time.monotonic()]
        self.check_connected()
        key = (address, size)
        if key not in self._sync_writers:
            self._sync_writers[key] = self.ft.GroupSyncWrite(
                self.port_handler, self.packet_handler, address, size)
        sync_writer = self._sync_writers[key]
        times.append(time.monotonic())
        errored_ids = []
        for motor_id, desired_pos in zip(motor_ids, values):
            value = signed_to_unsigned(int(desired_pos), size=size)
            value = value.to_bytes(size, byteorder='little')
            success = sync_writer.addParam(motor_id, value)
            if not success:
                errored_ids.append(motor_id)

        if errored_ids:
            logging.error('Sync write failed for: %s', str(errored_ids))
        times.append(time.monotonic())

        comm_result = sync_writer.txPacket()
        self.handle_packet_result(comm_result, context='sync_write')
        times.append(time.monotonic())

        sync_writer.clearParam()
        times.append(time.monotonic())
        return times

    def check_connected(self):
        """Ensures the robot is connected."""
        if self.lazy_connect and not self.is_connected:
            self.connect()
        if not self.is_connected:
            raise OSError('Must call connect() first.')

    def handle_packet_result(self,
                             comm_result: int,
                             ft_error: Optional[int] = None,
                             ft_id: Optional[int] = None,
                             context: Optional[str] = None):
        """Handles the result from a communication request."""
        error_message = None
        if comm_result != self.ft.COMM_SUCCESS:
            error_message = self.packet_handler.getTxRxResult(comm_result) 
        elif ft_error is not None:
            error_message = self.packet_handler.getRxPacketError(ft_error)
        if error_message:
            if ft_id is not None:
                error_message = '[Motor ID: {}] {}'.format(
                    ft_id, error_message)
            if context is not None:
                error_message = '> {}: {}'.format(context, error_message)
            logging.error(error_message)
            return False
        return True

    def convert_to_unsigned(self, value: int, size: int) -> int:
        """Converts the given value to its unsigned representation."""
        if value < 0:
            max_value = (1 << (8 * size)) - 1
            value = max_value + value
        return value

    def __enter__(self):
        """Enables use as a context manager."""
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, *args):
        """Enables use as a context manager."""
        self.disconnect()

    def __del__(self):
        """Automatically disconnect on destruction."""
        self.disconnect()


class FeetechReader:
    """Reads data from Feetech motors.

    This wraps a GroupBulkRead from the FeetechSDK.
    """

    def __init__(self, client: FeetechClient, motor_ids: Sequence[int],
                 address: int, size: int):
        """Initializes a new reader."""
        self.client = client
        self.motor_ids = motor_ids
        self.address = address
        self.size = size
        self._initialize_data()

        self.operation = self.client.ft.GroupSyncRead(client.port_handler,
                                                       client.packet_handler,
                                                       start_address=address,
                                                       data_length=size) # TOFIX: find out the sync/bulk read difference in hand instance

        for motor_id in motor_ids:
            success = self.operation.addParam(motor_id)
            if not success:
                raise OSError(
                    '[Motor ID: {}] Could not add parameter to bulk read.'
                    .format(motor_id))

    def read(self, retries: int = 1):
        """Reads data from the motors."""
        self.client.check_connected()
        success = False
        while not success and retries >= 0:
            comm_result = self.operation.txRxPacket()
            success = self.client.handle_packet_result(
                comm_result, context='read')
            retries -= 1

        # If we failed, send a copy of the previous data.
        if not success:
            return self._get_data()

        errored_ids = []
        for i, motor_id in enumerate(self.motor_ids):
            # Check if the data is available.
            available = self.operation.isAvailable(motor_id, self.address,
                                                   self.size)
            if not available:
                errored_ids.append(motor_id)
                continue

            try:
                self._update_data(i, motor_id)
            except Exception as e:
                logging.error(f'Error updating data for motor {motor_id}: {e}')
                errored_ids.append(motor_id)
                continue

        if errored_ids:
            logging.error('Bulk read data is unavailable for: %s',
                          str(errored_ids))

        return self._get_data()

    def _initialize_data(self):
        """Initializes the cached data."""
        self._data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        """Updates the data index for the given motor ID."""
        self._data[index] = self.operation.getData(motor_id, self.address,
                                                   self.size) * self.client._pos_scale

    def _get_data(self):
        """Returns a copy of the data."""
        return self._data.copy()

class FeetechPosReader(FeetechReader):
    """Reads present position (2 bytes) for each Feetech motor."""
    
    def _initialize_data(self):
        # We'll store one float per motor for the position values.
        self._pos_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        # 4096 => 2 pi
        raw_val = self.operation.getData(motor_id, self.address, self.size)
        self._pos_data[index] = float(raw_val) * self.client._pos_scale

    def _get_data(self):
        return self._pos_data.copy()

class FeetechCurrentReader(FeetechReader):
    """Reads present current (2 bytes) for each Feetech motor."""
    
    def _initialize_data(self):
        # We'll store one float per motor for the temperature values.
        self._current_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        # The raw value from the control table is 1 byte = 1 degree Celsius.
        raw_val = self.operation.getData(motor_id, self.address, self.size)
        self._current_data[index] = float(raw_val)

    def _get_data(self):
        return self._current_data.copy()

class FeetechTempReader(FeetechReader):
    """Reads present temperature (1 byte) for each Feetech motor."""
    
    def _initialize_data(self):
        # We'll store one float per motor for the temperature values.
        self._temp_data = np.zeros(len(self.motor_ids), dtype=np.float32)

    def _update_data(self, index: int, motor_id: int):
        # The raw value from the control table is 1 byte = 1 degree Celsius.
        raw_val = self.operation.getData(motor_id, self.address, self.size)
        self._temp_data[index] = float(raw_val)

    def _get_data(self):
        return self._temp_data.copy()

# Register global cleanup function.
atexit.register(Feetech_cleanup_handler)

if __name__ == '__main__':
    import argparse
    import itertools

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-m',
        '--motors',
        required=True,
        help='Comma-separated list of motor IDs.')
    parser.add_argument(
        '-d',
        '--device',
        default='/dev/cu.usbserial-FT62AFSR',
        help='The Feetech device to connect to.')
    parser.add_argument(
        '-b', '--baud', default=1000000, help='The baudrate to connect with.')
    parsed_args = parser.parse_args()
    motors = [int(motor) for motor in parsed_args.motors.split(',')]
    
    way_points = [np.zeros(len(motors)), np.full(len(motors), np.pi)]

    with FeetechClient(motors, parsed_args.device,
                         parsed_args.baud) as ft_client:
        for step in itertools.count():
            if step > 0 and step % 50 == 0:
                way_point = way_points[(step // 100) % len(way_points)]
                print('Writing: {}'.format(way_point.tolist()))
                ft_client.write_desired_pos(motors, way_point)
            read_start = time.time()
            pos_now, vel_now, cur_now = ft_client.read_pos_vel_cur()
            if step % 5 == 0:
                print('[{}] Frequency: {:.2f} Hz'.format(
                    step, 1.0 / (time.time() - read_start)))
                print('> Pos: {}'.format(pos_now.tolist()))
                print('> Vel: {}'.format(vel_now.tolist()))
                print('> Cur: {}'.format(cur_now.tolist()))