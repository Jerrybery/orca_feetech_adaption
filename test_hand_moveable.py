import orca_core.hardware.feetech_client as ft_client
from orca_core import OrcaHand
import numpy as np
import time 
JOINT_NAMES = ['thumb_mcp', 'thumb_abd', "thumb_pip", "thumb_dip" , "index_abd", 'index_mcp', 'index_pip', 'middle_abd', 'middle_mcp', 'middle_pip', 'ring_abd', 'ring_mcp', 'ring_pip', 'pinky_abd', 'pinky_mcp', 'pinky_pip']

client = ft_client.FeetechClient(motor_ids=[i for i in range(1, 17)])
client.connect()
# control mode
reader_mod = ft_client.FeetechReader(client=client, motor_ids=[i for i in range(1, 17)], address=33, size=1)
# speed
reader_vel = ft_client.FeetechReader(client=client, motor_ids=[i for i in range(1, 17)], address=46, size=2)
# acc
reader_acc = ft_client.FeetechReader(client=client, motor_ids=[i for i in range(1, 17)], address=41, size=1)
# goalpos
reader_pos = ft_client.FeetechReader(client=client, motor_ids=[i for i in range(1, 17)], address=0x2A, size=2)

print(client.read_position())
print(reader_pos.read())
print(reader_vel.read())
print(reader_acc.read())


print('\n')

hand = OrcaHand("orca_core/models/orcahand_v1_left")
hand.connect()
print('========')
print(hand._motor_client._position_reader._get_real_pos())
