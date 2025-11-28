import orca_core.hardware.feetech_client as ft_client
import numpy as np

client = ft_client.FeetechClient(motor_ids=[i for i in range(1, 17)])
client.connect()
reader = ft_client.FeetechReader(client=client, motor_ids=[i for i in range(1, 17)], address=0x21, size=1)
reader_vel = ft_client.FeetechReader(client=client, motor_ids=[i for i in range(1, 17)], address=0X2E, size=2)
reader_acc = ft_client.FeetechReader(client=client, motor_ids=[i for i in range(1, 17)], address=0x29, size=1)
print(client.read_position())
print(reader_vel.read())
print(reader_acc.read())
