import scservo_sdk

reader = scservo_sdk.GroupSyncRead('/dev/ttyUSB0', scservo_sdk.PacketHandler(0), 0x21, 1)
for i in range(1, 17):
    reader.addParam(i)
    print(reader.getData(scs_id=i, address=0x21, data_length=1))
