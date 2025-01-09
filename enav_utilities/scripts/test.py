#!/usr/bin/env python

from tracemalloc import start
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

import time as pytime

if __name__ == "__main__":
    # Create a typestore and get the string class.
    typestore = get_typestore(Stores.LATEST)

    start_time = pytime.time()

    # Create reader instance and open for reading.
    with Reader('/media/olamarre/My Passport/enav_dataset/run3_base_new.bag') as reader:
        # Topic and msgtype information is available on .connections list.

        # for connection in reader.connections:
        #     print(connection.topic, connection.msgtype)

        print(f"{reader.message_count}")

        # # Iterate over messages.
        # for connection, timestamp, rawdata in reader.messages():
        #     if connection.topic == '/imu_raw/Imu':
        #         msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
        #         print(msg.header.frame_id)

        # The .messages() method accepts connection filters.
        connections = [x for x in reader.connections if x.topic == '/imu']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            print(msg.header.frame_id)
            break


    print(f"Elapsed: {pytime.time()-start_time}")

    print(f"This is a test")
    print(f"uh ohhhh")