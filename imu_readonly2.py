# -*- coding: utf-8 -*-
#用的高层状态接口
import sys
import time
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

# 这里可能需要改！
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_


class SportStateReader:
    def __init__(self):
        self.msg = None
        self.lock = threading.Lock()

    def cb(self, msg):
        with self.lock:
            self.msg = msg

    def get(self):
        with self.lock:
            return self.msg


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"

    ChannelFactoryInitialize(0, iface)

    reader = SportStateReader()

    sub = ChannelSubscriber(
        "rt/sportmodestate",
        SportModeState_
    )

    sub.Init(reader.cb, 10)

    print("start reading sport state...")

    while True:
        msg = reader.get()

        if msg is not None:
            try:
                print("=" * 50)

                # 位置
                print("position:", list(msg.position))

                # 速度
                print("velocity:", list(msg.velocity))

                # imu rpy
                print("imu rpy:", list(msg.imu_state.rpy))

            except Exception as e:
                print("parse error:", e)

        time.sleep(0.5)


if __name__ == "__main__":
    main()