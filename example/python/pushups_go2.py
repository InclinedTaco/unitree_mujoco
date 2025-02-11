import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

stand_up_joint_pos = np.array([
    0.0, 0.67, -1.3,  # FR
    0.0, 0.67, -1.3,  # FL
    0.0, 0.67, -1.3,  # RR
    0.0, 0.67, -1.3   # RL
], dtype=float)

stand_down_joint_pos = np.array([
    0.0, 1.2, -2.4,  # FR
    0.0, 1.2, -2.4,  # FL
    0.0, 1.2, -2.4,  # RR
    0.0, 1.2, -2.4,  # RL
], dtype=float)

dt = 0.002
running_time = 0.0
cycle_duration = 4.0  # Time to complete one cycle of stand and sit
crc = CRC()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0

    # Initialize motor commands
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    input("Press enter to start")

    while True:
        step_start = time.perf_counter()
        running_time += dt

        if running_time < 2.0:
            # Stand up phase
            phase = np.tanh(running_time / 0.8)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
                cmd.motor_cmd[i].kp = 70.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.0
                cmd.motor_cmd[i].tau = 0.0
        elif running_time < 4.0:
            # Stand phase
            for i in range(12):
                cmd.motor_cmd[i].q = stand_up_joint_pos[i]
                cmd.motor_cmd[i].kp = 70.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.0
                cmd.motor_cmd[i].tau = 0.0
        elif running_time < 6.0:
            # Sit down phase
            phase = np.tanh((running_time - 4.0) / 0.8)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i]
                cmd.motor_cmd[i].kp = 70.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.0
                cmd.motor_cmd[i].tau = 0.0
        else:
            # Reset the running time after a full cycle (stand -> sit -> stand again)
            running_time = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

