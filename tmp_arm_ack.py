from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
mav.wait_heartbeat(timeout=30)
print('heartbeat')

mav.mav.command_long_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
)
print('arm sent')

start = time.time()
while time.time() - start < 10:
    msg = mav.recv_match(blocking=True, timeout=1)
    if not msg:
        continue
    msg_type = msg.get_type()
    if msg_type == 'COMMAND_ACK':
        print('ACK', msg.command, msg.result)
        break
    if msg_type == 'STATUSTEXT':
        print('TEXT', msg.text)
    if msg_type == 'HEARTBEAT':
        print('HB', msg.base_mode)
