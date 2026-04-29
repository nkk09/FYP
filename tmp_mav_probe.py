from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
mav.wait_heartbeat(timeout=30)
print('heartbeat')

for stream_id in range(1, 8):
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component, stream_id, 10, 1)
print('requested streams')

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

start = time.time()
while time.time() - start < 8:
    msg = mav.recv_match(blocking=True, timeout=1)
    if not msg:
        continue
    msg_type = msg.get_type()
    if msg_type == 'COMMAND_ACK':
        print('ACK', msg.command, msg.result)
    elif msg_type == 'STATUSTEXT':
        print('TEXT', msg.text)
    elif msg_type == 'HEARTBEAT':
        print('HB', msg.base_mode)
    elif msg_type == 'SERVO_OUTPUT_RAW':
        print('SERVO', msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw)
    elif msg_type == 'EKF_STATUS_REPORT':
        print('EKF', msg.flags)
    elif msg_type == 'GPS_RAW_INT':
        print('GPS', msg.fix_type, msg.satellites_visible)
