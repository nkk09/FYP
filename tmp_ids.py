from pymavlink import mavutil
mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
msg = mav.wait_heartbeat(timeout=30)
print('mav.target', mav.target_system, mav.target_component)
print('heartbeat src', msg.get_srcSystem(), msg.get_srcComponent())
print('heartbeat type', msg.get_type())
