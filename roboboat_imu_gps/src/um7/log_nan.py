import time
import math

import um7

name1 = 's1'
port1 = '/dev/ttyS0'
statevars = ['health', 'roll', 'pitch', 'yaw', 'accel_raw_x', 'accel_raw_y', 'accel_raw_z', 'accel_proc_x', 'accel_proc_y', 'accel_proc_z', 'gyro_proc_x', 'gyro_proc_y', 'gyro_proc_z', 'accel_raw_time', 'accel_proc_time', 'euler_time']

s1 = um7.UM7(name1, port1, statevars, baud=115200)

print('GET_FW_REVISION=' +     s1.get_fw_revision())
print('ZERO_GYROS ' + 'ok.' if s1.zero_gyros()      else 'failed.')
print('RESET_EKF ' + 'ok.'  if s1.reset_ekf()       else 'failed.')

fs = ''
hs = ''
for i in statevars:
    hs += '{:>13.13s} '
    if i == 'health':
        fs += ' {0['+i+']:012b} '
    else:
        fs += '{0['+i+']:13.3f} '

last_atime = 0.0
nan_reported = False
health = 0
state_history = []
def report(reason):
    global state_history
    dt = time.strftime('%Y-%m-%d %H:%M:%S ') + reason
    print('           datetime reason' + hs.format(*statevars))
    for j in state_history:
        print(dt + fs.format(j))
    
while True:
    s1.catchallsamples(['accel_raw_time', 'accel_proc_time', 'euler_time'], 1.0)

    if s1.state['accel_proc_time'] != last_atime:
        last_atime = s1.state['accel_proc_time']
        state_history.append(s1.state.copy())
        if len(state_history) > 20:
            del state_history[0]
    if s1.state['health'] != health:
        health = s1.state['health']
        report('health')
    if math.isnan(s1.state['accel_proc_x']) and not nan_reported:
        nan_reported = True
        report('nan   ')
