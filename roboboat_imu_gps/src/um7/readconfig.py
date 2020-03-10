import numpy as np
import struct
import um7

name1 = 's'
# port1 = '/dev/ttyS0'
port1 = '/dev/tty.usbserial-A903AAUJ'
statevars = []

sensor1 = um7.UM7(name1, port1, statevars, baud=115200)

np.set_printoptions(formatter={'float':lambda x: '{:11.5f}'.format(x)})

p = sensor1.readreg(um7.CREG_MISC_SETTINGS, 1)
cs = list(struct.unpack('!I', p.data))
print('CREG_MISC_SETTINGS={:032b}'.format(cs[0]))

p = sensor1.readreg(um7.CREG_ACCEL_BIAS_X, 3)
bx, by, bz = struct.unpack('!fff', p.data)
print('accel bias: {:9.2f} {:9.2f} {:9.2f}'.format(bx, by, bz))

p = sensor1.readreg(um7.CREG_ACCEL_CAL1_1, 9)
floats = struct.unpack('!fffffffff', p.data)
old_m = np.array([floats[0:3],floats[3:6],floats[6:9]])
print("accel cal matrix=")
print(old_m)

p = sensor1.readreg(um7.CREG_MAG_CAL1_1, 9)
floats = struct.unpack('!fffffffff', p.data)
old_m = np.array([floats[0:3],floats[3:6],floats[6:9]])
print("mag cal matrix=")
print(old_m)

p = sensor1.readreg(um7.H_CREG_GYRO_VARIANCE, 3)
bx, by, bz = struct.unpack('!fff', p.data)
print('sensor variance: gyro={:11.6f} accel={:11.6f} mag={:11.6f}'.format(bx, by, bz))

p = sensor1.readreg(um7.H_CREG_ACCEL_TAU, 3)
bx, by, bz = struct.unpack('!fff', p.data)
print('sensor tau:      gyro={:11.6f} accel={:11.6f} mag={:11.6f}'.format(by, bx, bz))

p = sensor1.readreg(um7.CREG_MAG_BIAS_X, 3)
bx, by, bz = struct.unpack('!fff', p.data)
print('mag bias: {:9.2f} {:9.2f} {:9.2f}'.format(bx, by, bz))

p = sensor1.readreg(um7.H_CREG_MAG_REF, 3)
bx, by, bz = struct.unpack('!fff', p.data)
print('mag ref:  {:9.2f} {:9.2f} {:9.2f}'.format(bx, by, bz))

p = sensor1.readreg(um7.CREG_COM_SETTINGS, 1)
cs = list(struct.unpack('!BBBB', p.data))
print('CREG_COM_SETTINGS = {}'.format(cs))
br = (cs[0] & 0xf0)>> 4
print('BAUD_RATE         = {0}'.format(br))
bg = cs[0] & 0x0f
print('GPS_BAUD          = {0}'.format(bg))

p = sensor1.readreg(um7.CREG_COM_RATES1, 1)
cr = list(struct.unpack('!BBBB', p.data))
print('CREG_COM_RATES1   = {}'.format(cr))

p = sensor1.readreg(um7.CREG_COM_RATES2, 1)
cr = list(struct.unpack('!BBBB', p.data))
print('CREG_COM_RATES2   = {}'.format(cr))

p = sensor1.readreg(um7.CREG_COM_RATES3, 1)
cr = list(struct.unpack('!BBBB', p.data))
print('CREG_COM_RATES3   = {}'.format(cr))

p = sensor1.readreg(um7.CREG_COM_RATES4, 1)
cr = list(struct.unpack('!BBBB', p.data))
print('CREG_COM_RATES4   = {}'.format(cr))

p = sensor1.readreg(um7.CREG_COM_RATES5, 1)
cr = list(struct.unpack('!BBBB', p.data))
print('CREG_COM_RATES5   = {}'.format(cr))

p = sensor1.readreg(um7.CREG_COM_RATES6, 1)
cr = list(struct.unpack('!BBBB', p.data))
print('CREG_COM_RATES6   = {}'.format(cr))

p = sensor1.readreg(um7.CREG_COM_RATES7, 1)
cr = list(struct.unpack('!BBBB', p.data))
print('CREG_COM_RATES7   = {}'.format(cr))

del sensor1
