from smbus import SMBus
import struct
import time

i2cbus = SMBus(1)
addr = 0x31

while True:
  px = struct.unpack('<f', bytes(i2cbus.read_i2c_block_data(addr,9,4)))[0]
  py = struct.unpack('<f', bytes(i2cbus.read_i2c_block_data(addr,10,4)))[0]
  h = struct.unpack('<f', bytes(i2cbus.read_i2c_block_data(addr,11,4)))[0]
  ex = struct.unpack('<i', bytes(i2cbus.read_i2c_block_data(addr,6,4)))[0]
  ey = struct.unpack('<i', bytes(i2cbus.read_i2c_block_data(addr,7,4)))[0]
  print('px', px, 'py', py, 'h', h, 'ex', ex, 'ey', ey)
  time.sleep(1)

