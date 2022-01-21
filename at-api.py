#!/usr/bin python

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.version import version
from pymodbus.payload import BinaryPayloadDecoder,BinaryPayloadBuilder, Endian
import time

_BYTEORDER_ = Endian.Big
_WORDORDER_ = Endian.Big
_IP_ADDRESS_ = '192.168.1.43'
#_IP_ADDRESS_ = 'localhost'
_PORT_ = 1124

_UNIT_ = 0x01

#0x43: Home Coil

class mbClientClass:
    def __init__(self):
        #connect to server
        self.client = ModbusClient(_IP_ADDRESS_, port=_PORT_)
        print("Client Connected: ",self.client.connect())
        
        while True:
            i1 = int(input("Set to Operational? [1 or 0]: "))
            self.client.write_coil(0x01,0x01,unit=_UNIT_)
            print("Operation Okay")
            time.sleep(0.2)
            j = int(input("Home? [1 or 0]: "))
            if (j == 1):
                self.client.write_coil(0x43,0x01,unit=_UNIT_)
                time.sleep(0.2)
                val,lengthVal = self.encode_float_and_negative(1000)
                self.client.write_registers(0x112,val,unit=_UNIT_)
                print("In Operation mode, default speed: 1000")
                val = self.client.read_input_registers(0x41,count=1,unit=_UNIT_)
                val = val.registers
                v = False
                if val[0] == 1:
                    v = True
                print("Machine Homed: ",v)

                while True:
                    cmd_x = float(input("X command: "))
                    cmd_x,lengthVal = self.encode_float_and_negative(cmd_x)
                    self.client.write_registers(0x100,cmd_x,unit=_UNIT_)
                    #time.sleep(0.1)
                    cmd_y = float(input("Y command: "))
                    cmd_y,lengthVal = self.encode_float_and_negative(cmd_y)
                    self.client.write_registers(0x103,cmd_y,unit=_UNIT_)
                    #time.sleep(0.1)
                    cmd_vel = float(input("Speed command: "))
                    cmd_vel,lengthVal = self.encode_float_and_negative(cmd_vel)
                    self.client.write_registers(0x112,cmd_vel,unit=_UNIT_)
                    #time.sleep(0.1)
                    executeVar = int(input("Execute? [1 or 0]: "))
                    if (executeVar == 1):
                        self.client.write_coil(0x05,1,unit=_UNIT_)
                        time.sleep(1)
                        while True:
                            #inRun = self.client.read_input_registers(0x42,count=1,unit=_UNIT_)
                            #r = inRun.registers
                            #if (r[0] == 0):
                            #    break
                            speed_fb = self.client.read_input_registers(0x514,count=2,unit=_UNIT_) #514,515 actual speed -> actual speed 
                            x_fb = self.client.read_input_registers(0x500,count=2,unit=_UNIT_)
                            y_fb = self.client.read_input_registers(0x502,count=2,unit=_UNIT_)
                            print("X: "+str(x_fb)+" "+"Y: "+str(y_fb)+" "+"Speed: "+str(speed_fb))
                            time.sleep(0.5)
                            eefState = int(input("Set (estop == 1) (enter next Command == -1) (read again == 0): "))
                            if (eefState == -1):
                                break
                            if(eefState == 1):
                                #0x02 estop 09 eef
                                self.client.write_coil(0x03,1,unit=_UNIT_)
                            if(eefState == 0):
	                            #0x02 estop 06 eef
    	                        self.client.write_coil(0x03,0,unit=_UNIT_)



    def decode_float_and_negative(self,val):
        d = BinaryPayloadDecoder.fromRegisters(val, byteorder=_BYTEORDER_, wordorder=_WORDORDER_)
        returnVal = d.decode_32bit_float()
        return returnVal
    
    def encode_float_and_negative(self,val):
        builder = BinaryPayloadBuilder(byteorder=_BYTEORDER_, wordorder=_WORDORDER_)
        builder.reset()
        builder.add_32bit_float(val)
        returnVal = builder.to_registers()
        return returnVal,len(returnVal)



if __name__=='__main__':
    try:
        mbClientClass()
    except:
        print("CannotStart")
