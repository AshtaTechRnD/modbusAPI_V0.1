#!/usr/bin/env python

from pymodbus.version import version
from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
#from pymodbus.transaction import ModbusRtuFramer, ModbusBinaryFramer
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder, Endian
#from twisted.internet.task import LoopingCall
import threading
import multiprocessing
import ctypes
import time
import hal
import linuxcnc

_BYTEORDER_ = Endian.Big
_WORDORDER_ = Endian.Big
_IP_ADDRESS_ = "0.0.0.0"
_PORT_ = 1124
_SLAVE_ID_ = 0x01

#import #LOGging
#LOGging.basicConfig()
#LOG = #LOGging.get#LOGger()
#LOG.setLevel(#LOGging.DEBUG)


class startServerClass:

    def __init__(self):
        self.initVariables()
        self.context = self.intializeAddress()
        self.identity = self.deviceInformation()
        self.initHal()
        self.startUpdatingThread()
        self.serverThread()

    def get_current_joint(self):
        x = hal.get_value("joint.0.pos-fb")
        y = hal.get_value("joint.1.pos-fb")
        return x,y

    def get_current_vel(self):
        v_m = hal.get_value("motion.current-vel")
        v= v_m * 60.0
        return v

    def serverThread(self):
        server = StartTcpServer(self.context, identity=self.identity, address=(_IP_ADDRESS_,_PORT_))


    def startUpdatingThread(self):
        x = threading.Thread(target=self.updationThread,name='updationThread')
        x.daemon=True
        x.start()

    def updationThread(self):
        #c.acquire()
        while True:
            #set values to ir and di
            self.readBus()
            time.sleep(0.01)
            self.evaluate()
            #print("Done Evaluation")
            #get values to hr and do
            #LOG.debug("Updating...")
            time.sleep(0.01)
        #pass

    def readBusVal(self,fx,addr,cnt):
        slave_id = _SLAVE_ID_
        values = self.context[slave_id].getValues(fx,addr,count=cnt)
        returnVal = values
        if (fx == 0x03):
            returnVal = [self.decode_float_and_negative(values)]
        #0x05 write_coil single 0x15 write_multiple coil
        return returnVal

    def machineHomed(self):
        retval = 0
        h0 = hal.get_value("halui.joint.0.is-homed")
        h1 = hal.get_value("halui.joint.1.is-homed")
        if((h0 == 1) and (h1 == 1)):
            retval = 1
        return retval

    def writeBusVal(self,fx,addr,val):
        slave_id = _SLAVE_ID_
        if ((fx == 0x04) or (fx == 0x03)):
            value,count = self.encode_float_and_negative(val)
        for i in range(count):
            self.context[slave_id].setValues(fx,addr,value[i])
        if(fx == 0x01):
            self.context[slave_id].setValues(fx,addr,[val])


    def readBus(self):
        self.estop_mb = self.readBusVal(0x01,0x03,1)
        #print(self.estop_mb)
        self.h["mb-estop"] = self.setBit(self.estop_mb[0])
        #print(self.h["mb-estop"])
        if((self.h["mb-estop"] == 1)):
            self.inExecution = 0
            self.lc.abort()
            self.ex_thread.terminate()
            self.writeBusVal(0x01,0x05,0)
        #self.context[0x01].setValues(0x04,0x42,[0])
        #self.lc.mode(linuxcnc.MODE_MANUAL)
        self.machineState = self.readBusVal(0x01,0x01,1)
        self.machineState = self.machineState[0]
        self.machineState = self.setBit(self.machineState)
        self.h["machine-state"] = self.machineState

        #go home
        if (self.machineHomed() == 0):
            self.homeIt = self.readBusVal(0x01,0x43,1)
            self.homeIt = self.homeIt[0]
            self.homeIt = self.setBit(self.homeIt)
            if(self.homeIt == 1):
                #self.lc.unhome(0)
                #self.lc.wait_complete()
                #self.lc.unhome(1)
                #self.lc.wait_complete()
                #print("Homing")
                self.lc.home(0)
                self.lc.wait_complete()
                self.lc.home(1)
                self.lc.wait_complete()
                self.homeIt = self.writeBusVal(0x01,0x43,0)

        #check homed
        if (self.machineHomed() == 1):
            #print("Homed")
            self.context[0x01].setValues(0x04,0x41,[1])
            #self.context[0x01].setValues(0x04,0x42,[0])
        else:
            self.context[0x01].setValues(0x04,0x41,[0])
            #self.context[0x01].setValues(0x04,0x42,[0])

        #Command
        self.X_cmd = self.readBusVal(0x03,0x100,2)
        self.Y_cmd = self.readBusVal(0x03,0x103,2)
        self.Z_cmd = self.readBusVal(0x03,0x106,2)
        self.A_cmd = self.readBusVal(0x03,0x110,2)

        self.X_cmd = self.X_cmd[0]
        #self.X_cmd = utils.decode_ieee(self.X_cmd,double=True)
        self.Y_cmd = self.Y_cmd[0]
        self.Z_cmd = self.Z_cmd[0] 
        self.A_cmd = self.A_cmd[0]


        #Speed
        self.Speed = self.readBusVal(0x03,0x112,2)
        self.Speed = self.Speed[0]

        #DO
        self.o1 = self.readBusVal(0x01,0x201,1)
        self.o2 = self.readBusVal(0x01,0x202,1)
        self.o3 = self.readBusVal(0x01,0x203,1)
        self.o4 = self.readBusVal(0x01,0x204,1)
        self.o1 = self.o1[0]
        self.o2 = self.o2[0]
        self.o3 = self.o3[0]
        self.o4 = self.o4[0]
        
        #self.output_control(self.o1)

        #Execution
        self.execute = self.readBusVal(0x01,0x05,1)
        self.execute = self.execute[0]
        self.execute = self.setBit(self.execute)
        #if(self.execute != self.prevExecute):
        #	self.prevExecute = self.execute
        #	DataBank.set_words(0x02,[0])
        #	print("reset_ok")

        #print(self.execute)
        #print("readComplete")

    def output_control(self,varPin):
        if (self.o1 == 1):
            self.lc.mode(linuxcnc.MODE_MDI)
            self.lc.wait_complete()
            self.lc.mdi("m64 p0")
            self.lc.wait_complete()
        elif (self.o1 == 0):
            self.lc.mode(linuxcnc.MODE_MDI)
            self.lc.wait_complete()
            self.lc.mdi("m65 p0")
            self.lc.wait_complete()
        else:
            pass


    def executionThread(self,cc):
        local_lc = linuxcnc.command()
        cmd = "g01"+" X"+str(self.X_cmd)+" Y"+str(self.Y_cmd) +" F"+str(self.Speed)
        local_lc.mode(linuxcnc.MODE_MDI)
        local_lc.wait_complete()
        local_lc.mdi(cmd)
        local_lc.wait_complete()
        #cc[0x01].setValues(0x04,0x42,[0])
        self.inExecution = 0
        time.sleep(5)
        cc.value = 0
        #self.ex_thread.terminate()

    def isRunning(self):
        retval = 0
        v = hal.get_value("halui.program.is-running")
        if v == True:
            retval = 1
        return retval


    def evaluate(self):
        if(self.machineState == 1):
            if ((self.execute == 1) and (self.h["mb-estop"] == 0)):
                #cmd = "g01"+" X"+str(self.X_cmd)+" Y"+str(self.Y_cmd)+" Z"+str(self.Z_cmd)+" F"+str(self.Speed)
                #self.execute = 0
                self.writeBusVal(0x01,0x05,0)
                #self.lc.mode(linuxcnc.MODE_MDI)
                #self.lc.wait_complete()
                #self.lc.mdi(cmd)
                #print("in exe once")
                #self.executionComplete = 1
                #self.ex_thread = threading.Thread(target=self.executionThread,name='executionThrerad')
                self.inExecution = 1
                self.inExecutionsharedVal.value = 1
                self.execute = 0
                #self.context[0x01].setValues(0x04,0x42,[1])
                self.ex_thread = multiprocessing.Process(target=self.executionThread,args=(self.inExecutionsharedVal,))
                self.ex_thread.start()
                #self.FeedbackVar['executeStatus'] = 1
                #execute linuxcnc "g01 x(self.CmdVar['xCmd']) y(self.CmdVar['yCmd']) f(self.CmdVar['SpeedCmd'])"


            if (self.isRunning() == 1):
                self.context[0x01].setValues(0x04,0x42,[1])
            else:
                self.context[0x01].setValues(0x04,0x42,[0])

            #set speed
            self.h["speed-cmd"] = self.Speed

            #set Pos Val
            self.h["x-cmd"] = self.X_cmd
            self.h["y-cmd"] = self.Y_cmd
            self.h["z-cmd"] = self.Z_cmd
            self.h["a-cmd"] = self.A_cmd
            #Setp feedback
            x,y = self.get_current_joint()
            self.h["x-fb"] = x
            self.h["y-fb"] = y

            self.X_fb = self.h["x-fb"]
            self.Y_fb = self.h["y-fb"]

            value,count = self.encode_float_and_negative(self.X_fb)
            #print("X",value)
            vx1 = value[0]
            vx2 = value[1]
            self.context[0x01].setValues(0x04,0x500,[vx1])
            self.context[0x01].setValues(0x04,0x501,[vx2])
            value,count = self.encode_float_and_negative(self.Y_fb)
            #print("Y",value)
            vy1 = value[0]
            vy2 = value[1]
            self.context[0x01].setValues(0x04,0x502,[vy1])
            self.context[0x01].setValues(0x04,0x503,[vy2])

            vel = self.get_current_vel()
            self.h["speed-fb"] = vel
            value,count = self.encode_float_and_negative(self.h["speed-fb"])
            vs1 = value[0]
            vs2 = value[1]
            self.context[0x01].setValues(0x04,0x514,[vs1])
            self.context[0x01].setValues(0x04,0x515,[vs2])

            #outputControl
            self.output_control(self.o1)

            #set Output halPins
            self.h["output-1"] = self.setBit(self.o1)
            self.h["output-2"] = self.setBit(self.o2)
            self.h["output-3"] = self.setBit(self.o3)
            self.h["output-4"] = self.setBit(self.o4)

            #set Input halPins
            self.i1 = self.getBit(self.h["input-1"]) 
            self.i2 = self.getBit(self.h["input-2"])
            self.i3 = self.getBit(self.h["input-3"])
            self.i4 = self.getBit(self.h["input-4"])

    def decode_float_and_negative(self,val):
        d = BinaryPayloadDecoder.fromRegisters(val, byteorder=_BYTEORDER_, wordorder=_WORDORDER_)
        returnVal = d.decode_32bit_float()
        return returnVal

    def encode_float_and_negative(self,val):
        builder = BinaryPayloadBuilder(byteorder=_BYTEORDER_, wordorder=_WORDORDER_)
        builder.add_32bit_float(val)
        returnVal = builder.to_registers()
        return returnVal,len(returnVal)

    def getBit(self,inObj):
        returnVal = 0
        if(inObj == 1):
            returnVal = 1
        return returnVal

    def setBit(self,inObj):
        returnVal = 0
        if(inObj == 1):
            returnVal = 1
        return returnVal


    def deviceInformation(self):
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'AshtaTech'
        identity.ProductCode = 'Br-2021011'
        identity.VendorUrl = 'http://ashtatech.com/'
        identity.ProductName = 'VSTR'
        identity.ModelName = 'BR-Flexy'
        identity.MajorMinorRevision = version.short()
        return identity

    def intializeAddress(self):
        di_block = ModbusSparseDataBlock({0x43:0,0x01:0,0x03:0,0x05:0,0x201:0,0x202:0,0x203:0,0x204:0})
        ir_block = ModbusSparseDataBlock({0x41:0,0x42:0,0x500:0,0x501:0,0x502:0,0x503:0,0x504:0,0x505:0,0x506:0,0x507:0,0x508:0,0x509:0,0x510:0,0x511:0,0x512:0,0x513:0,0x514:0,0x515:0})
        do_block = ModbusSparseDataBlock({0x02:0,0x07:0,0x08:0,0x601:0,0x602:0,0x603:0,0x604:0})
        hr_block = ModbusSparseDataBlock({0x100:0,0x101:0,0x102:0,0x103:0,0x104:0,0x105:0,0x106:0,0x107:0,0x108:0,0x109:0, 0x110:0, 0x111:0, 0x112:0,0x113:0})
        store = ModbusSlaveContext(di=di_block,ir=ir_block,do=do_block,hr=hr_block,zero_mode=True)
        #store = {0x01: store}
        context = ModbusServerContext(slaves=store, single=True)
        return context


    def initHal(self):
        self.lc = linuxcnc.command()
        self.stat = linuxcnc.stat()
        #Execution
        #self.executionComplete = self.executionComplete[0]
        self.h = hal.component("at-api")
        #print("initHAL")

        self.h.newpin("machine-state",hal.HAL_BIT,hal.HAL_IO)
        self.h.newpin("execute",hal.HAL_BIT,hal.HAL_OUT)
        self.h.newpin("mb-estop",hal.HAL_BIT,hal.HAL_IO)
        self.h.newpin("execution-complete",hal.HAL_BIT,hal.HAL_IN)
        #Command pins
        self.h.newpin("x-cmd",hal.HAL_FLOAT,hal.HAL_OUT)
        self.h.newpin("y-cmd",hal.HAL_FLOAT,hal.HAL_OUT)
        self.h.newpin("z-cmd",hal.HAL_FLOAT,hal.HAL_OUT)
        self.h.newpin("a-cmd",hal.HAL_FLOAT,hal.HAL_OUT)
        #self.h.newpin("b-cmd",hal.HAL_FLOAT,hal.HAL_OUT)
        #self.h.newpin("c-cmd",hal.HAL_FLOAT,hal.HAL_OUT)

        #Command Speed
        self.h.newpin("speed-cmd",hal.HAL_FLOAT,hal.HAL_OUT)

        #FeedBack pins
        self.h.newpin("x-fb",hal.HAL_FLOAT,hal.HAL_IN)
        self.h.newpin("y-fb",hal.HAL_FLOAT,hal.HAL_IN)
        self.h.newpin("z-fb",hal.HAL_FLOAT,hal.HAL_IN)
        self.h.newpin("a-fb",hal.HAL_FLOAT,hal.HAL_IN)
        self.h.newpin("speed-fb",hal.HAL_FLOAT,hal.HAL_IN)
        #self.h.newpin("b-fb",hal.HAL_FLOAT,hal.HAL_IN)
        #self.h.newpin("c-fb",hal.HAL_FLOAT,hal.HAL_IN)

        #Inputs
        self.h.newpin("input-1",hal.HAL_BIT,hal.HAL_IN)
        self.h.newpin("input-2",hal.HAL_BIT,hal.HAL_IN)
        self.h.newpin("input-3",hal.HAL_BIT,hal.HAL_IN)
        self.h.newpin("input-4",hal.HAL_BIT,hal.HAL_IN)
        #Outputs
        self.h.newpin("output-1",hal.HAL_BIT,hal.HAL_OUT)
        self.h.newpin("output-2",hal.HAL_BIT,hal.HAL_OUT)
        self.h.newpin("output-3",hal.HAL_BIT,hal.HAL_OUT)
        self.h.newpin("output-4",hal.HAL_BIT,hal.HAL_OUT)

        self.h.ready()

        #print("halReady")

    def initVariables(self):
        self.inExecutionsharedVal = multiprocessing.Value(ctypes.c_int,0)
        self.homeIt = [0]
        #MachineState
        self.machineState = [0]
        #Execution
        self.execute = [0]
        self.executionComplete = 0
        self.inExecution = 0
        #Command
        self.X_cmd = 0
        self.Y_cmd = 0
        self.Z_cmd = 0
        self.A_cmd = 0
        #Feedback
        self.X_fb = 0
        self.Y_fb = 0
        self.Z_fb = 0
        self.A_fb = 0
        #Speed
        self.Speed = [0]

        #DI/DO
        self.o1 = [0]
        self.o2 = [0]
        self.o3 = [0]
        self.o4 = [0]
        self.i1 = 0
        self.i2 = 0
        self.i3 = 0
        self.i4 = 0
        #print("variables ok")

        self.prevExecute = 0





if __name__=='__main__':
    startServerClass()
