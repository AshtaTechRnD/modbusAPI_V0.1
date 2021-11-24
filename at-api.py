#!/usr/bin/env python

from pymodbus.version import version
from pymodbus.server.sync import StartTcpServer, ModbusTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
# from pymodbus.transaction import ModbusRtuFramer, ModbusBinaryFramer
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder, Endian
# from twisted.internet.task import LoopingCall
import threading
import time
import hal
import linuxcnc

_BYTEORDER_ = Endian.Big
_WORDORDER_ = Endian.Big
_IP_ADDRESS_ = '10.10.1.25'
_PORT_ = 1025
_SLAVE_ID_ = 0x01
_HAL_COMP_NAME_ = "at-api"
_EEF_ON_CMD_ = "m64 p0"
_EEF_OFF_CMD_ = "m65 p0"

# 0x06 recordPos
# 0x120 x-return-recordedpos
# 0x122 y-return-recordedpos
# 0x07 getRecorded Pos Increment
# 0x08 homePin
# 0x11 homeState
# eef-ON 0x09
# 0x02 ESTOP

# import #LOGging
# LOGging.basicConfig()
# LOG = #LOGging.get#LOGger()
# LOG.setLevel(#LOGging.DEBUG)


class startServerClass:

    def __init__(self):
        self.initVariables()
        self.context = self.intializeAddress()
        self.identity = self.deviceInformation()
        self.initHal()
        self.startUpdatingThread()
        self.startTCPserver()

    def sstartTCPserver(self):
        s = threading.Thread(target=self.sstartTCPserver,
        name='sstartTCPserver')
        s.daemon = True
        s.start()

    def startTCPserver(self):
        server = StartTcpServer(
        self.context, identity=self.identity, address=(_IP_ADDRESS_, _PORT_))
        # server.start()
        # server = ModbusTcpServer(self.context, identity=self.identity, address=(_IP_ADDRESS_,_PORT_))
        # server.serve_forever()
        # try:
        #    server = StartTcpServer(self.context, identity=self.identity, address=(_IP_ADDRESS_,_PORT_))
        #    print("Server Started")
        # except:
        #    print("Server ERROR, CANNOT START")
        # finally:
        #    server.close()

    def startUpdatingThread(self):
        x = threading.Thread(target=self.updationThread, name='updationThread')
        x.daemon = True
        x.start()



    def updationThread(self):
        # c.acquire()
        while True:
            # set values to ir and di
            self.readBus()
            # time.sleep(0.1)
            self.evaluate()
            # print("Done Evaluation")
            # get values to hr and do
            # LOG.debug("Updating...")
            time.sleep(0.1)
        # pass

    def readBusVal(self, fx, addr, cnt):
        slave_id = _SLAVE_ID_
        values = self.context[slave_id].getValues(fx, addr, count=cnt)
        returnVal = values
        if (fx == 0x03):
            returnVal = [self.decode_float_and_negative(values)]
        # 0x05 write_coil single 0x15 write_multiple coil
        return returnVal

    def writeBusVal(self, fx, addr, val):
        slave_id = _SLAVE_ID_
        if ((fx == 0x04) or (fx == 0x03)):
            value, count = self.encode_float_and_negative(val)
            for i in range(count):
                self.context[slave_id].setValues(fx, addr+i, value[i])
        if(fx == 0x01):
            self.context[slave_id].setValues(fx, addr, [val])

    def readBus(self):
        # ESTOP
        self.eStop = self.readBusVal(0x01, 0x02, 1)
        self.eStop = self.eStop[0]
        self.eStop = self.setBit(self.eStop)
        self.h["mb-estop"] = self.eStop

        #homePin
        self.homeMachine = self.readBusVal(0x01, 0x08, 1)
        self.homeMachine = self.homeMachine[0]
        self.homeMachine = self.setBit(self.homeMachine)

        # eef-ON
        self.eefState = self.readBusVal(0x01,0x09,1)
        self.eefState = self.eefState[0]
        self.eefState = self.setBit(self.eefState)
        # MachineState
        self.machineState = self.readBusVal(0x01, 0x01, 1)
        self.machineState = self.machineState[0]
        self.machineState = self.setBit(self.machineState)
        self.h["machine-state"] = self.machineState

        # Record Pos
        self.recordPos = self.readBusVal(0x01,0x06,1)
        self.recordPos = self.recordPos[0]
        self.recordPos = self.setBit(self.recordPos)

        # getRecordPos
        self.getRecord = self.readBusVal(0x01,0x07,1) #0x07 getRecorded Pos Increment
        self.getRecord = self.getRecord[0]
        self.getRecord = self.setBit(self.getRecord)

        # Command
        self.X_cmd = self.readBusVal(0x03, 0x100, 2)
        self.Y_cmd = self.readBusVal(0x03, 0x103, 2)
        self.Z_cmd = self.readBusVal(0x03, 0x106, 2)
        self.A_cmd = self.readBusVal(0x03, 0x110, 2)

        self.X_cmd = self.X_cmd[0]
        # self.X_cmd = utils.decode_ieee(self.X_cmd,double=True)
        self.Y_cmd = self.Y_cmd[0]
        self.Z_cmd = self.Z_cmd[0]
        self.A_cmd = self.A_cmd[0]

        # Speed
        self.Speed = self.readBusVal(0x03, 0x112, 2)
        self.Speed = self.Speed[0]

        # DO
        self.o1 = self.readBusVal(0x01, 0x201, 1)
        self.o2 = self.readBusVal(0x01, 0x202, 1)
        self.o3 = self.readBusVal(0x01, 0x203, 1)
        self.o4 = self.readBusVal(0x01, 0x204, 1)
        self.o1 = self.o1[0]
        self.o2 = self.o2[0]
        self.o3 = self.o3[0]
        self.o4 = self.o4[0]

        # Execution
        self.execute = self.readBusVal(0x01, 0x05, 1)
        self.execute = self.exec#0x07 getRecorded Pos Incrementute[0]
        self.execute = self.setBit(self.execute)


    def evaluate(self):
        if(self.machineState == 1):
            # return Record
            k = 0
            if (self.getRecord == 1):
                if (k == (self.recordLength - 1)):
                    k = 0
                i = self.xRecord[k]
                j = self.yRecord[k]
                self.writeBusVal(0x04,0x120,i)
                self.writeBusVal(0x04,0x122,j)
                self.writeBusVal(0x01,0x07,0)
                k +=1
            # Record Pos
            if (self.recordPos == 1):
                x,y = self.get_current_joint()
                self.xRecord.append(x)
                self.yRecord.append(y)
                self.recordLength = len(self.xRecord)
                self.writeBusVal(0x01,0x06,0)
            
            # execution Status
            self.executionStatus = self.getBit(self.h["execution-complete"])
            self.writeBusVal(0x01, 0x05, self.executionStatus)

            if (self.execute == 1):
                cmd = "g01"+" X"+str(self.X_cmd)+" Y"+str(self.Y_cmd) +" F"+str(self.Speed)
                self.lc.mode(linuxcnc.MODE_MDI)
                self.lc.wait_complete()
                self.lc.mdi(cmd)
                # print(cmd)
                self.execute = 0

            # set speed
            self.h["speed-cmd"] = self.Speed

            # set Pos Val
            self.h["x-cmd"] = self.X_cmd
            self.h["y-cmd"] = self.Y_cmd
            self.h["z-cmd"] = self.Z_cmd
            self.h["a-cmd"] = self.A_cmd

            #eef
            if self.eefState == 1:
                self.lc.mode(linuxcnc.MODE_MDI)
                self.lc.wait_complete()
                self.lc.mdi(_EEF_ON_CMD_)
                self.lc.wait_complete()
            elif self.eefState == 0:
                self.lc.mode(linuxcnc.MODE_MDI)
                self.lc.wait_complete()
                self.lc.mdi(_EEF_OFF_CMD_)
                self.lc.wait_complete()
            
            #HOME IT
            if self.homeMachine == 1:
                self.lc.mode(linuxcnc.MODE_MANUAL)
                self.lc.wait_complete()
                self.lc.home(-1)
                self.lc.wait_complete()
                self.writeBusVal(0x01,0x11,1)
                self.homeState = 1
            elif self.homeState == 1:
                #AlreadyHomed
                self.writeBusVal(0x01,0x11,1)


            # set Output halPins
            self.h["output-1"] = self.setBit(self.o1)
            self.h["output-2"] = self.setBit(self.o2)
            self.h["output-3"] = self.setBit(self.o3)
            self.h["output-4"] = self.setBit(self.o4)


    def writeBus(self):
        # Setp feedback
        self.X_fb,self.Y_fb = self.get_current_joint()
        self.writeBusVal(0x04, 0x500, self.X_fb)
        self.writeBusVal(0x04, 0x502, self.Y_fb)
        self.h["x-fb"] = self.X_fb
        self.h["y-fb"] = self.Y_fb

        # set Input halPins
        self.i1 = self.getBit(self.h["input-1"])
        self.i2 = self.getBit(self.h["input-2"])
        self.i3 = self.getBit(self.h["input-3"])
        self.i4 = self.getBit(self.h["input-4"])

        # write input status to bus
        self.writeBusVal(0x01, 0x201, self.i1)
        self.writeBusVal(0x01, 0x202, self.i2)
        self.writeBusVal(0x01, 0x203, self.i3)
        self.writeBusVal(0x01, 0x204, self.i4)


    def get_current_joint(self):
        x = hal.get_value("joint.0.pos-fb")
        y = hal.get_value("joint.1.pos-fb")
        return x,y


    def decode_float_and_negative(self, val):
        d = BinaryPayloadDecoder.fromRegisters(
        val, byteorder=_BYTEORDER_, wordorder=_WORDORDER_)
        returnVal = d.decode_32bit_float()
        return returnVal

    def encode_float_and_negative(self, val):
        builder = BinaryPayloadBuilder(
        byteorder=_BYTEORDER_, wordorder=_WORDORDER_)
        builder.add_32bit_float(val)
        returnVal = builder.to_registers()
        return returnVal, len(returnVal)

    def getBit(self, inObj):
        returnVal = 0
        if(inObj == 1):
            returnVal = 1 
        return returnVal

    def setBit(self, inObj):
        returnVal = 0
        if(inObj == 1):
            returnVal = 1
        return returnVal

    def deviceInformation(self):
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'AshtaTech'
        identity.ProductCode = 'Br-2021011'
        identity.VendorUrl = 'http://ashtatech.com/'
        identity.ProductName = 'AT_VSTR'
        identity.ModelName = 'BR-Flexy'
        identity.MajorMinorRevision = version.short()
        return identity

    def intializeAddress(self):
        di_block = ModbusSparseDataBlock({0x01: 0, 0x05: 0,0x06: 0,0x07: 0, 0x09:0, 0x11: 0, 0x201: 0, 0x202: 0, 0x203: 0, 0x204: 0})
        ir_block = ModbusSparseDataBlock({0x500: 0, 0x501: 0, 0x502: 0, 0x503: 0, 0x504: 0, 0x505: 0, 0x506: 0, 0x507: 0, 0x508: 0, 0x509: 0, 0x510: 0, 0x511: 0, 0x512: 0, 0x513: 0, 0x120: 0, 0x122: 0})
        do_block = ModbusSparseDataBlock({0x02: 0, 0x07: 0,0x08:0, 0x601: 0, 0x602: 0, 0x603: 0, 0x604: 0})
        hr_block = ModbusSparseDataBlock({0x100: 0, 0x101: 0, 0x102: 0, 0x103: 0, 0x104: 0, 0x105: 0, 0x106: 0, 0x107: 0, 0x108: 0, 0x109: 0, 0x110: 0, 0x111: 0, 0x112: 0, 0x113: 0})
        store = ModbusSlaveContext(di=di_block, ir=ir_block, do=do_block, hr=hr_block, zero_mode=True)
        context = ModbusServerContext(slaves=store, single=True)
        return context

    def initHal(self):
        self.lc_stat = linuxcnc.stat()
        self.lc = linuxcnc.command()
        # Execution
        # self.executionStatus = self.executionStatus[0]
        self.h = hal.component(_HAL_COMP_NAME_)
        # print("initHAL")

        self.h.newpin("machine-state", hal.HAL_BIT, hal.HAL_IO)
        self.h.newpin("execute", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("execution-complete", hal.HAL_BIT, hal.HAL_IN)
        self.h.newpin("mb-estop", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("record-pos",hal.HAL_BIT,hal.HAL_OUT)
        # Command pins
        self.h.newpin("x-cmd", hal.HAL_FLOAT, hal.HAL_OUT)
        self.h.newpin("y-cmd", hal.HAL_FLOAT, hal.HAL_OUT)
        self.h.newpin("z-cmd", hal.HAL_FLOAT, hal.HAL_OUT)
        self.h.newpin("a-cmd", hal.HAL_FLOAT, hal.HAL_OUT)
        # self.h.newpin("b-cmd",hal.HAL_FLOAT,hal.HAL_OUT)
        # self.h.newpin("c-cmd",hal.HAL_FLOAT,hal.HAL_OUT)

        # Command Speed
        self.h.newpin("speed-cmd", hal.HAL_FLOAT, hal.HAL_OUT)

        # FeedBack pins
        self.h.newpin("x-fb", hal.HAL_FLOAT, hal.HAL_IN)
        self.h.newpin("y-fb", hal.HAL_FLOAT, hal.HAL_IN)
        self.h.newpin("z-fb", hal.HAL_FLOAT, hal.HAL_IN)
        self.h.newpin("a-fb", hal.HAL_FLOAT, hal.HAL_IN)
        # self.h.newpin("b-fb",hal.HAL_FLOAT,hal.HAL_IN)
        # self.h.newpin("c-fb",hal.HAL_FLOAT,hal.HAL_IN)

        # Inputs
        self.h.newpin("input-1", hal.HAL_BIT, hal.HAL_IN)
        self.h.newpin("input-2", hal.HAL_BIT, hal.HAL_IN)
        self.h.newpin("input-3", hal.HAL_BIT, hal.HAL_IN)
        self.h.newpin("input-4", hal.HAL_BIT, hal.HAL_IN)
        # Outputs
        self.h.newpin("output-1", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("output-2", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("output-3", hal.HAL_BIT, hal.HAL_OUT)
        self.h.newpin("output-4", hal.HAL_BIT, hal.HAL_OUT)

        self.h.ready()

        # print("halReady")

    def initVariables(self):
        # ESTOP
        self.eStop = [0]
        # MachineState
        self.machineState = [0]
        # Execution
        self.execute = [0]
        self.executionStatus = 0
        # Command
        self.X_cmd = 0
        self.Y_cmd = 0
        self.Z_cmd = 0
        self.A_cmd = 0
        # Feedback
        self.X_fb = 0
        self.Y_fb = 0
        self.Z_fb = 0
        self.A_fb = 0
        # Speed
        self.Speed = [0]

        # DI/DO
        self.o1 = [0]
        self.o2 = [0]
        self.o3 = [0]
        self.o4 = [0]
        self.i1 = 0
        self.i2 = 0
        self.i3 = 0
        self.i4 = 0
        # print("variables ok")

        self.prevExecute = 0
        self.recordPos = 0
        self.xRecord = []
        self.yRecord = []
        self.xrec_return = 0
        self.yrec_return = 0
        self.recordLength = 0
        self.getRecord = 0
        self.eefState = 0
        self.homeMachine = 0
        self.homeState = 0


if __name__ == '__main__':
    startServerClass()
    # try:
    #    startServerClass()
    # except:
    #    print("Error: Cannot Start Server")
    # finally:
    #    print("Server Closed")
