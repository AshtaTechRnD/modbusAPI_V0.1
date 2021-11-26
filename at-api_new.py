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

#ADDRESSES
#CMD
_X_CMD_h = 0x100
_X_CMD_l = 0x101
_Y_CMD_h = 0x103
_Y_CMD_l = 0x104
_SPEED_CMD_h = 0x106
_SPEED_CMD_l = 0x107

_MACHINE_STATE_ = 0x01
_ESTOP_ = 0x02
_HOME_ = 0x03
_EEF_ = 0x04
_EXECUTE_ = 0x05

#Feedback
_X_FB_h = 0x200
_X_FB_l = 0x201
_Y_FB_h = 0x203
_Y_FB_l = 0x204
_SPEED_FB_h = 0x206
_SPEED_FB_l = 0x207

_MACHINE_STATUS_ = 0x01
_ESTOP_STATUS_ = 0x02
_HOME_STATUS_ = 0x03
_EEF_STATUS_ = 0x04
_EXECUTE_STATUS_ = 0x05




class startServerClass:
    def __init__(self):
        self.initVariables()
        self.initModbusInfo()
        #self.initHal()
        self.evaluationThread()
        self.startServer()

    def startServer(self):
        #inThreading
        self.readBus()
        self.writeBus()

    def readBus(self):
        self.CmdVar['machineState'] = self.readBusVal(0x01,_MACHINE_STATE_,1)[0]
        self.CmdVar['estop'] = self.readBusVal(0x01,_ESTOP_,1)[0]
        self.CmdVar['homeMachine'] = self.readBusVal(0x01,_HOME_,1)[0]
        self.CmdVar['xCmd'] = self.readBusVal(0x03,_X_CMD_h,2)
        self.CmdVar['yCmd'] = self.readBusVal(0x03,_Y_CMD_h,2)
        self.CmdVar['SpeedCmd'] = self.readBusVal(0x03,_SPEED_CMD_h,2)
        self.CmdVar['executeCmd'] = self.readBusVal(0x01,_EXECUTE_,1)
    
    def writeBus(self):
        s = self.getBit(self.FeedbackVar['machineStatus'])
        self.writeBusVal(0x01,_MACHINE_STATUS_,s)
        e = self.getBit(self.FeedbackVar['estopStatus'])
        self.writeBusVal(0x01,_ESTOP_STATUS_,e)
        h = self.getBit(self.FeedbackVar['homeState'])
        self.writeBusVal(0x01,_HOME_STATUS_,h)
        x = self.FeedbackVar['xFb']
        self.writeBusVal(0x03,_X_FB_h,x)
        y = self.FeedbackVar['yFb']
        self.writeBusVal(0x03,_Y_FB_h,y)
        v = self.FeedbackVar['currentSpeed']
        self.writeBusVal(0x03,_SPEED_CMD_h,v)
        


    def evaluationThread(self):
        if(self.CmdVar['machineState'] == 1):
            self.FeedbackVar['machineStatus'] = 1
            if(self.CmdVar['estop'] == 1):
                linuxcnc.abort()
                self.FeedbackVar['estopStatus'] = 1
            else:
                self.FeedbackVar['estopStatus'] = 0

            if(self.CmdVar['homeMachine'] == 1):
                #linuxcnc.home(-1)
                pass
            if(self.CmdVar['executeCmd'] == 1):
                ex_thread = threading.Thread(target=self.executionThread,name='executionThrerad')
                ex_thread.daemon=True
                ex_thread.start()
                self.FeedbackVar['executeStatus'] = 1
                #execute linuxcnc "g01 x(self.CmdVar['xCmd']) y(self.CmdVar['yCmd']) f(self.CmdVar['SpeedCmd'])"
    
    def executionThread(self):
        local_lc = linuxcnc.command()
        cmd = "g01"+" X"+str(self.X_cmd)+" Y"+str(self.Y_cmd) +" F"+str(self.Speed)
        local_lc.mode(linuxcnc.MODE_MDI)
        local_lc.wait_complete()
        local_lc.mdi(cmd)
        self.FeedbackVar['executeStatus'] = 0



    def initModbusInfo(self):
        #initializeAddress
        do_block = ModbusSparseDataBlock({_MACHINE_STATE_,_ESTOP_,_HOME_,_EEF_,_EXECUTE_})
        ir_block = ModbusSparseDataBlock({_X_FB_h,_X_FB_l,_Y_FB_h,_Y_FB_l,_SPEED_FB_h,_SPEED_FB_l})
        di_block = ModbusSparseDataBlock({_MACHINE_STATUS_,_ESTOP_STATUS_,_HOME_STATUS_,_EEF_STATUS_,_EXECUTE_STATUS_})
        hr_block = ModbusSparseDataBlock({_X_CMD_h,_X_CMD_l,_Y_CMD_h,_Y_CMD_l,_SPEED_CMD_h,_SPEED_CMD_l})
        store = ModbusSlaveContext(di=di_block, ir=ir_block, do=do_block, hr=hr_block, zero_mode=True)
        context = ModbusServerContext(slaves=store, single=True)
        #deviceInformation
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'AshtaTech'
        identity.ProductCode = 'Br-2021011'
        identity.VendorUrl = 'http://ashtatech.com/'
        identity.ProductName = 'AT_VSTR'
        identity.ModelName = 'BR-Flexy'
        identity.MajorMinorRevision = version.short()


    def initVariables(self):
        self.CmdVar = {'xCmd': 0,'yCmd': 0,'speedCmd': 0,'eefON':0,'executeCmd': 0,'estop': 0,'machineState': 0,'homeMachine': 0}
        self.FeedbackVar = {'xFb': 0,'yFb': 0,'currentSpeed': 0,'eefStatus':0,'executeStatus': 0,'estopStatus': 0,'machineStatus': 0,'homeState': 0}


#HELPER FUNCTIONS
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

    def get_current_joint(self):
        x = hal.get_value("joint.0.pos-fb")
        y = hal.get_value("joint.1.pos-fb")
        return x,y


    def decode_float_and_negative(self, val):
        d = BinaryPayloadDecoder.fromRegisters(val, byteorder=_BYTEORDER_, wordorder=_WORDORDER_)
        returnVal = d.decode_32bit_float()
        return returnVal

    def encode_float_and_negative(self, val):
        builder = BinaryPayloadBuilder(byteorder=_BYTEORDER_, wordorder=_WORDORDER_)
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


if __name__ == '__main__':
    startServerClass()
