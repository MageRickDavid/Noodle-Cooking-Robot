import serial, time, ctypes
import serial.tools.list_ports
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


# SDK基础类
class ClawControl():
    def __init__(self):
        self.master = None
    
    def setMaster(self, master):
        self.master = master
    
    def getMaseter(self):
        return self.master

    def getS16(self, val):
        return ctypes.c_int16(val).value
    
    def getS8(self, val):
        return ctypes.c_int8(val).value

    def searchCom(self):
        """
            查询可用的串口
        """
        comList = []
        plist = list(serial.tools.list_ports.comports()) # 查询可用的串口
        for i in range(len(plist)):
            plist_0 = list(plist[i])
            comList.append(str(plist_0[0]))
        return comList

    def serialOperation(self, com, baudRate, state):
        """
            串口操作
            参数说明：
                com:串口号
                baudRate:波特率
                state:开关状态(打开:True, 关闭:False)
        """
        try:
            if state:
                self.master = modbus_rtu.RtuMaster(serial.Serial(port=com, baudrate=int(baudRate), bytesize=8, parity='N', stopbits=1))
                self.master.set_timeout(0.1)
                self.master.set_verbose(True)
            else:
                self.master._do_close()
            reVal = 1
            
        except Exception as exc:
            reVal = str(exc)
        
        return reVal
    
    def setBitValue(self, byte, index, val):
        """
            Change the value of a certain bit in a byte
            Parameter description:
                byte: The original value of the byte to be changed
                index: The sequence number of the bit to be changed, starting from 0 from right to left, 0-7 is the 8 bits of a complete byte
                val: the pre-changed value of the target bit, 0 or 1
        """
        if val:
            return byte | (1 << index)
        else:
            return byte & ~(1 << index)

    def readRegisterData(self, salveId, readMode, address, count):
        """
            Status query
            Parameter description:
                salveId: Gripper site id
                address:modbus register address
                readMode: modbus read status command number 03 or 04
                count: the number of registers to read
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, readMode, address, count) 
            return readBuf
        except Exception as exc:
            return str(exc)
    
    def writeDataToRegister(self, salveId, address, sendCmdBuf):
        """
            Write data to register
            Parameter description:
                salveId: Gripper site id
                address: modbus register starting address
                sendCmdBuf: written data (the format is a list, the length of the list represents the number of written registers)
        """
        if not self.master:
            return "Communication not connected"
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, address, output_value=sendCmdBuf) 
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def runWithoutParam(self, salveId, cmdId):
        """
            Run command without parameters
            Parameter description:
                salveId: Gripper site id
                cmdId: No parameter command number
        """
        if not self.master:
            return "Communication not connected"     
        try:
            sendCmdBuf = [0x000B]
            sendCmdBuf[0] |= cmdId << 8
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def runWithParam(self, salveId, pos, speed, torque):
        """
            Run command with parameters
            Parameter description:
                salveId: Gripper site id
                pos: running target location
                speed: maximum speed during operation
                torque: torque when grabbing an object
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x0009, 0x0000, 0x000]
            sendCmdBuf[1] = pos << 8
            sendCmdBuf[2] = speed | torque << 8   
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def getDeviceCurrentTemperature(self, salveId):
        """
            Get the current temperature of the device
            Parameter description:
                salveId: Gripper site id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D3, 1)
            return [readBuf[0] >> 8]
        except Exception as exc:
            return str(exc)
    
    def getDeviceCurrentVoltage(self, salveId):
        """
            Get the current voltage of the device
            Parameter description:
                salveId: Gripper site id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D3, 1)
            return [readBuf[0] & 0xff]
        except Exception as exc:
            return str(exc)
    
    def readSoftwareVersion(self, salveId):
        """
            Read software version
            Parameter description:
                salveId: Gripper site id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x138C, 1)
            return ["V" + str(readBuf[0] >> 8) + "." + str(readBuf[0] & 0xff)]
        except Exception as exc:
            return str(exc)

    def changeSalveId(self, oldId, newId):
        """
            Modify gripper site ID
            Parameter description:
                oldId: current id
                newId: modified id
        """
        if not self.master:
            return "Communication not connected"
        try: 
            sendCmdBuf = [newId]
            self.master.execute(oldId, cst.WRITE_MULTIPLE_REGISTERS, 0x138D, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def scanSalveId(self, startId, stopId):
        """
            Scan the connected gripper ID
            Parameter description:
                startId: start ID
                stopId: Termination ID
        """
        if not self.master:
            return "Communication not connected"
        idList = []
        for myId in range(startId, stopId+1):
            try:
                self.master.execute(myId, cst.READ_INPUT_REGISTERS, 0x07D0, 1)
                idList.append(myId)
                time.sleep(0.02)
            except Exception as exc:
                pass
        return idList

    def changeBaudRate(self, salveId, baudRate):
        """
            Modify gripper baud rate
            Parameter description:
                salveId: Gripper site ID
                baudRate: Target baud rate number (0:115200, 1:57600, 2:38400, 3:19200, 4:9600, 5:4800)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [baudRate]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x138E, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def writeCustomParam(self, salveId, address, pos, speed, torque):
        """
            Write custom parameters
            Parameter description:
                salveId: Gripper site ID
                address: modbus register starting address
                pos: default position
                speed: preset speed
                torque: preset torque
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0, 0]
            sendCmdBuf[0] = int(pos) | (int(speed) << 8)
            sendCmdBuf[1] = int(torque)
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, address, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

# EPG类型SDK
class EpgClawControl(ClawControl):
    def __init__(self):
        super().__init__()
        self.clampState = {

            0: 'The finger is moving towards the specified position',
            1: 'Object detected when finger is opened',
            2: 'The object is detected when the finger is closed',
            3: 'The finger has reached the specified position and no object was detected'
        }
    
    def enableClamp(self, salveId, state):
        """
            Clamping enable
            Parameter description:
                salveId: Gripper site id
                state: Whether to enable (enable: True, disable: False)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x01] if state else [0x00]             
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def stopClaw(self, salveId):
        """
            Stop gripper movement
            Parameter description:
                salveId: Gripper site ID
        """
        if not self.master:
            return "Communication not connected"    
        try:
            sendCmdBuf = [0x0001]  
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FF, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def forceOpenBrake(self, salveId, state):
        """
            Forced opening of the brake
            Parameter description:
                salveId: Gripper site ID
                state: switch state (on: True, off: False)
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [0x0001]  if state else [0x0000]
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FC, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def switchMode(self, salveId, modeIndex):
        """
            Select mode
            Parameter description:
                salveId: Gripper site ID
                modeIndex: mode number (0: serial communication mode, 1: IO mode)
        """
        if not self.master:
            return "Communication not connected"  
        if modeIndex == 0:  
            sendCmdBuf = [0x00]
        elif modeIndex == 1:                                        
            sendCmdBuf = [0x55] 
        else:
            sendCmdBuf = [0x00]
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FD, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def switchInputState(self, salveId, state):
        """
            Select IO input status
            Parameter description:
                salveId: Gripper site ID
                state: IO input positive and negative logic (False: positive logic, True: negative logic)
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf= [0x01] if state else [0x00]        
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FE, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def switchOutputState(self, salveId, state):
        """
            Select IO output status
            Parameter description:
                salveId: Gripper site ID
                state: IO output positive and negative logic (False: positive logic, True: negative logic)
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf= [0x01] if state else [0x00]       
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FF, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def readSoftwareVersion(self, salveId):
        """
            读取软件版本
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x1388, 5)
            if readBuf[0] == 0:
                return ["V" + str(readBuf[4] >> 8) + "." + str(readBuf[4] & 0xff)]
            else:
                return ["V" + str(readBuf[0]) + "." + str(readBuf[4])]
        except Exception as exc:
            return str(exc)
    
    def getClampCurrentState(self, salveId):
        """
            Get the current status of the clamping end
            Parameter description:
                salveId: Gripper site id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D0, 1)
            return [self.clampState[(readBuf[0] >> 6) & 0x03]]
        except Exception as exc:
            return str(exc)
    
    def getClampCurrentLocation(self, salveId):
        """
            Get the current position of the clamping end
            Parameter description:
                salveId: Gripper site id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D1, 1)
            return [readBuf[0] >> 8]
        except Exception as exc:
            return str(exc)
    
    def getClampCurrentSpeed(self, salveId):
        """
            Get the current speed of the clamping end
            Parameter description:
                salveId: Gripper site id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D2, 1)
            return [readBuf[0] & 0xff]
        except Exception as exc:
            return str(exc)
    
    def getClampCurrentTorque(self, salveId):
        """
            Get the current torque of the clamping end
            Parameter description:
                salveId: Gripper site id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D2, 1)
            return [readBuf[0] >> 8]
        except Exception as exc:
            return str(exc)


# RG类型SDK
class RgClawControl(EpgClawControl):
    def __init__(self):
        super().__init__()


# EPG-HP类型SDK
class EpgHpClawControl(EpgClawControl):
    def __init__(self):
        super().__init__()


# RCG类型SDK
class RcgClawControl(EpgClawControl):
    def __init__(self):
        super().__init__()

    def switchMode(self, salveId, modeIndex):
        """
            选择模式
            参数说明：
                salveId:夹爪站点ID
                modeIndex:模式编号(0:串口通讯模式, 1:IO模式)
        """
        if not self.master:
            return "Communication not connected"  
    
    def switchInputState(self, salveId, state):
        """
            选择IO输入状态
            参数说明：
                salveId:夹爪站点ID
                state:IO输入正反逻辑(False:正逻辑, True:反逻辑)
        """
        if not self.master:
            return "Communication not connected"

    def switchOutputState(self, salveId, state):
        """
            选择IO输出状态
            参数说明：
                salveId:夹爪站点ID
                state:IO输出正反逻辑(False:正逻辑, True:反逻辑)
        """
        if not self.master:
            return "Communication not connected"
    

# LEPG类型SDK
class LepgClawControl(EpgClawControl):
    def __init__(self):
        super().__init__()

    def enableCollision(self, salveId, state):
        """
            碰撞使能
            参数说明：
                salveId:夹爪站点ID
                state:使能状态(使能:True, 去使能:False)
        """
        if not self.master:
            return "Communication not connected"    
        sendCmdBuf = [0x55] if state else [0x00]        
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FE, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def setCollisionThreshold_1(self, salveId, data):
        """
            设置碰撞阈值1
            参数说明：
                salveId:夹爪站点ID
                data:碰撞阈值
        """
        if not self.master:
            return "Communication not connected"  
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FF, output_value=[data])
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def setCollisionThreshold_2(self, salveId, data):
        """
            设置碰撞阈值2
            参数说明：
                salveId:夹爪站点ID
                data:碰撞阈值
        """
        if not self.master:
            return "Communication not connected"  
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0400, output_value=[data])
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setCollisionThreshold_3(self, salveId, data):
        """
            设置碰撞阈值3
            参数说明：
                salveId:夹爪站点ID
                data:碰撞阈值
        """
        if not self.master:
            return "Communication not connected"  
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0401, output_value=[data])
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setDropThreshold(self, salveId, data):
        """
            掉落检测阈值设置
            参数说明：
                salveId:夹爪站点ID
                data:掉落检测阈值
        """
        if not self.master:
            return "Communication not connected"  
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0403, output_value=[data])
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def searchRange(self, salveId):
        """
            搜索行程
            参数说明：
                salveId:夹爪站点ID
        """
        if not self.master:
            return "Communication not connected" 
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0402, output_value=[0x01])
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def manualSavePosition(self, salveId):
        """
            手动保存位置
            参数说明：
                salveId:夹爪站点ID
        """
        if not self.master:
            return "Communication not connected" 
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0404, output_value=[0x01])
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def switchInputState(self, salveId, state):
        """
            选择IO输入状态
            参数说明：
                salveId:夹爪站点ID
                state:IO输入正反逻辑(False:正逻辑, True:反逻辑)
        """
        if not self.master:
            return "Communication not connected"

    def switchOutputState(self, salveId, state):
        """
            选择IO输出状态
            参数说明：
                salveId:夹爪站点ID
                state:IO输出正反逻辑(False:正逻辑, True:反逻辑)
        """
        if not self.master:
            return "Communication not connected"
    
    def setPreImpact(self, salveId, data):
        """
            设定预撞击点
            参数说明：
                salveId:夹爪站点ID
                data:设定预撞击点位置
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [data]             
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0405, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def setPreImpactSpeed(self, salveId, data):
        """
            设定预撞击点速度
            参数说明：
                salveId:夹爪站点ID
                data:预撞击点速度
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [data]             
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0406, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal


# EVS01类型SDK
class EvsClawControl_01(ClawControl):
    def __init__(self):
        super().__init__()
        self.deviceState = {
            0: "气压低于最低气压",
            1: "检测到工件，最低压力值已达到",
            2: "检测到工件，最高压力值已达到",
            3: "没有检测到对象，物体已丢失或未夹住脱落，或超时"
        }
    
    def enableDevice(self, salveId, state):
        """
            设备使能
            参数说明：
                salveId:夹爪站点id
                state:是否使能(使能:True, 去使能:False)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x01] if state else [0x00]             
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def runWithoutParam(self, salveId, cmdId):
        """
            无参数运行指令
            参数说明：
                salveId:夹爪站点id
                cmdId: 无参数指令编号
        """
        pass

    def runWithParam(self, salveId, pos, speed, torque):
        """
            有参数运行指令
            参数说明：
                salveId:夹爪站点id
                pos:运行目标位置
                speed:运行中最大速度
                torque:抓取物体时的力矩
        """
        pass
    
    def writeCustomParam(self, salveId, maxPressure, minPressure, timeout):
        """
            写入自定义参数
            参数说明：
                salveId:夹爪站点ID
                maxPressure:最大气压值
                minPressure:最小气压值
                timeout:超时时间
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x00, 0x00]
            maxPressure = 100 - maxPressure
            minPressure = 100 - minPressure
            sendCmdBuf[0] = maxPressure << 8
            sendCmdBuf[1] = (minPressure << 8) | (timeout & 0xff)
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E9, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def startOrStopDevice(self, salveId, mode, runFlag, breakState):
        """
            启动或停止设备
            参数说明：
                salveId: 夹爪站点id
                mode: 运行模式,0为自动控制,1为高级模式
                runFlag: 启停标志, 0为停止, 1为运行
                breakState: 破真空标志, True为破真空, False为不破真空
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x01]
            sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 1, mode)
            sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 2, 1) if breakState else self.setBitValue(sendCmdBuf[0], 2, 0)
            sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 3, runFlag)
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def switchMode(self, salveId, modeIndex):
        """
            选择模式
            参数说明：
                salveId:夹爪站点ID
                modeIndex:模式编号(0:串口通讯模式, 1:IO模式)
        """
        if not self.master:
            return "Communication not connected"  
        if modeIndex == 0:  
            sendCmdBuf = [0x00]
        elif modeIndex == 1:                                        
            sendCmdBuf = [0x55] 
        else:
            sendCmdBuf = [0x00]       
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03ED, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def readSoftwareVersion(self, salveId):
        """
            读取软件版本
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x138C, 1)
            return ["V" + str(readBuf[0] >> 8) + "." + str(readBuf[0] & 0xff)]
        except Exception as exc:
            return str(exc)
    
    def getDeviceCurrentState(self, salveId):
        """
            获取设备当前状态
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D0, 1)
            evsState = (readBuf[0] & 0xff) >> 6
            return [self.deviceState[evsState]]
        except Exception as exc:
            return str(exc)
    
    def getDeviceCurrentVacuumDegree(self, salveId):
        """
            获取设备当前真空度
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D2, 1)
            return [(readBuf[0] >> 8) - 100]
        except Exception as exc:
            return str(exc)

    
    def getDeviceCurrentTemperature(self, salveId):
        """
            获取设备当前温度
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D6, 1)
            return [readBuf[0] >> 8]
        except Exception as exc:
            return str(exc)
    
    def getDeviceCurrentVoltage(self, salveId):
        """
            获取设备当前电压
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D6, 1)
            return [readBuf[0] & 0xff]
        except Exception as exc:
            return str(exc)


# EVS08类型SDK
class EvsClawControl_08(EvsClawControl_01):
    def __init__(self):
        super().__init__()
    
    def writeCustomParam(self, salveId, channelNo, maxPressure, minPressure, timeout):
        """
            写入自定义参数
            参数说明：
                salveId:夹爪站点ID
                channelNo: 通道号(1: 通道1, 2:通道2)
                maxPressure:最大气压值
                minPressure:最小气压值
                timeout:超时时间
        """
        if not self.master:
            return "Communication not connected"
        try:
            if channelNo == 1:
                address = 0x03E9
            elif channelNo == 2:
                address = 0x03EB
            else:
                address = 0x03E9
            sendCmdBuf = [0x00, 0x00]
            maxPressure = 100 - maxPressure
            minPressure = 100 - minPressure
            setTime = int(timeout * 10)
            sendCmdBuf[0] = maxPressure << 8
            sendCmdBuf[1] = (minPressure << 8) | (setTime & 0xff)
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, address, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def startOrStopDevice(self, salveId, mode, runFlag, channelIndex, breakState):
        """
            启动或停止设备
            参数说明：
                salveId: 夹爪站点id
                mode: 运行模式,0为自动控制,1为高级模式
                runFlag: 启停标志, 0为停止, 1为运行
                channelIndex: 通道个数, 0为所有通道, 1为通道1, 2为通道2
                breakState: 破真空标志, True为破真空, False为不破真空
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x01]
            if channelIndex == 0:
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 1, mode)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 2, 1) if breakState else self.setBitValue(sendCmdBuf[0], 2, 0)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 3, runFlag)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 4, mode)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 5, 1) if breakState else self.setBitValue(sendCmdBuf[0], 5, 0)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 6, runFlag)
            elif channelIndex == 1:
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 1, mode)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 2, 1) if breakState else self.setBitValue(sendCmdBuf[0], 2, 0)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 3, runFlag)
            elif channelIndex == 2:
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 4, mode)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 5, 1) if breakState else self.setBitValue(sendCmdBuf[0], 5, 0)
                sendCmdBuf[0] = self.setBitValue(sendCmdBuf[0], 6, runFlag)
            else:
                pass
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def getDeviceCurrentState(self, salveId, channelNo):
        """
            获取设备当前状态
            参数说明：
                salveId:夹爪站点id
                channelNo: 通道号(1: 通道1, 2:通道2)
        """
        if not self.master:
            return "Communication not connected"
        try:
            if channelNo == 1:
                address = 0x07D0
            elif channelNo == 2:
                address = 0x07D3
            else:
                address = 0x07D0
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, address, 1)
            evsState = (readBuf[0] & 0xff) >> 6
            return [self.deviceState[evsState]]
        except Exception as exc:
            return str(exc)
    
    def getDeviceCurrentVacuumDegree(self, salveId, channelNo):
        """
            获取设备当前真空度
            参数说明：
                salveId:夹爪站点id
                channelNo: 通道号(1: 通道1, 2:通道2)
        """
        if not self.master:
            return "Communication not connected"
        try:
            if channelNo == 1:
                address = 0x07D2
            elif channelNo == 2:
                address = 0x07D5
            else:
                address = 0x07D2
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, address, 1)
            return [(readBuf[0] >> 8) - 100]
        except Exception as exc:
            return str(exc)


# ERG32类型SDK
class ErgClawControl(ClawControl):
    def __init__(self):
        super().__init__()
        self.clampState = {
            0: '初始状态',
            1: '手指正向指定位置移动',
            2: '手指在打开方向运动时，由于接触到物体已经停止',
            3: '手指在闭合方向运动时，由于接触到物体已经停止',
            4: '手指打开方向到达指定的位置，但没有检测到对象',
            5:'手指闭合方向到达指定的位置，但没有检测到对象'
        }
        self.rotateStatus = {
            0: '初始状态',
            1: '夹爪正向指定位置转动',
            2: '夹爪在顺时针方向运动时，由于受到阻力已经停止,拧紧',
            3: '夹爪在逆时针方向运动时，由于受到阻力已经停止，未拧开',
            4: '夹爪顺时针方向旋转到达指定的位置',
            5: '夹爪逆时针方向旋转到达指定的位置'
        }
    
    def enableClamp(self, salveId, state):
        """
            夹持使能
            参数说明：
                salveId:夹爪站点id
                state:是否使能(使能:True, 去使能:False)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x01] if state else [0x00]             
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def enableRotate(self, salveId, state):
        """
            夹爪旋转使能
            参数说明：
                salveId:夹爪站点id
                state:是否使能(使能:True,去使能:False)
        """
        if not self.master:
            return "Communication not connected"  
        try:
            sendCmdBuf = [0x01] if state else [0x00]  
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E9, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def runWithParam(self, salveId, pos, speed, torque):
        """
            有参数运行指令
            参数说明：
                salveId:夹爪站点id
                pos:运行目标位置
                speed:运行中最大速度
                torque:抓取物体时的力矩
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x0000, 0x0000]
            sendCmdBuf[0] = speed << 8 | pos
            sendCmdBuf[1] = torque << 8 | 0x01   
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03EA, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def rotateWithoutParam(self, salveId, cmdId):
        """
            无参数旋转指令
            参数说明：
                salveId:夹爪站点id
                cmdId:无参数指令编号
        """
        if not self.master:
            return "Communication not connected"      
        try:
            sendCmdBuf = [0x000B]
            sendCmdBuf[0] |= cmdId << 8
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E9, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def rotateWithParam(self, salveId, angle, speed, torque, absStatus, cycleNum=0):
        """
            有参数旋转指令
            参数说明：
                salveId:夹爪站点id
                angle:旋转角度
                speed:旋转时最大速度
                torque:旋转时的力矩
                absState:是否按绝对位置进行旋转(是:True, 否:False)
                cycleNum:圈数（配合绝对位置旋转使用）
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x0009, 0x0000, 0x0000, 0x0000]
            sendCmdBuf[0] = angle
            sendCmdBuf[1] = speed | torque << 8
            sendCmdBuf[2] = angle
            if absState:
                sendCmdBuf[3] = ctypes.c_int8(cycleNum).value << 8 | 0x01            
            else:
                sendCmdBuf[3] = 0x02

            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03EC, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def scanSalveId(self, startId, stopId):
        """
            扫描连接的夹爪ID
            参数说明：
                startId:起始ID
                stopId:终止ID
        """
        if not self.master:
            return "Communication not connected"
        idList = []
        for myId in range(startId, stopId+1):
            try:
                self.master.execute(myId, cst.READ_HOLDING_REGISTERS, 0x07D0, 4)
                idList.append(myId)
                myId += 1
                time.sleep(0.02)
            except Exception as exc:
                print(str(exc))
        return idList
    
    def changeBaudRate(self, salveId, baudRate):
        """
            修改夹爪波特率
            参数说明：
                salveId:夹爪站点ID
                baudRate:目标波特率编号(0:115200, 1:57600, 2:38400, 3:19200, 4:9600, 5:4800)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [baudRate]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x1395, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def getClampCurrentState(self, salveId):
        """
            获取夹持端当前状态
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D0, 1)
            return [self.clampState[(readBuf[0] & 0xff) >> 5]]
        except Exception as exc:
            return str(exc)
    
    def getClampCurrentLocation(self, salveId):
        """
            获取夹持端当前位置
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D2, 1)
            return [readBuf[0] & 0xff]
        except Exception as exc:
            return str(exc)
    
    def getClampCurrentSpeed(self, salveId):
        """
            获取夹持端当前速度
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D2, 1)
            return [readBuf[0] >> 8]
        except Exception as exc:
            return str(exc)
    
    def getClampCurrentTorque(self, salveId):
        """
            获取夹持端当前力矩
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D3, 1)
            return [readBuf[0] >> 8]
        except Exception as exc:
            return str(exc)
    
    def getDeviceCurrentTemperature(self, salveId):
        """
            获取设备当前温度
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D8, 1)
            return [readBuf[0] >> 8]
        except Exception as exc:
            return str(exc)
    
    def getDeviceCurrentVoltage(self, salveId):
        """
            获取设备当前电压
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D8, 1)
            return [readBuf[0] & 0xff]
        except Exception as exc:
            return str(exc)
    
    def readSoftwareVersion(self, salveId):
        """
            查询软件版本
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x9101, 3)
            return ["V" + str(readBuf[0]) + "." + str(readBuf[1]) +"." + str(readBuf[2])]
        except Exception as exc:
            return str(exc)

    def stopClampMotion(self, salveId):
        """
            停止夹持运动
            参数说明：
                salveId:夹爪站点ID
        """
        if not self.master:
            return "Communication not connected"    
        try:
            sendCmdBuf = [0x0011]  
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def getRotateCurrentState(self, salveId):
        """
            获取旋转端当前状态
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D1, 1)
            return [self.rotateStatus[(readBuf[0] & 0xff) >> 5]]
        except Exception as exc:
            return str(exc)
    
    def getAbsoluteAngle(self, salveId):
        """
            获取旋转端绝对角度
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D4, 1)
            return [self.getS16(readBuf[0])]
        except Exception as exc:
            return str(exc)
    
    def getRelativeAngle(self, salveId):
        """
            获取旋转端相对角度
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D7, 1)
            return [self.getS16(readBuf[0])]
        except Exception as exc:
            return str(exc)
    
    def getAbsoluteNumberOfTurns(self, salveId):
        """
            获取旋转端绝对圈数
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D6, 1)
            return [self.getS8(readBuf[0] >> 8)]
        except Exception as exc:
            return str(exc)
    
    def getRotateCurrentSpeed(self, salveId):
        """
            获取旋转端当前速度
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D5, 1)
            return [readBuf[0] & 0xff]
        except Exception as exc:
            return str(exc)
    
    def getRotateCurrentTorque(self, salveId):
        """
            获取旋转端当前力矩
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D5, 1)
            return [readBuf[0] >> 8]
        except Exception as exc:
            return str(exc)
    
    def stopRotateMotion(self, salveId):
        """
            停止旋转运动
            参数说明：
                salveId:夹爪站点ID
        """
        if not self.master:
            return "Communication not connected"    
        try:
            sendCmdBuf = [0x0011]  
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E9, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def forceOpenBrake(self, salveId, state):
        """
            强制打开抱闸
            参数说明：
                salveId:夹爪站点ID
                state:开关状态(开:True,关:False)
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [0x0001] if state else [0x0000]
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x1397, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setRotateToZero(self, salveId, angle):
        """
            旋转端设置零点偏置
            参数说明：
                salveId:夹爪站点ID
                angle:对零角度
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [self.getS16(angle)]           
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0410, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def switchMode(self, salveId, modeIndex):
        """
            选择模式
            参数说明：
                salveId:夹爪站点ID
                modeIndex:模式编号(0:串口通讯模式, 1:IO模式)
        """
        if not self.master:
            return "Communication not connected"  
        if modeIndex == 0:  
            sendCmdBuf = [0x00]
        elif modeIndex == 1:                                        
            sendCmdBuf = [0x55] 
        else:
            sendCmdBuf = [0x00]
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0413, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal


# ERG08类型SDK
class ErgClawControl_08(ErgClawControl):
    def __init__(self):
        super().__init__()
        self.clampState = {
            0: '手指正向指定位置移动', 
            1: '手指在张开检测到物体', 
            2: '手指在闭合检测到物体', 
            3: '手指已到达指定的位置，没有检测到物体'
        }
        self.rotateStatus = {
            0: '初始状态',
            1: '夹爪正向指定位置转动',
            2: '夹爪在顺时针方向运动时，由于受到阻力已经停止，拧紧',
            3: '夹爪在逆时针方向运动时，由于受到阻力已经停止，未拧开',
            4: '夹爪顺时针方向旋转到达指定的位置',
            5: '夹爪逆时针方向旋转到达指定的位置'
        }

    def changeBaudRate(self, salveId, baudRate):
        """
            修改夹爪波特率
            参数说明：
                salveId:夹爪站点ID
                baudRate:目标波特率编号(0:115200, 1:57600, 2:38400, 3:19200, 4:9600, 5:4800)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [baudRate]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x138E, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def stopClampMotion(self, salveId):
        """
            停止夹持运动
            参数说明：
                salveId:夹爪站点ID
        """
        if not self.master:
            return "Communication not connected"
    
    def stopRotateMotion(self, salveId):
        """
            停止旋转运动
            参数说明：
                salveId:夹爪站点ID
        """
        if not self.master:
            return "Communication not connected"    
        try:
            sendCmdBuf = [0x0001]  
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E9, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def getClampCurrentState(self, salveId):
        """
            获取夹持端当前状态
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x07D0, 1)
            return [self.clampState[(readBuf[0] & 0xff) >> 6]]
        except Exception as exc:
            return str(exc)
    
    def readSoftwareVersion(self, salveId):
        """
            查询软件版本
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x138C, 1)
            return ["V" + str(readBuf[0] >> 8) + "." + str(readBuf[0] & 0xff)]
        except Exception as exc:
            return str(exc)
    
    def switchMode(self, salveId, modeIndex):
        """
            选择模式
            参数说明：
                salveId:夹爪站点ID
                modeIndex:模式编号(0:串口通讯模式, 1:IO模式)
        """
        if not self.master:
            return "Communication not connected"
    
    def forceOpenBrake(self, salveId, state):
        """
            强制打开抱闸
            参数说明：
                salveId:夹爪站点ID
                state:开关状态(开:True,关:False)
        """
        if not self.master:
            return "Communication not connected"


# ZRG类型SDK
class ZrgClawControl(ErgClawControl_08):
    def __init__(self):
        super().__init__()
        self.zAxisState = {
            0: '设备正向指定位置移动', 
            1: '设备在上升过程中检测到物体', 
            2: '设备在下降过程中检测到物体', 
            3: '设备已到达指定的位置，没有检测到物体'
        }
    
    def enableZAxis(self, salveId, state, mode):
        """
            Z轴使能
            参数说明：
                salveId:夹爪站点id
                state:是否使能(使能:True, 去使能:False)
                mode:是否跟随(跟随模式:True, 非跟随模式:False)
        """
        if not self.master:
            return "Communication not connected"  
        try:
            if state:
                sendCmdBuf = [0x05 if mode else 0x01]
            else:
                sendCmdBuf = [0x04 if mode else 0x00]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0BB8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def runZAxisWithoutParam(self, salveId, cmdId, mode):
        """
            无参数旋转指令
            参数说明：
                salveId:夹爪站点id
                cmdId:无参数指令编号
                mode:是否跟随(跟随模式:True, 非跟随模式:False)
        """
        if not self.master:
            return "Communication not connected"      
        try:
            sendCmdBuf = [0x000D if mode else 0x000B]
            sendCmdBuf[0] |= cmdId << 8
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0BB8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def runZAxisWithParam(self, salveId, pos, speed, torque, mode):
        """
            有参数运行指令
            参数说明：
                salveId:夹爪站点id
                pos:运行目标位置
                speed:运行中最大速度
                torque:抓取物体时的力矩
                mode:是否跟随(跟随模式:True, 非跟随模式:False)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x000D if mode else 0x0009, 0x0000, 0x0000]
            sendCmdBuf[1] = pos
            sendCmdBuf[2] = torque << 8 | speed
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0BB8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setElecGearRatio(self, salveId, data):
        """
            设置电子齿轮比
            参数说明：
                salveId:夹爪站点id
                data:电子齿轮比数据
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [data]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x0BCC, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def readSoftwareVersionOfZAxis(self, salveId):
        """
            查询Z轴软件版本
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_HOLDING_REGISTERS, 0x232C, 1)
            return ["V" + str(readBuf[0] >> 8) + "." + str(readBuf[0] & 0xff)]
        except Exception as exc:
            return str(exc)


class ElsClawControl(ClawControl):
    def __init__(self):
        super().__init__()
        self.clampState = {
            0: '进入空闲状态',
            1: '空闲状态', 
            2: '离开空闲状态', 
            3: '进入使能状态',
            4: '使能状态', 
            5: '离开使能状态',
            6: '进入故障状态',
            7: '故障状态',
            8: '离开故障状态',
        }

        self.actionState = {
            0: '过渡状态',
            1: '开始搜索最大行程位置',
            2: '搜索最大行程位置',
            3: '搜索最大行程位置完成',
            4: '开始搜索最小行程位置',
            5: '搜索最小行程位置',
            6: '搜索最小行程位置完成',
            7: '开始推动作',
            8: '推动作运行中',
            9: '推动作离开',
            10: '推到位',
            11: '推受阻',
            12: '开始拉动作',
            13: '拉动作运行中',
            14: '拉动作离开',
            15: '拉到位',
            16: '进入示教模式',
            17: '示教模式',
            18: '离开示教模式',
        }
    
    def restartDevice(self, salveId):
        """
            设备重启
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x01]         
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E8, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def saveParam(self, salveId):
        """
            参数保存
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"     
        try:
            sendCmdBuf = [0x01]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03E9, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def enableDevice(self, salveId, state):
        """
            设备使能
            参数说明：
                salveId:夹爪站点id
                state:使能状态(1:使能 0:失能)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [state]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03EA, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def controlAction(self, salveId, state):
        """
            动作控制
            参数说明：
                salveId:夹爪站点id
                state:动作状态(1:推  0:拉)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [state]           
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03EB, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def resetDevice(self, salveId):
        """
            设备复位
            参数说明：
                salveId:夹爪站点ID
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [0x01]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03EC, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def switchTeachingMode(self, salveId, state):
        """
            示教模式
            参数说明：
                salveId:夹爪站点ID
                state:模式编号(0:关闭示教模式, 1:打开示教模式)
        """
        if not self.master:
            return "Communication not connected"   
        try:
            sendCmdBuf = [state] 
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03ED, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def joggingForward(self, salveId, state):
        """
            点动正向
            参数说明：
                salveId:夹爪站点ID
                state:执行状态(1:开始  0:拉停止)
        """
        if not self.master:
            return "Communication not connected"    
        try:
            sendCmdBuf = [state]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03EE, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def joggingReverse(self, salveId, state):
        """
            点动反向
            参数说明：
                salveId:夹爪站点ID
            state:执行状态(1:开始  0:拉停止)
        """
        if not self.master:
            return "Communication not connected"    
        try:
            sendCmdBuf = [state]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03EF, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setPushSpeed(self, salveId, data):
        """
            推速度设置
            参数说明：
                salveId:夹爪站点ID
                data:推速度数值, 输入范围0~65535, 单位0.01mm/s
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03F0, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setPullSpeed(self, salveId, data):
        """
            拉速度设置
            参数说明：
                salveId:夹爪站点ID
                data:拉速度数值, 输入范围0~65535, 单位0.01mm/s
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03F1, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setPushPower(self, salveId, data):
        """
            推力设置
            参数说明：
                salveId:夹爪站点ID
                data:推力数值, 输入范围30~100, 单位N
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03F2, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setPullPower(self, salveId, data):
        """
            拉力设置
            参数说明：
                salveId:夹爪站点ID
                data:拉力数值, 输入范围30~100, 单位N
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03F3, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setPushPosition(self, salveId, data):
        """
            推位置设置
            参数说明：
                salveId:夹爪站点ID
                data:推位置数值, 输入范围0~65535, 单位0.01mm
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03F4, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def setPullPosition(self, salveId, data):
        """
            拉位置设置
            参数说明：
                salveId:夹爪站点ID
                data:拉位置数值, 输入范围0~65535, 单位0.01mm
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03F5, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def ioForcedOutput(self, salveId, data):
        """
            IO强制输出
            参数说明：
                salveId:夹爪站点ID
                data:写入的数据
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03F6, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal

    def holdingBrakeForcedControl(self, salveId, data):
        """
            抱闸强制控制
            参数说明：
                salveId:夹爪站点ID
                data:写入的数据(1:强制开, 0:关闭)
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FC, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def selectCommandSource(self, salveId, data):
        """
            选择命令源
            参数说明：
                salveId:夹爪站点ID
                data:写入的数据(1:485控制 0:IO控制)
        """
        if not self.master:
            return "Communication not connected"
        sendCmdBuf = [data]         
        try:
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FD, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def changeSalveId(self, oldId, newId):
        """
            修改设备从站地址
            参数说明：
                oldId:当前的id
                newId:修改后的id(输入范围:1~247)
        """
        if not self.master:
            return "Communication not connected"
        try: 
            sendCmdBuf = [newId]
            self.master.execute(oldId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FE, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def changeBaudRate(self, salveId, baudRate):
        """
            修改设备波特率
            参数说明：
                salveId:夹爪站点ID
                baudRate:目标波特率编号(0:9600, 1:19200, 2:38400, 3:115200)
        """
        if not self.master:
            return "Communication not connected"
        try:
            sendCmdBuf = [baudRate]
            self.master.execute(salveId, cst.WRITE_MULTIPLE_REGISTERS, 0x03FF, output_value=sendCmdBuf)
            reVal = 1
        except Exception as exc:
            reVal = str(exc)
        return reVal
    
    def getDeviceCurrentState(self, salveId):
        """
            获取设备当前状态
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D0, 1)
            return [self.clampState.get(readBuf[0], "未知状态")]
        except Exception as exc:
            return str(exc)
    
    def getDeviceActionState(self, salveId):
        """
            获取设备动作状态
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D1, 1)
            return [self.actionState.get(readBuf[0], "未知状态")]
        except Exception as exc:
            return str(exc)
    
    def getDeviceErrorState(self, salveId):
        """
            获取设备故障状态
            参数说明：
                salveId:夹爪站点id
            返回值说明:
                bit0: 电机故障
                bit1: 行程错误
                bit2: 搜索超时
                bit3: 推超时
                bit4: 拉超时
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D2, 1)
            return [readBuf[0]]
        except Exception as exc:
            return str(exc)

    def getDeciveCurrentLocation(self, salveId):
        """
            获取设备当前位置
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D3, 1)
            return [readBuf[0]]
        except Exception as exc:
            return str(exc)
  
    def getDeviceCurrentTorque(self, salveId):
        """
            获取设备当前力矩
            参数说明：
                salveId:夹爪站点id
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D4, 1)
            return [readBuf[0]]
        except Exception as exc:
            return str(exc)
    
    def getIoInputMonitoring(self, salveId):
        """
            获取IO输入监测
            参数说明：
                salveId:夹爪站点id
            返回值说明:
                转为2进制, 检测IO输入(1bit对应1信号)
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D5, 1)
            return [readBuf[0]]
        except Exception as exc:
            return str(exc)
    
    def getIoOutputMonitoring(self, salveId):
        """
            获取IO输出监测
            参数说明：
                salveId:夹爪站点id
            返回值说明:
                转为2进制, 检测IO输出(1bit对应1信号)
        """
        if not self.master:
            return "Communication not connected"
        try:
            readBuf = self.master.execute(salveId, cst.READ_INPUT_REGISTERS, 0x07D6, 1)
            return [readBuf[0]]
        except Exception as exc:
            return str(exc)
