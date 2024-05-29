import time
from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.unitreeConnection_Go1 import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType
from ucl.complex import motorCmd

class myunitree:
    def __init__(self):
        self.connect_flag = False
    def connect(self):
        try:
            self.conn = unitreeConnection(HIGH_WIFI_DEFAULTS)  # 네트워크 연결
            self.conn.startRecv()

            self.hcmd = highCmd()
            self.hstate = highState()
            self.connect_flag = True
        except Exception as e:
            self.connect_flag = False
            print(f'Connect failed: {e}')
    def disconnect(self):
        if self.connect_flag:
            self.conn.stopRecv()

            self.connect_flag = False

    def cmdInit(self):
        time.sleep(0.02)
        data = self.conn.getData()
        for paket in data:
            self.hstate.parseData(paket)
            # print(f'SOC:\t\t\t{self.hstate.bms.SOC} %')
            self.hstate_bms_SOC = self.hstate.bms.SOC
            self.hstate_mode =self.hstate.mode
            self.hstate_gaitType =self.hstate.gaitType
            self.hstate_velocity = self.hstate.velocity

            # self.hstate_position = self.hstate.position

    def sendCmd(self):
        if not self.connect_flag:
            print('Cannot send command: Not connected')
            return

        try:
            self.cmd_bytes = self.hcmd.buildCmd(debug=False)
            self.conn.send(self.cmd_bytes)
            self.cmdInit()
        except Exception as e:
            print(f'Failed to send command: {e}')
            self.connect_flag = False

    #------ 뱡향키 입력 메소드 ---------------------------------
    def Move_Front(self, vel_0):
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.velocity = [vel_0, 0]  # 앞으로 이동 진행
    def Move_Back(self,vel_0):
        # self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK  # mode 2
        self.hcmd.velocity = [vel_0, 0]  # -1  ~ +1
    def Move_Left(self,vel_1):
        # self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK  # mode 2
        self.hcmd.velocity = [0, vel_1]  # -1  ~ +1
    def Move_Right(self,vel_1):
        # self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK  # mode 2
        self.hcmd.velocity = [0, vel_1]  # -1  ~ +1
    def Move_Stop(self):
        # self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.velocity = [0,0]  # -1  ~ +1

    def Turn_RL(self,yawspeed_value):
        # self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.yawSpeed = yawspeed_value
    def Turn_Stop(self):
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.yawSpeed = 0
    def Robot_force_Stop(self):
        # self.cmdInit()
        self.hcmd.mode = MotorModeHigh.IDLE
        self.hcmd.gaitType = GaitType.IDLE
        self.hcmd.velocity = [0,0]  # -1  ~ +1
        self.hcmd.yawSpeed = 0
        self.hcmd.euler = [0, 0, 0]
        print("강제 STOP")

    def Move_mult(self,vel_0,vel_1):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK
        self.hcmd.velocity = [vel_0,vel_1]


    # -------------------------------------------------------
    def click_Up(self):
        # self.cmdInit()
        self.hcmd.mode = MotorModeHigh.STAND_UP
        print("Click Up")
    def click_Down(self):
        # self.cmdInit()
        self.hcmd.mode = MotorModeHigh.STAND_DOWN
        print("Click Down")

    def click_Euler(self,vel_row,vel_pitch,vel_yaw): #self,vel_row,vel_pitch,vel_yaw
        self.cmdInit()

        # euler = [Roll, Pitch, Yaw]
        # Row: (+)왼쪽, (-)오른쪽
        # Pitch: (+)고개 숙이기 ,(-)고개 들기
        # Yaw: (+)왼쪽, (-)오른쪽
        self.hcmd.euler = [vel_row,vel_pitch,vel_yaw]

    def click_Height(self,vel_bodyHeight):
        self.cmdInit()
        # self.hcmd.mode = MotorModeHigh.FORCE_STAND
        self.hcmd.bodyHeight = vel_bodyHeight # default: 0.28m

    # ------ Mode ComboBox 입력 메소드 --------------------
    def click_ModeCombo_IDLE(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.IDLE # mode  0
    def click_ModeCombo_Force_Stand(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.FORCE_STAND # mode  1
    def click_ModeCombo_VEL_WALK(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.VEL_WALK # mode 2
    def click_ModeCombo_STAND_DOWN(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.STAND_DOWN # mode 5
    def click_ModeCombo_STAND_UP(self):
        self.cmdInit()
        self.hcmd.mode = MotorModeHigh.STAND_UP # mode 6

    #------ GaitType ComboBox 입력 메소드 --------------------
    def click_GaitTypeCombo_IDLE(self):
        self.cmdInit()
        self.hcmd.gaitType = GaitType.IDLE  # GaitType 0
    def click_GaitTypeCombo_Trot(self):
        self.cmdInit()
        self.hcmd.gaitType = GaitType.TROT  # GaitType 1
    def click_GaitTypeCombo_CLIMB_STAIR(self):
        self.cmdInit()
        self.hcmd.gaitType = GaitType.CLIMB_STAIR  # GaitType 2
    def click_GaitTypeCombo_TROT_OBSTACLE(self):
        self.cmdInit()
        self.hcmd.gaitType = GaitType.TROT_OBSTACLE  # GaitType 3

    def LED_OON(self):
        self.hcmd.led = [1,1,1]