#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode,  SetModeRequest, CommandBoolRequest
from mavros_msgs.msg import PositionTarget, State
from sensor_msgs.msg import Range, Image
from std_msgs.msg import Bool

# класс П регулятора
class P_reg:
    def __init__(self, p_r, min_p, max_p):
        self.p_r_ = p_r
        self.min_p_ = min_p
        self.max_p_ = max_p

    def set_param(self, p, min_out, max_out):
        self.p_r_ = p
        self.min_p_ = min_out
        self.max_p_ = max_out

    def p_culc(self, err_in):
        temp = err_in * self.p_r_
        if temp > self.max_p_:
            temp = self.max_p_
        if temp < self.min_p_:
            temp = self.min_p_
        return temp

# класс упоавления и контроля БЛА
class UavController:
    def __init__(self, uavName="mavros"):
        self.uavName_ = uavName
        self.max_high = 3.5 # максимальная набираемая высота 
        self.start_y_flag = True # после набора высоты движение вдоль оси Y
        self.back_y_flag = False # движение обратно вдоль оси Y
        #self.p_x_ = P_reg(5, -5.5, 5.5)
        #self.p_y_ = P_reg(5, -5.5, 5.5)
        self.p_z_ = P_reg(0.2, -0.1, 0.4)
        self.setPoint_ = PositionTarget()
        self.range = Range()
        self.currentState_ = State()
        self.setModeName_ = SetMode()
        self.offb_set_mode_ = SetModeRequest()
        self.arm_cmd_ = CommandBoolRequest()
        self.currentPoseLocal_ = PoseStamped()
        self.range = Range()
        self.img = Image()
        self.land = False
        # необходимые топики и сервисы для работы
        rospy.wait_for_service("mavros/cmd/arming")
        self.armingModeClient_ = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)   
        self.stateSub_ = rospy.Subscriber("mavros/state", State, self.uavStateCallback)    
        rospy.wait_for_service("/mavros/set_mode")
        self.setModeClient_ = rospy.ServiceProxy("mavros/set_mode", SetMode) 
        self.imageSub_ = rospy.Subscriber("/uav_ex_dev/uav_cam/image_raw", Image, self.imageCallback)
        self.range_ = rospy.Subscriber("/range_down", Range, self.rangeCallback)
        self.localPositionSub_ = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.localPositionCallback)
        self.landSub = rospy.Subscriber("/extra_land", Bool, self.extraLandCallback)
        self.setPointPub_ = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        rospy.wait_for_service("/mavros/set_mode")
        self.setModeClient_ = rospy.ServiceProxy("mavros/set_mode", SetMode)        
        self.rosNodeInit()

    # получение и обработка изображения 
    def imageCallback(self, img_msg):        
        pass
        #self.img = img_msg        
              
    # запуск двигателей
    def arm(self, cmd):
        if cmd:
            self.arm_cmd_.value = True
            if (self.armingModeClient_.call(self.arm_cmd_).success == True):
                rospy.loginfo("Vehicle armed")
                return True
        else:
            self.arm_cmd_value = False
            if (self.armingModeClient_.call(self.arm_cmd_).success == True):
                rospy.loginfo("Vehicle disarmed")
                return False

    def set_offboard(self):
        if self.currentState_.mode != "OFFBOARD":
            if(self.setModeClient_.call(self.offb_set_mode_).mode_sent == True):
                return True
            else:
                return False
        return True
    
    # Метод для посадки дрона
    def land_vehicle(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            land_response = land_service(0, 0, 0, 0, 0)
            if land_response.success:
            #    print("Landing command sent successfully!")
                return True
            else:
            #    print("Failed to send landing command.")
                return False
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)
            return False
       
    # инициализация и подключение
    def rosNodeInit(self):
        self.offb_set_mode_.custom_mode = "OFFBOARD"
        self.setPointTypeInit()
        rate = rospy.Rate(20.0)
        # ждём подключения...
        while not rospy.is_shutdown() and not self.currentState_.connected:
              rate.sleep()
        if self.arm(True):
           if self.set_offboard():
              rospy.loginfo("Offboard enabled")

    def setPointTypeInit(self):
        # задаем тип используемого нами сообщения для желаемых параметров управления аппаратом
        _ =  PositionTarget()
        _.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
        _.coordinate_frame = PositionTarget.FRAME_LOCAL_NED


        self.setPoint_.type_mask = _.type_mask
        self.setPoint_.coordinate_frame = _.coordinate_frame

    def uavStateCallback(self, msg):
        self.currentState_ = msg
        if self.currentState_.mode == 'AUTO.LAND':
            self.land = True
            if self.land_vehicle():
               rospy.loginfo("Emergency landing") 


    def rangeCallback(self, rangePose):
        self.range = rangePose
        # вывод данных лазерного дальномера
        rospy.loginfo("Rangefinder data: %s" % self.range.range)
        
    def localPositionCallback(self, localPose):
        if self.land:            
            return
        self.currentPoseLocal_ = localPose
        self.calculateAndSendSetpoint()
    
    def extraLandCallback(self, land_str):
        self.land = land_str        
        if (not self.land.data):
            if self.land_vehicle():
               rospy.loginfo("Emergency landing")            
            

    def calculateAndSendSetpoint(self):
        #errX = self.desPoseLocal_.pose.position.x - self.currentPoseLocal_.pose.position.x
        #errY = self.desPoseLocal_.pose.position.y - self.currentPoseLocal_.pose.position.y
        
        self.setPoint_.velocity.x = 0.0
        self.setPoint_.velocity.y = 0.0
        # проверяем высоту по лазерному дальномеру
        if self.range.range > self.max_high and self.start_y_flag:
            self.start_y_flag = False           
        
        if self.start_y_flag == False:
           self.setPoint_.velocity.y = 0.2
           if self.currentPoseLocal_.pose.position.y > 11 and (not self.back_y_flag):
              self.back_y_flag = True
           if self.back_y_flag:
              self.setPoint_.velocity.y = -0.2
           if self.currentPoseLocal_.pose.position.y < -1 and self.back_y_flag:
              self.back_y_flag = False                    

        errZ = self.max_high - self.range.range
        self.setPoint_.velocity.z = 1
        if self.range.range > self.max_high or abs(errZ) < 0.2:
            self.setPoint_.velocity.z = self.p_z_.p_culc(errZ)
        
        # self.setPoint_.velocity.x = self.p_x_.p_culc(errX)
        # self.setPoint_.velocity.y = self.p_y_.p_culc(errY)        
        self.setPoint_.yaw = 0
        if self.currentState_.mode != "OFFBOARD":
            self.set_offboard()
        # print(self.setPoint_.velocity.x, self.setPoint_.velocity.y, self.setPoint_.velocity.z, self.range.range)
        self.setPointPub_.publish(self.setPoint_)

def main():
    rospy.init_node("extra_delivery_node")    
    rate = rospy.Rate(20)
    controller = UavController()
    while not rospy.is_shutdown():
        rospy.spin()        
        rate.sleep()

if __name__ == "__main__":
    main()


