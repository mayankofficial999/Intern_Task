#!/usr/bin/python3
import rospy
import tf
import math
import PID
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

current=Twist()
goal=Twist()
input_location = Odometry()
feedback=0
velocity=0

def getOrientation(goal,angle):
    x2=goal.linear.x
    x1=current.linear.x
    y2=goal.linear.y
    y1=current.linear.y
    if (x2<x1 and y2<y1) :
        return 1.0*(angle)+math.pi
    elif (x2>x1 and y2<y1):
        return 2*math.pi-angle
    elif (x2>x1 and y2>y1) or (x2<x1 and y2>y1):
        return angle
    elif x1==x1 or y1==y2:
        return angle

def getFeedback(inp):
    it=0
    if inp>0:
        it=inp
    else:
        it=2*math.pi-abs(inp)
    return it

def getTangent(x0,x1,y0,y1,r):
    a=((x0-x1)**2-r**2)
    b=-2*(y0-y1)*(x0-x1)
    c=(y0-y1)**2-r**2
    print('A= ',a)
    print('B= ',b)
    print('C= ',c)
    #Calc slope values
    if b**2-4*a*c>0:
        m1=(-b+math.sqrt(b**2-4*a*c))/(2*a)
    else:
        m1=1000
    if b**2-4*a*c>0:
        m2=(-b-math.sqrt(b**2-4*a*c))/(2*a)
    else:
        m2=1000
    #Choose smallest slope and get slope in radians
    m=min([m1,m2])
    m=math.atan(m)
    #Transform slope to [0,2*pi]
    if x0>x1 and y0>y1:
        m=m
    elif x0>x1 and y0<y1:
        m=2*math.pi-abs(m)
    elif x0<x1 and y0<y1:
        m=math.pi+abs(m)
    elif x0<x1 and y0>y1:
        m=math.pi-abs(m)
    print("Slope: ",math.degrees(m))
    return m

def quat2rpy(quat):
    euler=tf.transformations.euler_from_quaternion(quat)
    return euler[0],euler[1],euler[2]

def currentLocation(data):
    global feedback,velocity
    input_location=data
    current.linear.x=input_location.pose.pose.position.x
    current.linear.y=input_location.pose.pose.position.y
    current.linear.z=input_location.pose.pose.position.z
    velocity=input_location.twist.twist.linear.x
    quat1 = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    current.angular.x,current.angular.y,current.angular.z=quat2rpy(quat1)
    feedback=current.angular.z

def speed_control(ang_speed=0,speed_linear=0):
    global angle_lateral,angle_left,angle_right,thrust_lateral,thrust_left,thrust_right
    #angle_lateral.publish(-math.pi/2.0)
    #thrust_lateral.publish(-2*ang_speed)
    if -1*ang_speed+speed_linear>1.0:
        thrust_left.publish(1.0)
    else:
        thrust_left.publish(-1*ang_speed+speed_linear)
    if 1*ang_speed+speed_linear>1.0:
        thrust_right.publish(1.0)
    else:
        thrust_right.publish(1*ang_speed+speed_linear)

def moveAngle(goal,theta,speed):
    global feedback
    x2=goal.linear.x
    x1=current.linear.x
    x1=x1-2
    y2=goal.linear.y
    y1=current.linear.y
    print("Goal : {:.3f},{:.3f}".format(x2,y2))
    print("Current : {:.3f},{:.3f}".format(x1,y1))
    #Work Something here
    if(theta>0):
        print("+ve is executed")
        if (x2<x1 and y2<y1):
            angle_lateral.publish(-1.0*(math.pi/2.0-theta))
            thrust_lateral.publish(1.0*speed)
        elif (x2>x1 and y2>y1):
            angle_lateral.publish((-1.0*math.pi/2.0-theta))
            thrust_lateral.publish(-1.0*speed)
    else:
        print("-ve is executed")
        if (x2<x1 and y2>y1):
            print("Moving correct engine !")
            angle_lateral.publish((math.pi/2.0-abs(theta)))
            thrust_lateral.publish(-1.0*speed)
        elif (x2>x1 and y2<y1):
            angle_lateral.publish((math.pi/2.0-abs(theta)))
            thrust_lateral.publish(1.0*speed)

def Navigate(P=0.2,I=0,D=0):
    global feedback
    goal=Twist()
    g_param=rospy.get_param("Final_Point")
    #Final Point Geographic Lat and Lon
    goal.linear.x=g_param[0]
    goal.linear.y=g_param[1]
    goal.linear.z=0
    goal.angular.x=0
    goal.angular.y=0
    #Goal Position Heading (Yaw)
    goal.angular.z=g_param[2]
    #1st PID for angular control
    pid = PID.PID(P, I, D)
    pid.setSampleTime(0.01)
    pid.setWindup(1.0)
    distance = abs(math.sqrt(((goal.linear.x-current.linear.x) ** 2) + ((goal.linear.y-current.linear.y) ** 2)))
    #2nd PID for linear control
    pid_1=PID.PID(P,I,D)
    pid_1.setSampleTime(0.01)
    pid_1.setWindup(1.0)
    pid_1.SetPoint=0
    pid.SetPoint=math.acos((goal.linear.x-current.linear.x)/(distance))
    rate=rospy.Rate(100)
    c=0
    while distance>0.2 and rospy.get_param("Goal_Reach"):
        print("Goal x,y: ",goal.linear.x,',',goal.linear.y)
        print("Current x,y: ",current.linear.x,',',current.linear.y)
        distance = abs(math.sqrt(((goal.linear.x-(current.linear.x)) ** 2) + ((goal.linear.y-current.linear.y) ** 2)))
        print("Distance: ",distance)
        if distance>1:
            goal_angle=math.acos((goal.linear.x-(current.linear.x))/(distance))
        else:
            goal_angle=0
        goal_angle=getOrientation(goal,goal_angle)
        #goal_angle_1=math.atan((goal.linear.y-current.linear.y)/(goal.linear.x-(current.linear.x)))
        print("Goal Angle: ",(goal_angle)*180/math.pi)
        print("Feedback: ",(getFeedback(feedback))*180/math.pi)
        pid.update(getFeedback(feedback))
        output=pid.output
        #Code Block to control orientation at 0 degree (discontinuous point)
        if pid.SetPoint>=0 or pid.SetPoint<=math.radians(20) or pid.SetPoint==2*math.pi or pid.SetPoint>2*math.pi-math.radians(20):
            if getFeedback(feedback)>0 and getFeedback(feedback)<math.radians(40):
                output=-(output)
                print(output)
            elif getFeedback(feedback)>math.radians(320) and getFeedback(feedback)<2*math.pi:
                output=output
                print(output)

        pid_1.update(distance)
        output_1=pid_1.output
        #speed_control(output)
        #Series of 3 Controls to reach the goal : Go to 10 units before the goal, Align to goal yaw , Reduce the distance gradually to 0
        if distance>5:
            pid.SetPoint=goal_angle
            if abs(goal_angle-getFeedback(feedback))>math.radians(2):
                speed_control(output,0)
                thrust_lateral.publish(0)
            else:
                speed_control(0.0,abs(output_1))
                thrust_lateral.publish(0)
        else:
            pid.SetPoint=getFeedback(goal.angular.z)
            if abs(getFeedback(goal.angular.z)-getFeedback(feedback))>math.radians(2):
                speed_control(output,0)
                thrust_lateral.publish(0)
            else:
                #moveAngle(goal,goal_angle_1,0.25)
                print("Reached")
                rospy.set_param("Goal_Reach",False)
        rate.sleep()

def MoveCircle():
    try:
        global feedback,velocity,current
        c_param=rospy.get_param("Center")
        #Geographic Center
        x0=c_param[0]
        y0=c_param[1]
        x1=current.linear.x
        y1=current.linear.y
        r=rospy.get_param("Radius")
        m=getTangent(x0,x1,y0,y1,r)
        print('m= ',math.degrees(m))
        rospy.set_param('Final_Point',[x0,y0,m])
        rospy.set_param('Goal_Reach',True)
        Navigate(1.0,0.01,4.0)
        ang_pid=PID.PID(1,0.01,8.0)
        ang_pid.SetPoint=m         
        ang_pid.setSampleTime(0.01)
        ang_pid.setWindup(1.0)
        distance = abs(math.sqrt(((x0-current.linear.x) ** 2) + ((y0-current.linear.y) ** 2)))
        rate=rospy.Rate(100)
        #print("Distance: ",distance)
        #print("Velocity: ",velocity)
        while rospy.get_param('Move_Circle'):
            speed_control(velocity/r,velocity)
            angle_lateral.publish(0)
            thrust_lateral.publish(-((180*velocity**2)/r)/250)
            rate.sleep()
    except rospy.ROSInterruptException:
        exit(0)

def MoveLawn():
    global feedback,current
    l_param=rospy.get_param("Lawn")
    #Lawn length and width
    len=l_param[0]
    width=l_param[1]
    while current.linear.x==0 and current.linear.y==0:
        pass
    i=1
    sign=1
    rate=rospy.Rate(10)
    while rospy.get_param('Lawn_Move'):
        if i%2==0:
            sign=+1
        else:
            sign=-1
        x0=current.linear.x
        y0=current.linear.y+sign*len
        distance = abs(math.sqrt(((x0-(current.linear.x)) ** 2) + ((y0-current.linear.y) ** 2)))
        goal_angle=math.acos((x0-current.linear.x)/(distance))
        goal_angle=getOrientation(goal,goal_angle)
        rospy.set_param('Final_Point',[x0,y0,getFeedback(goal_angle+sign*math.radians(90))])
        rospy.set_param('Goal_Reach',True)
        Navigate()
        rate.sleep()
        x0=current.linear.x-width
        y0=current.linear.y
        distance = abs(math.sqrt(((x0-(current.linear.x)) ** 2) + ((y0-current.linear.y) ** 2)))
        goal_angle=math.acos((x0-current.linear.x)/(distance))
        goal_angle=getOrientation(goal,goal_angle)
        rospy.set_param('Final_Point',[x0,y0,getFeedback(goal_angle+sign*math.radians(90))])
        rospy.set_param('Goal_Reach',True)
        Navigate()
        i+=1
        rate.sleep()
    
def Initiate_Task():
    while not rospy.is_shutdown():
        if rospy.get_param("Move_Circle"):
            MoveCircle()
        elif rospy.get_param("Lawn_Move"):
            MoveLawn()
        elif rospy.get_param("Goal_Reach"):
            Navigate(1.0,0.01,4.0)

if __name__ == '__main__':
    try:
        global angle_lateral,angle_left,angle_right,thrust_lateral,thrust_left,thrust_right
        rospy.init_node('Task_Backend', anonymous=False)
        angle_lateral=rospy.Publisher('/wamv/thrusters/lateral_thrust_angle',Float32,queue_size=10)
        thrust_lateral=rospy.Publisher('/wamv/thrusters/lateral_thrust_cmd',Float32,queue_size=10)
        angle_left=rospy.Publisher('/wamv/thrusters/left_thrust_angle',Float32,queue_size=10)
        thrust_left=rospy.Publisher('/wamv/thrusters/left_thrust_cmd',Float32,queue_size=10)
        angle_right=rospy.Publisher('/wamv/thrusters/right_thrust_angle',Float32,queue_size=10)
        thrust_right=rospy.Publisher('/wamv/thrusters/right_thrust_cmd',Float32,queue_size=10)
        rospy.Subscriber("/wamv/robot_localization/odometry/filtered",Odometry , currentLocation)
        Initiate_Task()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Backend Terminated")