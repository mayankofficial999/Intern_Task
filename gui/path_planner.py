#!/usr/bin/python3
import sys
import rospy
import math
import roslaunch
from PyQt5 import QtGui
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QPushButton, QLabel,QGridLayout,QRadioButton,QLineEdit, QMessageBox
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot

class App(QMainWindow):

    def __init__(self):
        super().__init__()
        self.title = 'Path Planner'
        self.left = 10
        self.top = 10
        self.choice=1
        self.Goal_Reach=False
        self.Move_Circle=False
        self.Lawn_Move =False
        self.width = 400
        self.height = 140
        self.initUI()
    
    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.Window = QWidget(self)
        self.setCentralWidget(self.Window)
        self.layout = QGridLayout()
        self.chooseBox()
        self.showParamLine()
        self.showParamCircle()
        self.showParamLawn()
        self.showActionButtons()
        self.Window.setLayout(self.layout)
        self.show()

    def chooseBox(self):
        #Choice Text
        choose = QLabel(self)
        choose.setText("Choose Pattern Type: ")
        #choose.resize(200,30)
        #choose.move(40,0)
        self.layout.addWidget(choose,0,0,1,3)

        #Radio Buttons
        self.Line = QRadioButton("Line")
        self.Line.setChecked(True)
        self.Line.toggled.connect(lambda:self.LineState(self.Line))
        self.layout.addWidget(self.Line,1,0)

        self.Circle = QRadioButton("Circle")
        self.Circle.setChecked(False)
        self.Circle.toggled.connect(lambda:self.CircleState(self.Circle))
        self.layout.addWidget(self.Circle,1,1)

        self.Lawn = QRadioButton("Lawn")
        self.Lawn.setChecked(False)
        self.Lawn.toggled.connect(lambda:self.LawnState(self.Lawn))
        self.layout.addWidget(self.Lawn,1,2)
    
    def showParamLine(self):
        self.LineText = QLabel(self)
        self.LineText.setText("Line: ")
        self.layout.addWidget(self.LineText,2,0,1,6)

        self.GoalText = QLabel(self)
        self.GoalText.setText("Goal Location:  ")
        self.layout.addWidget(self.GoalText,3,1)

        self.Lat = QLabel(self)
        self.Lat.setText("X: ")
        self.layout.addWidget(self.Lat,3,2)

        self.LatIn = QLineEdit(self)
        self.LatIn.resize(50,30)
        self.layout.addWidget(self.LatIn,3,3)

        self.Long = QLabel(self)
        self.Long.setText("Y: ")
        self.layout.addWidget(self.Long,3,4)

        self.LongIn = QLineEdit(self)
        self.LongIn.resize(50,30)
        self.layout.addWidget(self.LongIn,3,5)

        self.Yaw = QLabel(self)
        self.Yaw.setText("Yaw (Deg): ")
        self.layout.addWidget(self.Yaw,3,6)

        self.YawIn = QLineEdit(self)
        self.YawIn.resize(50,30)
        self.layout.addWidget(self.YawIn,3,7)

    def showParamCircle(self):
        self.CircleText = QLabel(self)
        self.CircleText.setText("Circle: ")
        self.layout.addWidget(self.CircleText,4,0)

        self.CenterText = QLabel(self)
        self.CenterText.setText("Center:  ")
        self.layout.addWidget(self.CenterText,5,1)

        self.X = QLabel(self)
        self.X.setText("x: ")
        self.layout.addWidget(self.X,5,2)

        self.XIn = QLineEdit(self)
        self.XIn.resize(50,30)
        self.layout.addWidget(self.XIn,5,3)

        self.Y = QLabel(self)
        self.Y.setText("y: ")
        self.layout.addWidget(self.Y,5,4)

        self.YIn = QLineEdit(self)
        self.YIn.resize(50,30)
        self.layout.addWidget(self.YIn,5,5)

        self.Radius = QLabel(self)
        self.Radius.setText("Radius: ")
        self.layout.addWidget(self.Radius,5,6)

        self.RIn = QLineEdit(self)
        self.RIn.resize(50,30)
        self.layout.addWidget(self.RIn,5,7)

    def showParamLawn(self):
        self.LawnText = QLabel(self)
        self.LawnText.setText("Lawn: ")
        self.layout.addWidget(self.LawnText,6,0)

        self.ParamText = QLabel(self)
        self.ParamText.setText("Parameters:  ")
        self.layout.addWidget(self.ParamText,7,1)

        self.Length = QLabel(self)
        self.Length.setText("Length: ")
        self.layout.addWidget(self.Length,7,2)

        self.LengthIn = QLineEdit(self)
        self.LengthIn.resize(50,30)
        self.layout.addWidget(self.LengthIn,7,3)

        self.Width = QLabel(self)
        self.Width.setText("Width: ")
        self.layout.addWidget(self.Width,7,4)

        self.WidthIn = QLineEdit(self)
        self.WidthIn.resize(50,30)
        self.layout.addWidget(self.WidthIn,7,5)

    def showActionButtons(self):
        self.Start = QPushButton('Start', self)
        self.Start.clicked.connect(lambda:self.StartAction())
        self.layout.addWidget(self.Start,8,1)

        self.Stop = QPushButton('Stop', self)
        self.Stop.clicked.connect(lambda:self.StopAction())
        self.layout.addWidget(self.Stop,8,2)

        self.StartSim = QPushButton('Start Simulation', self)
        self.StartSim.clicked.connect(lambda:self.StartSimAction())
        self.layout.addWidget(self.StartSim,8,3)

        self.Exit = QPushButton('Stop Simulation', self)
        self.Exit.clicked.connect(lambda:self.ExitSimAction())
        self.layout.addWidget(self.Exit,8,4)
    
    def LineState(self,b):
        print(b.isChecked(),'1','Choice: ',self.choice)
        if b.isChecked() == True:
            self.choice=1
            self.Goal_Reach=True
            #self.showParamLine()

    def CircleState(self,b):
        print(b.isChecked(),'2','Choice: ',self.choice)
        if b.isChecked() == True:
            self.choice=2
            self.Move_Circle=True
            #self.showParamCircle()

    def LawnState(self,b):
        print(b.isChecked(),'3','Choice: ',self.choice)
        if b.isChecked() == True:
            self.choice=3
            self.Lawn_Move=True
            #self.showParamLawn()

    @pyqtSlot()
    def StartAction(self):
        Final_Point=[float(self.LatIn.text()),float(self.LongIn.text()),float(self.YawIn.text())]
        Center1=[float(self.XIn.text()),float(self.YawIn.text())]
        r=math.radians(float(self.RIn.text()))
        Lawn=[float(self.LengthIn.text()),float(self.WidthIn.text())]
        if self.choice==1:
            rospy.set_param('Goal_Reach',self.Goal_Reach)
            rospy.set_param('Final_Point',Final_Point)
        elif self.choice==2:
            rospy.set_param('Move_Circle',self.Move_Circle)
            rospy.set_param('Center',Center1)
            rospy.set_param('Radius',r)
        elif self.choice==3:
            print(self.Lawn_Move)
            rospy.set_param('Lawn_Move',self.Lawn_Move)
            rospy.set_param('Lawn',Lawn)

    def StopAction(self):
        self.Goal_Reach=False
        self.Move_Circle=False
        self.Lawn_Move=False
        rospy.set_param('Goal_Reach',False)
        rospy.set_param('Move_Circle',False)
        rospy.set_param('Lawn_Move',False)

    def StartSimAction(self):
        self.LatIn.setText("0")
        self.LongIn.setText("0")
        self.YawIn.setText("0")
        self.XIn.setText("0")
        self.YIn.setText("0")
        self.RIn.setText("0")
        self.LengthIn.setText("0")
        self.WidthIn.setText("0")
        self.Goal_Reach=True
        rospy.init_node('Frontend', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        #Change the username and workspace name here for your computer.
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/parallels/vrx_ws/src/intern_task/launch/backend.launch"],False)
        self.launch.start()
        rospy.sleep(3)

    def ExitSimAction(self):
        self.launch.shutdown()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())