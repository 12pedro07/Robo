#!/usr/bin/env python
import numpy
import rospy
import smach
import smach_ros
import math
from std_msgs.msg import Float32MultiArray, Float64, String
from geometry_msgs import Twist
from sensor_msgs.msg import LaserScan


class get_close(smach.State):

    def __init__(self, outcomes=['success', 'in_progress', 'fail']):
        smach.State.__init__(self, outcomes=['success', 'in_progress', 'fail'])
        self.check = False  #booleano para checar se a tf do objeto foi encontrada
        self.cont = 0  #auxiliar para mover o angulo do kinect
        self.distance_laser = 0.4  #distancia minima entre laser e obstaculo
        self.done = False  #booleano para checar se a tarefa terminou
        self.dist_grip = 0.37  #distancia ideal entre manipulador e objeto
        self.first_object = []
        self.i = -1  #auxiliar para inverter a direção de movimento da cabeça
        self.increment = 15 * math.pi / 180  #incremento no angulo do kinect (15 graus em radianos)
        self.kin_pub = rospy.Publisher('/tilt_head/command', Float64, queue_size=1)  #publisher do servo da cabeça
        self.laser = 0
        self.n = 0  #valor inicial do contador (nao alterar)
        self.rot = 0
        self.trans = 0
        self.vel = Twist()
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  #publisher para movimentar a base
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def laser_callback(self, msg):
        #retornar o valor da menor distancia captada pelo laser
        self.laser = min(msg.ranges[130:180])

    def move_kinect(self):
        #move o kinect para cima ou para baixo com um incremento pre-definido
        if self.cont < 3:
            self.kin_pub.publish((12 * self.increment * self.i / 20))
            self.cont += 1
        else:
            self.cont = 0
            self.i = -1 * self.i

    def move_base(self):
        #move a base ate o objeto, de forma a deixar o manipulador posicionado para pegar o objeto
        (trans_manip, rot_manip) = listener.lookupTransform("/object_%i" % self.first_object, "/manipulator", rospy.Time(0))
        vel.linear.z = 0
        if trans_manip[1] < -0.1 and self.laser < 0.4:
            vel.linear.y = 0.1
        if trans_manip[1] > 0.1 and self.laser < 0.4:
            vel.linear.y = -0.1
        else:
            ################################################
            # trans_manip[0] tem que ser normalizado antes
            # de realizar este if
            length = sqrt(self.vel.linear.x ^ 2 + self.vel.linear.y ^ 2 + self.vel.linear.z ^ 2)
            self.vel.linear.x = self.vel.linear.x / length
            if self.vel.linear.x > 0.37 and self.laser < 0.4:
                vel.linear.x = 0.1
            ################################################
            else:
                vel.linear.x = 0
            vel.linear.y = 0
        self.pub_vel.publish(vel)

    def too_close(self):
        # anda para traz por 4 segundos
        rate = rospy.Rate(10)  # 10hz
        self.vel = Twist()
        time = rospy.get_time()
        while (rospy.get_time() - time <= 4):
            vel.linear.x = -0.1
            vel.linear.y = 0
            vel.linear.z = 0
            self.pub_vel.publish(vel)
        vel.linear.x = 0
        self.pub_vel.publish(vel)

    def tf_check(self, userdata):
        #tenta fazer a transformada entre objeto e kinect
        while self.check == False:
            try:
                (self.trans, self.rot) = listener.lookupTransform("/object_%i" % self.first_object, "/kinect_center", rospy.Time(0))
                self.check = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.move_kinect()
                self.check = False
                self.n += 1
                if self.n == 14:
                    self.check = True

    def execute(self, userdata):
        listener = tf.TransformListener()
        self.first_object = userdata.namesids[0][1]
        while self.done == False:
            self.tf_check()
            if self.n == 14:
                return 'fail'
            self.n = 0
            (trans_manip, rot_manip) = listener.lookupTransform("/object_%i" % self.first_object, "/manipulator", rospy.Time(0))
            if trans_manip[0] > self.dist_grip and self.laser < self.distance_laser:
                self.move_base()
            elif trans_manip[0] == self.dist_grip:
                del userdata.namesids[0]
                self.done = True
                if userdata.namesids != []:
                    return 'in_progress'
                else:
                    return 'success'
            else:
                self.too_close()
