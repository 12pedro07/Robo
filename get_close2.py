#!/usr/bin/env python
import numpy
import rospy
import smach
import smach_ros
import math
import tf
from std_msgs.msg import Float32MultiArray, Float64, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

###################################################################################
###################################################################################
###                                                                                ###
### Mapa de outcomes:                                                            ###
### X = not using /// - = using                                                    ###
###                                                                             ###
###    - success = estado completo e nao existem outros objetos na lista            ###
###    X in_progress = estado completo mas ainda existem outros objetos            ###
###    - no_tf = kinect nao foi capaz de encontrar a transformada dos objetos        ###
###    - no_objects = nao foram passados objetos para o estado                        ###
###                                                                                ###
###################################################################################
###################################################################################


class get_close2(smach.State):
    #------------------------------------------------------------------------
    # Variaveis que so precisam ser declaradas uma vez na maquina de estados
    def __init__(self, outcomes=['success', 'no_tf', 'no_objects']):
        smach.State.__init__(self, outcomes=['success', 'no_tf', 'no_objects'], output_keys=['namesids', 'coordX', 'coordY', 'coordZ'], input_keys=['namesids'])
        self.distance_laser = 0.3  #distancia minima entre laser e obstaculo
        self.first_object = []
        self.max_range = 0.4
        self.increment = 2 * math.pi / 180  #incremento no angulo do kinect (5 graus em radianos)
        self.kin_pub = rospy.Publisher('/tilt_head/command', Float64, queue_size=1)  #publisher do servo da cabeca
        self.laser = 0
        self.n = 0  #valor inicial do contador (nunca alterar)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  #publisher para movimentar a base
        self.vel = Twist()
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

#------------------------------------------------------------------------
# Retornar o valor da menor distancia captada pelo laser

    def laser_callback(self, msg):
        self.laser = min(msg.ranges[130:180])

#------------------------------------------------------------------------
# Move a base ate o objeto, de forma a deixar o manipulador posicionado para pegar o objeto

    def move_base(self):
        print("no move_base")
        self.vel.linear.z = 0
        #print "y = " + str(trans_manip[1])
        if self.laser >= (self.distance_laser - self.max_range):
            self.vel.linear.x = 0.1
            print("vel = " + str(self.vel.linear.x))
        else:
            self.vel.linear.x = 0
            self.vel.linear.y = 0
            print("vel = " + str(self.vel.linear.x))
        self.pub_vel.publish(self.vel)
#------------------------------------------------------------------------
# Move a base para traz para nao bater

    def too_close(self):
        print("no too_close")
        print("laser = " + str(self.laser))
        time = rospy.get_time()
        while (rospy.get_time() - time <= 1.0):
            self.vel.linear.x = -0.1
            self.vel.linear.y = 0
            self.vel.linear.z = 0
            self.pub_vel.publish(self.vel)
        self.vel.linear.x = 0
        self.pub_vel.publish(self.vel)
#------------------------------------------------------------------------
# Move o kinect para cima ou para baixo para encontrar a transformada

    def move_kinect(self):
        print("no kinect")
        if self.cont < 35:
            rospy.sleep(0.5)
            self.j = self.j + 12 * self.increment * self.i / 20
            print("angulo kinect (self.j) = " + str(self.j))
            self.kin_pub.publish(self.j)
            self.cont += 1
        else:
            self.cont = 0
            self.i = -1 * self.i
#------------------------------------------------------------------------
# Tenta fazer a transformada entre objeto e manipulador

    def tf_check(self):
        while self.check == False:
            try:
                print("entrou tf")
                print("laser = " + str(self.laser))
                (self.trans, self.rot) = self.listener.lookupTransform("/object_%i" % self.first_object, "/manipulator", rospy.Time(0))
                self.check = True
            except tf.Exception:
                self.move_kinect()
                self.check = False
                self.n += 1
                print("n = " + str(self.n))
                if self.n == 70:
                    self.check = True
#------------------------------------------------------------------------
# Main code com as variaveis que precisam sofrer reset cada vez que o estado chamado mais de uma vez

    def execute(self, userdata):
        self.check = False  #booleano para checar se a tf do objeto foi encontrada
        self.cont = 0  #auxiliar para mover o angulo do kinect
        self.done = False  #booleano para checar se a tarefa terminou
        self.i = -1  #auxiliar para inverter a direcao de movimento da cabeca
        self.j = 0
        self.rot = []
        self.trans = []
        self.listener = tf.TransformListener()
        self.kin_pub.publish(0)  #inicia o servo em angulo em 0
        #------------------------------------------------------------------------
        # Garantindo presenca de um objeto
        if userdata.namesids == []:
            self.first_object = []
        else:
            self.first_object = userdata.namesids[0][1]
#------------------------------------------------------------------------
# Caso o kinect ja tenha atingido seu limite antes
        if self.n == 70:
            print("resetando")
            self.n = 0
#------------------------------------------------------------------------
# Main loop
        while self.done == False:
            rospy.logwarn('general info')
            print("laser                          = " + str(self.laser))
            print("self.distance_laser     (padrao) = " + str(self.distance_laser))
            rospy.logwarn("-" * 50)
            #------------------------------------------------------------------------
            # Aproximacao
            if (self.laser > self.distance_laser):
                self.move_base()
    #------------------------------------------------------------------------
    # Posicao ideal
            elif self.laser >= (self.distance_laser - self.max_range):
                self.vel.linear.x = 0
                self.pub_vel.publish(self.vel)
                #del userdata.namesids[0]
                if self.first_object == []:
                    return 'no_objects'
                else:
                    self.tf_check()
                if self.n == 70:
                    rospy.loginfo('no tf found')
                    self.kin_pub.publish(0)
                    return 'no_tf'
#                elif userdata.namesids != []:
#                    rospy.loginfo('in_progress')
#                    return 'in_progress'
                else:
                    rospy.logwarn('ending state, success')
                    print('self.trans              =' + str(self.trans) + '\n')
                    print("coordenadas p/ manipulador:")
                    print("x' = " + str(self.trans[0] * 1000))
                    print("y' = " + str(self.trans[1] * 1000))
                    print("z' = " + str(self.trans[2] * 1000))
                    print("#" * 30)
                    return 'success'
    #------------------------------------------------------------------------
    # Evita bater
            elif self.laser < (self.distance_laser - self.max_range):
                self.too_close()

    #------------------------------------------------------------------------


#    def execute2(self,userdata):
#        try:
#            execute_main()
#        except rospy.ROSInterruptException:
#            pass
