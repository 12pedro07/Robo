#!/usr/bin/env python

import roslib
import rospy
import tf
import turtlesim.msg
import numpy as np
import math


class kinect:

    def __init__(self):
        self.R = 0.022  #raio do centro do motor ao centro do kinect em metros
        self.xb = 0.012  #distancia em x (para frente) do centro do motor ao centro da base
        self.zb = 1.43  #distancia em z (altura) do centro do motor ao centro da base
        rospy.Subscriber('/tilt_head/state', JointState, self.center_callback, queue_size=1)

    def center_callback(self, data):
        try:
            while True:
                rospy.sleep(0.1)
                self.servo_head = data.current_pos  #relacao engrenagens = 12 pra 20
                ang_kinect = 12 * servo_head / 20  #relacao entre servo e kinect
                ang_kinect_correto = 1.56 + ang_kinect  #angulo do kinect em radianos, tarado para 0 radianos na posicao horizontal para frente
                xk = np.round(self.R * np.cos(ang_servo), 2)
                zk = np.round(self.R * np.sen(ang_servo), 2)
                if ang_servo >= (math.pi / 2):
                    x = xk - self.xb
                    x = math.fabs(x)
                else:
                    x = xk + self.xb
                z = zk + self.zb
                rotx = (math.pi / 2) - ang_servo
                br = tf.TransformBroadcaster()
                br.sendTransform((x, 0, z), (rotx, 0, 0), rospy.Time.now(), "/base_link", "/kinect_center")
        except KeyboardInterrupt:
            print("Shutting down")


def main(args):
    rospy.init_node('dinamic_tf_kinect')
    kinect()


if __name__ == '__main__':
    main(sys.argv)
