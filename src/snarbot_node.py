#! /usr/bin/python3
from snarbot.msg import P
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt
import numpy as np
import rospy

last_wr, last_wl = 0, 0
time_from_last_callback = 0
coord_x = [0]
coord_y = [0]
angle = 0
fig, = plt.plot([], [])


def encoder_from_twist(lin_vel, ang_vel, callback_duration):
    base_len = 0.2
    wheel_r = 0.05
    T = 3*callback_duration
    N= 4096
    global last_wl, last_wr

    beta = callback_duration/(T + callback_duration + 0.01)

    w_r = (base_len*ang_vel/(2*wheel_r)) + lin_vel/wheel_r
    w_l = 2*lin_vel/wheel_r - w_r

    w_r = beta*last_wr + (1 - beta)*w_r
    w_l = beta*last_wl + (1 - beta)*w_l

    last_wl = w_l
    last_wr = w_r

    en_r = w_r*N*callback_duration/2*3.14
    en_l = w_l*N*callback_duration/2*3.14

    return en_l, en_r



def coords_from_twist(lin_vel, ang_vel, callback_duration):
    global fig
    global angle  
    
    old_angle = angle
    angle = ang_vel*callback_duration
    x_old = coord_x[-1]
    y_old = coord_y[-1]
    coord_x.append(x_old + lin_vel*np.sin(angle+old_angle)*callback_duration)
    coord_y.append(y_old + lin_vel*np.cos(angle+old_angle)*callback_duration)

    fig.set_xdata(coord_x)
    fig.set_ydata(coord_y)
    plt.draw()




def talker(data):

    global time_from_last_callback, last_wl, last_wr

    callback_duration = rospy.Time.now().secs - time_from_last_callback
    time_from_last_callback = rospy.Time.now().secs

    lin_vel = (data.linear.x**2 + data.linear.y**2)**0.5
    ang_vel = data.angular.z

    rospy.loginfo("i heard lin_vel: %s ang_vel %s", lin_vel, ang_vel)

    pub = rospy.Publisher("en_pub", P, queue_size=10)

    msg = P()
    msg.head.stamp = rospy.Time.now()
    msg.left_enc, msg.right_enc = encoder_from_twist(lin_vel, ang_vel, callback_duration)
    coords_from_twist(lin_vel, ang_vel, callback_duration)

    rospy.loginfo(" and calculated encoder l: %s r: %s", msg.left_enc, msg.right_enc)
    pub.publish(msg)



def listener():

    rospy.init_node('twist_listener')
    sub = rospy.Subscriber('cmd_vel', Twist, talker)
    plt.axis([-10, 10, -10, 10])
    plt.show()
    plt.draw()
    #plt.autoscale(True,True,True)
    rospy.spin()



if __name__ == "__main__":
    rospy.loginfo("launching_listener")
    listener()

