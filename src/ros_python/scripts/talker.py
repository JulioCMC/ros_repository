#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion


import ang2PI
import angPI

def send_vel(u,w):    
    commands = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('path_following_controller', anonymous=True)    
    rate = rospy.Rate(10) # 10hz
    vel_msg = Twist()

    vel_msg.linear.x = u
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0    
    vel_msg.angular.z = w
    
    
    if not rospy.is_shutdown():
        
        rospy.loginfo(vel_msg)
        commands.publish(vel_msg)        
        rate.sleep()

def get_odom():
    rate = rospy.Rate(10) # 10hz
    if not rospy.is_shutdown():        
        rospy.Subscriber('/odom',Odometry,robot_position)
        rate.sleep()

def robot_position(data):
    
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q1 = data.pose.pose.orientation.x
    q2 = data.pose.pose.orientation.y
    q3 = data.pose.pose.orientation.z
    q4 = data.pose.pose.orientation.w
    q = (q1, q2, q3, q4)
    e = euler_from_quaternion(q)
    phi = e[2]

    return x,y,phi
    

# Parameters 

n=1;
s_max=6;
ts=0.1;
a=0;
alpha = 0*np.pi/6;
u_d=0.4;
ds=0.001

# Design Constants

kx = 1;
ky = 1;
kw = 5; 

lx = 0.4;
ly = 0.4;
lw = 0.75;

# Defining the path

s = np.arange(0,s_max,ds)

# Cartesian Path

x_d = s
#y_d = 0.8*np.sin(1.5*s)
y_d = 0.0*np.ones(x_d.size)

xp_a = np.diff(x_d)/ts
yp_a = np.diff(y_d)/ts

# Angle over the path (phi_p)

phi_p = np.arctan2(yp_a,xp_a)


### Reserved Space of Memory

x_t    = np.array([])
y_t    = np.array([])

nu_x = np.array([])
nu_y = np.array([])
nu_p = np.array([])

phi_d = np.array([])

phip_d = np.array([])

u = np.array([])
v = np.array([])
w = np.array([])

x    = np.array([0])
y    = np.array([1])
phi  = np.array([0*np.pi/4])

xa    = np.array([0])
ya    = np.array([2])
phia  = np.array([0*np.pi/4])

xpr   = np.array(x + a*np.cos(phi+alpha))
ypr   = np.array(y + a*np.sin(phi+alpha))

df    = 100; k=0;

if __name__ == '__main__':
    
    #Controller

    while (df > 0.1 and k<1000):        

        dist   = np.sqrt(np.power(xpr[k]-x_d,2)+ np.power(ypr[k]-y_d,2))
        k_p = list(dist).index(np.min(dist))

        
        df     = np.sqrt(np.power(xpr[k]-x_d[x_d.size-1],2)+ np.power(ypr[k]-y_d[y_d.size-1],2))

        xp_d = u_d*np.cos(phi_p[k_p])
        yp_d = u_d*np.sin(phi_p[k_p])

        x_t = np.append(x_t,x_d[k_p] - xpr[k])
        y_t = np.append(y_t,y_d[k_p] - ypr[k])        

        nu_x = np.append(nu_x, xp_d + lx*np.tanh(kx*x_t[k]/lx))
        nu_y = np.append(nu_y, yp_d + ly*np.tanh(ky*y_t[k]/ly))
        
        if a==0:

                phi_d = np.append(phi_d,np.arctan2(nu_y[k],nu_x[k]))
                #phi_d = np.append(phi_d,np.arctan2(np.tanh(y_t[k]),np.tanh(x_t[k])))
                                

                if k>1:                        
                        phi_d[k] = ang2PI.ver(phi_d[k-1],phi_d[k])
                        phip_d   = np.append(phip_d,(phi_d[k]-phi_d[k-1])/ts)
                else:
                        phip_d   = np.append(phip_d,0)

                
                phi_t = phi_d - phi[k]
                nu_p  = phip_d[k]+ lw*np.tanh(kw*phi_t/lw)

        else:
                nu_p = np.append(nu_p,(nu_y[k]*np.cos(phi[k])-nu_x[k]*np.sin(phi[k]))/(a*np.cos(alpha)))    

        u = np.append(u,(nu_x[k]*np.cos(phi[k])+ nu_y[k]*np.sin(phi[k])+ nu_p[k]*a*np.sin(alpha)))
        w = np.append(w,(nu_p[k]))

        ##### Send Commands ####
        
        try:
            
            send_vel(u[k],w[k])            
            
        except rospy.ROSInterruptException:
            pass

        ##### Simulation #######        

        xp   = u[k]*np.cos(phi[k])
        yp   = u[k]*np.sin(phi[k])
        phip = w[k]

        phia      = np.append(phia, phi[k] + phip*ts)        
        phia[k+1] = angPI.ver(phia[k+1])
        phia[k+1] = ang2PI.ver(phia[k],phi[k+1])       

        v   = np.append(v,-xp*np.sin(phi[k])+ yp*np.cos(phi[k]))
        
        xa   = np.append(xa, xa[k]+ xp*ts)
        ya   = np.append(ya, ya[k]+ yp*ts)
        
        xpra   = np.append(xpr,xa[k+1]+ a*np.cos(phi[k+1]+alpha))
        ypra   = np.append(ypr,ya[k+1]+ a*np.sin(phi[k+1]+alpha))


        ###### Get Odometry #######

        odom = get_odom()

        x = np.append(x,odom[0])
        y = np.append(y,odom[1])
        
        phi      = np.append(phi,odom[2])
        phi[k+1] = ang2PI.ver(phi[k],phi[k+1]) 

        k+=1
        

try:    
            
    send_vel(0,0)            
            
except rospy.ROSInterruptException:
    
    pass

        
