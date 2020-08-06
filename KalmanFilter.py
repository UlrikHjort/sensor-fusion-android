########################################################################
#                                                                           
#                                                                    
#                           Kalman Filter                                                           
#                           KalmanFilter.py                                      
#                                                                           
#                                MAIN                                      
#                                                                           
#                 Copyright (C) 2010 Ulrik Hoerlyk Hjort                   
#                                                                        
#  Kalman Filter is free software;  you can  redistribute it                          
#  and/or modify it under terms of the  GNU General Public License          
#  as published  by the Free Software  Foundation;  either version 2,       
#  or (at your option) any later version.                                   
#  Kalman Filter is distributed in the hope that it will be                           
#  useful, but WITHOUT ANY WARRANTY;  without even the  implied warranty    
#  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  
#  See the GNU General Public License for  more details.                    
#  You should have  received  a copy of the GNU General                     
#  Public License  distributed with Yolk.  If not, write  to  the  Free     
#  Software Foundation,  51  Franklin  Street,  Fifth  Floor, Boston,       
#  MA 02110 - 1301, USA.                                                    
########################################################################        

import numpy as np

##########################################################################
#
# 
#
############################################################################
class Kalman :
    
    ########################################################################
    #
    # 
    #
    ########################################################################
    def __init__(self):       
        self.QAngle = 0.001
        self.QBias = 0.003
        self.R = 0.02
        self.angle = 0.0
        self.bias = 0.0
        self.acc = 0
        self.gyro = 0
        self.P = np.array([[0.0, 0.0], [0.0, 0.0]])
        self.lastT = 0
                         
    ########################################################################
    #
    # 
    #
    ########################################################################
    def filter(self,acc, gyro,t):
                
        dt = (t-self.lastT) / 1000.0
        
        # Predict state: 
        self.gyro = gyro - self.bias;
        self.angle += dt * self.gyro;
    
        # Error covariance:
        self.P[0][0] = self.P[0][0] + (dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.QAngle))
        self.P[0][1] = self.P[0][1] - (dt * self.P[1][1])
        self.P[1][0] = self.P[1][0] - (dt * self.P[1][1])
        self.P[1][1] = self.P[1][1] + (self.QBias * dt)


        # Estimate error and compute Kalman gain:           
        S = self.P[0][0] + self.R 
        K = np.array([0.0, 0.0]) 
        K[0] = self.P[0][0] / S
        K[1] = self.P[1][0] / S

        # Compute the estimate:
        angleDelta = acc - self.angle
        self.angle = self.angle + (K[0] * angleDelta)
        self.bias = self.bias + (K[1] * angleDelta)
        
        #Compute error covariance:
        p00 = self.P[0][0]
        p01 = self.P[0][1]

        self.P[0][0] = self.P[0][0] - (K[0] * p00)
        self.P[0][1] = self.P[0][1] - (K[0] * p01)
        self.P[1][0] = self.P[1][0] - (K[1] * p00)
        self.P[1][1] = self.P[1][1] - (K[1] * p01)

        return self.angle;

