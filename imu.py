########################################################################
#                                                                           
#                                                                    
#                         Inertial Measurement Unit                                                           
#                                imu.py                                      
#                                                                           
#                                MAIN                                      
#                                                                           
#                 Copyright (C) 2010 Ulrik Hoerlyk Hjort                   
#                                                                        
#  Inertial Measurement Unit is free software;  you can  redistribute it                          
#  and/or modify it under terms of the  GNU General Public License          
#  as published  by the Free Software  Foundation;  either version 2,       
#  or (at your option) any later version.                                   
#  Inertial Measurement Unit is distributed in the hope that it will be                           
#  useful, but WITHOUT ANY WARRANTY;  without even the  implied warranty    
#  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  
#  See the GNU General Public License for  more details.                    
#  You should have  received  a copy of the GNU General                     
#  Public License  distributed with Yolk.  If not, write  to  the  Free     
#  Software Foundation,  51  Franklin  Street,  Fifth  Floor, Boston,       
#  MA 02110 - 1301, USA.                                                    
########################################################################   
import matplotlib.pyplot as plt
import math
#import ComplementaryFilter as Filter
import KalmanFilter as Filter

class Plots :
    ROLL = 0
    PITCH = 1
    YAW = 2
    MAGNO = 3

############################################################################
#
# 
#
############################################################################
class Accelerometer :
    
    ########################################################################
    #
    # 
    #
    ########################################################################
    def __init__(self):
        self.timeStamp = []
        self.x = []
        self.y = []
        self.z = []
        self.angle = []

    ########################################################################
    #
    # 
    #
    ########################################################################
    def getXYZ(self):
        return list(zip(self.x, self.y, self.z))

    ########################################################################
    #
    # 
    #
    ########################################################################
    def read(self,filename):
        first = True;
        with open(filename, 'r')  as file:
            for line in file:
                if first:
                    first = False
                    continue

                e = line.split(',')
                self.timeStamp.append(int(e[1]))            
                self.x.append(float(e[2]))
                self.y.append(float(e[3]))
                self.z.append(float(e[4]))

    ########################################################################
    #
    # Roll for Android phone (roll around y-axis)
    #
    ########################################################################    
    def roll(self):
        self.angle = []
        for x,z in zip(self.x, self.z):
            self.angle.append(float(math.atan2(float(x) ,float(z)) * 57.3))
        return self.angle


    ########################################################################
    #
    # Pitch for Android phone (pitch around x-axis)
    #
    ########################################################################    
    def pitch(self):
        self.angle = []        
        for x,y,z in zip(self.x, self.y,self.z):            
            self.angle.append(float(math.atan2(-float(y), math.sqrt(float(x)*float(x) + float(z)*float(z))) * 57.3))
        return self.angle


    def plot(self):
        plt.plot(self.x)
        plt.plot(self.y)
        plt.plot(self.z)
            
 
 
############################################################################
#
# 
#
############################################################################ 
class Gyro :
    
    ########################################################################
    #
    # 
    #
    ########################################################################
    def __init__(self):
        self.timeStamp = []
        self.x = []
        self.y = []
        self.z = []
        self.angle = []

    ########################################################################
    #
    # 
    #
    ########################################################################
    def read(self,filename):
        file = open(filename, "r") 
        first = True;

        for line in file:
            if first:
                first = False
                continue

            e = line.split(',')
            self.timeStamp.append(int(e[1]))
            self.x.append(float(e[2]))
            self.y.append(float(e[3]))
            self.z.append(float(e[4]))
            
    ########################################################################
    #
    # Android y-axis
    #
    ########################################################################    
    def roll(self):
        self.angle = [y * 57.3 for y in self.y]
        return self.angle

    ########################################################################
    #
    # Andoid x-axis
    #
    ########################################################################    
    def pitch(self):
        self.angle = [x * 57.3 for x in self.x]
        return self.angle

    ########################################################################
    #
    # 
    #
    ########################################################################    
    def yaw(self):
        self.angle = [z * 57.3 for z in self.z]
        return self.angle


############################################################################
#
# 
#
############################################################################
class Magnetometer :
    ########################################################################
    #
    # 
    #
    ########################################################################
    def __init__(self):
        self.x = []
        self.y = []
        self.z = []
        self.angle = []


    ########################################################################
    #
    # 
    #
    ########################################################################
    def getXYZ(self):
        return list(zip(self.x, self.y, self.z))


    ########################################################################
    #
    # 
    #
    ########################################################################
    def read(self,filename):
        file = open(filename, "r") 
        first = True;

        for line in file:
            if first:
                first = False
                continue

            e = line.split(',')
            self.x.append(float(e[2]))
            self.y.append(float(e[3]))
            self.z.append(float(e[4]))


    ########################################################################
    #
    # 
    #
    ########################################################################
    def getAngle(self):
        self.angle = []        
        for x,y,z in zip(self.x, self.y,self.z):
            a = math.atan2(y,x)
            a = a * 57.3
            a = a + 90.0
            a = (a+360) % 360
            self.angle.append(a)
        return self.angle;
            
############################################################################
#
#    I M U  
#
############################################################################
class Imu :
    ########################################################################
    #
    # 
    #
    ########################################################################
    def __init__(self):
        self.accelerometer = Accelerometer()
        self.gyro = Gyro()
        self.magnometer = Magnetometer()
        #self.filter = Filter.Complementary()
        self.filter = Filter.Kalman()

    ########################################################################
    #
    # 
    #
    ########################################################################    
    def read(self, dataPath):
        self.accelerometer.read(dataPath + "/Accelerometer.csv")
        self.gyro.read(dataPath + "/Gyroscope.csv")
        self.magnometer.read(dataPath + "/Compass.csv")


    ########################################################################
    #
    # 
    #
    ########################################################################    
    def roll(self):
        return (self.accelerometer.roll(), self.gyro.roll())


    ########################################################################
    #
    # 
    #
    ########################################################################    
    def pitch(self):
        return (self.accelerometer.pitch(), self.gyro.pitch())

    ########################################################################
    #
    # 
    #
    ########################################################################    
    def pitchFilter(self):
        l = []
        a,g,t = (self.accelerometer.pitch(), self.gyro.pitch(), self.gyro.timeStamp)
        length = min(len(a),len(g), len(t))
        a = a[:length]
        g = g[:length]
        t = t[:length]
        
        for ae,ge,te in zip(a,g,t):
            l.append(self.filter.filter(ae,ge,te))
        return l

    ########################################################################
    #
    # 
    #
    ########################################################################    
    def yaw(self):
        return self.gyro.yaw()

    ########################################################################
    #
    #  Get the device attitude based on the accelerometer gravity vector and 
    #  the compass reading from themagnetometer.
    #
    ########################################################################    
    def getOrientation(self):
        orientationList = []
        aList = self.accelerometer.getXYZ()
        mList = self.magnometer.getXYZ()
        
        length = min(len(aList),len(mList))
        aList = aList[:length]
        mList = mList[:length]
        
        for (ax, ay, az),(ex, ey ,ez) in zip(aList, mList):            
            normsqA = (ax * ax + ay * ay + az * az)
            g = 9.81
            freeFallGravitySquared = 0.01 * g * g

            if normsqA < freeFallGravitySquared: 
                return None
                
            hx = ey * az - ez * ay
            hy = ez * ax - ex * az
            hz = ex * ay - ey * ax

            normH = math.sqrt(hx * hx + hy * hy + hz * hz)
            if normH < 0.1: 
                return None
        
            invH = 1.0 / normH
            hx = hx *invH
            hy = hy *invH
            hz = hz *invH
            invA = 1.0 / math.sqrt(ax * ax + ay * ay + az * az);
            ax = ax *invA
            ay = ay *invA
            az = az *invA
            mx = ay * hz - az * hy
            my = az * hx - ax * hz;
            mz = ax * hy - ay * hx;
        
            R = [hx, hy, hz, 
                 mx, my, mz, 
                 ax, ay, az
                ]

            orientationList.append(( (((math.atan2(R[1], R[4])  * 57.3) +360) %360), (math.asin(-R[7]) * 57.3), (math.atan2(-R[6], R[8]) * 57.3)))
        return orientationList

    ########################################################################
    #
    # 
    #
    ########################################################################    
    def plot(self, plots):
        if plots == Plots.PITCH :
            plt.plot(self.accelerometer.pitch(), "-b", label="acc")
            plt.plot(self.gyro.pitch(), "-r", label="gyro")                       
        elif plots == Plots.ROLL:
            plt.plot(self.accelerometer.roll())
            plt.plot(self.gyro.roll())
        elif plots == Plots.YAW:
            plt.plot(self.gyro.yaw())
        elif plots == Plots.MAGNO:
            plt.plot(self.magnometer.getAngle())
            