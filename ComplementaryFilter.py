########################################################################
#                                                                           
#                                                                    
#                          Complementary Filter                                                           
#                           Complementary.py                                      
#                                                                           
#                                MAIN                                      
#                                                                           
#                 Copyright (C) 2010 Ulrik Hoerlyk Hjort                   
#                                                                        
#  Complementary Filter is free software;  you can  redistribute it                          
#  and/or modify it under terms of the  GNU General Public License          
#  as published  by the Free Software  Foundation;  either version 2,       
#  or (at your option) any later version.                                   
#  Complementary Filter is distributed in the hope that it will be                           
#  useful, but WITHOUT ANY WARRANTY;  without even the  implied warranty    
#  of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  
#  See the GNU General Public License for  more details.                    
#  You should have  received  a copy of the GNU General                     
#  Public License  distributed with Yolk.  If not, write  to  the  Free     
#  Software Foundation,  51  Franklin  Street,  Fifth  Floor, Boston,       
#  MA 02110 - 1301, USA.                                                    
########################################################################        

############################################################################
#
# 
#
############################################################################
class Complementary :
    
    ########################################################################
    #
    # 
    #
    ########################################################################
    def __init__(self,initialAngle=0.0, alpha=0.02):    
        self.alpha = alpha
        self.oneMinusAlpha = 1- self.alpha
        self.angle = initialAngle
        self.lastT = 0

    ########################################################################
    #
    # 
    #
    ########################################################################
    def reset(initialAngle=0.0, alpha=0.02):    
        self.alpha = alpha
        self.oneMinusAlpha = 1- self.alpha
        self.angle = initialAngle
        self.lastT = 0
        
    ########################################################################
    #
    # 
    #
    ########################################################################
    def filter(self, acc, gyro,t):
        dt = (t-self.lastT) / 1000.0
        self.lastT=t
        self.angle = self.oneMinusAlpha*(self.angle + gyro * dt) + (self.alpha)*acc        
        return self.angle
        