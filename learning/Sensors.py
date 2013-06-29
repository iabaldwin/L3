#!/usr/bin/python

import array

class Sensor:

    def __init__(self):
        self.time = float( 'nan' )
        pass

class INS( Sensor ):

    @staticmethod
    def Size():
        return (6)+1

    @staticmethod
    def Identity():
        return INS( [0,0,0,0,0,0,0])

    def __init__(self, data ):
        Sensor.__init__( self )
    
        self.time = data[0]
        self.x = data[1]
        self.y = data[2]
        self.z = data[3]
        self.r = data[4]
        self.p = data[5]
        self.q = data[6]

class LIDAR( Sensor ):

    @staticmethod
    def Size():
        return (541*2)+1

    def __init__(self, data ):

        Sensor.__init__( self )

        assert( len(data) == (541*2)+1 )
    
        self.time = data[0]
        self.ranges = data[1:541+1]
    
   
if __name__=="__main__":

    LIDAR()
