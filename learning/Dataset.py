#!/usr/bin/python 
import os 
import numpy 
import pprint 
import bisect
from array import array 

import cPickle

import Math
import Sensors 

from matplotlib import pyplot

class Dataset:

    def __init__(self, path, **kwargs ):

        try:
            start = kwargs.pop('start')
        except:
            start = 0

        try:
            end = kwargs.pop('end')
        except:
            end = float('inf')

        self._horizontal_LIDAR = 'LMS1xx_10420001_192.168.0.51.lidar'
        self._vertical_LIDAR    = 'LMS1xx_10420002_192.168.0.50.lidar'
        self._INS              = 'OxTS.ins'
        
        if os.path.exists( '.dataset' ):
            data = cPickle.load( open( '.dataset', 'r' ) )
            self.horizontal_LIDAR_data = data[0] 
            self.vertical_LIDAR_data   = data[1] 
            self.INS_data              = data[2] 
        else:
            self.horizontal_LIDAR_data = self.readLIDAR( path, self._horizontal_LIDAR, (start,end) )
            self.vertical_LIDAR_data   = self.readLIDAR( path, self._vertical_LIDAR, (start,end) )
            self.INS_data              = self.readINS( path, self._INS, (start,end) )

            data = []
            data.append( self.horizontal_LIDAR_data )
            data.append( self.vertical_LIDAR_data )
            data.append( self.INS_data )

            cPickle.dump( data, open( '.dataset', 'w' ) )

    def readINS( self, path, fname, bounds ):

        target = os.path.join( path, 'L3', fname )
        
        arr = self.readDoubles( target )

        sensor_size = Sensors.INS.Size()

        num_elements = len(arr)/sensor_size 

        INS_frame = numpy.array( arr.tolist() )
        
        INS_frame = numpy.reshape( INS_frame, (num_elements, sensor_size ) ) 

        # Create LIDAR
        #return [ Sensors.INS(frame) for frame in INS_frame]
       
        frames = []
        for frame in INS_frame:
            
            relative_time = frame[0] - INS_frame[0][0] 

            if ( (relative_time >= bounds[0] ) and (relative_time < bounds[1] ) ):
                frames.append( Sensors.INS( frame ) )

        return frames

    
        
    def readLIDAR( self, path, fname, bounds ):

        target = os.path.join( path, 'L3', fname )
        
        arr = self.readDoubles( target )

        sensor_size = Sensors.LIDAR.Size()

        num_elements = len(arr)/sensor_size 

        LIDAR_frame = numpy.array( arr.tolist() )

        LIDAR_frame = numpy.reshape( LIDAR_frame, (num_elements, sensor_size ) ) 

        # Create LIDAR
        frames = []
        for frame in LIDAR_frame:
            
            relative_time = frame[0] - LIDAR_frame[0][0] 

            if ( (relative_time >= bounds[0] ) and (relative_time < bounds[1] ) ):
                frames.append( Sensors.LIDAR( frame ) )

        return frames


    def readDoubles( self, target ):

        file_handle = open( target, 'rb' )

        arr = array('d')

        size = os.path.getsize( target )

        # Read doubles
        arr.fromfile( file_handle, size/8 )

        return arr

class Threader:

    def Thread( self, LIDAR_frames, INS_frames ):

        threaded = [] 

        INS_frames.sort( key=lambda x: x.time ) 

        keys = [ frame.time for frame in INS_frames ]
     
        for LIDAR_frame in LIDAR_frames:
           
            try:
                INS_frame = INS_frames[ bisect.bisect_left( keys, LIDAR_frame.time ) ]
                threaded.append( (LIDAR_frame, INS_frame) ) 
       
            except:
                break

        return threaded

def plot( dataset ):

    poses = dataset.INS_data

    X = []
    Y = []

    for pose in poses:
        X.append( pose.x )
        Y.append( pose.y )
    
    pyplot.scatter( X, Y )
    pyplot.show()


def plot_pairs( pairs ):


    X = []
    Y = []

    for element in pairs:
        X.append( element[0] )
        Y.append( element[1] )
    
    pyplot.scatter( X, Y )
    pyplot.show()


if __name__=="__main__":

    #D = Dataset( '/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/', start=0, end=float('inf') )
    D = Dataset( '/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/', start=100, end=120 )

    threaded = Threader().Thread( D.horizontal_LIDAR_data, D.INS_data )

    pts = []

    for element in threaded:

        pts.extend( Math.projectScanAtPose( element[0], element[1] ) )
  
    plot_pairs( pts )
    



    #plot(D)

