#!/usr/bin/python

import os
import sys
import array
import pprint
import numpy
from matplotlib import pyplot
import matplotlib.animation as animation

class Sensor:

    def __init__(self,size):
        self._fundamental_size = size

    def size(self):
        return self._fundamental_size

class LMS151(Sensor):

    def __init__(self):
        Sensor.__init__(self,(541)+1)


class Visualiser:

    def __init__(self, path ):
        self._target_path_ = path
        if not os.path.exists( path ):
            raise IOError()
        self._file_handle_ = open( self._target_path_, 'rb' )

        self._drawable_ = None

    def render(self):
        fig, ax = pyplot.subplots()

        self._drawable_, = pyplot.scatter( [], [] )

        ani = animation.FuncAnimation(fig, 
                self.draw, 
                None, 
                blit=True, 
                interval=10, 
                repeat=False)
        pyplot.show()

    def draw(self, index ):

        #print index

        #arr = self.read()

        #ranges = arr[1:541+1]

        return self._drawable_


    def __iter__(self):
        return self

    def next(self):

        arr = self.read()

        if not arr:
            raise StopIteration
        else:
            return arr


class LIDARViz(Visualiser):

    def __init__(self, path ):
        Visualiser.__init__(self, path)

        self._sensor_ = LMS151()
    
    def read(self):

        range_array = array.array( 'd' )
        reflectance_array = array.array( 'f' )

        try:
            range_array.fromfile( self._file_handle_, self._sensor_.size() )
            reflectance_array.fromfile( self._file_handle_, 541 )
            return (range_array,reflectance_array)
        except:
            return []


if __name__=="__main__":
    
    if len(sys.argv)<2:
        collection = LIDARViz( os.path.expanduser( '~/code/datasets/2013-10-01_08-11-43/L3/laser_0_lms151.lidar' ) )
    else:
        collection = LIDARViz( os.path.join( sys.argv[1], 'L3/laser_0_lms151.lidar' ) )

    collection.render()

    #for scan in collection:
        #print scan
