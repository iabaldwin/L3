import os
import cPickle

import Math
import Dataset

import numpy

from matplotlib import pyplot
from matplotlib import animation 


class Generator:

    def __init__(self, dataset ):

        D = Dataset.Dataset( dataset )

        threaded = Dataset.Threader.Thread( D.horizontal_LIDAR_data, D.INS_data )

        self.clouds = []

        
        for index, threaded_elements in enumerate( threaded ):

            pt_cloud = Dataset.Projector.Project( threaded_elements )

            self.clouds.append( pt_cloud )

            if ( index > 20 ):
                break

        self.figure = pyplot.figure()                
        self.axes = self.figure.add_subplot(111)


    def Plot(self):

        # Create figure
        self.scatter = pyplot.scatter( [], [], c='tomato' )

        # Create animation
        anim = animation.FuncAnimation(self.figure, self.plot_helper, frames=len(self.clouds) )

        # Run
        pyplot.show()

    def plot_helper( self, i ):
        
        # Extract XY
        dat = self.clouds[i].pts[ 0::, 0::2 ]
        
        self.scatter.set_offsets( dat )

        mins = numpy.amin( dat, axis=0 )
        maxs = numpy.amax( dat, axis=0 )

        self.axes.set_xlim( [mins[0], maxs[0] ] )
        self.axes.set_ylim( [mins[1], maxs[1] ] )

        return self.scatter,



if __name__=="__main__":

    G = Generator( '/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/' ).Plot()
