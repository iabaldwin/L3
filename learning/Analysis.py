import sys, os
import cPickle

import numpy

import Math
import Dataset, Sensors

from matplotlib import pyplot
from matplotlib import animation 

from sklearn import cluster

import matplotlib.cm as cm

class Generator:

    def __init__(self, dataset ):

        #D = Dataset.Dataset( dataset, regenerate=True, start=20, end=30 )
        D = Dataset.Dataset( dataset )

        self.threaded = Dataset.Threader.Thread( D.horizontal_LIDAR_data, D.INS_data )

        self.poses  = []
        self.clouds = []
 
    def BuildLocal( self ):

        zero_pose = Sensors.INS.Identity()

        for index, threaded_elements in enumerate( self.threaded ):

            zeroed_thread = (threaded_elements[0], zero_pose )

            pt_cloud = Dataset.Projector.Project( zeroed_thread )

            self.clouds.append( pt_cloud )

        return self;

    def BuildGlobal( self ):

        for index, threaded_elements in enumerate( self.threaded ):

            pt_cloud = Dataset.Projector.Project( threaded_elements )

            self.clouds.append( pt_cloud )
            self.poses.append( threaded_elements[1] )

        return self;

class Plotter:

    def __init__(self, generator ):
        assert( isinstance( generator, Generator ) )
    
        self.generator = generator
        

    def Plot(self):

        # Create figure
        self.figure = pyplot.figure()                
        self.axes = self.figure.add_subplot(111)
        
        master = []
        clouds = [] 

        for cloud in self.generator.clouds:
            clouds.append( cloud.pts )

        # Stack em
        master = numpy.vstack( clouds )
    
        # Create figure
        self.scatter = pyplot.scatter( master[0::, 0], master[0::, 1] , c='tomato' )
        
        pyplot.show()

    def Animate(self):

        self.figure = pyplot.figure()                
        #self.axes = self.figure.add_subplot(111)

        # Create figure
        self.scatter = pyplot.scatter( [], [], c='tomato' )

        # Create animation
        #anim = animation.FuncAnimation(self.figure, self.plot_helper, frames=len(self.generator.clouds), init_func=self.init )
        anim = animation.FuncAnimation(self.figure, self.plot_helper, frames=len(self.generator.clouds) )

        # Run
        pyplot.show()

    def init( self ):
        self.figure.clf()

    def plot_helper( self, i ):
        
        self.figure.clf()
        
        # Extract XY
        dat = self.generator.clouds[i].pts[  0::, 0:2 ]

        self.scatter = pyplot.scatter( dat[0::, 0], dat[0::, 1] , c='tomato' )
        #self.scatter.set_offsets( dat )

        mins = numpy.amin( dat, axis=0 )
        maxs = numpy.amax( dat, axis=0 )
        
        #mins = (x_min, y_min)
        #maxs = (x_max, y_max)
        #self.axes.set_xlim( [mins[0], maxs[0] ] )
        #self.axes.set_ylim( [mins[1], maxs[1] ] )

        #dbscan = cluster.DBSCAN(eps=2)
        dbscan = cluster.DBSCAN(eps=5)

        dbscan.fit( dat )

        # Get labels
        y_pred = dbscan.labels_.astype(numpy.int)

        # Build sub clusters
        sub_clusters = []

        for i in range( 0, numpy.amax( y_pred ) ):
            sub_clusters.append(  dat[y_pred == i, 0:: ] )

        for sub_cluster in sub_clusters:

            mins = numpy.amin( sub_cluster, 0 )
            maxs = numpy.amax( sub_cluster, 0 )

            pyplot.hlines( mins[1], mins[0], maxs[0] )
            pyplot.hlines( maxs[1], mins[0], maxs[0] )
            
            pyplot.vlines( mins[0], mins[1], maxs[1] )
            pyplot.vlines( maxs[0], mins[1], maxs[1] )


        # Plot clusters
        colors = cm.hot( y_pred ) 
        
        self.scatter.set_array( colors[ ::, 0] )

        self.scatter.axes.set_xlim( -25, 25 )
        self.scatter.axes.set_ylim( -20, 30 )

        return self.scatter,


if __name__=="__main__":

    G = Generator( '/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/' ).BuildLocal()
    
    P = Plotter( G ).Animate()

