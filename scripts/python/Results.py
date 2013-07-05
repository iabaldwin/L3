#!/usr/bin/python
import os, sys
import array
import pprint

import numpy
from numpy import random

import Listings

sys.path.append( '/Users/ian/code/L3/learning/' )

import Dataset
import Configuration

from matplotlib import pyplot
import matplotlib.cm as cm

def datasetName( dataset ):

    sp = dataset.split( '/' )

    index = -1

    if ( len(sp) == 0 ):
        index -= 1
    

    return sp[index]



class Region:

    def __init__(self):
        pass


class Global(Region):

    def __init__(self):
        Region.__init__(self)

        self.lim = 500
        self.centre = (500,500/2)

class Corner(Region):

    def __init__(self):
        Region.__init__(self)

        self.lim = 20
        self.centre = (890, 386)

class Road(Region):

    def __init__(self):
        Region.__init__(self)

        self.lim = 20
        self.centre = (700, 440)

class Trajectory:

    def __init__(self):
        self.trajectories = []

        self._buildPlotMap()

    def _buildPlotMap(self):

        datasets = Listings.Listing.listDatasets();

        # Remove experience
        exp = Listings.Listing.listExperiences()

        del datasets[ datasets.index( exp[0] ) ]

        counter = 0

        self.colormap = {}

        self.colormap[ datasetName(exp[0]) ] = 'k'

        for d in datasets:

            #color = cm.summer(counter*20)
            color = cm.Paired(counter*5)

            self.colormap[ datasetName(d) ] = color

            counter += 1


    def Plot(self, region, figure=None ):

        assert( isinstance( region, Region ) )

        if not figure:

            self.figure  = pyplot.figure()

            self.figure.patch.set_facecolor('white')

        else:
        
            self.figure = figure
            
        self.figure.hold( True )

        for t in self.trajectories:

            color = self.colormap[t[2]]

            pyplot.plot( t[0], t[1], c=color, ls=t[3] )

        pyplot.xlim( (region.centre[0]-region.lim, region.centre[0]+region.lim ) )
        pyplot.ylim( (region.centre[1]-region.lim, region.centre[1]+region.lim) )

        pyplot.xlabel( 'X (m)' )
        pyplot.ylabel( 'Y (m)' )

        pyplot.grid(True)
       
        return self

    def Show( self ):
        pyplot.show()

class Experience( Trajectory ):

    def __init__(self):
        Trajectory.__init__(self) 

        exp = Listings.Listing.listExperiences()

        d = Dataset.Dataset( exp[0] )

        X = []
        Y = []

        for pose in d.INS_data:
            X.append( pose.x )
            Y.append( pose.y )

        c =  'k'

        self.trajectories.append( (X,Y, datasetName(exp[0]), '-' ) )


class INSTrajectories(Trajectory):

    def __init__(self):
        Trajectory.__init__(self) 

        datasets = Listings.Listing.listDatasets();

        # Remove experience
        exp = Listings.Listing.listExperiences()

        del datasets[ datasets.index( exp[0] ) ]

        counter = 0

        for dataset in datasets:

            try:

                d = Dataset.Dataset( dataset )

                dataset_name = dataset.split( '/' )[-1]

                c = Configuration.Mission( os.path.join( Configuration.Configuration.configuration_directory, dataset_name+ '.config' ) )

                if (c.locale != 'Woodstock' ):
                    continue
                else :
                    print c.locale

                X = []
                Y = []


                for pose in d.INS_data:
                    X.append( pose.x )
                    Y.append( pose.y )


                print datasetName( dataset )
                self.trajectories.append( (X,Y, datasetName(dataset), '-' ) )

            except IOError:
                continue

class L3Trajectories(Trajectory):

    def __init__(self):
        Trajectory.__init__(self) 

        datasets = Listings.Listing.listDatasets();

        # Remove experience
        exp = Listings.Listing.listExperiences()

        del datasets[ datasets.index( exp[0] ) ]

        counter = 0
        
        for dataset in datasets:

            pose_file = os.path.join( dataset, 'L3', 'poses.dat' ) 
            if os.path.exists( pose_file ):

                print 'Rendering : %s ' % dataset
                

                poses = open( pose_file, 'r' ).read().split( '\n' ) 
            
                X = []
                Y = []

                for pose in poses:
                    spl = pose.split()

                    if len(spl)< 6:
                        continue

                    X.append( float(spl[0] ) )
                    Y.append( float(spl[1] ) )


                print datasetName(dataset)
                self.trajectories.append( (X,Y, datasetName(dataset), '--' )) 


if __name__=="__main__":

    #location = Global()
    location = Road()
    #location = Corner()
    
    L3 = Experience().Plot( location )

    I = INSTrajectories().Plot( location, L3.figure )
    L3 = L3Trajectories().Plot( location, L3.figure ).Show()
    
