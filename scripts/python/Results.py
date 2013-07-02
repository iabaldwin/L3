#!/usr/bin/python
import os, sys
import array
import pprint

from numpy import random

import Listings

sys.path.append( '/Users/ian/code/L3/learning/' )

import Dataset
import Configuration

from matplotlib import pyplot
import matplotlib.cm as cm

if __name__=="__main__":

    datasets = Listings.Listing.listDatasets();

    exp = Listings.Listing.listExperiences()

    del datasets[ datasets.index( exp[0] ) ]

    f  = pyplot.figure()

    trajectories = []

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
            
            c = cm.summer(counter*50)
            
            counter += 1


            trajectories.append( (X,Y,c ) )

        except IOError:
            pass

    lim = 20

    corner = (890, 386)

    f.hold( True )

    for t in trajectories:

        pyplot.plot( t[0], t[1], c=t[2] ) 

    pyplot.xlim( (corner[0]-lim, corner[0]+lim ) )
    pyplot.ylim( (corner[1]-lim, corner[1]+lim) )

    pyplot.grid(True)
    pyplot.show()

