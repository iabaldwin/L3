#!/usr/bin/python

import os
import array
import pprint
import Listings
from matplotlib import pyplot

class INSPlotter:

    def __init__( self, dataset ):

        self._file_handle = open( dataset, 'rb' )
    
        self.readPoses()

    def readPoses( self ):

        self.poses = array.array( 'd' )
     
        # This is blazingly fast
        self.poses.fromfile( self._file_handle, 100000 )

        times = self.poses[0:-1:7]
        self.X = self.poses[1:-1:7]
        self.Y = self.poses[2:-1:7]
        

    def show(self):

        pyplot.scatter( self.X, self.Y, marker='+', s=1 )
        pyplot.show()


if __name__=="__main__":

    datasets = Listings.Listing.listDatasets()

    for dataset in datasets:

        try:

            INSPlotter( os.path.join( dataset, 'L3/OxTS.ins' ) ).show()
        except:
            pass
