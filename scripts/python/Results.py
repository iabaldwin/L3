#!/usr/bin/python
import os
import array
import pprint

import Listings

class Trajectory:

    def __init__(self, dataset ):
        self.dataset = dataset


    def Plot( self ):

        target =os.path.join( dataset  , 'L3' , 'OxTS.ins' ) 
       
        if not os.path.exists( target ):
           raise Exception()

        

#print os.path.getsize( 
if __name__=="__main__":

    datasets = Listings.Listing.listDatasets();

    exp = Listings.Listing.listExperiences()

    del datasets[ datasets.index( exp[0] ) ]
    
    trajectories = []
    [trajectories.append( Trajectory( x )  ) for x in datasets ]
