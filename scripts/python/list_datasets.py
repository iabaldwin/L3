#!/usr/bin/python

import pprint
import Listings as L

#for a,b,c in os.walk( os.path.expanduser( '~/code/datasets/' ) ):

    #if a.split( '/' )[-1] == 'L3':
        #print a

pprint.pprint( L.Listing.listDatasets() )
