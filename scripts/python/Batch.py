#!/usr/bin/python

import os
import pprint

import Creator
import Listings
import Extractor
import Configuration

class Bulk:

    def __init__(self):

        self._configuration = Configuration.Configuration()


if __name__=="__main__":

    root = '~/code/datasets/' 

    for dataset in os.listdir( os.path.expanduser( root ) ):

        print '\nDataset: ---%s---' % dataset 

        try:
            #Create extractor dataset
            e = Extractor.Dataset( os.path.join( os.path.expanduser( root) , dataset ) ) 

            sessions = e._mission.sessions
    
            if len( sessions ) == 0 :
                continue

            target = []
            for session in sessions:
                target = session 
                break

    
            # Parse the data
            e.Parse( target[0], target[1] )
            
            # Run creator
            Creator.Creator( e )

        except Exception as e:
            print e
            print '-----------------------' 

    print '-----------------------' 
