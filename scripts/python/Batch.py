#!/usr/bin/python

import os
import pprint

import Creator
import Listings
import Extractor
import Configuration

class Bulk:

    def __init__(self):

        root = '~/code/datasets/' 

        for dataset in os.listdir( os.path.expanduser( root ) ):

            if not os.path.exists( os.path.join( os.path.expanduser( root ), dataset, 'L3' ) ):
                continue

            # Have we already done it?
            if not self.validate( os.path.join( os.path.expanduser( root ), dataset ) ):

                print '\nDataset: ---%s---' % dataset 

                #Create extractor dataset
                e = Extractor.Dataset( os.path.join( os.path.expanduser( root) , dataset ) ) 

                sessions = e._mission.sessions
        
                if len( sessions ) == 0 :
                    print 'No sessions for  <%s> ' % dataset
                    continue

                target = []
                for session in sessions:
                    target = session 
                    break

        
                # Parse the data
                e.Parse( target[0], target[1] )
                
                # Run creator
                Creator.Creator( e )


                break


    def validate( self, dataset ):

        return os.path.exists( os.path.join( dataset, 'L3', 'LMS1xx_10420001_192.168.0.51.lidar' ) ) &  \
        os.path.exists( os.path.join( dataset, 'L3', 'LMS1xx_10420002_192.168.0.50.lidar' ) ) & \
        os.path.exists( os.path.join( dataset, 'L3', 'OxTS.ins' ) ) & \
        os.path.exists( os.path.join( dataset, 'L3', 'OxTS.lhlv' ) ) & \
        os.path.exists( os.path.join( dataset, 'L3', 'vel.sm' ) ) 

if __name__=="__main__":

    Bulk()
    
