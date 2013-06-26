#!/usr/bin/python

import os
import pprint

import Creator
import Listings
import Extractor
import Configuration

class Bulk:

    def __init__(self):

        self._root = '~/code/datasets/' 

    def all( self ):

        for dataset in os.listdir( os.path.expanduser( self._root ) ):

            self._parseDataset( dataset )

    def single( self, dataset ):
            
            self._parseDataset( dataset )

    def _parseDataset( self, dataset ):

            if not dataset.startswith( '20' ):
                print 'Unknown dataset %s' % dataset
                return  


            target = os.path.join( os.path.expanduser( self._root ), dataset, 'L3' )  

            if not os.path.exists( target ):
                print '<%s> does not exist' % target + ' creating...'
                os.mkdir( target )

            dataset_root = os.path.join( os.path.expanduser( self._root ), dataset ) 
            
            # Have we already done it?
            if not self.validate( dataset_root ):

                print '\nDataset: ---%s---' % dataset 

                # Create extractor dataset
                e = Extractor.Dataset( os.path.join( os.path.expanduser( self._root) , dataset ) ) 

                # Get the relative time sessions
                sessions = e._mission.sessions
        
                if len( sessions ) == 0 :
                    print 'No sessions for  <%s> ' % dataset
                    return

                target = []
                for session in sessions:
                    target = session 
                    break
        
                # Parse the data
                e.Parse( target[0], target[1] )
                
                # Run creator
                Creator.Creator( e )

            else:
                print 'Dataset %s is valid' % dataset 


    def validate( self, dataset ):

        return os.path.exists( os.path.join( dataset, 'L3', 'LMS1xx_10420001_192.168.0.51.lidar' ) ) &  \
        os.path.exists( os.path.join( dataset, 'L3', 'LMS1xx_10420002_192.168.0.50.lidar' ) ) & \
        os.path.exists( os.path.join( dataset, 'L3', 'OxTS.ins' ) ) & \
        os.path.exists( os.path.join( dataset, 'L3', 'OxTS.lhlv' ) ) & \
        os.path.exists( os.path.join( dataset, 'L3', 'vel.sm' ) ) 

if __name__=="__main__":

    #Bulk().single( '2012-04-16-20-05-30NightWoodstock1' )
    Bulk().all()
    
