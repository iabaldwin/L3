import os
import sys
import pprint 
import random

from pylibconfig import Config

class Parser:

    def __init__(self):
        pass

class Configuration(Parser):
        
    configuration_directory = '/Users/ian/code/datasets/configuration/missions/' 
        

    def __init__(self):
    
        Parser.__init__(self) 

class Mission(Configuration):
    
    def __init__(self, dataset_name ):
      
        Configuration.__init__(self) 
     
        self.x = 0
        self.y = 0
        self.z = 0
        self.name = dataset_name

        mission_config = Config()

        if not os.path.isfile( dataset_name ):
            raise OSError(2, 'No such file or directory', dataset_name )

        mission_config.readFile( dataset_name )

        locale_data = mission_config.value( 'mission.locale' )

        if not locale_data[1]:
            raise Exception('No locale data for %s' % dataset_name )
        else:
            self.locale = locale_data[0]

        # Load all the sessions
        counter = 0 
      
        self.sessions = set()

        while True:
      
            start_pair = mission_config.value( 'results.sessions.[%d].start' % counter )
            end_pair = mission_config.value( 'results.sessions.[%d].end' % counter )

            if not start_pair[1]:
                break
            else:
                self.sessions.add( (start_pair[0], end_pair[0] )  )
                counter += 1

        # Load the locale data
        locale_config = Config()

        locale_config.readFile( os.path.join( os.path.expanduser( '~/code/datasets/configuration/datums/%s_datum.config' % self.locale.lower() ) ) ) 

        self.x = locale_config.value( 'datum.X.lower' )[0]
        self.y = locale_config.value( 'datum.Y.lower' )[0]
        self.z = 73.0


    def __repr__( self ):
        return self.name

    def __str__( self ):
        return self.name


if __name__=="__main__":

    c = Configuration()

    dataset_listing = os.listdir( c.configuration_directory )

    for dataset in dataset_listing:

        if not os.path.isdir( os.path.join( c.configuration_directory, dataset ) ):
            
            try:
                m = Mission( os.path.join( c.configuration_directory, dataset ) )
                print m.locale + ':' + m.name 
            except:
                pass
