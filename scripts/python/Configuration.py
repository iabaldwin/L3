import os
import sys
import pprint 
import random

class Parser:

    def __init__(self):
        pass

    @staticmethod
    def read( fname ):
        return open( fname ).read().split('\n' )

class Configuration(Parser):

    def __init__(self):
    
        Parser.__init__(self) 

        self.configuration_directory = '/Users/ian/code/datasets/configuration/missions/' 
        
class Mission(Configuration):
    
    def __init__(self, dataset_name ):
        Configuration.__init__(self) 
     
        self.x = 0
        self.y = 0
        self.z = 0
        self.name = dataset_name

        data = Parser.read( dataset_name )

        npos = ' '.join(data).find( 'Begbroke' ) 

        if ( npos != -1 ):
            self.locale = 'Begbroke'
            self.x -= 616648.701255
            self.y -= 5742069.101970
            self.z -= 73.0

        elif ' '.join(data).find( 'Woodstock'  ):
            self.locale = 'Woodstock'
            self.x -= 613210.368664
            self.y -= 5745105.147059
            self.z -= 73.0


        else:
            self.locale = ''

    def repr( self ):
        return self.name

    def str( self ):
        return self.name


if __name__=="__main__":

    c = Configuration()

    dataset_listing = os.listdir( c.configuration_directory )

    for dataset in dataset_listing:

        if not os.path.isdir( os.path.join( c.configuration_directory, dataset ) ):
            m = Mission( os.path.join( c.configuration_directory, dataset ) )
            print m.locale + ':' + m.name 
