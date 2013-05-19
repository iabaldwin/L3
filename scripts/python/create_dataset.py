#!/usr/bin/python

import os
import sys
import Parsers 
import Configuration

def printUsage():
    return "Usage: %s <dataset_name>" % sys.argv[0]

if __name__=="__main__":

    if len( sys.argv ) < 2:
        sys.exit( printUsage() )

    dataset = Parsers.dataset( sys.argv[1] )
    #dataset = Parsers.dataset( '/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow' )

    c = Configuration.Configuration()
    mission = Configuration.Mission( os.path.join( c.configuration_directory, os.path.split( sys.argv[1] )[-1] + '.config' )  )

    parsers = []
    
    # Start
    start = 2*60

    # End
    limit = 25*60
    
    parsers.append( Parsers.INS( dataset.root, mission ).binary().duration( (start,limit) ).parse() )
    parsers.append( Parsers.LHLV( dataset.root ).binary().duration( (start,limit) ).parse() )
    parsers.append( Parsers.LIDAR( dataset.root ).binary().duration( (start,limit) ).parse() )

    [ parser.write(dataset.target_directory) for parser in parsers ]

