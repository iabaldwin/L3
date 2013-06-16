#!/usr/bin/python

import os
import sys
import Parsers 
import Configuration

class Dataset:

    def __init__(self, dataset, start, limit ):

        self._name = dataset
    
        self._dataset = Parsers.dataset( self._name )
        
        self._start = start
        self._limit = limit

        self._configuration = Configuration.Configuration()
        self._mission = Configuration.Mission( os.path.join( self._configuration.configuration_directory, os.path.split( sys.argv[1] )[-1] + '.config' )  )

    def Parse(self):

        print self._mission

        parsers = []

        parsers.append( Parsers.INS( self._dataset.root, self._mission ).binary().duration( (self._start,self._limit) ).parse() )
        parsers.append( Parsers.LHLV( self._dataset.root ).binary().duration( (self._start,self._limit) ).parse() )
        parsers.append( Parsers.LIDAR( self._dataset.root ).binary().duration( (self._start,self._limit) ).parse() )

        [ parser.write(self._dataset.target_directory) for parser in parsers ]

        return self

