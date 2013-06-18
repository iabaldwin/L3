#!/usr/bin/python

import os
import sys
import Parsers 
import Configuration

class Dataset:

    def __init__(self, dataset ):
               
        self._name = dataset
    
        self._dataset = Parsers.dataset( self._name )

        self._configuration = Configuration.Configuration()

        dataset_tag = dataset.split('/')[-1]

        self._mission = Configuration.Mission( os.path.join( self._configuration.configuration_directory, dataset_tag + '.config' )  )

    def Parse(self, start, limit ):

        parsers = []

        parsers.append( Parsers.INS( self._dataset.root, self._mission ).binary().duration( (start, limit) ).parse() )
        parsers.append( Parsers.LHLV( self._dataset.root ).binary().duration( (start, limit) ).parse() )
        parsers.append( Parsers.LIDAR( self._dataset.root ).binary().duration( (start, limit) ).parse() )

        [ parser.write(self._dataset.target_directory) for parser in parsers ]

        return self

