#!/usr/bin/python

import sys
import os
import pprint
import subprocess

import Extractor

def printUsage():
    return "Usage: %s <dataset_name> <double:start> <double:end>" % sys.argv[0]

if __name__=="__main__":

    if len( sys.argv ) < 4:
        sys.exit( printUsage() )

    # Start
    start = float(sys.argv[2])

    # End
    limit = float(sys.argv[3])

    if( limit  < start ):
        sys.exit( "%f seconds requested..." % ( limit - start  ) )

    dataset = Extractor.Dataset( sys.argv[1], start, limit ).Parse()

    # Filter LHLV
    lhlv_filter_bin = os.path.expanduser( '~/code/L3.build/release/core/apps/LHLV_filter' )
        
    args = [lhlv_filter_bin, dataset._name]
    subprocess.call( args )

    # Do scan-matching
    scan_matcher_bin = os.path.expanduser( '~/code/octave/scan_matching/standalone.m' )

    args = [scan_matcher_bin, dataset._name, sys.argv[2], sys.argv[3]]
    subprocess.call( args )

