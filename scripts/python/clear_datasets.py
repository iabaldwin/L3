#!/usr/bin/python

import os
import sys
import shutil

for a,b,c in os.walk( os.path.expanduser( '~/code/datasets/' ) ):

    if a.split( '/' )[-1] == 'L3':
        print 'Removing ' +  a
        shutil.rmtree( a )
