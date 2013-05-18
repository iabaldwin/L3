#!/usr/bin/python

import os
import sys

for a,b,c in os.walk( os.path.expanduser( '~/code/datasets/' ) ):

    for entry in c:
        name, extension = os.path.splitext( entry )

        if name == 'experience' :

            print 'Experience [%s]' % os.path.join( a, entry) 
