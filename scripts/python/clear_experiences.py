#!/usr/bin/python

import os
import sys
import shutil


for a,b,c in os.walk( os.path.expanduser( '~/code/datasets/' ) ):

    for entry in c:
        name, extension = os.path.splitext( entry )

        if name == 'experience' :

            print 'Clearing [%s]' % os.path.join( a, entry) 

            # Erase it
            os.remove( os.path.join( a, entry) )
