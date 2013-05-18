#!/usr/bin/python

import os
import sys

for a,b,c in os.walk( os.path.expanduser( '~/code/datasets/' ) ):

    if a.split( '/' )[-1] == 'L3':
        print a

