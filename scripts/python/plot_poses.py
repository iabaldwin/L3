#!/usr/bin/python

import sys, os
from matplotlib import  pyplot

if len( sys.argv )< 2:
    sys.exit( 'Usage: %s <dataset>' % sys.argv[0] )


target = os.path.join( sys.argv[1], 'L3', 'poses.dat' ) 
if not os.path.exists( target ):
    sys.exit( 'No such file: %s ' % target )

#poses = filter( lambda x: x, open( os.path.expanduser( '~/code/L3.build/release/poses.dat' ) ).read().split( '\n')  )
poses = filter( lambda x: x, open( target ).read().split( '\n')  )

X = []
Y = []

for pose in poses:


    sp = pose.split()

    if (len(sp) < 4 ):
        break

    X.append( float(sp[0]) )
    Y.append( float(sp[1]) )


pyplot.scatter( X, Y )
pyplot.show()
