#!/usr/bin/python

import os
from matplotlib import  pyplot

poses = filter( lambda x: x, open( os.path.expanduser( '~/code/L3.build/release/poses.dat' ) ).read().split( '\n')  )

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
