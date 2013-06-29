import numpy
import math

import Sensors

def poseToHomogeneous( pose ):

    Rz = numpy.identity( 4 )

    Rz[0,0] = math.cos( pose.q)
    Rz[0,1] = -1*math.sin( pose.q)
    Rz[1,0] = math.sin( pose.q)
    Rz[1,1] = math.cos( pose.q)

    Rx = numpy.identity( 4 )

    Rx[1,1] = math.cos( pose.p)
    Rx[1,2] = -1*math.sin( pose.p)
    Rx[2,1] = math.sin( pose.p)
    Rx[2,2] = math.cos( pose.p)

    Ry = numpy.identity( 4 )

    Ry[0,0] = math.cos( pose.r)
    Ry[0,2] = math.sin( pose.r)
    Ry[2,0] = -1*math.sin( pose.r)
    Ry[2,2] = math.cos( pose.r)

    homogeneous = numpy.dot( numpy.dot( Rz, Ry ), Rx )

    homogeneous[0,3] = pose.x
    homogeneous[1,3] = pose.y
    homogeneous[2,3] = pose.z

    return homogeneous

def projectScanToXY( LIDAR ):

    angles = numpy.linspace( -45, 225, 541 )

    angles = [ math.radians(x) for x in angles ] 

    return [ ( rng*math.cos(angle), rng*math.sin(angle)) for rng, angle in zip( LIDAR.ranges, angles ) ]


def projectScanAtPose( LIDAR, INS ):

    mat = poseToHomogeneous( INS )

    xy = projectScanToXY( LIDAR )

    projected = []

    for x_y in xy:
        R = numpy.identity( 4 )
        
        R[0,3] = x_y[0] 
        R[1,3] = x_y[1] 
   
        res = numpy.dot( mat, R )
        projected.append( (res[0,3], res[1,3] ) )

    return projected


if __name__=="__main__":

    pose = Sensors.INS( numpy.array( [0,0,0,0,0,0,0] ) )

    poseToHomogeneous( pose )

