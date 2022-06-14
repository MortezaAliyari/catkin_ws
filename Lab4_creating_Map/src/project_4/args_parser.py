import argparse
from math import radians
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

def parse_args():
    parser = argparse.ArgumentParser(description='Map Maker')
    parser.add_argument('bagfile')
    parser.add_argument('-r', '--resolution', default=0.5, type=float)
    parser.add_argument('-d', '--dimension', default=20.0, type=float)
    parser.add_argument('-lx', '--laser_x_offset', default=0.0, type=float)
    parser.add_argument('-ly', '--laser_y_offset', default=0.0, type=float)
    parser.add_argument('-lt', '--laser_theta_offset', default=0.0, type=float)
    parser.add_argument('-p', type=int)
    args = parser.parse_args()

    if args.p is not None:
        if args.p==1:
            args.dimension = 20.0
            args.laser_x_offset = 0.0
            args.laser_y_offset = 0.0
            args.laser_theta_offset = 0.0
        elif args.p==2:
            args.dimension = 25.0
            args.laser_x_offset = 0.12
            args.laser_y_offset = 0.12
            args.laser_theta_offset = radians(45)
        else:
            print ("Invalid part number!")
            exit(0)
    
    size = int( args.dimension / args.resolution )
    origin = -args.dimension / 2

    q = quaternion_from_euler(0,0,args.laser_theta_offset)
    
    to_laser = TransformStamped()
    to_laser.header.frame_id = '/base_link'
    to_laser.child_frame_id = '/base_laser_link'
    to_laser.transform.translation.x = args.laser_x_offset
    to_laser.transform.translation.y = args.laser_y_offset
    to_laser.transform.rotation.x = q[0]
    to_laser.transform.rotation.y = q[1]
    to_laser.transform.rotation.z = q[2]
    to_laser.transform.rotation.w = q[3]

    return args.bagfile, size, origin, args.resolution, to_laser
    
