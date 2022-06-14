#!/usr/bin/python
#import roslib; roslib.load_manifest('localization')
from localization import *
from localization.bag import get_dict
from geometry import *
from laser import *
from math import pi
import tf
from tf.transformations import euler_from_quaternion
import argparse

import rospy
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *

# Parse Args
parser = argparse.ArgumentParser(description='Pose Scorer')
parser.add_argument('mapbag')
parser.add_argument('databag')

args = parser.parse_args()

# Get Data From Bag Files
the_map = get_dict( args.mapbag )['/map']
test_files = get_dict( args.databag )
scan = test_files['/base_scan']
truth = test_files['/base_pose_ground_truth']

pose = truth.pose.pose
true_pos = pose.position.x, pose.position.y, euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
print "True Position:", true_pos

scan2 = LaserScan()
scan2.header = scan.header
scan2.angle_min = scan.angle_min
scan2.angle_max = scan.angle_max
scan2.angle_increment = scan.angle_increment
scan2.range_max = scan.range_max

rospy.init_node('query')
mpub = rospy.Publisher('/map', OccupancyGrid, latch=True, queue_size=10)
mpub.publish(the_map)
pub_true = rospy.Publisher('/base_scan', LaserScan, queue_size=10)
pub_expected = rospy.Publisher('/base_scan_expected', LaserScan, queue_size=10)

tposepub = rospy.Publisher('/truth', PoseStamped, latch=True, queue_size=10)
truth = PoseStamped()
truth.header.frame_id = '/map'
truth.pose = apply(to_pose, true_pos)

posepub = rospy.Publisher('/estimate', PoseStamped, queue_size=10)
estimate = PoseStamped()
estimate.header.frame_id = '/map'

rospy.sleep(1)
tposepub.publish(truth)

br = tf.TransformBroadcaster()

publish_update(pub_true, scan, br, true_pos)

def pose_sub(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    theta = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
    result = to_grid(x,y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)#轉換成網格座標
    if not result: #判斷是否超出地圖外，如果超過則回傳INVALID，反之回傳該點的網格座標
        print "INVALID"
        return
    else:
        mx, my = result#該點網格座標
    
    ex_scan = expected_scan(mx, my, theta, scan.angle_min, scan.angle_increment, len(scan.ranges), scan.range_max, the_map)#主要取得點與障礙物之間的距離，次要取得整條掃描的路徑(如果判斷為沒有障礙物)
    scan2.ranges = ex_scan

    (wx, wy) = to_world(mx, my, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)#將點座標轉換成世界座標
    publish_update(pub_expected, scan2, br, (wx,wy,theta)) #更新資料

    estimate.pose = apply(to_pose, (wx,wy,theta))
    posepub.publish(estimate)
    
    score = scan_similarity(scan.ranges, ex_scan, scan.range_max)#計算相似度
    print "Score: " + str(score)
    

sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_sub)
rospy.spin()