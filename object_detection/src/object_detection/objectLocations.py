#!/usr/bin/env python
import rospy
import csv
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import Point 
from object_detection_msgs import ObjectDetectionInfoArray

def lcoations(data):
    tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
    tf2_ros.TransformListener(tf_buffer)
    source_frame = "world_graph_msf"
    target_frame = data.header.frame_id

    # get the transformation from source_frame to target_frame.
    try:
        transformation = tf_buffer.lookup_transform(source_frame, target_frame, data.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s', source_frame, target_frame)

    for detection in data.info:
        object_id = detection.class_id
        x, y, z = detection.position.x, detection.position.y, detection.position.z

        with open('/workspaces/rss_workspace/src/object_detection/object_detection/data/objects.csv', 'a') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([object_id, x, y, z])

        rospy.loginfo(f"Saved object {object_id} position: ({x}, {y}, {z})")
    
def savelocations():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('savelocations', anonymous=True)

    rospy.Subscriber("lcoations", ObjectDetectionInfoArray, lcoations)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    savelocations()
