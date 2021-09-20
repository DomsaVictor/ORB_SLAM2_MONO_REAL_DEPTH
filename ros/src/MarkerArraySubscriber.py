#!/usr/bin/env python3

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import rospy
import numpy as np
from probreg import cpd
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import sys

class MarkerArrays:
    def __init__(self, subscribe, load_name, publisher_name, save_name=None):
        self.publisher_name = publisher_name
        self.load_name = load_name
        self.subscribe = subscribe
        self.save_name = save_name
        if self.subscribe:
            self.marker_sub = rospy.Subscriber("/visualization_marker_array", MarkerArray, self.callback, queue_size=1)
        else:
            self.read_from_file()
        self.marker_array_pub = rospy.Publisher(self.publisher_name, MarkerArray, queue_size=1)

        self.marker_np = np.array([])
        self.marker_array = None

    def callback(self, msg):
        
        self.marker_array = msg

        if self.save_name is not None and self.marker_array is not None:
            aux_marker_np = np.array([[0, 0, 0]])
            for marker in msg.markers:
                # print(marker.pose.position.x)
                x = marker.pose.position.x
                y = marker.pose.position.y
                z = marker.pose.position.z
                #print(x)
                new_row = np.array([[x, y, z]])
                aux_marker_np = np.append(aux_marker_np, new_row, axis=0)
            self.marker_np = np.array(aux_marker_np[1:].T)
            self.write_to_file()
            self.marker_sub.unregister()
        elif self.marker_array is not None:
            self.marker_sub.unregister()

    def publish_markers(self):
        if self.marker_array is not None:
            self.marker_array_pub.publish(self.marker_array)

    def write_to_file(self):
        f = open(self.save_name, "w")
        for x, y, z in self.marker_np.T:
            f.write(str(x) + " ")
            f.write(str(y) + " ")
            f.write(str(z) + " ")
            f.write("\n")
        f.close()
        print ("File saved!")

    def read_from_file(self):
        with open(self.load_name, 'r') as fobj:
            all_lines = [[float (num) for num in line.split()] for line in fobj]
        fobj.close()
        
        aux_array = np.array(all_lines, copy=True)
        self.marker_array = MarkerArray()
        for x, y, z in aux_array:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            count_transformed_markers += 1
            marker.id = count_transformed_markers
            marker.pose.orientation.w = 1.0

            marker.scale.x = 1.5
            marker.scale.y = 1.5  
            marker.scale.z = 2

            marker.color.a = 1.0
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z

            self.marker_array.append(marker)
        print("Completed read from file!")



if __name__ == '__main__':

    '''
    Args: 
        1: read location type: 1 - use subscribers; 2 - from file
        2: publisher name
        3: read location name: if reads form subscribers the parameter refers to the topic name, if it reads
            from file than it is the file name
        4: (optional) save file location + name: name of the file (and path) where the data will be saved on the disk. 
        
        Note: save function is only available if reading from subsciber... if reading from file the 4th argument is ignored
    '''

    arg_count = len(sys.argv) - 1
    print(arg_count)
    if arg_count != 3 and arg_count != 4:
        print("ERROR! WRONG NUMBER OF ARGUMENTS")
        exit()
    read_location  = sys.argv[1]
    if read_location == "1":
        subscribe = True
        print("Reading from subscriber: ")
    elif read_location == "2":
        subscribe = False
        print("Reading from file: ")
    else:
        print("WRONG LOAD TYPE. PROGRAM WILL EXIT!")

    load_name       = sys.argv[3]
    print(load_name)

    publisher_name  = sys.argv[2]
    print("Publishing on: " + publisher_name)

    if arg_count == 4:
        save_name   = sys.argv[4]
        print("Markers will be saved at: " + save_name)
    else:
        save_name   = None

    rospy.init_node("MarkerArraySub", anonymous=True)
    r = rospy.Rate(1)

    marker_arrays = MarkerArrays(subscribe=subscribe, load_name=load_name, publisher_name=publisher_name, save_name=save_name)
    print("~~~ Publishing markers! ~~~")
    while not rospy.is_shutdown():
        marker_arrays.publish_markers()
        r.sleep()