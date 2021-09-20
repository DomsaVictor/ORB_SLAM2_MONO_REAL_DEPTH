#!/usr/bin/env python3

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from std_msgs.msg import Header
import rospy
import numpy as np
from sklearn.neighbors import NearestNeighbors
from matplotlib.cm import get_cmap
import pcl
import copy
from probreg import cpd


class MapsSubscriber:
    def __init__(self):
        ''' initialize ros '''
        self.counter = 0
        self.np_cloud1 = np.array([[0, 0, 0]])
        self.np_cloud2 = np.array([[0, 0, 0]])
        self.cloud1_sub = rospy.Subscriber("/orb_point_cloud_1", PointCloud2, self.callback, queue_size=1, callback_args=1)
        self.cloud2_sub = rospy.Subscriber("/orb_point_cloud_2", PointCloud2, self.callback, queue_size=1, callback_args=2)
        self.loaded_cloud1 = False
        self.loaded_cloud2 = False

    def callback(self, msg, args):
        '''
        Both subscribers use the same callback function.
        Args is used to specify which subscriber calls this function (can be either 1 or 2).
        As the cloud points do not change (are read from files by a C++ program and published in ROS) we only want to 
            read them once -> class parameters 'loaded_cloud1/2'
        After the map is read the respective subscriber unsubscribes.
        NOT OPTIMAL BUT I DON'T CARE.
        '''
        if args == 1 and self.loaded_cloud1 == False:
            np_cloud1 = np.array([[0, 0, 0]])
            gen = pc2.read_points(msg, skip_nans=True)
            int_data = list(gen)
            for x in int_data:
                np_cloud1 = np.append(np_cloud1, [[x[0],x[1],x[2]]], axis = 0)
            
            self.np_cloud1 = np_cloud1
            self.loaded_cloud1 = True
            self.cloud1_sub.unregister()

            print("DONE CLOUD 1 WITH: " + str(np_cloud1.shape) + " POINTS")
        
        elif args == 2 and self.loaded_cloud2 == False:
            np_cloud2 = np.array([[0, 0, 0]])
            gen = pc2.read_points(msg, skip_nans=True)
            int_data = list(gen)
            for x in int_data:
                np_cloud2 = np.append(np_cloud2, [[x[0],x[1],x[2]]], axis = 0)
            
            self.np_cloud2 = np_cloud2
            self.loaded_cloud2 = True
            self.cloud2_sub.unregister()

            print("DONE CLOUD 2 WITH: " + str(np_cloud2.shape) + " POINTS")


class PointCloudRegistrator:
    def __init__(self, point_cloud1, point_cloud2):
        self.source = np.array(point_cloud1, copy=True)
        self.target = np.array(point_cloud2, copy=True)
        self.result = np.array([])


    def cpd(self, th=None):
        print("Starting CPD algorithm.")
        if th is not None:
            th = np.deg2rad(30.0)
            c, s = np.cos(th), np.sin(th)
            # example Rotation matrix, might be useful if we know initial pose of the clouds
            self.target.transform(np.array([ [  c,  -s, 0.0, 0.0],
                                        [  s,   c, 0.0, 0.0],
                                        [0.0, 0.0, 1.0, 0.0],
                                        [0.0, 0.0, 0.0, 1.0]]))

        tf_param, _, _ = cpd.registration_cpd(self.source, self.target)
        self.result = copy.deepcopy(self.source)
        self.result = tf_param.transform(self.result)
        print("Transformation succesful.")
        return tf_param

def point_cloud_from_np(points, parent_frame):
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgb')]

    header = Header(frame_id=parent_frame, stamp=rospy.Time.now())
    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data
    )


if __name__ == '__main__':
    print("Initializing 'map_merger' node.")
    rospy.init_node('map_merger')
    
    print("Creating Maps Subscribers.")
    maps_subscriber = MapsSubscriber()
    
    print("Loading Cloud Points from publishers.")
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        # let 'ROS run' until both maps are loaded
        if maps_subscriber.loaded_cloud1 and maps_subscriber.loaded_cloud2:
            point_cloud_registrator = PointCloudRegistrator(maps_subscriber.np_cloud1, maps_subscriber.np_cloud2)
            # after the maps are loaded and PCR is created we no longer care about ROS (until the new merged map is ready to be published)
            break
        r.sleep()

    # get the transform that needs to be used to also transform the MarkerArrays from RVIZ !! not yet implemented !!
    tf_param = point_cloud_registrator.cpd()
    result = np.array(point_cloud_registrator.result)

    print(result.shape)
    
    # create a publisher for the new map
    pub_points1 = rospy.Publisher('/merged_map1', PointCloud2, queue_size=1)    
    
    # put colors for the point cloud - TESTING - DO NOT DELETE AS THE CODE WILL CRUSH
    colors1 = np.array(get_cmap('cubehelix')(
        np.cos(2 * np.pi / 10 * np.arange(result.shape[0])) / 2 + 0.5))
    
    result = np.hstack((result[:, 0:3], colors1))

    print("DONE. PUBLISHING")
    
    while not rospy.is_shutdown():
        # publish the points. the point_cloud_from_np functions need to be called every time to update the time stamp in header
        # not sure if necessary but rn does not hurt...
        pub_points1.publish(point_cloud_from_np(result, '/map'))
        r.sleep()
