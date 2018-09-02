#!/usr/bin/env python
# ------------------------------------------------------------------------
# Laser assembler 
# Prepared by : Marion
# 2018
# ------------------------------------------------------------------------

## OLD CODE
# import rospy
# from sensor_msgs.msg import PointCloud2
# from laser_assembler.srv import AssembleScans2
# from lidar_3d_scan.msg import StartEnd


# def requestTransform(msg):
#     """
#     Callback that receives a message from motor_control.py with start and end values
#     for the assemble_scans2 request, requests the point cloud and publishes it
#     """
#     try:
#         resp = assemble_scans_srv(msg.start, msg.end)
#         pc_pub.publish(resp.cloud)
#         print "Got cloud with %u points" % len(resp.cloud.data)
#     except rospy.ServiceException, e:
#         print("scan_assembler.py: service call failed: %s" % (e))


# if (__name__ == "__main__"):
#     global assemble_scans_srv
#     global pc_pub

#     rospy.init_node("assembles_scans_to_cloud")

#     cloud_out               = rospy.get_param('~cloud_out')
#     assemble_times_topic    = rospy.get_param('~times_in')
#     rospy.wait_for_service("assemble_scans2")
#     assemble_scans_srv = rospy.ServiceProxy('assemble_scans2', AssembleScans2)

#     pc_pub  = rospy.Publisher(cloud_out, PointCloud2, queue_size=1)
#     assemble_times_sub = rospy.Subscriber(assemble_times_topic, StartEnd, requestTransform)

#     rospy.spin()


# WORKING CODE
import rospy
from sensor_msgs.msg import PointCloud2
from laser_assembler.srv import AssembleScans2

if (__name__ == "__main__"):
    rospy.init_node("assembles_scans_to_cloud")
    cloud_out               = rospy.get_param('~cloud_out')

    rospy.wait_for_service("assemble_scans2")
    assemble_scans_srv = rospy.ServiceProxy('assemble_scans2', AssembleScans2)

    pc_pub  = rospy.Publisher(cloud_out, PointCloud2, queue_size=1)

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        try:
            resp = assemble_scans_srv(rospy.Time(0,0), rospy.get_rostime())
            pc_pub.publish(resp.cloud)
            # print "Got cloud with %u points" % len(resp.cloud.data)
        except rospy.ServiceException, e:
            print("scan_assembler.py: service call failed: %s" % (e))

        r.sleep()

