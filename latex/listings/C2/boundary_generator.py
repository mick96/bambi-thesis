#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('bambi_msgs')
import rospy
from bambi_msgs.msg import OrthoPhoto, Field, GeoPosition2D
import sys
# parse kml files
from pykml import parser
class BoundaryGeneratorNode():

    def __init__(self):
        self.m_boundaryPublisher = rospy.Publisher("~boundary", Field, queue_size=5)
        rospy.Subscriber('/bambi/mission_controller/trigger_boundary', OrthoPhoto, self.cb_boundary_trigger)
        rospy.spin()
        
    def cb_boundary_trigger(self, OrthoPhoto):
        field = Field();
        rospy.loginfo("Trying to read file %s", OrthoPhoto.filenameWithFullPath)
        with open(OrthoPhoto.filenameWithFullPath) as f:
            root = parser.parse(f).getroot()
            coordinates = root.Document.Placemark.LineString.coordinates.text.split()
            for c in coordinates:
                splitted = c.split(',')
                lon = splitted[0]
                lat = splitted[1]
                pos = GeoPosition2D();
                pos.latitude = float(lat)
                pos.longitude = float(lon)
                field.boundary_path.append(pos)
        rospy.loginfo("Publishing field with %d coordinates", len(field.boundary_path))
        self.m_boundaryPublisher.publish(field);
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('boundary_generator')
    rospy.loginfo("BoundaryGenerator STARTUP")
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        boundaryGeneratorNode = BoundaryGeneratorNode()
    except rospy.ROSInterruptException: pass