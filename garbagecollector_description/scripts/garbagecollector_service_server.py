#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
from garbagecollector import GarbageCollector



class GarbageCollectorService(object):
    def __init__(self):
        rospy.init_node('garbagecollector_pick_service_server') 
        my_service = rospy.Service('/garbagecollector_pick', Empty , self.my_callback) # create the Service called my_service with the defined callback
        self.garbagecollector_object = GarbageCollector()
        self.garbagecollector_object.init_position()
        rospy.spin() # mantain the service open.
        
    def my_callback(self,request):
        print "My_callback has been called"
        self.garbagecollector_object.pick()
        return EmptyResponse()
        

if __name__ == "__main__":
    garbagecollector_service_object = GarbageCollectorService()
