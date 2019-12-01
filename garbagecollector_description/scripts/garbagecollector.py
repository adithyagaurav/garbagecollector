#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float64
"""
Topics To Write on:
type: std_msgs/Float64
/garbagecollector/left_arm_shovel_lid_joint_position_controller/command
/garbagecollector/left_arm_joint_position_controller/command
/garbagecollector/right_arm_shovel_lid_joint_position_controller/command
/garbagecollector/right_arm_joint_position_controller/command
"""

class GarbageCollector(object):

    def __init__(self):
        rospy.loginfo("GarbageCollector Initialising...")
        self.pub_left_lid_position = rospy.Publisher('/garbagecollector/left_arm_shovel_lid_joint_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_left_arm_position = rospy.Publisher('/garbagecollector/left_arm_joint_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_right_lid_position = rospy.Publisher('/garbagecollector/right_arm_shovel_lid_joint_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_right_arm_position = rospy.Publisher('/garbagecollector/right_arm_joint_position_controller/command',
                                                            Float64,
                                                            queue_size=1)

    def init_position(self):
        self.move_garbagecollector_all_joints(  left_lid=0.0,
                                                right_lid=0.0,
                                                left_arm_pos=0.0,
                                                right_arm_pos=0.0)

        
    def wait_publishers_to_be_ready(self):
        
        publishers_ready = False
        
        rate_wait = rospy.Rate(10)
        while not publishers_ready:
            pub_left_lid_num = self.pub_left_lid_position.get_num_connections()
            pub_right_lid_num = self.pub_right_lid_position.get_num_connections()
            pub_left_arm_num = self.pub_left_arm_position.get_num_connections()
            pub_right_arm_num = self.pub_right_arm_position.get_num_connections()
            publishers_ready = (pub_left_lid_num > 0) and (pub_right_lid_num > 0) and (pub_left_arm_num > 0) and (pub_right_arm_num > 0)
            rate_wait.sleep()


    def move_garbagecollector_all_joints(self, left_lid, right_lid, left_arm_pos, right_arm_pos):
        left_lid_msg = Float64()
        left_lid_msg.data = left_lid
        
        right_lid_msg = Float64()
        right_lid_msg.data = right_lid
        
        left_arm_pos_msg = Float64()
        left_arm_pos_msg.data = left_arm_pos
        
        right_arm_pos_msg = Float64()
        right_arm_pos_msg.data = right_arm_pos
        
        # Wait for publishers to be ready
        self.wait_publishers_to_be_ready()
        # Publish Joint Position
        self.pub_left_lid_position.publish(left_lid_msg)
        self.pub_right_lid_position.publish(right_lid_msg)
        self.pub_left_arm_position.publish(left_arm_pos_msg)
        self.pub_right_arm_position.publish(right_arm_pos_msg)
    
    def close_slowly(self):
        
        step_num = 50
        angle = 0.2
        left_lid = -angle
        right_lid = -angle
        delta = angle / step_num
        
        
        for i in range(step_num):
            print "Clossing..."
            self.move_garbagecollector_all_joints(  left_lid=left_lid,
                                                right_lid=left_lid,
                                                left_arm_pos=1.7,
                                                right_arm_pos=1.7)
            left_lid += delta
            right_lid += delta
            
    def lower_slowly(self):
        
        step_num = 50
        angle = 1.7
        left_arm_pos = 0.0
        right_arm_pos = 0.0
        delta = angle / step_num
        
        
        for i in range(step_num):
            print "Lowering..."
            self.move_garbagecollector_all_joints(  left_lid=-0.2,
                                                right_lid=-0.2,
                                                left_arm_pos=left_arm_pos,
                                                right_arm_pos=right_arm_pos)
            left_arm_pos += delta
            right_arm_pos += delta
            
            
    def go_up_slowly(self):
        
        step_num = 50
        angle = 1.7+0.5
        left_arm_pos = 1.7
        right_arm_pos = 1.7
        delta = angle / step_num
        
        
        for i in range(step_num):
            print "Lowering..."
            self.move_garbagecollector_all_joints(  left_lid=0.0,
                                                right_lid=0.0,
                                                left_arm_pos=left_arm_pos,
                                                right_arm_pos=right_arm_pos)
            left_arm_pos -= delta
            right_arm_pos -= delta


    
    def pick(self):
        """
        Generated Picking Movement
        """
        self.lower_slowly()
        self.close_fast()
        self.go_up_slowly()
        self.shake_to_fall()
        self.init_position()
    
    def hight_calibration(self):
        self.move_garbagecollector_all_joints(  left_lid=-0.2,
                                                right_lid=-0.2,
                                                left_arm_pos=1.7,
                                                right_arm_pos=1.7)
                                                
    def close_fast(self):
        self.move_garbagecollector_all_joints(  left_lid=0.0,
                                                right_lid=0.0,
                                                left_arm_pos=1.7,
                                                right_arm_pos=1.7)
                                                
    def go_up(self):
        self.move_garbagecollector_all_joints(  left_lid=0.0,
                                                right_lid=0.0,
                                                left_arm_pos=-0.4,
                                                right_arm_pos=-0.4)
                                                
    def shake_to_fall(self):
        
        for i in range(5):
            self.move_garbagecollector_all_joints(  left_lid=0.0,
                                                    right_lid=0.0,
                                                    left_arm_pos=-0.3,
                                                    right_arm_pos=-0.3)
            
            time.sleep(0.5)                                
            self.move_garbagecollector_all_joints(  left_lid=0.0,
                                                    right_lid=0.0,
                                                    left_arm_pos=-0.5,
                                                    right_arm_pos=-0.5)
        
        
    
if __name__ == "__main__":
    rospy.init_node("GarbageCollector_HightCalibration")
    garbagecollector_object = GarbageCollector()
    garbagecollector_object.pick()
    
    
    