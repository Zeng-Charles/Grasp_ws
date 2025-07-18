#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
import numpy as np

class QuickTest:
    def __init__(self):
        rospy.init_node('quick_test')
        self.initial_joints = None
        self.current_joints = None
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        
    def joint_callback(self, msg):
        if self.initial_joints is None:
            self.initial_joints = np.array(msg.position)
        self.current_joints = np.array(msg.position)
        
    def test_movement(self):
        print("Quick movement test...")
        
        # Wait for initial data
        rospy.sleep(2)
        if self.initial_joints is None:
            print("No joint data received!")
            return
            
        print(f"Initial joints: {self.initial_joints[:5]}...")  # Show first 5
        
        # Start grasp
        try:
            start_grasp = rospy.ServiceProxy('/start_grasp', Empty)
            start_grasp()
            print("Grasp started, waiting for movement...")
        except Exception as e:
            print(f"Failed to start grasp: {e}")
            return
        
        # Monitor for 10 seconds
        for i in range(100):  # 10 seconds at 10Hz
            rospy.sleep(0.1)
            if self.current_joints is not None:
                diff = np.linalg.norm(self.current_joints - self.initial_joints)
                if diff > 0.01:  # Significant movement
                    print(f"✓ Movement detected! Difference: {diff:.6f}")
                    print(f"Changed joints: {self.current_joints[:5]}...")
                    return
                    
        print("✗ No significant movement detected")
        print(f"Final difference: {np.linalg.norm(self.current_joints - self.initial_joints):.6f}")

if __name__ == '__main__':
    try:
        tester = QuickTest()
        tester.test_movement()
    except rospy.ROSInterruptException:
        pass
