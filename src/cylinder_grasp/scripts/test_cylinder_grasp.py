#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Bool
import time

class GraspTester:
    """Simple grasp tester"""
    
    def __init__(self):
        rospy.init_node('grasp_tester', anonymous=True)
        self.grasp_active = False
        
        # Subscribe to grasp status
        rospy.Subscriber('/grasp_status', Bool, self.status_callback)
        
        rospy.loginfo("Grasp Tester ready")
    
    def status_callback(self, msg):
        if msg.data != self.grasp_active:
            self.grasp_active = msg.data
            print(f"Grasp: {'ACTIVE' if msg.data else 'INACTIVE'}")
    
    def call_service(self, service_name):
        """Call a service and return success"""
        try:
            rospy.wait_for_service(service_name, timeout=3.0)
            service = rospy.ServiceProxy(service_name, Empty)
            service()
            print(f"✓ {service_name}")
            return True
        except:
            print(f"✗ {service_name} failed")
            return False
    
    def wait_for_completion(self, timeout=30):
        """Wait for grasp to complete"""
        print("Waiting for grasp completion...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if not self.grasp_active:
                print("✓ Grasp completed!")
                return True
            time.sleep(0.5)
        
        print("✗ Grasp timed out")
        return False
    
    def run_interactive(self):
        """Interactive mode"""
        print("\n=== Grasp Tester ===")
        print("1 - Start viewer")
        print("2 - Start grasp") 
        print("3 - Stop grasp")
        print("4 - Reset")
        print("5 - Auto test")
        print("q - Quit")
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\nCommand: ").strip()
                
                if cmd == '1':
                    self.call_service('/start_viewer')
                elif cmd == '2':
                    self.call_service('/start_grasp')
                elif cmd == '3':
                    self.call_service('/stop_grasp')
                elif cmd == '4':
                    self.call_service('/reset_grasp')
                elif cmd == '5':
                    self.run_auto_test()
                elif cmd == 'q':
                    break
                else:
                    print("Invalid command")
                    
            except KeyboardInterrupt:
                break
    
    def run_auto_test(self):
        """Automatic test sequence"""
        print("\n=== Auto Test ===")
        
        # Test sequence
        steps = [
            ("Reset system", lambda: self.call_service('/reset_grasp')),
            ("Start viewer", lambda: self.call_service('/start_viewer')),
            ("Start grasp", lambda: self.call_service('/start_grasp')),
            ("Wait for completion", lambda: self.wait_for_completion(30))
        ]
        
        for step_name, step_func in steps:
            print(f"\n{step_name}...")
            if not step_func():
                print(f"✗ Test failed at: {step_name}")
                return
            time.sleep(1.0)
        
        print("✓ Auto test completed successfully!")


if __name__ == '__main__':
    try:
        tester = GraspTester()
        
        import sys
        if len(sys.argv) > 1 and sys.argv[1] == 'auto':
            tester.run_auto_test()
        else:
            tester.run_interactive()
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
