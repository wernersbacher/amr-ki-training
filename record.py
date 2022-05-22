import rospy
import cv2
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

FPS_SAVE = 2

class DataRecorder:

    def __init__(self) -> None:
    
        rospy.loginfo("Setting Up the Node...")
        rospy.init_node('amr_ki_training')

        # --- Create the Subscriber to Twist commands
        self.subscriber_twist = rospy.Subscriber("/cmd_vel", Twist, self.update_twist)
        self.subscriber_img = rospy.Subscriber("/camera/image_raw", Image, self.update_img)

        self.throttle = 0
        self.steering = 0

        self.last_imgage_saved = 0


    def update_twist(self, msg):

        self.throttle = msg.linear.x
        self.steering = msg.angular.z
        rospy.loginfo(f"Updated current throttle and steering to {self.throttle, self.steering}")

    def update_img(self, msg):

        now = time.time()

        if now - self.last_imgage_saved < 1/FPS_SAVE:
            return
            
        self.last_imgage_saved = now

        rospy.loginfo(f"Received an image, processing...")
        file_name = f"img_{self.throttle:.4f}_{self.steering:.4f}_{time.time()}.png"

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')	
        cv2.imwrite('/var/recorded_data/'+file_name, cv_image) 
        rospy.loginfo(f"Saved new image at {'/var/recorded_data/'+file_name}")


if __name__ == '__main__':
    try:
        print("Starting Datarecorder...")
        DataRecorder()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)