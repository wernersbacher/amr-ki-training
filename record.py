import rospy
import random
import cv2
import time
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

FPS_SAVE = 0  # set 0 to save every image!

class DataRecorder:

    def __init__(self, folder_name) -> None:
    
        rospy.loginfo("Setting Up the Node...")
        rospy.init_node('amr_ki_training')

        self.folder_name = folder_name
        self.counter = 1

        # --- Create the Subscriber to Twist commands
        self.subscriber_twist = rospy.Subscriber("/cmd_vel", Twist, self.update_twist)
        self.subscriber_img = rospy.Subscriber("/camera/image_raw", Image, self.update_img)

        self.throttle = 0
        self.steering = 0

        self.last_imgage_saved = 0

        self.base_path = f"/var/recorded_data/{self.folder_name}/"
        os.mkdir(self.base_path)

        rospy.loginfo(f"Saving images in {self.base_path}. \n Stop with Strg+C")


    def update_twist(self, msg):

        self.throttle = msg.linear.x
        self.steering = msg.angular.z
        rospy.logdebug(f"Updated current throttle and steering to {self.throttle, self.steering}")

    def update_img(self, msg):

        now = time.time()

        if FPS_SAVE != 0 and now - self.last_imgage_saved < 1/FPS_SAVE:
            rospy.logdebug("Skipping frame.")
            return

        if self.throttle <= 0:  # only capturing frames if driving forwards (throttle > 0)
            return
            
        self.last_imgage_saved = now

        rospy.logdebug(f"Received an image, processing...")

        file_name = f"img_{self.counter}_{self.throttle:.4f}_{self.steering:.4f}_{time.time()}.png"
        path = f"{self.base_path}{file_name}"
        m1 = time.time()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')	
        cv2.imwrite(path, cv_image) 
        self.counter += 1
        m2 = time.time()
        rospy.logdebug(f"Saved new image at {path}")

        rospy.logdebug(f"Saving image took {m2-m1} seconds.")


if __name__ == '__main__':
    try:

        folder_name = input("Type a folder name [random]: ")
        if len(folder_name) < 1:
            folder_name = random.randint(1000, 9999)

        print("Starting Datarecorder...")
        DataRecorder(folder_name)
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo("Goodbye.")