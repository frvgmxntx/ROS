#!/usr/bin/env python3

# imports
import rospy
import cv2
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

# config header
MODEL_PATH = 'test1.pt'            # path to pytorch .pt model
ROS_IMAGE_TOPIC = '/quadrotor/ovc3/rgb' # ROS image topic to inference on
RESULT_FILE = 'result.mp4' # result file
RESULT_FPS = 30.0                      # FPS of result file

# ROS node
class YoloROSDetector:
    def __init__(self):
        rospy.loginfo("starting node...")

        self.model = YOLO(MODEL_PATH)
        rospy.loginfo(f"loading model: {MODEL_PATH}")

        # cv_bridge is used to pass the image from the image topic to the model
        self.bridge = CvBridge()
        self.video_out = None
        self.latest_frame = None
        self.frame_lock = threading.Lock() # thread safe access

        # we use a lock to prevent a race condition
        # between the image_callback (writing) and the main loop (reading).
        # without it, the loop might try to run inference on a half-written
        # or corrupted frame, which would cause a crash

        # shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

        # subscriber
        self.image_sub = rospy.Subscriber(ROS_IMAGE_TOPIC, Image, self.image_callback)
        rospy.loginfo(f"subscribed {ROS_IMAGE_TOPIC}")
    
    def image_callback(self, msg):
        """function called everytime a new frame appears on the topic."""
        try:
            # convert ROS image frame (RGB) to OpenCV frame (BGR)
            cv_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # lock frame
            with self.frame_lock:
                self.latest_frame = cv_frame
        
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")

    # OpenCV video writer to save video result file
    def initialize_video_writer(self, frame):
        h, w = frame.shape[:2]
        tamanho_frame = (w, h)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_out = cv2.VideoWriter(RESULT_FILE, fourcc, RESULT_FPS, tamanho_frame)
        rospy.loginfo(f"cv2 video writer started: {RESULT_FILE}")

    # main loop, run inference on each frame
    def run(self):
        rate = rospy.Rate(RESULT_FPS) 
        
        while not rospy.is_shutdown():
            frame_to_process = None
            
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame_to_process = self.latest_frame.copy()

            if frame_to_process is not None:
                if self.video_out is None:
                    self.initialize_video_writer(frame_to_process)

                # yolo prediction
                resultado = self.model.predict(frame_to_process, verbose=False)
                
                # draw box
                frame_com_caixas = resultado[0].plot()

                # save frame
                if self.video_out:
                    self.video_out.write(frame_com_caixas)
             
            rate.sleep()

    # start shutdown after Ctrl-C
    def shutdown_hook(self):
        rospy.loginfo("shutdown hook, saving file....")
        if self.video_out:
            self.video_out.release()
            rospy.loginfo(f"result saved: {RESULT_FILE}")

if __name__ == '__main__':
    try:
        # start ROS node
        rospy.init_node('yolo_detector_node', anonymous=True)
        detector = YoloROSDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
