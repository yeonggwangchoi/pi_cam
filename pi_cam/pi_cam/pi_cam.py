import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Pi_Cam_Publisher(Node):   
    def __init__(self):      
        super().__init__('pi_cam_publisher')      
        self.publisher = self.create_publisher(Image, 'pi_cam_topic', 10)      
        timer_period = 1     
        self.timer = self.create_timer(timer_period, self.timer_callback)      
        self.cap = cv2.VideoCapture(0)      
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)      
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)      
        self.br = CvBridge()   

    def timer_callback(self):      
        ret, frame = self.cap.read()       
        if ret:         
            self.publisher.publish(self.br.cv2_to_imgmsg(frame))         
            cv2.imshow('frame', frame)         
            cv2.waitKey(1)      
        else:          
            print("cam..")

def main(args=None):   
    rclpy.init(args=args)   
    pi_cam_publisher = Pi_Cam_Publisher()   
    rclpy.spin(pi_cam_publisher)   
    pi_cam_publisher.destroy_node()   
    rclpy.shutdown()
    
if __name__ == '__main__':   
    main()