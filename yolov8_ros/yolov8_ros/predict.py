import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from ultralytics import YOLO

HALF_WIDTH, HALF_HEIGHT = 640, 480

class Yolo_Predict_Node(Node):   
    def __init__(self):      
        super().__init__('yolo_predict_node')    
        self.yolo = YOLO('yolov8n.pt')
        self.cv_bridge = CvBridge()   
        self.sub = self.create_subscription(Image, 'pi_cam_topic', self.image_callback, 10)
        self.publisher = self.create_publisher(Int16, 'angle_topic', 10)  
        self.angle = 0
        self.msg = Int16()

    def image_callback(self, msg: Image) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        results = self.yolo(cv_image)
        results = results[0].cpu()
        
        for boxes in results.boxes.xywh:
            x, y = int(boxes[0]), int(boxes[1])
            print('center_x, center_y : {}, {}'.format(x, y))

            if abs(x - HALF_WIDTH) < 10:
                self.angle = 0
            else:
                self.angle = self.map_range((x - HALF_WIDTH))
        self.msg.data = int(self.angle)
        self.publisher.publish(self.msg)

        
    def map_range(self, value):
        normalized = (value + 320) / 640
        
        mapped_value = normalized * 10 - 10
        
        return mapped_value

def main(args=None):   
    rclpy.init(args=args)   
    yolo_predict_node = Yolo_Predict_Node()   
    rclpy.spin(yolo_predict_node)   
    yolo_predict_node.destroy_node()   
    rclpy.shutdown()
    
if __name__ == '__main__':   
    main()