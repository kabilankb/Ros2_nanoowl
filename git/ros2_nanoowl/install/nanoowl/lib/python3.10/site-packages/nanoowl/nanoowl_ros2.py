import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as im
from nanoowl.owl_predictor import OwlPredictor
from nanoowl.owl_drawing import draw_owl_output

class NanoOWLSubscriber(Node):
    def __init__(self):
        super().__init__('nano_owl_subscriber')
        
        self.declare_parameter('model', 'google/owlvit-base-patch32')
        self.declare_parameter('image_encoder_engine', '../data/owl_image_encoder_patch32.engine')
        self.declare_parameter('thresholds', rclpy.Parameter.Type.DOUBLE)

        self.cv_br = CvBridge()
        self.query = "a person, a box"
        
        # Subscribers
        self.query_subscription = self.create_subscription(
            String, 'input_query', self.query_listener_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, '/rgb', self.listener_callback, 10)

        # Publishers
        self.output_publisher = self.create_publisher(Detection2DArray, 'output_detections', 10)
        self.output_image_publisher = self.create_publisher(Image, 'output_image', 10)

        self.image_encoder_engine = self.get_parameter('image_encoder_engine').get_parameter_value().string_value
        self.predictor = OwlPredictor(
            'google/owlvit-base-patch32',
            image_encoder_engine=self.image_encoder_engine
        )
    
    def query_listener_callback(self, msg):
        self.query = msg.data
    
    def listener_callback(self, data):
        input_query = self.query
        thresholds = self.get_parameter('thresholds').get_parameter_value().double_value

        cv_img = self.cv_br.imgmsg_to_cv2(data, 'rgb8')
        PIL_img = im.fromarray(cv_img)

        text = input_query.strip("][()").split(',')
        self.get_logger().info('Your query: %s' % text)
        thresholds = [thresholds] * len(text)

        text_encodings = self.predictor.encode_text(text)  
        output = self.predictor.predict(
            image=PIL_img, 
            text=text, 
            text_encodings=text_encodings,
            threshold=thresholds,
            pad_square=False
        )

        detections_arr = Detection2DArray()
        detections_arr.header = data.header
        
        for i, box in enumerate(output.boxes):
            box = [float(x) for x in box]
            obj = Detection2D()
            obj.bbox.size_x = abs(box[2] - box[0])
            obj.bbox.size_y = abs(box[1] - box[3])
            obj.bbox.center.position.x = (box[0] + box[2]) / 2.0 
            obj.bbox.center.position.y = (box[1] + box[3]) / 2.0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(output.labels[i]))
            obj.results.append(hyp)
            obj.header = data.header
            detections_arr.detections.append(obj)

        self.output_publisher.publish(detections_arr)

        image = draw_owl_output(PIL_img, output, text=text, draw_text=True)
        image = np.array(image)[:, :, ::-1].copy()
        self.output_image_publisher.publish(self.cv_br.cv2_to_imgmsg(image, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = NanoOWLSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

