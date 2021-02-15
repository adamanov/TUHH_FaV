#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('controller')
import sys
import rospy
import cv2
import numpy as np
import time
import math as m

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
#roslib.load_manifest('my_package')


class person_detection:

    def __init__(self, name):
        rospy.init_node(name)
        rospy.set_param('prediction_rate',20.0)
        self.prediction_rate = rospy.get_param('prediction_rate')

        # global Variables
        self.img_frame = None    
        self.header = None
        self.img = None
        self.missionStatement = False
        # Parameters
        self.frequency = 100.0  # hz 
        
        # Load CvBridge
        self.bridge = CvBridge()
        # Load Yolo 
        # Check it in lab !!! where you are located a weigths and cfg files
        self.folder_name = rospy.get_param("weight_folder")

        modelWeights = self.folder_name + "/yolov3.weights"
        modelConfiguration = self.folder_name +"/yolov3.cfg"
        print(modelWeights, "/n")

        self.net = cv2.dnn.readNet(modelWeights,modelConfiguration )
        self.classes = []
        with open(self.folder_name + "/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3)) 

        #publisher
        #self.image_pub = rospy.Publisher("image_topic_2", Image)

        #subscriber
        self.image_sub = rospy.Subscriber("front_camera/image_raw/compressed", CompressedImage, self.bridge_callback)

        self.publish_Detected = rospy.Publisher("personDetection/statement",Bool,queue_size=1)
        # self.timer = rospy.Timer(rospy.Duration(1.0/self.frequency), self.timer_callback) 

    def bridge_callback(self, data):
        try:
            self.img_frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8") #encode image as bgr8
        except CvBridgeError as e:
            print(e)

        #print("I got all data from rosbag file")
        self.detect_person()


        #(rows,cols,channels) = cv_image.shape
        # if cols > 60 and rows > 60 :
        #   cv2.circle(cv_image, (50,50), 10, 255)

        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)

        #try:
        #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_frame, "bgr8"))
        #except CvBridgeError as e:
        #  print(e)

    # def timer_callback(self, timer):
        #print("timer")
        # if img_frame is None or boxes_mid_pub is None:
        # return
        #boxes, confidences, classids = yolo.evaluate(img_frame)
        # self.detect_person()
        
    def detect_person(self):
        # resizing img
        
        # self.img_frame = cv2.imread("dnn/Face.jpg")

        if self.img_frame is None:
            sys.exit("could not load the image")
        #else:
            #print("I loaded")
        
        # cv2.imshow('img',self.img_frame)
        # cv.wait(100)
        img = cv2.resize(self.img_frame, None, fx=0.4, fy=0.4)
        
        height, width, channels = img.shape
        image_center = [width/2, height/2]
        #print("img resized")

        # Detecting objects
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)


        # Showing informations on the screen
        class_ids = []
        confidences = []
        boxes = []
        center = []
        persons_center = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    # cv2.circle(img, (center_x, center_y), 60, (0, 255, 0), 2)
                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    
                    center.append([center_x, center_y])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        font = cv2.FONT_HERSHEY_PLAIN
        p = 0
        for i in range(len(boxes)):
            if i in indexes:
                
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                
                print(label, i, center[i])
                color = self.colors[class_ids[i]]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
                if class_ids[i] == 0: 
                    persons_center.append(center[i])
                    p = p + 1
                    # print('Amount of persons seen:', p)
                    # print('Persons center:', persons_center)
                    # print('image center:', image_center)     

                    if p>0:
                        # I detected a person!!! 
                        self.missionStatement = True
        # cv2.imshow('img',img)
        # cv2.waitKey(1)


        # if cv2.waitKey(10) & 0xFF == ord('q'):
        #    break

    def publishIfDetected (self):
        msg = Bool()
        msg = self.missionStatement
        str = "Msg before publishing: ", msg, "  " 
        rospy.loginfo_once(str)
        self.publish_Detected.publish(msg)
        
    def run(self):
        rate = rospy.Rate(self.prediction_rate)
        while not rospy.is_shutdown():
            self.publishIfDetected()
            rate.sleep()

    #def run(self):
     #   rate = rospy.Rate(50.0)
    #    while not rospy.is_shutdown():
     #       rate.sleep()

#def main(args):
    #c = Person_Detection()
    #rospy.init_node('Person_Detection', anonymous=true)
    #while not rospy.is_shutdown():
     #   rospy.spin()
    #try:
       # rospy.spin()
    #except KeyboardInterrupt:
    #    print("Shutting down")
    #    cv2.destroyAllWindows()


#if __name__ == "__main__":
    #node = Person_Detection("Person_Detection")
    #node.run()
   # main(sys.argv)

def main():
    ic = person_detection('person_detection')
    ic.run()
    
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
   main()
