#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
from ultralytics import YOLO
from time import time

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox3DArray
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import PointStamped

# TODO: use scan for depth; subscribe to tracking output when detection is not stable ( obs track -> detection -> lidar_preprocess -> localization -> bev_tracker (obs track) ) 

class Yolo_Dect:
    def __init__(self):

        # load parameters
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param(
            '~image_topic', '/mirte/depth_cam/image_raw')
        depth_topic = rospy.get_param(
            '~depth_topic', '/mirte/depth_cam/depth/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov8/BoundingBoxes2D')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = rospy.get_param('~conf', '0.5')
        self.visualize = rospy.get_param('~visualize', 'True')

        self.method = rospy.get_param('~method', "color_filter")

        self.human_class = rospy.get_param('~human_class', "human")
        self.cow_class = rospy.get_param('~cow_class', "cow")
        self.manure_class = rospy.get_param('~manure_class', "manure")

        self.camera_height = rospy.get_param('~camera_height', "0.1")

        # TODO: camera frame in base_link
        self.base_T_cam = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        self.cam_K = np.array([
            [542.4134521484375, 0.0, 310.2947692871094],
            [0.0, 542.4134521484375, 245.37026977539062],
            [0.0, 0.0, 1.0]
        ])

        # which device will be used
        if (rospy.get_param('/use_cpu', 'false')):
            self.device = 'cpu'
        else:
            self.device = 'cuda'

        self.model = YOLO(weight_path)
        self.model.fuse()

        self.model.conf = conf
        self.color_image = Image()
        self.depth_image = Image()
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)

        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback,
                                          queue_size=1, buff_size=52428800)

        # output publishers
        self.position_pub_2D = rospy.Publisher(
            pub_topic,  BoundingBoxes, queue_size=1)

        self.position_pub_3D = rospy.Publisher(
            '/yolov8/BoundingBoxes3DArray', BoundingBox3DArray, queue_size=1 # TODO: not sure about queue size
        )

        self.image_pub = rospy.Publisher(
            '/yolov8/detection_image',  Image, queue_size=1)

        self.human_position_base_pub = rospy.Publisher('/human_position', PointStamped, queue_size=10)
        self.cow_position_base_pub = rospy.Publisher('/cow_position', PointStamped, queue_size=10)
        self.manure_position_base_pub = rospy.Publisher('/manure_position', PointStamped, queue_size=10)

        # if no image messages
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):

        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        height, width, _ = self.color_image.shape

        new_width = int(width * 0.5)
        new_height = int(height * 0.5)
        self.color_image = cv2.resize(self.color_image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

        results = self.model(self.color_image, show=False, conf=0.3)

        self.dectshow(results, image.height, image.width)

        current_detections = []
        self.calcPositions(results, current_detections, image.height)
        self.detectionsPostProcessing(current_detections)
        self.publishDetections(current_detections)

        cv2.waitKey(3)


    def depth_callback(self, image):
        return

        # self.boundingBoxes = BoundingBoxes()
        # self.boundingBoxes.header = image.header
        # self.boundingBoxes.image_header = image.header
        # self.getImageStatus = True
        # self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
        #     image.height, image.width, -1)

        # self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        # height, width, _ = self.color_image.shape

        # new_width = int(width * 0.5)
        # new_height = int(height * 0.5)
        # self.color_image = cv2.resize(self.color_image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

        # results = self.model(self.color_image, show=False, conf=0.3)

        # self.dectshow(results, image.height, image.width)

        # cv2.waitKey(3)


    def pixelToCamera3D(self, bottom_center):
        uv1 = np.append(bottom_center, 1)
        print("uv1: ", uv1)
        xy1 =  np.linalg.inv(self.cam_K) * uv1
        print("K: ", K)
        print("uv1: ", xy1)
        assert xy1[2] == 1
        Y_cam = self.camera_height
        Z_cam = Y_cam / xy1[1]
        X_cam = xy1[0] * Z_cam
        return np.array([X_cam, Y_cam, Z_cam, 1.0])


    def calcPositions(self, results, current_detections, height):
        for result in results[0].boxes:
            class_label = results[0].names[result.cls.item()]
            # if class_label not in [self.human_class, self.cow_class, self.manure_class]:
            #     return
            # else:
            if(True):
                # x is horizontal (u), y is vertical (v), origin is at left top center
                bottom_left = result.xyxyn[0][0].item() # xmin
                bottom_right = result.xyxyn[0][2].item() # xmax
                bottom_center_u = (bottom_left + bottom_right) / 2.0
                bottom_center_v= result.xyxyn[0][3].item() # ymax
                bottom_center = np.array([bottom_center_u, bottom_center_v])
                print("detection: ")
                # print("x_min: ", result.xyxyn[0][0].item())
                # print("x_max: ", result.xyxyn[0][2].item())
                # print("y_min: ", result.xyxyn[0][1].item())
                # print("y_max: ", result.xyxyn[0][3].item())
                print("u: ", bottom_center_u)
                print("v: ", bottom_center_v)
                if bottom_center_v > 0.5:
                    position_cam = self.pixelToCamera3D(bottom_center)
                    print("position_cam: ", position_cam)
                    position_base = self.base_T_cam * position_cam
                    probs = result.probs
                    if class_label == self.human_class:
                        current_detections.append(["human", probs, position_cam])
                    elif class_label == self.cow_class:
                        current_detections.append(["cow", probs,position_cam])
                    else: # all manure if debuging and not judging class labels before running this line
                        current_detections.append(["manure", probs, position_cam])
        return


    def detectionsPostProcessing(self, current_detections):
        # TODO: same label IOU > threshold --> only keep higher prob; prob < threshold --> discard
        return

    def publishDetections(self, current_detections):
        # TODO: combine them as one topic
        time_stamp = rospy.Time.now()

        for detection in current_detections:
            position_msg = PointStamped()
            position_msg.header.frame_id = "base_link"
            position_msg.header.stamp = time_stamp
            obstable_position = detection[2]
            position_msg.point.x = obstable_position[0]
            position_msg.point.y = obstable_position[1]
            position_msg.point.z = obstable_position[2]
            if detection[0] == "human":
                self.human_position_base_pub.publish(position_msg)
            elif detection[0] == "cow":
                self.cow_position_base_pub.publish(position_msg)
            elif detection[0] == "manure":
                self.manure_position_base_pub.publish(position_msg)
            else: 
                rospy.logerr("unknown detection type")
        return


    def dectshow(self, results, height, width):

        self.frame = results[0].plot()
        print(str(results[0].speed['inference']))
        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        for result in results[0].boxes:
            boundingBox = BoundingBox()
            boundingBox.xmin = np.int64(result.xyxy[0][0].item())
            boundingBox.ymin = np.int64(result.xyxy[0][1].item())
            boundingBox.xmax = np.int64(result.xyxy[0][2].item())
            boundingBox.ymax = np.int64(result.xyxy[0][3].item())
            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()
            self.boundingBoxes.bounding_boxes.append(boundingBox)
        self.position_pub_2D.publish(self.boundingBoxes)
        self.publish_image(self.frame, height, width)

        if self.visualize :
            cv2.imshow('YOLOv8', self.frame)

    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('yolov8_ros', anonymous=False)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":

    main()