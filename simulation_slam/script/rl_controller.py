#!/usr/bin/env python

import rospy
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
import torchvision.models as models
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError


class DualCameraResnetModel(nn.Module):
    def __init__(self):
        super(DualCameraResnetModel, self).__init__()
        # Load a pre-trained ResNet-18 model for both cameras
        self.resnet18_cam1 = models.resnet18(pretrained=True)
        self.resnet18_cam2 = models.resnet18(pretrained=True)

        # Remove the final fully connected layer
        num_features = self.resnet18_cam1.fc.in_features
        self.resnet18_cam1.fc = nn.Identity()
        self.resnet18_cam2.fc = nn.Identity()

        # New fully connected layers after concatenation
        self.fc = nn.Linear(num_features * 2, 128)  # Assuming concatenation

    def forward(self, x1, x2):
        # Extract features from both cameras
        features1 = self.resnet18_cam1(x1)
        features2 = self.resnet18_cam2(x2)

        # Concatenate along the feature dimension (ensure that tensor dimensions align)
        combined_features = torch.cat((features1, features2), dim=1)

        # Apply the fully connected layer
        output = self.fc(combined_features)
        return output


# class SingleCameraCnnFcModel(nn.Module):
#     def __init__(self):
#         super(SingleCameraCnnFcModel, self).__init__()
#         # Define the convolutional layers
#         self.conv1 = nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=1)
#         self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1)
#         self.conv3 = nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1)
#         self.conv4 = nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1)
#         self.conv5 = nn.Conv2d(256, 512, kernel_size=3, stride=2, padding=1)
#         self.conv6 = nn.Conv2d(512, 512, kernel_size=5, stride=1, padding=0)
        
#         # Flatten layer to transition to fully connected layers
#         self.flatten = nn.Flatten()
       
#         # Define the fully connected layers
#         self.fc1 = nn.Linear(512 * 2 * 2, 128)
    
#     def forward(self, x):
        
#         # Apply convolutional layers with activation functions
#         x = F.relu(self.conv1(x))
#         x = F.relu(self.conv2(x))
#         x = F.relu(self.conv3(x))
#         x = F.relu(self.conv4(x))
#         x = F.relu(self.conv5(x))
#         x = F.relu(self.conv6(x))
        
#         # Flatten the output for the fully connected layer
#         x = self.flatten(x)
#         # Fully connected layer
#         x = self.fc1(x)
#         return x


# class SingleCameraResnetModel(nn.Module):
#     def __init__(self):
#         super(SingleCameraResnetModel, self).__init__()
#         # Load a pre-trained ResNet-18 model
#         self.resnet18 = models.resnet18(pretrained=True)
#         # Replace the final fully connected layer with custom one
#         num_features = self.resnet18.fc.in_features
#         self.resnet18.fc = nn.Linear(num_features, 128)

#     def forward(self, x):
#         # Pass data through ResNet-18; the modified fc layer outputs 128 features
#         x = self.resnet18(x)
#         return x


class ModelInferenceNode:
    def __init__(self):
        rospy.init_node('rl_controller')
        self.model = DualCameraResnetModel()
        state_dict = torch.load('/home/m4/Documents/weights/CNN19/m4_cnn.pth', map_location=torch.device('cpu'))
        self.model.fc.load_state_dict(state_dict) 
        
        self.model.eval()  # Set the model to evaluation mode

        self.bridge = CvBridge()
        self.image1 = None
        self.image2 = None
        
        self.front_cam_sub = rospy.Subscriber("/realsense/depth/image_rect_raw", Image, self.front_image_callback)
        self.rear_cam_sub = rospy.Subscriber("/realsense2/depth/image_rect_raw", Image, self.rear_image_callback)
        self.pub = rospy.Publisher("/rl/hip_pos", Float64, queue_size=10)

        # Transform for image preprocessing
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize(224),  # Assuming the model expects 72x72 input
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  # Example values
        ])

        rate = rospy.Rate(10) # 10 Hz

        while not rospy.is_shutdown():
            rate.sleep()

    def front_image_callback(self, msg):
        self.image1 = self.process_image(msg)
        self.try_inference()

    def rear_image_callback(self, msg):
        self.image2 = self.process_image(msg)
        self.try_inference()

    def process_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            cv_image[cv_image > 6.0] = 6.0
            cv_image = cv_image / 6.0
            cv_image_3c = cv2.merge([cv_image, cv_image, cv_image])
            # print("Image Shape: ", cv_image_3c.shape)
            rospy.logwarn("Image Shape: ", cv_image_3c.shape)
            # apply transform here
            image_tensor = self.transform(cv_image_3c).unsqueeze(0)
            return image_tensor
        except CvBridgeError as e:
            print(e)
            return None

    def try_inference(self):
        if self.image1 is not None and self.image2 is not None:
            with torch.no_grad():
                output = self.model(self.image1, self.image2)
            # print("Hip control: ", output)
            rospy.logwarn("Hip control: ", output)
            output_msg = Float64(data=output.flatten().tolist())
            self.pub.publish(output_msg)
            self.image1 = None
            self.image2 = None

if __name__ == '__main__':
    node = ModelInferenceNode()
