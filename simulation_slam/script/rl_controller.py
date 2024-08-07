#!/usr/bin/env python3

import rospy
import torch
import os
import time
import torch.nn as nn
import numpy as np
import torch.nn.functional as F
import torchvision.transforms as transforms
import torchvision.models as models
import cv2
from sensor_msgs.msg import Image
from simulation_world.msg import HipPos
from cv_bridge import CvBridge, CvBridgeError


class DualCameraResnetModel(nn.Module):
    def __init__(self, output_dim):
        super(DualCameraResnetModel, self).__init__()
        
        rospy.logwarn("Test")

        # Load a pre-trained ResNet-18 model for both cameras
        self.resnet18_cam1 = models.resnet18(pretrained=True)
        self.resnet18_cam2 = models.resnet18(pretrained=True)

        # Remove the final fully connected layer
        num_features = self.resnet18_cam1.fc.in_features
        self.resnet18_cam1.fc = nn.Identity()
        self.resnet18_cam2.fc = nn.Identity()

        # New fully connected layers after concatenation
        self.fc = nn.Linear(num_features * 2, 128)  # Assuming concatenation
        self.fc2 = nn.Linear(128, output_dim)
        self.fc_sigma = nn.Linear(128, output_dim)  # Sigma output

    def forward(self, x1, x2):
        # Extract features from both cameras
        features1 = self.resnet18_cam1(x1)
        features2 = self.resnet18_cam2(x2)

        # Concatenate along the feature dimension (ensure that tensor dimensions align)
        combined_features = torch.cat((features1, features2), dim=1)

        # Apply the fully connected layer
        output = self.fc(combined_features)

        sigma = self.fc_sigma(output)  # Predict sigma
        sigma = torch.exp(sigma)  # Ensure sigma is positive

        output = self.fc2(output)

        return output, sigma


class ModelInferenceNode:
    def __init__(self):
        rospy.init_node('rl_controller')

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")

        self.filtered_output = None
        self.filtered_front_command = None
        self.filtered_rear_command = None
        self.alpha = 0.1

        # 2 ACTIONS
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN24/nn/last_m4_cnn_ep_100_rew_-13607.754.pth') # COULD WORK
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN28/nn/last_m4_cnn_ep_60_rew_-43324.34.pth') # DO NOT USE
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN23/nn/last_m4_cnn_ep_130_rew_-8516.387.pth') # DO NOT USE
        
        # 4 ACTIONS
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN19/nn/m4_cnn.pth') # BEST
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN19/nn/last_m4_cnn_ep_315_rew_-4499.462.pth') # COULD WORK
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN19/nn/last_m4_cnn_ep_390_rew_-4534.2935.pth')
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN19/nn/last_m4_cnn_ep_205_rew_-5253.5923.pth')
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN19/nn/last_m4_cnn_ep_125_rew_-5923.9536.pth')
        state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN19/nn/last_m4_cnn_ep_170_rew_-5382.9204.pth') # Very stable under obstacles (with sigma check), would be the BEST if filter added when not crawling
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN19/nn/last_m4_cnn_ep_255_rew_-4889.1235.pth')
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN18/nn/last_m4_cnn_ep_190_rew_-5114.941.pth')
        # state_dict = torch.load('/home/m4/IsaacLab/logs/rl_games/m4_cnn/CNN16/nn/last_m4_cnn_ep_260_rew_-6527.487.pth') # No0t so bad wihout the sigma
        
        state_dict = state_dict['model']
        
        model_dict = {
            'weight': state_dict['a2c_network.actor_mlp.0.weight'],
            'bias': state_dict['a2c_network.actor_mlp.0.bias']
        }

        model_dict2 = {
            'weight': state_dict['a2c_network.mu.weight'],
            'bias': state_dict['a2c_network.mu.bias']
        }

        model_dict3 = {
            'weight': state_dict['a2c_network.value.weight'],
            'bias': state_dict['a2c_network.value.bias']
        }

        self.sigma = {
            'sigma': state_dict['a2c_network.sigma']
        }

        output_dim = state_dict['a2c_network.mu.weight'].shape[0]

        self.model = DualCameraResnetModel(output_dim=output_dim)
        self.model.to(self.device)

        keys_str = str(list(state_dict.keys()))
        rospy.logwarn("State dict keys: " + keys_str)

        rospy.loginfo(f"Shape of model_dict['weight']: {model_dict['weight'].shape}")
        rospy.loginfo(f"Shape of model_dict2['weight']: {model_dict2['weight'].shape}")
        rospy.loginfo(f"Shape of model_dict3['weight']: {model_dict3['weight'].shape}")
        rospy.loginfo(f"Shape of sigma['sigma']: {self.sigma['sigma'].shape}")
        
        self.model.fc.load_state_dict(model_dict)
        self.model.fc2.load_state_dict(model_dict2)
        
        self.model.eval()  # Set the model to evaluation mode

        self.bridge = CvBridge()
        self.image1 = None
        self.image2 = None

        self.hip_pos_msg = HipPos()
        
        self.front_cam_sub = rospy.Subscriber("/realsense/depth/image_rect_raw", Image, self.front_image_callback)
        self.rear_cam_sub = rospy.Subscriber("/realsense2/depth/image_rect_raw", Image, self.rear_image_callback)
        self.pub = rospy.Publisher("/rl/hip_pos", HipPos, queue_size=10)

        # Transform for image preprocessing
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),  # Assuming the model expects 72x72 input
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),  # Example values
        ])

        rate = rospy.Rate(20) # 10 Hz

        while not rospy.is_shutdown():
            self.try_inference()
            rate.sleep()

    def front_image_callback(self, msg):
        self.image1 = self.process_image(msg)

    def rear_image_callback(self, msg):
        self.image2 = self.process_image(msg)

    def process_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            cv_image[cv_image > 6.0] = 6.0
            cv_image[np.isinf(cv_image)] = 6.0
            cv_image[np.isnan(cv_image)] = 6.0

            cv_image = cv_image / 6.0
            cv_image_3c = cv2.merge([cv_image, cv_image, cv_image])

            # apply transform
            image_tensor = self.transform(cv_image_3c).unsqueeze(0).to(self.device)

            return image_tensor
        except CvBridgeError as e:
            rospy.logwarn(e)
            return None

    def try_inference(self):
        if self.image1 is not None and self.image2 is not None:
            with torch.no_grad():
                output, sigma = self.model(self.image1, self.image2)

            # rospy.loginfo(f"Output shape: {output.shape}")
            rospy.loginfo(f"Hip control: {output}")
            rospy.logwarn(f"Standard Deviation (sigma) output: {sigma}")

            # Apply low-pass filter
            if self.filtered_output is None:
                self.filtered_output = output
            else:
                self.filtered_output = self.alpha * output + (1 - self.alpha) * self.filtered_output
                output = self.filtered_output

            # Check if the output tensor has only one element and replicate it
            if output.numel() == 1:

                value = torch.clamp(output[0, 0], min=0.0, max=1.0).item().item()
                self.hip_pos_msg.FL_hip = value
                self.hip_pos_msg.FR_hip = value
                self.hip_pos_msg.RL_hip = value
                self.hip_pos_msg.RR_hip = value
            
            elif output.numel() == 2:

                front_command = torch.clamp(output[0, 0], min=0.0, max=0.7).item()
                rear_command = torch.clamp(output[0, 1], min=0.0, max=0.7).item()
                self.hip_pos_msg.FL_hip = front_command
                self.hip_pos_msg.FR_hip = front_command
                self.hip_pos_msg.RL_hip = rear_command
                self.hip_pos_msg.RR_hip = rear_command


            elif output.numel() == 4:

                max_clamp = 1.0
                front_command = max(torch.clamp(output[0, 0], min=0.0, max=max_clamp).item(), torch.clamp(output[0, 1], min=0.0, max=max_clamp).item())
                rear_command = max(torch.clamp(output[0, 2], min=0.0, max=max_clamp).item(), torch.clamp(output[0, 3], min=0.0, max=max_clamp).item())

                front_command = front_command * 0.7
                rear_command = rear_command * 0.7
                
                max_sigma = 1.0
                if (sigma[0,0] < max_sigma or sigma[0,1] < max_sigma): 
                    self.hip_pos_msg.FL_hip = front_command
                    self.hip_pos_msg.FR_hip = front_command
                
                if (sigma[0,2] < max_sigma or sigma[0,3] < max_sigma):
                    self.hip_pos_msg.RL_hip = rear_command
                    self.hip_pos_msg.RR_hip = rear_command


            rospy.loginfo("Publishing HipPos message: FL_hip={}, FR_hip={}, RL_hip={}, RR_hip={}".format(
            self.hip_pos_msg.FL_hip, self.hip_pos_msg.FR_hip, self.hip_pos_msg.RL_hip, self.hip_pos_msg.RR_hip))

            # Publish the HipPos message
            self.pub.publish(self.hip_pos_msg)
            self.image1 = None
            self.image2 = None


if __name__ == '__main__':
    node = ModelInferenceNode()
