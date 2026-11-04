#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import numpy as np
import torch
from torch.utils.data import Dataset
from PIL import Image
import cv2
import json
import random
import itertools
from torchvision.transforms import functional
from torchvision.transforms import InterpolationMode
import torch.nn.functional as F
from scipy.ndimage import label
import matplotlib.pyplot as plt

# set seed values (python, numpy, pytorch)
seed_value = 1
random.seed(seed_value)
np.random.seed(seed_value)
torch.manual_seed(seed_value)
torch.cuda.manual_seed(seed_value)

# values for normalization
AXIS_UNIT_CUBE_MAX = 10
EDGINESS_MIN = 0.01
EDGINESS_MAX = 1
ANGLE_MIN = 0
ANGLE_MAX = 89
ORIGIN_SHIFT_MAX = 0


# translate contour pixesl randomly (dx, dy)
def add_noise_to_mask_edges(mask, max_offset=3, scale_factor=4):  
    
    original_shape = mask.shape
    
    # downscale the mask
    if scale_factor > 1:
        small_shape = (original_shape[1] // scale_factor, original_shape[0] // scale_factor)
        small_mask = cv2.resize(mask, small_shape, interpolation=cv2.INTER_NEAREST)
        # scale the offset proportionally
        scaled_offset = max(1, max_offset // scale_factor)
    else:
        small_mask = mask
        scaled_offset = max_offset
    
    # process the smaller mask
    noisy_mask_small = np.zeros_like(small_mask)
    contours, _ = cv2.findContours(small_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        noisy_cnt = []
        for pt in cnt:
            x, y = pt[0]
            dx = np.random.randint(-scaled_offset, scaled_offset + 1)
            dy = np.random.randint(-scaled_offset, scaled_offset + 1)
            noisy_cnt.append([[x + dx, y + dy]])
        noisy_cnt = np.array(noisy_cnt, dtype=np.int32)
        cv2.drawContours(noisy_mask_small, [noisy_cnt], -1, 255, thickness=cv2.FILLED)
    
    # upscale back to original size
    if scale_factor > 1:
        noisy_mask = cv2.resize(noisy_mask_small, (original_shape[1], original_shape[0]), 
                               interpolation=cv2.INTER_NEAREST)
    else:
        noisy_mask = noisy_mask_small
    
    return noisy_mask


# use perturbed contour and fill "new pixels"
def perturb_and_fill_depth_tensor(depth_tensor, mask_threshold=1e-5, max_offset=3):
    
    depth_np = depth_tensor.squeeze(0).detach().clone().cpu().numpy()

    # create mask from depth values (non-zero or valid pixels)
    mask = ((depth_np > mask_threshold) & np.isfinite(depth_np)).astype(np.uint8) * 255

    # perturb mask
    noisy_mask = add_noise_to_mask_edges(mask, max_offset=max_offset)

    # identify new region (expanded mask minus original)
    new_area = cv2.subtract(noisy_mask, mask)

    # normalize depth to 0–255 for inpainting
    depth_min, depth_max = np.nanmin(depth_np), np.nanmax(depth_np)
    depth_norm = cv2.normalize(depth_np, None, 0, 255, cv2.NORM_MINMAX)
    depth_uint8 = np.uint8(depth_norm)

    # inpaint
    inpainted = cv2.inpaint(depth_uint8, new_area, 3, cv2.INPAINT_TELEA)

    # convert back to float32 and rescale
    depth_filled = inpainted.astype(np.float32)
    depth_filled = depth_filled / 255.0 * (depth_max - depth_min) + depth_min

    # merge inpainted region into original depth
    depth_np[new_area > 0] = depth_filled[new_area > 0]
    depth_np = np.where(depth_np > 1, depth_np, 0)

    return torch.from_numpy(depth_np).unsqueeze(0).float()


# Gaussian noise inside tmask region
def denoise_depth_values(depth_tensor, mask_threshold=1e-5):
    
    depth_np = depth_tensor.squeeze(0).detach().clone().cpu().numpy()

    # mask for valid pixels
    mask = ((depth_np > mask_threshold) & np.isfinite(depth_np))
    valid_indices = np.argwhere(mask)

    noise_ratio = np.random.uniform(0.005, 0.8)
    noise_scale = np.random.uniform(0.0001, 0.5) 

    # compute mean and std from valid depth values
    valid_depths = depth_np[mask]
    mean = valid_depths.mean()
    std = valid_depths.std()

    # choose a random subset of valid pixels
    num_noisy = int(len(valid_indices) * noise_ratio)
    chosen_indices = valid_indices[np.random.choice(len(valid_indices), size=num_noisy, replace=False)]

    # apply noise to selected pixels
    noisy_depth = depth_np.copy()
    for y, x in chosen_indices:
        noise = np.random.normal(loc=0.0, scale=std * noise_scale)
        noisy_depth[y, x] += noise

    # clip to avoid negative depth
    noisy_depth = np.clip(noisy_depth, a_min=0.0, a_max=None)

    return torch.from_numpy(noisy_depth).unsqueeze(0).float()


class ExrDataset(Dataset):
    def __init__(self, root_dir, limit_lower, limit_upper, augmentation=False):
        
        self.root_dir = root_dir
        self.limit_lower = limit_lower
        self.limit_upper = limit_upper
        self.augmentation = augmentation
        self.samples = self.collect_valid_samples()
       

    def collect_valid_samples(self):
        valid_samples = []

        # collect all depth images and corresponding GT files 
        for i in range(self.limit_lower, self.limit_upper):
            object_dir = os.path.join(self.root_dir, f'object_{int(i)}')
            for root, _, files in os.walk(object_dir):
                for file in files:
                    if file.endswith('.tiff') and 'depth' in file:
                        depth_path = os.path.join(root, file)
                        gt_path = os.path.join(object_dir, 'object.json')

                        depth_data = self.load_tiff(depth_path)
                        binary_mask = (depth_data != 0) & np.isfinite(depth_data)
                        _, num_features = label(binary_mask)

                        # there should be only one connected component (one object)
                        if num_features == 1:  
                            valid_samples.append((depth_data, gt_path))

        return valid_samples
  

    def __len__(self):
        
        return len(self.samples)


    def __getitem__(self, idx):

        depth_data, gt_path = self.samples[idx]

        ground_truth = self.convert_json_parameters_to_gt_matrix_tblr(gt_path)
        gt_tensor = self.to_tensor(ground_truth)
        depth_tensor = self.to_tensor(depth_data).unsqueeze(0)

        if self.augmentation:
            # this augmentation is nice, but makes things slower 
            depth_tensor_perturbed = perturb_and_fill_depth_tensor(depth_tensor, max_offset=15)
            depth_tensor_perturbed = self.down_scale_image(depth_tensor_perturbed, image_scale=(1/5.024))
            depth_tensor_noisy = denoise_depth_values(depth_tensor_perturbed, mask_threshold=1e-5)

            return depth_tensor_noisy, gt_tensor
        
        else:
            # without augmentation (if things need to be faster)
            depth_tensor = self.down_scale_image(depth_tensor, image_scale=(1/5.024))

            return depth_tensor, gt_tensor
        
    
    def load_tiff(self, tiff_path):
        
        depth_img = Image.open(tiff_path)
        # convert to NumPy array (float32)
        depth_np = np.array(depth_img, dtype=np.float32)
        # convert to PyTorch tensor
        depth_tensor = torch.from_numpy(depth_np)
        
        return depth_tensor


    def to_tensor(self, data):
        return torch.tensor(np.array(data), dtype=torch.float32)
    
    
    def normalize_values(self, values, min_value, max_value):
        # normalize values between 0 and 1
        norm_values = (values - min_value) / (max_value - min_value) 

        return norm_values
        

    def convert_json_parameters_to_gt_matrix_tblr(self, json_path):

        # load parameters from JSON file
        with open(json_path, 'r') as file:
            params = json.load(file)
        
        # initialize the ground truth matrix with zeros
        gt_matrix = np.zeros((1, 30))
            
        center = np.array([0.0, 0.0, 0.0])

        components = sorted(
            zip(
            params["all_translated_locations"],  # (x, y, z) positions
            params["all_edginesses"],            # (n_1, n_2) edginess values
            params["all_rescaled_main_axes"],    # (a_x, a_y, a_z) main axes values
            params["all_rotations"]              # (azimuth, elevation)
            ),
            key=lambda comp: (-comp[0][2], comp[0][1])  # ascending distance to center
        )
        
        # assign values to the gt_matrix for each component
        for i, ((x, y, z), (n_1, n_2), (a_x, a_y, a_z), (azimuth, elevation)) in enumerate(components):
            # base component at index 0, others start from index 10 and increment by 8
            if i == 0:
                idx = 0  # base component
            elif i == 1:
                idx = 10  # first additional component
            elif i == 2:
                idx = 20  # second additional component (new)
            else:
                continue  # ignore extra components if they exist

            # normalize and store position values (x, y, z)
            gt_matrix[0][idx:idx+3] = [
                self.normalize_values(x, -AXIS_UNIT_CUBE_MAX/2, AXIS_UNIT_CUBE_MAX/2),
                self.normalize_values(y, -AXIS_UNIT_CUBE_MAX/2, AXIS_UNIT_CUBE_MAX/2),
                self.normalize_values(z, -AXIS_UNIT_CUBE_MAX/2, AXIS_UNIT_CUBE_MAX/2)
            ]

            # normalize and store edginess and main axes values
            gt_matrix[0][idx+3:idx+8] = [
                self.normalize_values(n_1, EDGINESS_MIN, EDGINESS_MAX),
                self.normalize_values(n_2, EDGINESS_MIN, EDGINESS_MAX),
                a_x / AXIS_UNIT_CUBE_MAX, 
                a_y / AXIS_UNIT_CUBE_MAX, 
                a_z / AXIS_UNIT_CUBE_MAX
            ]

            # normalize and store azimuth and elevation
            gt_matrix[0][idx+8:idx+10] = [
                self.normalize_values(azimuth, ANGLE_MIN, ANGLE_MAX),
                self.normalize_values(elevation, ANGLE_MIN, ANGLE_MAX/2),
            ]
        
        return gt_matrix


    def down_scale_image(self, tensor, image_scale):
        
        _, h, w = tensor.shape
        resize_h = int(h * image_scale)
        resize_w = int(w * image_scale)
        
        return functional.resize(tensor, (resize_h, resize_w), interpolation=InterpolationMode.BILINEAR, antialias=True)
    
    

    
    





