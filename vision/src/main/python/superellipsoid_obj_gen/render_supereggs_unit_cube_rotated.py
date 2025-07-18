#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# I have blender-2.90.1 installed, not sure if this works with other blender versions out of the box

import math
import os
import sys
import bpy
import numpy as np
import json
import bmesh
import mathutils  
import random
import argparse
from PIL import Image
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
from math import radians

random.seed(5)
np.random.seed(5)

START_IDX = 0
NUMBER_OF_SAMPLES = 1
MAX_SIDELENGTH = 3
MIN_SIDELENGTH = 1
EDGINESS_MIN = 0.01
EDGINESS_MAX = 1
ANGLE_MIN = 0
ANGLE_MAX = 89
FORMAT = 'OPEN_EXR'  # 'PNG'
RESOLUTION = 256
UNIT_CUBE_SIDELENGTH = 10

# side components oriented bounding boxes (obb) should overlap with base component obb
def test_obb_overlap(center1, axes1, half_sizes1,
                     center2, axes2, half_sizes2, epsilon=1e-6):
    
    R = [[axes1[i].dot(axes2[j]) for j in range(3)] for i in range(3)]
    AbsR = [[abs(R[i][j]) + epsilon for j in range(3)] for i in range(3)]

    # compute translation vector
    t = center2 - center1
    # bring translation into box1's coordinate frame
    t = Vector([t.dot(axes1[i]) for i in range(3)])

    # test axes L = A0, A1, A2
    for i in range(3):
        ra = half_sizes1[i]
        rb = sum(half_sizes2[j] * AbsR[i][j] for j in range(3))
        if abs(t[i]) > ra + rb:
            return False  # Separation found

    # test axes L = B0, B1, B2
    for i in range(3):
        ra = sum(half_sizes1[j] * AbsR[j][i] for j in range(3))
        rb = half_sizes2[i]
        if abs(sum(t[j] * R[j][i] for j in range(3))) > ra + rb:
            return False

    # test cross products of axes: L = Ai x Bj
    for i in range(3):
        for j in range(3):
            ra = (half_sizes1[(i + 1) % 3] * AbsR[(i + 2) % 3][j] +
                  half_sizes1[(i + 2) % 3] * AbsR[(i + 1) % 3][j])
            rb = (half_sizes2[(j + 1) % 3] * AbsR[i][(j + 2) % 3] +
                  half_sizes2[(j + 2) % 3] * AbsR[i][(j + 1) % 3])
            t_ = (t[(i + 2) % 3] * R[(i + 1) % 3][j] -
                  t[(i + 1) % 3] * R[(i + 2) % 3][j])
            if abs(t_) > ra + rb:
                return False

    # No separating axis found, boxes must be intersecting
    return True


def get_obb_data_from_superellipsoid(se_location, a_x, a_y, a_z, rot_x_deg, rot_z_deg):
    # Euler rotation angles
    rot_x = radians(-rot_x_deg)
    rot_z = radians(-rot_z_deg)
    
    # compose rotation: first around X, then Z
    rot_matrix = (
        Matrix.Rotation(rot_z, 4, 'Z') @
        Matrix.Rotation(rot_x, 4, 'X')
    ).to_3x3()
    
    # get rotated local axes
    axes = [rot_matrix @ Vector((1, 0, 0)),
            rot_matrix @ Vector((0, 1, 0)),
            rot_matrix @ Vector((0, 0, 1))]

    # center and half extents
    center = Vector(se_location)
    half_extents = Vector((a_x, a_y, a_z))

    return center, axes, half_extents

# Euclidean distance between two points in 3D space
def euclidean_distance(p1, p2):

    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

# distance between two side components should be less than the minimum distance
def is_not_too_close(coord1, coord2, min_distance):

    distance = euclidean_distance(coord1, coord2)

    return distance > min_distance

# function to create a superegg at location (0, 0, 0)
def create_superellipsoid(superegg_name, se_location, n_1, n_2, a_x, a_y, a_z, rot_x, rot_z):
    
    mesh = bpy.data.meshes.new(name=superegg_name)
    obj = bpy.data.objects.new(superegg_name, mesh)
    # link the object to the scene
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    
    obj.select_set(True)

    bm = bmesh.new()
    resolution = RESOLUTION
    
    for u in range(resolution + 1):
        for v in range(resolution + 1):
            theta = (u / resolution) * 2 * math.pi
            phi = (v / resolution) * math.pi
    
            x = a_x * math.copysign(1, math.cos(phi)) * (abs(math.cos(phi)) ** n_1) * math.copysign(1, math.cos(theta)) * (abs(math.cos(theta)) ** n_2)
            y = a_y * math.copysign(1, math.cos(phi)) * (abs(math.cos(phi)) ** n_1) * math.copysign(1, math.sin(theta)) * (abs(math.sin(theta)) ** n_2)
            z = a_z * math.copysign(1, math.sin(phi)) * (abs(math.sin(phi)) ** n_1)
    
            bm.verts.new((x, y, z))

    # create faces for the upper half
    bm.verts.ensure_lookup_table()
    for u in range(resolution):
        for v in range(resolution):
            v1 = bm.verts[(u * (resolution + 1)) + v]
            v2 = bm.verts[(u * (resolution + 1)) + v + 1]
            v3 = bm.verts[((u + 1) * (resolution + 1)) + v + 1]
            v4 = bm.verts[((u + 1) * (resolution + 1)) + v]
            bm.faces.new((v1, v2, v3, v4))
            
    # mirror the upper half to create the lower half
    bmesh.ops.mirror(bm, geom=bm.verts[:], axis='Z')
    bm.to_mesh(mesh)
    
    # select the object you want to mirror
    obj = bpy.context.active_object
    
    # create a new object that is a mirrored copy of the original
    mirror_obj = obj.copy()
    mirror_obj.data = obj.data.copy()
    bpy.context.collection.objects.link(mirror_obj)
    
    # mirror the new object along the X-axis
    mirror_obj.location.z = -obj.location.z
    mirror_obj.scale.z = -obj.scale.z
    
    bpy.ops.object.select_all(action='DESELECT')  # deselect all objects first
    obj.select_set(True)
    mirror_obj.select_set(True)
    bpy.ops.object.join()
    bpy.ops.object.make_single_user(object=True, obdata=True)  # ensure mesh data is unique
    
    superegg = bpy.context.object
    superegg.name = superegg_name
    superegg.location = se_location
    # rot_x = -azimuth
    # rot_z = -elevation
    superegg.rotation_euler = (math.radians(- rot_x), 0, math.radians(- rot_z))
    
    return superegg

# function to convert spherical coordinates to cartesian coordinates
def convert_angles_to_coordinates(phi, theta, radius):
    phi = phi * math.pi / 180.0
    theta = theta * math.pi / 180.0

    x = radius * math.sin(theta) * math.cos(phi)
    y = radius * math.sin(theta) * math.sin(phi)
    z = radius * math.cos(theta)

    return x, y, z


def create_directory_if_not_exists(directory_path):
    # check if the directory exists
    if not os.path.exists(directory_path):
        # create the directory if it does not exist
        os.makedirs(directory_path)
        

def get_geometry_centroid(component_names):
    total_x, total_y, total_z = 0.0, 0.0, 0.0
    total_vertices = 0

    for obj_name in component_names:
        obj = bpy.data.objects.get(obj_name)
        if obj:  # only process mesh objects
            mesh = obj.data  # get mesh data
            
            # iterate over all vertices in the object's mesh
            for vert in mesh.vertices:
                world_vert = obj.matrix_world @ vert.co  # convert to world coordinates
                
                total_x += world_vert.x
                total_y += world_vert.y
                total_z += world_vert.z
                total_vertices += 1
        else:
            print(f"Object {obj_name} not found or is not a mesh.")

    if total_vertices == 0:
        print("No valid mesh objects found.")
        return None

    # compute the centroid as the mean position of all vertices
    centroid = (total_x / total_vertices, total_y / total_vertices, total_z / total_vertices)
    
    return centroid


def get_bounding_box(component_names):
    bpy.context.view_layer.update()  # ensure transformations are updated

    # initialize extreme values for the bounding box
    min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
    max_x, max_y, max_z = -float('inf'), -float('inf'), -float('inf')

    depsgraph = bpy.context.evaluated_depsgraph_get()

    for obj_name in component_names:
        obj = bpy.data.objects.get(obj_name)
        if obj:
            obj_eval = obj.evaluated_get(depsgraph)  # ensure transformations are correct
            
            # get the world matrix
            world_matrix = obj_eval.matrix_world
            
            # ensure object has geometry data
            if obj_eval.type == 'MESH':
                mesh = obj_eval.to_mesh()
                
                # iterate over all vertices (this ensures a tighter bounding box)
                for vertex in mesh.vertices:
                    world_vertex = world_matrix @ vertex.co  # transform to world space

                    min_x, min_y, min_z = min(min_x, world_vertex.x), min(min_y, world_vertex.y), min(min_z, world_vertex.z)
                    max_x, max_y, max_z = max(max_x, world_vertex.x), max(max_y, world_vertex.y), max(max_z, world_vertex.z)

                obj_eval.to_mesh_clear()  # cleanup to avoid memory issues

            else:
                print(f"Skipping {obj_name}: Not a mesh object.")

        else:
            print(f"Object {obj_name} not found in the scene.")

    bbox_size = (max_x - min_x, max_y - min_y, max_z - min_z)
    bbox_center = ((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2)

    return {
        "min": (min_x, min_y, min_z),
        "max": (max_x, max_y, max_z),
        "size": bbox_size,
        "center": bbox_center
    }


def scale_object_to_bbox(empty, target_bbox, bbox_info):
    
    if not bbox_info:
        print("Bounding box could not be computed.")
        return

    actual_size = bbox_info["size"]  # (width, height, depth)
    
    # compute scaling factor per axis
    scale_x = target_bbox[0] / actual_size[0]
    scale_y = target_bbox[1] / actual_size[1]
    scale_z = target_bbox[2] / actual_size[2]
    
    # apply uniform or non-uniform scaling
    scale_factor = min(scale_x, scale_y, scale_z)  # use uniform scale if necessary
    empty.scale = (scale_factor, scale_factor, scale_factor)

    return scale_factor


def select_shape_parameters(ax_scale, ay_scale, az_scale, n1_ref, n2_ref, vol_A, size_type):
    n1 = np.clip(n1_ref + np.random.normal(0, 0.2), 0.01, 1)
    n2 = np.clip(n2_ref + np.random.normal(0, 0.2), 0.01, 1)

    axis_ref = np.random.uniform(MIN_SIDELENGTH, MAX_SIDELENGTH)

    # apply proportions with some noise
    ax = axis_ref * (ax_scale + np.random.normal(0, 0.2))
    ay = axis_ref * (ay_scale + np.random.normal(0, 0.2))
    az = axis_ref * (az_scale + np.random.normal(0, 0.2))

    # clamp to minimum size
    ax = max(ax, MIN_SIDELENGTH)
    ay = max(ay, MIN_SIDELENGTH)
    az = max(az, MIN_SIDELENGTH)

    # adjust volume based on size_type
    if vol_A is not None:
        current_vol = ax * ay * az

        if current_vol > 0:
            if "larger" in size_type:
                target_vol = max(vol_A * 1.8, current_vol)  # ensure it's at least 1.8×
            elif "smaller" in size_type:
                target_vol = min(vol_A * 0.2, current_vol)  # ensure it's at most 0.2×
            elif "similar" in size_type:
                target_vol = vol_A * np.random.uniform(0.9, 1.1)  # ±10%
            else:
                target_vol = vol_A  # default

            scale_factor = (target_vol / current_vol) ** (1/3)

            ax *= scale_factor
            ay *= scale_factor
            az *= scale_factor

    return n1, n2, ax, ay, az


def create_rotate_rescale_translate_objects(output_object_file_path, side_component_number, shape_types, positions, sizes):
    
    axis_to_attach = ['y', 'minus y', 'minus z', 'z']
    x_A = 0
    y_A = 0
    z_A = 0
    dfac = 1.2
    min_distance = 1.5*MAX_SIDELENGTH

    shape_parameters = define_shape_parameters(shape_types[0])

    n1_ref_A = shape_parameters[0]
    n2_ref_A = shape_parameters[1]
    scale_ax_A = shape_parameters[2]
    scale_ay_A = shape_parameters[3]
    scale_az_A = shape_parameters[4]
    max_angle_A = shape_parameters[5]

    vol_A = None
    size_type = ""
    n_1_A, n_2_A, a_x_A, a_y_A, a_z_A = select_shape_parameters(scale_ax_A, 
                                                                scale_ay_A, 
                                                                scale_az_A, 
                                                                n1_ref_A, 
                                                                n2_ref_A,
                                                                vol_A,
                                                                size_type, 
                                                                )

    rot_x_A = np.random.uniform(ANGLE_MIN, max_angle_A)
    rot_z_A = np.random.uniform(ANGLE_MIN, max_angle_A)
    vol_A = a_x_A * a_y_A * a_z_A
    
    jitter_scale = UNIT_CUBE_SIDELENGTH / 30  # Small jitter in overall object placement within unit cube

    x_offset = np.clip(np.random.normal(loc=0, scale=jitter_scale), -jitter_scale*3, jitter_scale*3)
    y_offset = np.clip(np.random.normal(loc=0, scale=jitter_scale), -jitter_scale*3, jitter_scale*3)
    z_offset = np.clip(np.random.normal(loc=0, scale=jitter_scale), -jitter_scale*3, jitter_scale*3)

    object_successfully_created = True
    create_directory_if_not_exists(output_object_file_path)

    component_names = []
    components = []
    all_locations = []
    all_rescaled_translated_locations = []
    all_edginesses = []
    all_main_axes = []
    all_rescaled_main_axes = []
    all_rotations = []
    bbox_center = []
    rescale_factor = []
    offset = []

    # camera records object from the top (z-axis)
    rotation_x = math.radians(-90)  
    rotation_y = math.radians(-90)  
    rotation_z = math.radians(0) 
    
    # Create an empty object to act as the parent
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.empty_add(location=(0, 0, 0))
    empty = bpy.context.object
    empty.name = 'Rotation_Empty'
    
    # set up base object
    base_object = create_superellipsoid("BaseComponent", (x_A, y_A, z_A), 
                                        n_1_A, n_2_A, a_x_A, a_y_A, a_z_A, rot_x_A, rot_z_A)
    base_object.parent = empty  # Parent base object to the empty

    # get base object oriented bounding box (obb)
    base_center, base_axes, base_half = get_obb_data_from_superellipsoid(
        (x_A, y_A, z_A), a_x_A, a_y_A, a_z_A, rot_x_A, rot_z_A
    )

    shrink_factor = 0.95  # shrink by 5% for curvy object overlap check

    base_half_shrunk = tuple(h * shrink_factor for h in base_half)

    all_locations.append(base_object.location[:])
    all_edginesses.append((n_1_A, n_2_A))
    all_main_axes.append((a_x_A, a_y_A, a_z_A))
    all_rotations.append((rot_x_A, rot_z_A))
    offset.append((x_offset, y_offset, z_offset))

    component_names.append("BaseComponent")
    components.append(base_object)
    
    for i in range(side_component_number):

        shape_parameters = define_shape_parameters(shape_types[i + 1])

        n1_ref = shape_parameters[0]
        n2_ref = shape_parameters[1]
        scale_ax = shape_parameters[2]
        scale_ay = shape_parameters[3]
        scale_az = shape_parameters[4]
        max_angle = shape_parameters[5]
            
        n_1, n_2, a_x, a_y, a_z = select_shape_parameters(scale_ax, scale_ay, scale_az, n1_ref, n2_ref, vol_A, sizes[i])
            
        distance_is_okay = False
        overlap_is_okay = False
        number_of_overlaps_with_base = 0
            
        counter = 0
            
        while not (distance_is_okay and overlap_is_okay):
                
            counter += 1

            # choose axis based on positioning the user specified
            if "right" in positions[i]:
                axis = 'y'
            elif "left" in positions[i]:
                axis = 'minus y'
            elif "top" in positions[i]:
                axis = 'z'    
            elif "bottom" in positions[i]:
                axis = 'minus z'
            elif "random":
                # esle choose axis to which we attach the side object randomly
                axis = random.choice(axis_to_attach)
            else:
                raise ValueError(f"Unsupported position type: {positions[i]}")
                    
            if axis == 'y':
                
                y = a_y_A / dfac + a_y / dfac
                x = np.random.uniform(-(a_x_A / dfac), (a_x_A / dfac + 1))
                z = np.random.uniform(-(a_z_A / dfac), (a_z_A / dfac + 1))
                    
            if axis == 'minus y':
                y = -(a_y_A / dfac + a_y / dfac)
                x = np.random.uniform(-(a_x_A / dfac), (a_x_A / dfac + 1))
                z = np.random.uniform(-(a_z_A / dfac), (a_z_A / dfac + 1))
                    
            if axis == 'minus z':
                z = -(a_z_A / dfac + a_z / dfac)
                x = np.random.uniform(-(a_x_A / dfac), (a_x_A / dfac + 1))
                y = np.random.uniform(-(a_y_A / dfac), (a_y_A / dfac + 1))
                    
            if axis == 'z':
                z = a_z_A / dfac + a_z / dfac
                x = np.random.uniform(-(a_x_A / dfac), (a_x_A / dfac + 1))
                y = np.random.uniform(-(a_y_A / dfac), (a_y_A / dfac + 1))
       
            if i == 0:
                coordinates_side_comp_1 = (x, y, z)
                distance_is_okay = True
                    
            if i == 1:
                if (is_not_too_close(coordinates_side_comp_1, (x, z, z), min_distance)):
                    distance_is_okay = True
                        
            rot_x = np.random.uniform(ANGLE_MIN, max_angle)
            rot_z = np.random.uniform(ANGLE_MIN, max_angle)

            # get side object obb
            side_center, side_axes, side_half = get_obb_data_from_superellipsoid(
                (x, y, z), a_x, a_y, a_z, rot_x, rot_z
            )

            side_half_shrunk = tuple(h * shrink_factor for h in side_half)

            # test for overlap (side components should overlap with base)
            overlap_is_okay = test_obb_overlap(
                base_center, base_axes, base_half_shrunk,
                side_center, side_axes, side_half_shrunk
            )
            
            if counter > 10:
                distance_is_okay = True
                overlap_is_okay = True
                object_successfully_created = False
                
        object_name = "SideComponent_" + str(i)
        side_object = create_superellipsoid(object_name, (x, y, z), 
                                            n_1, n_2, a_x, a_y, a_z, rot_x, rot_z)
        
        side_object.parent = empty
        component_names.append(object_name) 
            
        side_object.location = (x, y, z)

        # collect superellipsoid parameters           
        all_locations.append(side_object.location[:])
        all_edginesses.append((n_1, n_2))
        all_main_axes.append((a_x, a_y, a_z))
        all_rotations.append((rot_x, rot_z))
       
    if object_successfully_created:

        bpy.ops.object.select_all(action='DESELECT')
        
        bbox_info = get_bounding_box(component_names)
        empty.select_set(True)
        bpy.context.view_layer.objects.active = empty
        
        target_bbox_size = (UNIT_CUBE_SIDELENGTH*2, UNIT_CUBE_SIDELENGTH*2, UNIT_CUBE_SIDELENGTH*2)  
        scale_fac = scale_object_to_bbox(empty, target_bbox_size, bbox_info)
        rescale_factor.append(scale_fac)
        bbox_info_rescaled = get_bounding_box(component_names)
        
        bpy.context.view_layer.update()  # ensure Blender updates object transforms


        bbox_info_rescaled = get_bounding_box(component_names)
        #centroid = get_geometry_centroid(component_names)
        combined_bbox_center = bbox_info_rescaled['center']
        bbox_center.append(combined_bbox_center)
        empty.rotation_euler = (rotation_x, rotation_y, rotation_z)
        empty.location.x -= combined_bbox_center[1]
        empty.location.y -= combined_bbox_center[2]
        empty.location.z -= combined_bbox_center[0]
        
        # fit object within unit cube by shifting based on bbox center
        empty.location.x += y_offset
        empty.location.y += z_offset
        empty.location.z += x_offset
        
        #empty.location.x -= centroid[1]
        #empty.location.y -= centroid[2]
        #empty.location.z -= centroid[0]
        
        # update all locations with the applied translation
        for loc in all_locations:
            rescaled_translated_loc = (
                loc[0]*scale_fac - combined_bbox_center[0],
                loc[1]*scale_fac - combined_bbox_center[1],
                loc[2]*scale_fac - combined_bbox_center[2]
            )
            all_rescaled_translated_locations.append(rescaled_translated_loc)
            
        # update all locations with the applied translation
        for axes in all_main_axes:
            rescaled_axes = (
                axes[0]*scale_fac,
                axes[1]*scale_fac,
                axes[2]*scale_fac
            )
            all_rescaled_main_axes.append(rescaled_axes)
            
        # export objects with applied transformations
        for obj_name in component_names:
            obj = bpy.data.objects.get(obj_name)
            if obj:
                bpy.ops.object.select_all(action='DESELECT')
                obj.select_set(True)
                bpy.context.view_layer.objects.active = obj
                object_file_path = output_object_file_path + '/' + obj_name + '.obj'
                bpy.ops.export_scene.obj(filepath=object_file_path, use_selection=True)
                obj.select_set(False)
            else:
                print(f"Object {obj_name} not found for export.")
        
        # write component parameters to json file
        object_details = {
            "all_locations": all_locations,
            "all_translated_locations": all_rescaled_translated_locations,
            "all_edginesses": all_edginesses,
            "all_main_axes":  all_main_axes,
            "all_rescaled_main_axes":  all_rescaled_main_axes,
            "all_rotations": all_rotations,
            "bbox_center": bbox_center,
            "rescale_factor": rescale_factor,
            "offset": offset,
        }

        json_file_path = output_object_file_path + '/object.json'
        # dump parameters of base / side object to json file
        with open(json_file_path, 'w') as f:
            json.dump(object_details, f, indent=4)
            
    return object_successfully_created


def generate_topdown_depth_image(ID_path, resolution=(1256, 1256)):

    bounds = ((-2*UNIT_CUBE_SIDELENGTH, -2*UNIT_CUBE_SIDELENGTH), 
              (2*UNIT_CUBE_SIDELENGTH, 2*UNIT_CUBE_SIDELENGTH))
    
    # place "camera" for simple projection above unit cube 
    reference_z = 2*UNIT_CUBE_SIDELENGTH
    width, height = resolution
    x_min, y_min = bounds[0]
    x_max, y_max = bounds[1]

    # initialize empty (0 valued) image canvas
    depth_map = np.zeros((height, width), dtype=np.float32)
    max_depth = 0.0

    # ray cast through object to create simple projection of object
    for i in range(width):
        for j in range(height):
            x = x_min + (i / (width - 1)) * (x_max - x_min)
            y = y_min + (j / (height - 1)) * (y_max - y_min)

            origin = Vector((x, y, reference_z))
            direction = Vector((0, 0, -1))

            if bpy.app.version < (2, 91, 0):
                result, location, _, _, _, _ = bpy.context.scene.ray_cast(
                    bpy.context.view_layer, origin, direction)
            else:
                result, location, _, _, _, _ = bpy.context.scene.ray_cast(
                    bpy.context.evaluated_depsgraph_get(), origin, direction)

            if result:
                depth = reference_z - location.z
                depth_map[height - j - 1, i] = depth
                if depth > max_depth:
                    max_depth = depth
            else:
                depth_map[height - j - 1, i] = 0.0

    depth_img = Image.fromarray(depth_map.astype(np.float32), mode='F')
    tiff_path = ID_path + '/depth.tiff'
    depth_img.save(tiff_path)


def generate_blender_superegg_dataset(args):

    object_ID = args.object_id
    number_of_samples = args.number_of_samples
    start_idx = START_IDX
    number_of_side_components = args.num_of_side_components
    shape_types = [args.shape_type1, args.shape_type2, args.shape_type3]
    positions = [args.position2, args.position3]
    sizes = [args.size2, args.size3]

    safety_counter = 0

    for object_idx in range(start_idx, start_idx + number_of_samples):

        print('Current object number is ', object_idx, flush=True)
        
        successful_object_creation = False
        
        while not successful_object_creation:
            safety_counter += 1
            # delete everything that's present, we want a fresh start
            bpy.ops.object.select_all(action="SELECT")
            bpy.ops.object.delete(use_global=True)

            ID_path = 'superellipsoid_learning/data_unit_cube_rotated_simprojc/objects_' + object_ID + '/object_' + f'{object_idx}'
           
            # create object composed of base object and side object of height object height
            successful_object_creation = create_rotate_rescale_translate_objects(ID_path, 
                                                                               number_of_side_components, 
                                                                               shape_types, 
                                                                               positions, 
                                                                               sizes)
            
            if safety_counter > 100:
                print("We reached too many iterations, are you sure the parameters are reasonable?", flush=True)
                break

        
        generate_topdown_depth_image(ID_path)


def define_shape_parameters(shape_type):
    # n1, n2, ax, ay, az, max_angle
    if "sphere" in shape_type:
        return EDGINESS_MAX, EDGINESS_MAX, 1, 1, 1, ANGLE_MAX #?

    elif "cube" in shape_type:
        return EDGINESS_MIN, EDGINESS_MIN, 1, 1, 1, ANGLE_MAX #?

    elif "block" in shape_type:
        return EDGINESS_MIN, EDGINESS_MIN, 1, 1, 3, ANGLE_MAX

    elif "cylinder" in shape_type:
        return EDGINESS_MIN, EDGINESS_MAX, 1, 1, 3, ANGLE_MAX*2

    elif "capsule" in shape_type:
        return EDGINESS_MAX, EDGINESS_MAX, 1, 1, 3, ANGLE_MAX

    elif "disk" in shape_type:
        return EDGINESS_MIN, EDGINESS_MAX, 3, 3, 1, ANGLE_MAX*2

    elif "rod" in shape_type:
        return EDGINESS_MIN, EDGINESS_MAX, 1, 1, 3, ANGLE_MAX*2
    
    elif "part" in shape_type:
        n1 = np.random.uniform(EDGINESS_MIN, EDGINESS_MAX)
        n2 = np.random.uniform(EDGINESS_MIN, EDGINESS_MAX)
        ax = np.random.uniform(1, 3)
        ay = np.random.uniform(1, 3)
        az = np.random.uniform(1, 3)

        return n1, n2, ax, ay, az, ANGLE_MAX*2
        
    else:
        raise ValueError(f"Unsupported shape type: {shape_type}")
    
# parse command line arguments for the superellispoid object dataset generation
def parse_arguments():
    
    try:
        idx = sys.argv.index("--")
        script_args = sys.argv[(idx + 1):]  # only take arguments after --
    except ValueError:
        script_args = []  # no custom arguments passed
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--object_id', type=str, required=True)
    parser.add_argument('--number_of_samples', type=int, required=True)
    parser.add_argument('--num_of_side_components', type=int, required=True)
    parser.add_argument('--shape_type1', type=str, default='firstshape_part', 
                       choices=['firstshape_rod', 'firstshape_disk', 'firstshape_part'],
                       help='Type of shape parameters to use for the supereggs')
    parser.add_argument('--shape_type2', type=str, default='secondshape_other_part', 
                       choices=['secondshape_rod', 'secondshape_disk', 'secondshape_other_part'],
                       help='Type of shape parameters to use for the supereggs')
    parser.add_argument(
        '--size2',
        type=str,
        default='random',
        choices=[
            'secondsize_larger',
            'secondsize_smaller',
            'secondsize_similar_size',
        ],
        help='Size of the second component relative to the first'
    )
    parser.add_argument(
        '--position2',
        type=str,
        default='random',
        choices=[
            'secondposition_to_the_right',
            'secondposition_to_the_left',
            'secondposition_on_top',
            'secondposition_at_the_bottom'
        ],
        help='Position of the second component relative to the first'
    )
    parser.add_argument('--shape_type3', type=str, default='thirdshape_other_part', 
                       choices=['thirdshape_rod', 'thirdshape_disk', 'thirdshape_other_part'],
                       help='Type of shape parameters to use for the supereggs')
    parser.add_argument(
        '--position3',
        type=str,
        default='random',
        choices=[
            'thirdposition_to_the_right',
            'thirdposition_to_the_left',
            'thirdposition_on_top',
            'thirdposition_at_the_bottom'
        ],
        help='Position of the third component relative to the first'
    )
    parser.add_argument(
        '--size3',
        type=str,
        default='random',
        choices=[
            'thirdsize_larger',
            'thirdsize_smaller',
            'thirdsize_similar_size',
        ],
        help='Size of the third component relative to the first'
    )
    
    return parser.parse_args(script_args)


def main():
    
    args = parse_arguments()
    generate_blender_superegg_dataset(args)

if __name__ == '__main__':
    main()
     



