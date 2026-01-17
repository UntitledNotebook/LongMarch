#!/usr/bin/env python3
"""
Blender Python script to load the G1 humanoid robot from exported state data.

Usage:
    blender --python load_robot_blender.py -- [robot_state.json]
    
This will open Blender with the robot loaded and materials applied.
You can then adjust the camera, lighting, and render settings manually.
"""

import bpy
import json
import sys
import os
import mathutils

def clear_scene():
    """Remove all objects from the scene."""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    # Clear all mesh data
    for mesh in bpy.data.meshes:
        bpy.data.meshes.remove(mesh)
    
    # Clear all materials
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)

def import_stl(filepath, name="imported_mesh"):
    """Import an STL file and return the object."""
    if not os.path.exists(filepath):
        print(f"Warning: STL file not found: {filepath}")
        return None
    
    # Import STL
    bpy.ops.wm.stl_import(filepath=filepath)
    
    # Get the imported object (should be the active object)
    obj = bpy.context.active_object
    if obj:
        obj.name = name
    return obj

def create_material(name, color, roughness=0.3, metallic=0.9):
    """Create a principled BSDF material with the given color."""
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    
    # Clear default nodes
    nodes.clear()
    
    # Add nodes
    node_principled = nodes.new(type='ShaderNodeBsdfPrincipled')
    node_output = nodes.new(type='ShaderNodeOutputMaterial')
    
    # Set material properties for realistic metal
    node_principled.inputs['Base Color'].default_value = (*color, 1.0)
    node_principled.inputs['Metallic'].default_value = metallic
    node_principled.inputs['Roughness'].default_value = roughness
    node_principled.inputs['Specular IOR Level'].default_value = 0.5
    node_principled.inputs['Coat Weight'].default_value = 0.1
    node_principled.inputs['Coat Roughness'].default_value = 0.1
    
    # Link nodes
    links = mat.node_tree.links
    links.new(node_principled.outputs['BSDF'], node_output.inputs['Surface'])
    
    return mat

def apply_transform(obj, transform_matrix):
    """Apply a 4x4 transformation matrix to an object."""
    # Convert list to Blender Matrix
    matrix = mathutils.Matrix([
        transform_matrix[0],
        transform_matrix[1],
        transform_matrix[2],
        transform_matrix[3]
    ])
    
    obj.matrix_world = matrix

def setup_camera(position, target):
    """Create and position a camera looking at the target."""
    bpy.ops.object.camera_add()
    camera = bpy.context.active_object
    camera.name = "Camera"
    
    camera.location = position
    
    # Point camera at target
    direction = mathutils.Vector(target) - mathutils.Vector(position)
    rot_quat = direction.to_track_quat('-Z', 'Y')
    camera.rotation_euler = rot_quat.to_euler()
    
    # Set camera as active
    bpy.context.scene.camera = camera
    
    # Adjust camera settings
    camera.data.lens = 35  # 35mm focal length (roughly 45 degree FOV)
    camera.data.clip_end = 1000
    
    return camera

def setup_lighting(light_position, light_emission):
    """Create area light at specified position with given emission."""
    bpy.ops.object.light_add(type='AREA', location=light_position)
    light = bpy.context.active_object
    light.name = "AreaLight"
    
    # Configure area light
    light.data.energy = light_emission[0] * 10  # Scale for Blender
    light.data.size = 1.0
    light.data.color = (1.0, 1.0, 1.0)
    
    # Point light downward
    light.rotation_euler = (0, 0, 0)  # Adjust as needed
    
    return light

def setup_world():
    """Setup world environment with ambient lighting."""
    world = bpy.context.scene.world
    world.use_nodes = True
    nodes = world.node_tree.nodes
    
    # Clear existing nodes
    nodes.clear()
    
    # Add background node
    node_background = nodes.new(type='ShaderNodeBackground')
    node_output = nodes.new(type='ShaderNodeOutputWorld')
    
    # Set ambient color and strength
    node_background.inputs['Color'].default_value = (0.8, 0.8, 0.8, 1.0)
    node_background.inputs['Strength'].default_value = 0.5
    
    # Link nodes
    links = world.node_tree.links
    links.new(node_background.outputs['Background'], node_output.inputs['Surface'])

def add_ground_plane():
    """Add a ground plane for reference."""
    bpy.ops.mesh.primitive_cube_add(size=2, location=(0, 0, -1.0))
    ground = bpy.context.active_object
    ground.name = "Ground"
    ground.scale = (10, 10, 0.01)
    
    # Create ground material - less reflective than robot
    mat = create_material("GroundMaterial", (0.3, 0.3, 0.3), roughness=0.9, metallic=0.1)
    ground.data.materials.append(mat)
    
    return ground

def setup_render_settings(output_path, width=1920, height=1080):
    """Configure render settings."""
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'GPU'  # Use GPU if available
    scene.cycles.samples = 128  # Number of samples for quality
    
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    
    scene.render.image_settings.file_format = 'PNG'
    scene.render.filepath = output_path

def load_robot(json_path):
    """Load robot state from JSON and setup the scene."""
    print(f"Loading robot state from: {json_path}")
    
    # Load JSON data
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # Clear the scene
    clear_scene()
    
    # Setup world environment
    setup_world()
    
    # Add ground plane
    add_ground_plane()
    
    # Load and position robot links
    print("Loading robot meshes...")
    for i, link in enumerate(data['links']):
        link_index = link['link_index']
        mesh_path = link['mesh_path']
        color = link['color']
        transform = link['transform']
        
        print(f"  Loading link {link_index}: {mesh_path}")
        
        # Import mesh
        obj = import_stl(mesh_path, name=f"link_{link_index}")
        
        if obj:
            # Create and apply material
            mat = create_material(f"Material_{link_index}", color)
            if obj.data.materials:
                obj.data.materials[0] = mat
            else:
                obj.data.materials.append(mat)
            
            # Apply transformation
            apply_transform(obj, transform)
    
    # Setup default camera (user can adjust)
    camera_pos = (2.0, -2.0, 1.5)
    camera_target = (0.0, 0.0, 0.5)
    setup_camera(camera_pos, camera_target)
    
    # Setup default lighting (user can adjust)
    light_pos = (4.0, -3.0, 3.0)
    light_emission = (100.0, 100.0, 100.0)
    setup_lighting(light_pos, light_emission)
    
    # Add additional light for better illumination
    bpy.ops.object.light_add(type='SUN', location=(5, 5, 10))
    sun = bpy.context.active_object
    sun.data.energy = 2.0
    
    print("Robot loaded successfully!")
    print("You can now adjust the camera, lighting, and render settings manually.")
    print("Use Render > Render Image (F12) to render the scene.")

def main():
    """Main entry point."""
    # Parse command line arguments
    # Blender passes args after '--' to the script
    argv = sys.argv
    
    # Find the '--' separator
    if '--' in argv:
        argv = argv[argv.index('--') + 1:]
    else:
        argv = []
    
    # Get JSON path
    json_path = argv[0] if len(argv) > 0 else "robot_state.json"
    
    # Ensure absolute path
    json_path = os.path.abspath(json_path)
    
    print("=" * 60)
    print("Blender Robot Loader")
    print("=" * 60)
    print(f"Input JSON: {json_path}")
    print("=" * 60)
    
    # Load robot
    load_robot(json_path)

if __name__ == "__main__":
    main()
