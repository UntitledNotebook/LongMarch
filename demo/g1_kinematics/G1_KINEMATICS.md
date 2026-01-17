# G1 Robot Kinematics and Blender Rendering

Tools to compute forward kinematics for the Unitree G1 humanoid robot and visualize the results in Blender.

## Overview

1. **C++ Program** (`main.cpp`): Computes forward kinematics and exports robot state to JSON
2. **Blender Python Script** (`load_robot_blender.py`): Loads meshes and materials into Blender for visualization

## Quick Start

### Step 1: Export Robot State

```bash
# Build the project
cd build
cmake --build . --target demo_g1_kinematics

# Export robot state to JSON
./demo/g1_kinematics/demo_g1_kinematics robot_state.json
```

### Step 2: Visualize in Blender

```bash
/path/to/blender \
  --python /path/to/demo/g1_kinematics/load_robot_blender.py \
  -- robot_state.json
```

Blender will open with the robot loaded.

## Customization

### Modify Joint Positions

Edit `main.cpp` around line 147 to change initial joint values:

```cpp
info.value = (info.lower_bound + info.upper_bound) * 0.5f;  // Mid-range
// Or set specific values:
// info.value = 0.0f;  // All joints at zero (default)
```

Then rebuild and re-export.

### Change URDF Model

The program uses `g1_23dof_mode_10.urdf` by default. To use a different URDF:

```cpp
// In main.cpp, change:
std::string urdf_path = grassland::FileProbe::GetInstance().FindFile("g1_23dof_mode_10.urdf");
// To another model like:
std::string urdf_path = grassland::FileProbe::GetInstance().FindFile("g1_29dof.urdf");
```

Available URDF files in `assets/urdfs/unitree_g1/`:
- `g1_23dof_mode_10.urdf` (default)
- `g1_23dof.urdf`
- `g1_29dof.urdf`
- `g1_29dof_mode_*.urdf` (various modes)

## Asset Location

URDF and mesh files are centralized in:
- `assets/urdfs/unitree_g1/` - URDF files
- `assets/urdfs/unitree_g1/meshes/` - STL mesh files