from yade import pack, geom, qt, ymport, export
import math
import os
import numpy as np
import random

# --- 0. Global Variables & Setup ---
RPM = 26.0  
angular_vel = RPM * (math.pi / 30.0)

# Manufacturer specifications
baseline_friction_deg = 21.801 # arctan(0.4)
baseline_rad = math.radians(baseline_friction_deg)

# The extended E-Force sweep parameters (Removed 0.0, starting at 1e-8)
e_force_multipliers = [
    1.0e-8, 6.0e-8, 
    1.0e-7, 6.0e-7, 
    1.0e-6, 6.0e-6, 
    1.0e-5, 6.0e-5, 
    1.0e-4
]
current_e_idx = 0
current_e_mag = e_force_multipliers[current_e_idx]
phase_duration_virtual = 30.0 # 30 virtual seconds per E-force step

sim_initialized = False
phase_start_time = 0.0 
global_start_time = 0.0

# --- 1. File & Output Initialization ---
if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

# Lightweight real-time angle tracker
with open('live_repose_angles.csv', 'w') as f:
    f.write("sim_time,e_force,live_angle\n")

# Massive raw data tracker for Pandas offline analysis
with open('massive_repose_data.csv', 'w') as f:
    f.write("sim_time,e_force,x_pos,y_pos,z_pos\n")

# --- 2. Materials ---
sphere_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, density=1190, frictionAngle=baseline_rad))
wall_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, frictionAngle=baseline_rad))

# --- 3. Geometry ---
R_base = 81.5e-3  
drum_h = 3.2e-3   
gap = 0.2e-3      

drum_facets = ymport.stl('serrated_drum.stl', material=wall_mat, scale=0.001)
drum_ids = O.bodies.append(drum_facets)

faceplates = geom.facetBox(
    center=(0, 0, gap/2), 
    extents=(R_base * 1.5, R_base * 1.5, (drum_h + gap) / 2), 
    material=wall_mat, 
    wallMask=48 
)
faceplate_ids = O.bodies.append(faceplates)
rotating_ids = drum_ids + faceplate_ids

# --- 4. Particles (Hexagonal Grid Packing, PRE-TILTED) ---
R_bead = 1.6e-3
target_beads = 500
count = 0

spacing = (R_bead * 2.0) * 1.01 
row_height = spacing * math.sqrt(3) / 2.0 
grid_size = int((R_base * 2) / spacing) + 2

# Pre-tilt the bed to match the 21.8 degree friction angle
tilt_rad = math.radians(baseline_friction_deg)

for row in range(-grid_size, grid_size):
    if count >= target_beads: break
    for col in range(-grid_size, grid_size):
        if count >= target_beads: break
            
        x = col * spacing
        y = row * row_height
        
        if row % 2 != 0:
            x += spacing / 2.0
            
        x_rot = x * math.cos(tilt_rad) - y * math.sin(tilt_rad)
        y_rot = x * math.sin(tilt_rad) + y * math.cos(tilt_rad)
            
        if math.sqrt(x_rot**2 + y_rot**2) < (R_base - 0.003):
            s = sphere((x_rot, y_rot, gap/2), R_bead, material=sphere_mat)
            # Multicolored ball pit look!
            s.shape.color = (random.random(), random.random(), random.random())
            O.bodies.append(s)
            count += 1

print(f"\n--- SUCCESS: Placed {count} multicolored beads, pre-tilted to {baseline_friction_deg:.1f} degrees. ---\n")

# --- 5. Custom Python Engines ---

def e_force_stepper():
    """Automatically ramps up the electrostatic wall pull every 30 seconds."""
    global current_e_idx, current_e_mag, phase_start_time, global_start_time, sim_initialized
    
    if not sim_initialized:
        phase_start_time = O.time
        global_start_time = O.time
        sim_initialized = True
        return
        
    if (O.time - phase_start_time) >= phase_duration_virtual:
        current_e_idx += 1
        
        if current_e_idx >= len(e_force_multipliers):
            print(f"\n--- SWEEP COMPLETE! Total sim time: {(O.time - global_start_time):.1f}s ---")
            O.pause()
            return
            
        current_e_mag = e_force_multipliers[current_e_idx]
        print(f"\n[Sim Time: {O.time:.1f}s] Stepping up E-Force -> Now at {current_e_mag}")
        phase_start_time = O.time

def apply_wall_electrostatics():
    """Calculates and injects 3D boundary attraction forces."""
    wall_cutoff = R_bead * 4.0 
    Z_center = gap / 2.0
    Z_max = Z_center + (drum_h / 2.0)
    Z_min = Z_center - (drum_h / 2.0)
    
    for b in O.bodies:
        if isinstance(b.shape, Sphere):
            pos = b.state.pos
            fx, fy, fz = 0.0, 0.0, 0.0
            
            # Radial Pull (Serrated track)
            rad_dist = math.sqrt(pos[0]**2 + pos[1]**2)
            if rad_dist > 1e-6:
                dist_to_rad_wall = R_base - rad_dist
                if 0 < dist_to_rad_wall < wall_cutoff:
                    r_eff = max(dist_to_rad_wall, R_bead)
                    f_rad_mag = current_e_mag / (r_eff**2)
                    fx += f_rad_mag * (pos[0] / rad_dist)
                    fy += f_rad_mag * (pos[1] / rad_dist)
                    
            # Axial Pull (Faceplates)
            dist_to_front = Z_max - pos[2]
            dist_to_back = pos[2] - Z_min
            
            if 0 < dist_to_front < wall_cutoff:
                r_eff_front = max(dist_to_front, R_bead)
                fz += current_e_mag / (r_eff_front**2) 
                
            if 0 < dist_to_back < wall_cutoff:
                r_eff_back = max(dist_to_back, R_bead)
                fz -= current_e_mag / (r_eff_back**2)
                
            if fx != 0.0 or fy != 0.0 or fz != 0.0:
                O.forces.addF(b.id, (fx, fy, fz))

def live_angle_calc():
    """Calculates the dynamic angle of repose and saves it to a lightweight CSV."""
    x_coords, y_coords = [], []
    for b in O.bodies:
        if isinstance(b.shape, Sphere):
            x_coords.append(b.state.pos[0])
            y_coords.append(b.state.pos[1])
            
    if not x_coords: return
    x, y = np.array(x_coords), np.array(y_coords)
    
    num_bins = 30
    bins = np.linspace(x.min(), x.max(), num_bins)
    digitized = np.digitize(x, bins)
    
    surface_x, surface_y = [], []
    for i in range(1, num_bins):
        in_bin = (digitized == i)
        if np.any(in_bin):
            max_y_idx = np.argmax(y[in_bin])
            surface_x.append(x[in_bin][max_y_idx])
            surface_y.append(y[in_bin][max_y_idx])
            
    surface_x, surface_y = np.array(surface_x), np.array(surface_y)
    
    if len(surface_x) > 0:
        x_center = np.mean(surface_x)
        x_range = surface_x.max() - surface_x.min()
        valid_mask = (surface_x > x_center - 0.3 * x_range) & (surface_x < x_center + 0.3 * x_range)
        clean_x, clean_y = surface_x[valid_mask], surface_y[valid_mask]
        
        if len(clean_x) > 1:
            slope, _ = np.polyfit(clean_x, clean_y, 1)
            live_angle = np.degrees(np.arctan(abs(slope)))
            
            print(f"[LIVE] Time: {O.time:.2f}s | E-Force: {current_e_mag} | Angle: {live_angle:.2f}°")
            with open('live_repose_angles.csv', 'a') as f:
                f.write(f"{O.time},{current_e_mag},{live_angle}\n")

def export_raw_positions():
    """Dumps every bead's XYZ coordinate for offline Pandas analysis"""
    with open('massive_repose_data.csv', 'a') as f:
        for b in O.bodies:
            if isinstance(b.shape, Sphere):
                f.write(f"{O.time},{current_e_mag},{b.state.pos[0]},{b.state.pos[1]},{b.state.pos[2]}\n")

# --- 6. Engines ---
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    
    # 1. Inject Electrostatics every single physics tick
    PyRunner(command='apply_wall_electrostatics()', iterPeriod=1),
    
    GlobalStiffnessTimeStepper(active=1, timeStepUpdateInterval=100, timestepSafetyCoefficient=0.5),
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    RotationEngine(ids=rotating_ids, 
                   rotationAxis=(0, 0, 1), 
                   rotateAroundZero=True, 
                   angularVelocity=angular_vel),
                   
    # 2. Ramp up the E-Force every 30 seconds
    PyRunner(command='e_force_stepper()', iterPeriod=100),               
    
    # 3. Calculate and save the live angle at 10Hz (virtPeriod=0.1)
    PyRunner(command='live_angle_calc()', virtPeriod=0.1),
    
    # 4. Save the massive raw dataset at 100Hz (virtPeriod=0.01)
    PyRunner(command='export_raw_positions()', virtPeriod=0.01),
    
    # 5. Save the 3D visualization files at 20Hz (virtPeriod=0.05)
    VTKRecorder(fileName='vtk_output/drum_', recorders=['spheres', 'colors', 'velocity', 'id'], virtPeriod=0.05)
]

# --- 7. Execution Setup ---
qt.View()
qt.center()