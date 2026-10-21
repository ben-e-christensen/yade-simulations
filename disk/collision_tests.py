from yade import pack, geom, qt, ymport, export
import math
import os
import random

# --- 0. Global Variables & Setup ---
# The RPM & Duration Schedule
rpm_schedule = [1.0, 6.0, 11.0, 16.0, 21.0, 26.0]
duration_schedule = [60.0, 60.0, 30.0, 30.0, 30.0, 30.0] # 1 min for low speeds, 30s for high speeds

current_step = 0
current_rpm = rpm_schedule[current_step]
current_duration = duration_schedule[current_step]
angular_vel = current_rpm * (math.pi / 30.0)

# Stable 25-degree friction for this experiment
constant_friction_deg = 25.0
baseline_rad = math.radians(constant_friction_deg)

# Easily change this to 1000 for your second machine!
target_beads = 750  

# Relative Velocity Filter (m/s)
# Anything hitting slower than this is considered static bed creep and ignored
impact_velocity_threshold = 0.02 

sim_initialized = False
phase_start_time = 0.0 
global_start_time = 0.0

# Create output directory
if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

# Initialize the continuous tracking CSV
with open(f'bead_impact_stats_{target_beads}vol.csv', 'w') as f:
    f.write("sim_time,rpm,bead_id,total_collisions,avg_peak_force,max_peak_force\n")

# --- 1. Materials ---
sphere_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, density=1190, frictionAngle=baseline_rad))
wall_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, frictionAngle=baseline_rad))

# --- 2. Geometry ---
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

# --- 3. Particles (Hexagonal Grid Packing, Multicolored) ---
R_bead = 1.6e-3
count = 0

spacing = (R_bead * 2.0) * 1.01 
row_height = spacing * math.sqrt(3) / 2.0 
grid_size = int((R_base * 2) / spacing) + 2
tilt_rad = math.radians(constant_friction_deg)

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
            # Classic vibrant ball-pit look for long simulations
            s.shape.color = (random.random(), random.random(), random.random())
            O.bodies.append(s)
            count += 1

print(f"\n--- SUCCESS: Placed {count} multicolored beads (Target: {target_beads}). Friction locked at {constant_friction_deg}°. ---\n")

# --- 4. Custom Tracking & Ramping Engines ---

active_contacts = {} 
ignored_contacts = set()
bead_stats = {b.id: {'collisions': 0, 'sum_peak_forces': 0.0, 'max_peak_force': 0.0} 
              for b in O.bodies if isinstance(b.shape, Sphere)}

def track_collisions():
    """Monitors Newtonian impact forces ONLY for fast-moving, falling beads."""
    global active_contacts, ignored_contacts, bead_stats
    current_pairs = set()
    
    for i in O.interactions:
        if i.isReal and isinstance(O.bodies[i.id1].shape, Sphere) and isinstance(O.bodies[i.id2].shape, Sphere):
            id1, id2 = min(i.id1, i.id2), max(i.id1, i.id2)
            pair = (id1, id2)
            current_pairs.add(pair)
            
            # If flagged as static bed creep, bypass tracking
            if pair in ignored_contacts:
                continue
                
            fn = i.phys.normalForce.norm()
            
            if pair not in active_contacts:
                # NEW CONTACT: Evaluate initial incoming speed
                v1 = O.bodies[id1].state.vel
                v2 = O.bodies[id2].state.vel
                rel_vel = (v1 - v2).norm() 
                
                if rel_vel >= impact_velocity_threshold:
                    # High energy impact caught! Lock on to track peak force
                    active_contacts[pair] = fn
                else:
                    # Slow moving bed contact. Mute it for its entire lifecycle
                    ignored_contacts.add(pair)
            else:
                # Ongoing dynamic collision: keep the maximum peak force seen so far
                if fn > active_contacts[pair]:
                    active_contacts[pair] = fn
                    
    # Process contacts that just ended
    ended_pairs = (set(active_contacts.keys()) | ignored_contacts) - current_pairs
    
    for pair in ended_pairs:
        if pair in active_contacts:
            peak_f = active_contacts[pair]
            id1, id2 = pair
            
            for bead_id in (id1, id2):
                if bead_id in bead_stats:
                    bead_stats[bead_id]['collisions'] += 1
                    bead_stats[bead_id]['sum_peak_forces'] += peak_f
                    if peak_f > bead_stats[bead_id]['max_peak_force']:
                        bead_stats[bead_id]['max_peak_force'] = peak_f
                        
            del active_contacts[pair]
            
        if pair in ignored_contacts:
            ignored_contacts.remove(pair)

def export_bead_stats():
    """Appends the filtered rolling metrics to the CSV, tagged with the current RPM."""
    with open(f'bead_impact_stats_{target_beads}vol.csv', 'a') as f:
        for bead_id, stats in bead_stats.items():
            count = stats['collisions']
            avg_force = (stats['sum_peak_forces'] / count) if count > 0 else 0.0
            max_force = stats['max_peak_force']
            
            f.write(f"{O.time:.2f},{current_rpm},{bead_id},{count},{avg_force},{max_force}\n")

def rpm_stepper():
    """Manages the RPM schedule and resets all internal trackers per phase."""
    global current_step, current_rpm, current_duration, phase_start_time, global_start_time, sim_initialized, bead_stats, active_contacts, ignored_contacts
    
    if not sim_initialized:
        phase_start_time = O.time
        global_start_time = O.time
        print(f"\n[START] Starting schedule at {current_rpm} RPM for {current_duration}s")
        sim_initialized = True
        return
        
    if (O.time - phase_start_time) >= current_duration:
        # Final export for the finishing speed phase
        export_bead_stats()
        
        current_step += 1
        
        if current_step >= len(rpm_schedule):
            print(f"\n--- ALL SPEEDS COMPLETE! Total sim time: {(O.time - global_start_time):.1f}s ---")
            O.pause()
            return
            
        # Update Speed & Timing Variables
        current_rpm = rpm_schedule[current_step]
        current_duration = duration_schedule[current_step]
        
        # Inject new speed directly into the RotationEngine
        drum_rotator.angularVelocity = current_rpm * (math.pi / 30.0)
        
        # WIPE THE MEMORY SLATE CLEAN FOR THE NEW RPM PHASE
        active_contacts = {}
        ignored_contacts = set()
        bead_stats = {b.id: {'collisions': 0, 'sum_peak_forces': 0.0, 'max_peak_force': 0.0} 
                      for b in O.bodies if isinstance(b.shape, Sphere)}
                      
        print(f"\n[Sim Time: {O.time:.1f}s] Speed shifted -> Now running at {current_rpm} RPM for {current_duration}s")
        phase_start_time = O.time

# --- 5. Engines ---
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    
    GlobalStiffnessTimeStepper(active=1, timeStepUpdateInterval=100, timestepSafetyCoefficient=0.5),
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    # RotationEngine with remote label handle
    RotationEngine(ids=rotating_ids, 
                   rotationAxis=(0, 0, 1), 
                   rotateAroundZero=True, 
                   angularVelocity=angular_vel,
                   label='drum_rotator'),
                   
    # The scheduling brain (checked every 100 iterations)
    PyRunner(command='rpm_stepper()', iterPeriod=100),
                   
    # High-speed collision peak force monitor (checked every 10 physics steps)
    PyRunner(command='track_collisions()', iterPeriod=10),
    
    # Data logging period (Appends to CSV every 5 virtual seconds)
    PyRunner(command='export_bead_stats()', virtPeriod=5.0),
    
    # Exports for Paraview visualization
    VTKRecorder(fileName='vtk_output/drum_', recorders=['spheres', 'colors', 'velocity', 'id'], virtPeriod=0.05)
]

# --- 6. Execution Setup ---
qt.View()
qt.center()