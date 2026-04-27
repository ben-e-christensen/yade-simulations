from yade import pack, geom, qt, ymport
import math
import os

# --- 0. Global Variables & Setup ---
RPM = 26.0  
angular_vel = RPM * (math.pi / 30.0)

if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

with open('repose_data.csv', 'w') as f:
    f.write("sim_time,bead_id,x_pos,y_pos,z_pos,rot_w,rot_x,rot_y,rot_z,ang_vel_x,ang_vel_y,ang_vel_z\n")

def export_positions():
    with open('repose_data.csv', 'a') as f:
        for b in O.bodies:
            if isinstance(b.shape, Sphere):
                pos = b.state.pos
                ori = b.state.ori
                vel = b.state.angVel
                
                f.write(f"{O.time},{b.id},{pos[0]},{pos[1]},{pos[2]},"
                        f"{ori[0]},{ori[1]},{ori[2]},{ori[3]},"
                        f"{vel[0]},{vel[1]},{vel[2]}\n")

# --- 1. Materials (THE FRICTION HACK) ---
# We are overriding the 0.4 coefficient and forcing a massive 70-degree friction angle
fric_angle = math.radians(45) 

sphere_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, density=1190, frictionAngle=fric_angle))
wall_mat = O.materials.append(FrictMat(young=1e7, poisson=0.3, frictionAngle=fric_angle))

# --- 2. Geometry ---
R_base = 81.5e-3  
drum_h = 3.2e-3   
gap = 0.2e-3      
cohesion_strength = 1e4 

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

# --- 3. Particles (Hexagonal Grid Packing) ---
R_bead = 1.6e-3
target_beads = 500
count = 0

spacing = (R_bead * 2.0) * 1.01 
row_height = spacing * math.sqrt(3) / 2.0 
grid_size = int((R_base * 2) / spacing) + 2

for row in range(-grid_size, grid_size):
    if count >= target_beads: break
        
    for col in range(-grid_size, grid_size):
        if count >= target_beads: break
            
        x = col * spacing
        y = row * row_height
        
        if row % 2 != 0:
            x += spacing / 2.0
            
        if math.sqrt(x**2 + y**2) < (R_base - 0.003):
            O.bodies.append(sphere((x, y, gap/2), R_bead, material=sphere_mat))
            count += 1

print(f"\n--- SUCCESS: Placed {count} high-friction beads. ---\n")

# --- 4. Engines (REVERTED TO STANDARD, STABLE PHYSICS) ---
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom(), Ig2_Facet_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    RotationEngine(ids=rotating_ids, 
                   rotationAxis=(0, 0, 1), 
                   rotateAroundZero=True, 
                   angularVelocity=angular_vel),
                   
    PyRunner(command='export_positions()', virtPeriod=0.01),
    VTKRecorder(fileName='vtk_output/drum_', recorders=['spheres', 'facets', 'colors'], virtPeriod=0.01)
]

# --- 5. Execution Setup ---
O.dt = .5 * PWaveTimeStep()
qt.View()
qt.center()