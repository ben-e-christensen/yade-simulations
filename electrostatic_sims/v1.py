from yade import pack, geom, qt, ymport
import math
import os
import random

# --- 0. Global Variables & Setup ---
RPM = 26.0  # Change this single variable to speed up or slow down the drum!
angular_vel = RPM * (math.pi / 30.0)

if not os.path.exists('vtk_output'):
    os.makedirs('vtk_output')

# --- 1. Cohesive Materials ---
# We use CohFrictMat to allow particles to bond/glue together
glue_strength = 1e5  # Newtons of force required to pull them apart

mat_pos = O.materials.append(CohFrictMat(young=1e7, poisson=0.3, density=1190, 
                                         normalCohesion=glue_strength, shearCohesion=glue_strength, 
                                         frictionAngle=math.atan(0.4), label='Pos'))

mat_neg = O.materials.append(CohFrictMat(young=1e7, poisson=0.3, density=1190, 
                                         normalCohesion=glue_strength, shearCohesion=glue_strength, 
                                         frictionAngle=math.atan(0.4), label='Neg'))

# Walls have NO cohesion, so beads don't permanently stick to the glass
mat_wall = O.materials.append(CohFrictMat(young=1e7, poisson=0.3, frictionAngle=math.atan(0.4), 
                                          normalCohesion=0, shearCohesion=0, label='Wall'))

# --- 2. Geometry ---
R_base = 81.5e-3  
drum_h = 3.2e-3   
gap = 0.2e-3      

drum_facets = ymport.stl('serrated_drum.stl', material=mat_wall, scale=0.001)
drum_ids = O.bodies.append(drum_facets)

faceplates = geom.facetBox(center=(0, 0, gap/2), extents=(R_base * 1.5, R_base * 1.5, (drum_h + gap) / 2), 
                           material=mat_wall, wallMask=48)
faceplate_ids = O.bodies.append(faceplates)
rotating_ids = drum_ids + faceplate_ids

# --- 3. Particles ---
R_bead = 1.6e-3
sp = pack.SpherePack()

sp.makeCloud(minCorner=(-R_base, -R_base, -drum_h/2), 
             maxCorner=(R_base, R_base, drum_h/2 + gap), 
             rMean=R_bead, rRelFuzz=0.0, num=800)

count = 0
for center, radius in sp:
    x, y, z = center
    if math.sqrt(x**2 + y**2) < (R_base - 0.002):
        if count < 500:
            
            # THE COIN FLIP: 50% Positive, 50% Negative
            if random.random() > 0.5:
                b = sphere(center, radius, material=mat_pos)
                b.shape.color = (0.8, 0.1, 0.1) # RED for Positive
            else:
                b = sphere(center, radius, material=mat_neg)
                b.shape.color = (0.1, 0.1, 0.8) # BLUE for Negative
                
            O.bodies.append(b)
            count += 1

# --- 4. The "Electromagnetic" Arbiter ---
def manage_charges():
    # Scan every active collision in the simulation
    for i in O.interactions:
        if not i.isReal: continue
        
        b1 = O.bodies[i.id1]
        b2 = O.bodies[i.id2]
        
        # Rule 1: If it's hitting a wall, strip cohesion
        if not isinstance(b1.shape, Sphere) or not isinstance(b2.shape, Sphere):
            i.phys.normalAdhesion = 0
            i.phys.shearAdhesion = 0
            continue
            
        # Rule 2: SAME CHARGE (Pos+Pos or Neg+Neg) -> Repulsive/Slippery
        if b1.mat == b2.mat:
            i.phys.normalAdhesion = 0  # Break the glue
            i.phys.shearAdhesion = 0
            i.phys.frictionAngle = math.atan(0.1) # Make them slippery to mimic sliding away
            
        # Rule 3: OPPOSITE CHARGE (Pos+Neg) -> Sticky/Cohesive
        else:
            # Keep the high adhesion, and crank up friction so they lock together
            i.phys.frictionAngle = math.atan(0.8)

# --- 5. Engines ---
O.engines = [
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Facet_Aabb()]),
    # We must use 6D geometry and CohFrictPhys to calculate Cohesion
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom6D(), Ig2_Facet_Sphere_ScGeom6D()],
        [Ip2_CohFrictMat_CohFrictMat_CohFrictPhys(setCohesionNow=True, setCohesionOnNewContacts=True)],
        [Law2_ScGeom6D_CohFrictPhys_CohesionMoment()]
    ),
    NewtonIntegrator(gravity=(0, -9.81, 0), damping=0.2),
    
    # UPDATED: We now pass our calculated angular_vel variable here
    RotationEngine(ids=rotating_ids, rotationAxis=(0, 0, 1), rotateAroundZero=True, angularVelocity=angular_vel),
    
    # Run our charge arbiter constantly to check new collisions
    PyRunner(command='manage_charges()', virtPeriod=0.005),
    
    # Export 'colors' so Paraview can see the Red/Blue charges
    VTKRecorder(fileName='vtk_output/cohesive_', recorders=['spheres', 'facets', 'colors'], virtPeriod=0.01)
]

O.dt = .5 * PWaveTimeStep()