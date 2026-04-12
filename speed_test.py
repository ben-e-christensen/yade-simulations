from yade import pack
import time

# 1. Cranked up to 20,000 particles. 
# We shrunk rMean to .02 so they physically fit inside the 1x1x1 bounding box.
sp = pack.SpherePack()
sp.makeCloud((0,0,0), (1,1,1), rMean=.02, num=20000)
sp.toSimulation()

O.engines=[
    ForceResetter(),
    InsertionSortCollider([Bo1_Sphere_Aabb()]),
    InteractionLoop(
        [Ig2_Sphere_Sphere_ScGeom()],
        [Ip2_FrictMat_FrictMat_FrictPhys()],
        [Law2_ScGeom_FrictPhys_CundallStrack()]
    ),
    NewtonIntegrator()
]

start_time = time.time()

# 2. Cranked up to 50,000 calculation steps for a sustained burn
O.run(50000, True) 

elapsed_time = time.time() - start_time

# Write directly to a file to bypass terminal buffering
with open("benchmark_result.txt", "w") as f:
    f.write(f"{elapsed_time:.2f} seconds")

# Hard exit
O.exitNoBacktrace()