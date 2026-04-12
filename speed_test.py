from yade import pack
import time

sp = pack.SpherePack()
sp.makeCloud((0,0,0), (1,1,1), rMean=.05, num=5000)
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

# Run exactly 10,000 steps
O.run(10000, True) 

elapsed_time = time.time() - start_time

# Write directly to a file to completely bypass terminal buffering
with open("benchmark_result.txt", "w") as f:
    f.write(f"{elapsed_time:.2f} seconds")

# Hard exit
O.exitNoBacktrace()