import cupy as cp
import numpy as np
import time
 
N = 1080
 
A = cp.random.rand(N, N)
b = cp.random.rand(1000)

start = time.time()
x = np.linalg.solve(A, b)
end = time.time()
print(f"NP Inv Time: {end - start:.4f} seconds")

start = time.time()
x = np.linalg.solve(A, b)
end = time.time()
print(f"NP Solve Time: {end - start:.4f} seconds")
 
start = time.time()
x = cp.linalg.solve(A, b)
cp.cuda.Device(0).synchronize()
end = time.time()
 
print(f"CUDA Solve Time method1: {end - start:.4f} seconds")

# Matrix size
 
# Generate a float32 matrix
A = cp.random.rand(N, N, dtype=cp.float32)
identity = cp.eye(A.shape[0])
 
# Synchronize and start timing
cp.cuda.Device(0).synchronize()
start = time.time()
 
# Fastest available GPU inverse in CuPy using cuSolver
A_inv =  cp.linalg.solve(A, identity)
print(A_inv.shape)
 
cp.cuda.Device(0).synchronize()
end = time.time()
print(f"CUDA Solve Time method2: {end - start:.4f} seconds")