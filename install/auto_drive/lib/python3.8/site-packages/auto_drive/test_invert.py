import cupy as cp
import numpy as np
import time
 
N = 1000
 
A = cp.random.rand(N, N)
b = cp.eye(N)

A = cp.random.rand(N, N, dtype=cp.float32)
identity = cp.eye(A.shape[0])
 
start = time.time()
x = np.linalg.solve(A, identity)
end = time.time()
print(f"NP Inv Time: {end - start:.4f} seconds")

start = time.time()
x = np.linalg.solve(A, identity)
end = time.time()
print(f"NP Solve Time: {end - start:.4f} seconds")
 
start = time.time()
x = cp.linalg.inv(A)
#cp.cuda.Device(0).synchronize()
end = time.time()
 
#print(f"CUDA Solve Time method1: {end - start:.4f} seconds")

# Matrix size
 
# Generate a float32 matrix
A = cp.random.rand(N, N, dtype=cp.float32)
identity = cp.eye(A.shape[0])
_ =  cp.linalg.solve(A, identity)
# Synchronize and start timing
#cp.cuda.Device(0).synchronize()

start = time.time()
 
# Fastest available GPU inverse in CuPy using cuSolver
A_inv =  cp.linalg.solve(A, identity)
 
#cp.cuda.Device(0).synchronize()
end = time.time()
#print(f"CUDA Solve Time method2: {end - start:.4f} seconds")
cp.get_default_memory_pool().free_all_blocks()

N = 1000
A = cp.random.rand(N, N, dtype=cp.float32)
cp.linalg.inv(A)
cp.cuda.Stream.null.synchronize()
# Benchmark multiple runs
num_runs = 10
times = []
for _ in range(num_runs):
    start = time.time()
    A_inv = cp.linalg.inv(A)
    cp.cuda.Stream.null.synchronize()
    times.append((time.time() - start) * 1000)  # convert to ms

avg_time = sum(times) / num_runs
print(f"CUDA average inversion time: {avg_time:.2f} ms")

b = cp.eye(N)
cp.linalg.solve(A,b)
cp.cuda.Stream.null.synchronize()
# Benchmark multiple runs
num_runs = 10
times = []
for _ in range(num_runs):
    start = time.time()
    A_inv = cp.linalg.solve(A,b)
    cp.cuda.Stream.null.synchronize()
    times.append((time.time() - start) * 1000)  # convert to ms

avg_time = sum(times) / num_runs
print(f"CUDA average solve time: {avg_time:.2f} ms")

# other math
A = cp.random.rand(N,N)
b = cp.random.rand(N,1)
c = cp.random.rand(1,N)
d = cp.random.rand(N,N)
A_inv = cp.matmul(A,d)
cp.cuda.Stream.null.synchronize()
# Benchmark multiple runs
num_runs = 10
times = []
for _ in range(num_runs):
    start = time.time()
    A_inv = cp.matmul(A,d)
    cp.cuda.Stream.null.synchronize()
    times.append((time.time() - start) * 1000)  # convert to ms

avg_time = sum(times) / num_runs
print(f"CUDA average time Axd: {avg_time:.2f} ms")