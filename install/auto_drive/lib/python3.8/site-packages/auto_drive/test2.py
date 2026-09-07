#  USING TORCH
import torch
import numpy as np
import time
import cupy
import scipy.sparse as sparse
import numpy as np


An = (sparse.random(1000, 1000, density=0.25)).toarray()

#print(An)

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
iden = torch.eye(1000)
# Create a random 1000x1000 matrix
A = torch.randn(1000, 1000, device=device)

# Warm-up (especially important for GPU)
_ = torch.pinverse(A)
_ = torch.linalg.solve(A,iden)
# Start timing
start = time.time()
 
# Invert matrix
solve = torch.linalg.solve(A,iden)
#inv_A = torch.pinverse(A)
 
# Wait for GPU to finish (important!)
if device.type == 'cuda':
    torch.cuda.synchronize()
 
end = time.time()
 
print(f"Time taken on {device} using Torch: {end - start:.6f} seconds")

#An = np.random.rand(1000,1000) 
# Start timing
start = time.time()
 
# Invert matrix
inv_A = np.linalg.solve(An,np.eye(1000))
 
end = time.time()
print(f"Time taken on {device} using Numpy: {end - start:.6f} seconds")