import numpy as np

def rbf_kernel(X1, X2, length_scale, sigma_f):
    """
    Computes the RBF (Radial Basis Function) kernel between X1 and X2.
    """
    sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * X1 @ X2.T  # distance between points in X1 and X2
                                                                                # note the dimentions in the sums!
                                                                                # This is to create a matrix containing
                                                                                # all distances between pairs of points
    return sigma_f**2 * np.exp(-0.5 / length_scale**2 * sqdist) 


x = np.array([[0,0]])
y = np.array([[1,2]])
grad = np.array([[3.64, 7.29]])
length_scale = 1.0
sigma_f = 1.0
kernel_matrix = rbf_kernel(x, y, length_scale, sigma_f)
print("RBF Kernel Matrix:\n", kernel_matrix)
h = (grad * 0.082 * y)
h = np.array([2.985,1.9556]).reshape((2,1))
f = np.array([1.5, 0.0, 0.0 ,0.0]).reshape((4,1))
g = np.array([[0.0, 0.0], [0.0, 1.5], [1.0, 0.0], [0.0, 1.0]]).reshape((4,2))
print(f.shape)
print(h.shape)
print(f@h.T)
print(h@f.T)
print(g.T@h)