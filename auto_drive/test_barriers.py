import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from scipy.linalg import cholesky, solve, solve_triangular
 
# function for RBF kernel
def rbf_kernel(X1, X2, length_scale, sigma_f):
    """
    Computes the RBF (Radial Basis Function) kernel between X1 and X2.
    """
    sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * X1 @ X2.T  # distance between points in X1 and X2
                                                                                 # note the dimentions in the sums!
                                                                                 # This is to create a matrix containing
                                                                                 # all distances between pairs of points
    return sigma_f**2 * np.exp(-0.5 / length_scale**2 * sqdist)  # Same kernel as in paper
 
# function defining CBF (simple - not the one on GP Lidar paper)
def cbf_function(K_star, alpha):
    """
    Computes the CBF at certain distance.
    """
    return  (K_star @ alpha) + 1
    return  rbf_kernel(x_test, X_train, length_scale, sigma_f) @ alpha - safe_dist
 
 
# Function to apply a 2D rotation matrix
def rotate(x, y, angle_deg):
    angle_rad = np.radians(angle_deg)
    cos_theta, sin_theta = np.cos(angle_rad), np.sin(angle_rad)
    x_rot = x * cos_theta - y * sin_theta
    y_rot = x * sin_theta + y * cos_theta
    return x_rot, y_rot
 
# Function to draw a detailed TurtleBot with orientation
def draw_detailed_turtlebot(ax, center=(0, 0), base_radius=1, wheel_width=0.1, wheel_height=0.5,
                            sensor_radius=0.15, caster_wheel_radius=0.1, orientation_deg=0):
    x, y = center  # Unpack the center coordinates
 
    # Draw the base (outer circle) with rotation
    #base_x, base_y = rotate(x, y, orientation_deg)
    #base = plt.Circle((base_x, base_y), base_radius, color='lightblue', fill=True)
    req_width = 0.2
    req_length = 0.35
    base = plt.Rectangle((-req_width/2, -req_length/2),req_width, req_length, color='black',) # type: ignore
    ax.add_patch(base)
 
    # Draw the rear left wheel with rotation
    left_wheel_x = x - req_width/2 - wheel_width
    left_wheel_y = y + req_length/2 * .8 - wheel_height / 2
    
    left_wheel = plt.Rectangle((left_wheel_x, left_wheel_y),
                               wheel_width, wheel_height, color='gray', angle=0)
    ax.add_patch(left_wheel)
 
    # Draw the rear right wheel with rotation
    right_wheel_x = x + req_width/2  #wheel_width / 2
    right_wheel_y = y + req_length/2 * 0.8 - wheel_height / 2
    #right_wheel_x_rot, right_wheel_y_rot = rotate(right_wheel_x - x, right_wheel_y - y, orientation_deg)
    right_wheel = plt.Rectangle((x + right_wheel_x, y + right_wheel_y),
                                wheel_width, wheel_height, color='gray', angle=0)
    ax.add_patch(right_wheel)

    # Draw the front left wheel with rotation
    left_wheel_x = x - req_width/2 - wheel_width
    left_wheel_y = y - req_length/2 * .8 - wheel_height / 2
    
    front_left_wheel = plt.Rectangle((left_wheel_x, left_wheel_y),
                               wheel_width, wheel_height, color='gray', angle=0)
    ax.add_patch(front_left_wheel)
 
    # Draw the front wheel with rotation
    right_wheel_x = x + req_width/2  #wheel_width / 2
    right_wheel_y = y - req_length/2 * 0.8 - wheel_height / 2
    #right_wheel_x_rot, right_wheel_y_rot = rotate(right_wheel_x - x, right_wheel_y - y, orientation_deg)
    front_right_wheel = plt.Rectangle((x + right_wheel_x, y + right_wheel_y),
                                wheel_width, wheel_height, color='gray', angle=0)
    ax.add_patch(front_right_wheel)
 
    # Draw a small lidar or camera sensor on top with rotation
    sensor = plt.Circle((x, y), sensor_radius, color='orange', fill=True)
    ax.add_patch(sensor)
 

 
# # Step 1: Simulate LiDAR Data
# num_points = 50
x_width = 2
y_width = 2
x0, y0 = 0, 0
 
# num_lines = num_points
 
# x_startL = np.array([-17, 13, -9])
# x_endL = np.array([-2, 15, 13])
# y_startL = np.array([5, -10, -10])
# y_endL = np.array([17, 15, -10])
 
# num_barriers = 3
 
# x_lidar = []
# y_lidar = []
 
# for nB in np.arange(0, num_barriers):
#     x_start = x_startL[nB]
#     x_end = x_endL[nB]
#     y_start = y_startL[nB]
#     y_end = y_endL[nB]
   
#     # Line params
#     mL = (y_end - y_start) / (x_end - x_start)
#     bL = y_start - mL * x_start
 
#     angles = np.linspace(0, 2*np.pi, num_lines)
 
#     line_start = np.array([x_start, y_start])
#     line_end = np.array([x_end, y_end])
    

#     for theta in angles:
#         m = np.tan(theta)
#         # since the lines start at (0,0) then the intercept is 0
#         b = 0
#         if m != 0:
#             x_intersect = (bL - 0) / (m - mL)
#             y_intersect = mL*x_intersect + bL
#             if min(x_start, x_end) <= x_intersect <= max(x_start, x_end):
#                 x_lidar.append(x_intersect)
#                 if min(y_start, y_end) <= y_intersect <= max(y_start, y_end):
#                     y_lidar.append(y_intersect)
                   
               
               
 
# x_lidar = np.array(x_lidar)
# y_lidar = np.array(y_lidar)

point = np.load('auto_drive/points.npy', allow_pickle=True)
x_lidar = point[:, 0]
y_lidar = point[:, 1]

distances = np.sqrt(x_lidar**2 + y_lidar**2)
angles = np.arctan2(y_lidar, x_lidar)
 
 
 
 
 
#plt.figure(1)
plt.scatter(x0, y0, color='red', label='Sensor Position')
plt.scatter(x_lidar, y_lidar, color='blue', label='LiDAR Points')
plt.title('Simulated LiDAR Data')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.xlim([-x_width, x_width])
plt.ylim([-y_width, y_width])
plt.grid(True)
plt.legend()
 
 
# Step 2: Gaussian Process (GP) Model Training
sigma_f = 1.0  # set to 1 as in paper
length_scale = 0.15 # what im using in real code
noise_variance = 0 #1e-4 #1e-4

X_train = np.column_stack((x_lidar, y_lidar))
Y_train = distances
Y_train = -1*np.ones(distances.shape)  # Initialize Y_train with -1 values
 
# Compute the covariance matrices
K = rbf_kernel(X_train, X_train, length_scale, sigma_f) + noise_variance * np.eye(len(X_train))
alpha = np.linalg.pinv(K) @ (Y_train - 1)

# Define a grid for visualization
grid_size = 100
x_grid, y_grid = np.meshgrid(np.linspace(-x_width, x_width, grid_size), np.linspace(-y_width, y_width, grid_size))
X_test = np.column_stack((x_grid.ravel(), y_grid.ravel()))
 
# Predict GP mean and variance
K_star = rbf_kernel(X_test, X_train, length_scale, sigma_f)
K_ss = rbf_kernel(X_test, X_test, length_scale, sigma_f) + noise_variance * np.eye(len(X_test))

 
# Step 4: Define and Visualize trivial Control Barrier Function (CBF)
k_star = rbf_kernel(X_test, X_train, length_scale, sigma_f)
cbf_values = cbf_function(k_star, alpha)
#cbf_values = np.clip(cbf_values,-1,1)
print(cbf_values)

cbf_grid = cbf_values.reshape(grid_size, grid_size)
 
fig, ax = plt.subplots()
plt.contourf(x_grid, y_grid, cbf_grid, 1000, cmap='turbo')
plt.colorbar(label='CBF Value', orientation='vertical')
# plt.colorbar()
plt.title('Control Barrier Function (CBF) Visualization length scale = 0.15')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.grid(False)
scatter = plt.scatter(x_lidar, y_lidar, color='red', label='LiDAR Points', s=30)
# Plotting contour at h(x) = 0
# plt.contour(x_grid, y_grid, cbf_grid, [0], colors='black', linewidths=2)  # CBF boundary
# Plotting contour for CBF boundary at h(x) = 0
cbf_contour = plt.contour(x_grid, y_grid, cbf_grid, [0], colors='black', linewidths=2)

 
# Create legend entries
# Add legend for contour level of interest
contour_legend = mlines.Line2D([], [], color='black', linewidth=2, label='CBF Boundary (h(x)=0)')
plt.legend(handles=[contour_legend, plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='LiDAR Points')], loc='lower right')
#plt.colorbar(cbf_contour, label='CBF Value')
 
plt.xlim([-x_width, x_width])
plt.ylim([-y_width, y_width])
 
draw_detailed_turtlebot(ax, center=(0, 0), base_radius=.2, wheel_width=0.04, wheel_height=.1,
                        sensor_radius=0.03, caster_wheel_radius=0.02, orientation_deg=45)
 
# Draw lines from the origin to each LiDAR point
for x_l, y_l in zip(x_lidar, y_lidar):
    plt.plot([x0, x_l], [y0, y_l], color='green', linewidth=1.0, linestyle=':')
 
plt.show()  # Non-blocking, so both figures appear
 
#plt.waitforbuttonpress()
 
#plt.close('all')