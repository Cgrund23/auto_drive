# CBF Simulation for Fast Debugging

## What's New: Tangent Controller Added! 🎯

The CBF now includes a **tangent controller** (nominal controller) that:
- Steers toward goal when path is clear
- Generates tangent steering commands when obstacles detected
- CBF filters these commands to ensure safety

This is the key missing piece from the paper! CBF = Tangent Controller (guidance) + Safety Filter

## Quick Start on Jetson

```bash
cd ~/f1tenth_ws/src/auto_drive

# If qpsolvers not installed:
pip3 install qpsolvers[clarabel]

# Run CPU version (faster, good for debugging)
python3 simulate_cbf_cpu.py

# Or run GPU version (uses CuPy on Jetson)
python3 simulate_cbf.py
```

## What to Expect

**Goal:** Robot starts at (0, 0) and must drive to (3, 0) while avoiding a center obstacle at (1.5, 0).

**Success:** Robot steers around obstacle and reaches x > 3.0m

**Failure:** Robot hits obstacle or stops before reaching goal

## Tuning Parameters

Edit at top of `simulate_cbf_cpu.py`:

```python
self.r_min_obstacle = 0.25  # Detection distance (m)
self.length_scale = 0.5     # GP kernel width
self.lambda_0 = 0.5         # CBF aggressiveness
self.lambda_1 = 0.5         # CBF aggressiveness  
self.c_q = 0.05             # Safety margin (smaller = less conservative)
```

## Results

- Creates `cbf_simulation_results.png` with 6 plots
- Shows trajectory, barrier value, B_q estimates, control commands
- Terminal output shows real-time status

## Interpreting Results

**Good signs:**
- B_q,v stays between 0.05-3.0 (positive and bounded)
- Barrier q stays above 0 (safe)
- Robot trajectory goes around obstacle
- Reaches x > 3.0m

**Bad signs:**
- B_q,v goes negative or > 3.0 (divergence)
- Barrier q < 0 (collision)
- Robot trajectory hits obstacle
- "QP INFEASIBLE" errors

## Once Working in Simulation

Deploy to hardware:
```bash
cd ~/f1tenth_ws
colcon build --packages-select auto_drive
source install/setup.bash
ros2 run auto_drive cbf_Node_refactored
```

**10x faster iteration with simulation!**
