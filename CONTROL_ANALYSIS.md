# Control Law Analysis: Model-Free CBF

## Overview
This F1Tenth implementation uses **Fliess model-free control** with **Gaussian Process barriers** and **High-Order Control Barrier Functions (HOCBF)**.

## Theoretical Foundation

### 1. Ultra-Local Model (ULM)
The system dynamics are represented as:
```
q̈ = F_q + B_q · u
```

Where:
- `q`: Barrier function (distance to closest obstacle)
- `F_q`: Lumped disturbance (captures all unmodeled dynamics)
- `B_q`: Input sensitivity matrix [B_v, B_ω]
- `u`: Control input [v, ω]

**Key Insight**: No geometric model required! Parameters F_q and B_q are estimated online via EKF.

### 2. Gaussian Process Barrier Learning
The barrier function h(p) is learned from sparse LiDAR observations using GP regression:
```
h(p) = 1 + k(p, P)ᵀ K⁻¹ (Y - 1)
```

Where:
- `k(p, P)`: RBF kernel between query point p and obstacles P
- `K`: Kernel matrix of obstacle observations
- `Y = -1`: Obstacles labeled as unsafe
- Shift by +1 makes far regions → h=1 (safe)

**Key Insight**: Continuous barrier representation from discrete sensors!

### 3. High-Order CBF Constraint
Safety is enforced via:
```
B̂_q · u ≥ -F̂_q - (λ₀ + λ₁)q̇ - λ₀λ₁q + σ
```

Where:
- `λ₀, λ₁`: Convergence rate parameters (tune for responsiveness)
- `σ`: Safety margin from EKF uncertainty
- `q̇`: Barrier time derivative

**Key Insight**: Second-order barrier allows smooth, anticipatory control!

## Control Flaws Fixed

### CRITICAL FIX 1: B_q Clamping Range
**Problem**: [Original line 198]
```python
B_q[0] = cp.clip(B_q[0], 0.7, 3.0)  # Too conservative!
```

This assumes velocity ALWAYS has strong effect on barrier (B_q,v ≥ 0.7). But when moving perpendicular to an obstacle, velocity has weak effect → QP infeasible.

**Fix**:
```python
B_q[0] = cp.clip(B_q[0], 0.3, 3.0)  # Wider adaptation range
```

**Why it matters**: Allows CBF to adapt to different approach angles. B_q,v ≈ 0.3 when skirting past obstacle, B_q,v ≈ 2.0 when head-on.

---

### CRITICAL FIX 2: Gradient-Based Noise Explosion
**Problem**: [Original line 506]
```python
R_qdot = grad_norm² σ_pdot² + σ_gp² / (ℓ² (grad_norm² + ε))
```

When `grad_norm → 0` (at barrier local max/min), second term → ∞, causing EKF divergence.

**Fix**:
```python
R_qdot = min(R_qdot, 1.0)  # Cap uncertainty
```

**Why it matters**: Barrier gradient is small when equidistant from multiple obstacles. Unbounded noise causes EKF to ignore measurements → parameter drift.

---

### IMPORTANT FIX 3: EKF Reset Hysteresis
**Problem**: [Original line 161-187]
Hard reset when B_q diverges can cause oscillation:
1. B_q drifts low → reset to 1.0
2. Next observation pulls it back down
3. Repeat every few cycles

**Fix**:
```python
# Require 1 second between resets
if current_time - self._last_reset_time < 1.0:
    return False
```

**Why it matters**: Prevents "reset thrashing". EKF needs time to converge after reset.

---

### IMPORTANT FIX 4: Velocity-Dependent Safety Margin
**Problem**: [Original CBF_refactored.py:231]
Safety margin σ is constant, but stopping distance scales as v².

**Fix**:
```python
v_scale = 1.0 + 0.5 * (v_current / v_max)
sigma_k = compute_safety_margin(P, u_max) * v_scale
```

**Why it matters**: At high speed, need more conservative margin. At low speed, can operate closer to obstacles.

---

### MINOR FIX 5: QP Solver Speed
**Problem**: Clarabel solver is robust but slow for small problems

**Fix**:
```python
solver='osqp',
eps_abs=1e-4,
eps_rel=1e-4,
max_iter=100,
polish=False
```

**Why it matters**: OSQP is 3-5x faster for 2D QPs. Relaxed tolerances acceptable for real-time control.

## Remaining Control Issues

### 1. QP Weight Ratio (NOT FIXED)
Current weights:
```python
w_v = 10.0   # Velocity change cost
w_omega = 0.1  # Steering change cost
```

**Problem**: 100:1 ratio strongly prefers steering. For obstacles directly ahead, causes steering oscillation before braking.

**Potential Fix**: Make weights adaptive:
```python
# Compute obstacle angle from LiDAR
obstacle_angle = atan2(obstacle_y, obstacle_x)
w_omega = 0.1 if abs(obstacle_angle) < 0.2 else 1.0
```

**Why not implemented**: Requires passing obstacle geometry to CBF. Current design is geometry-agnostic (only uses barrier values).

---

### 2. No Slip Angle Compensation
Current Ackermann conversion:
```python
steering_angle = arctan(L * omega / v)
```

**Problem**: Ignores tire slip at high speed/steering. Can underestimate actual turning.

**Potential Fix**: Add empirical slip model:
```python
beta = 0.1 * (v / v_max) * steering_angle  # Slip angle
steering_angle_compensated = steering_angle + beta
```

**Why not implemented**: Requires vehicle-specific calibration. F1Tenth at v<2m/s has minimal slip.

---

### 3. EKF Covariance Inflation at Startup
**Problem**: P starts small → sigma_k small → aggressive early control → potential crash before convergence.

**Potential Fix**: Inflate P for first 2 seconds:
```python
if self._iteration_count < 200:  # First 2s at 100Hz
    P = P * 2.0  # Conservative during warmup
```

**Why not implemented**: Current "safe if no obstacles" logic (line 543-545) provides implicit warmup protection.

## Parameter Tuning Guide

### HOCBF Parameters
- `lambda_0, lambda_1`: Higher → faster convergence, but less smooth
  - **Current**: 0.3 (tuned for 100Hz)
  - **Range**: 0.1-0.5
  - **Rule**: lambda ≈ 2π / (desired_convergence_time * dt)
  
- `c_q`: Safety margin multiplier
  - **Current**: 0.05 (minimal for feasibility)
  - **Range**: 0.01-0.2
  - **Rule**: Higher c_q → more conservative, but can cause infeasibility

### GP Kernel Parameters
- `length_scale`: Spatial smoothness
  - **Current**: 0.25m (tight kernel)
  - **Effect**: Smaller → more localized influence
  - **Rule**: Should be ~2x sensor resolution
  
- `sigma_f`: Signal variance
  - **Current**: 1.0
  - **Effect**: Higher → trust observations more
  - **Rule**: Keep at 1.0, tune via R_q instead

### EKF Process Noise
- `Q[F_q]`: Disturbance variability
  - **Current**: 1e-3
  - **Effect**: Higher → faster parameter adaptation
  - **Rule**: Increase if F_q not tracking reference well
  
- `Q[B_q]`: Input sensitivity variability
  - **Current**: 1e-3
  - **Effect**: Higher → faster gain adaptation
  - **Rule**: Increase if QP frequently infeasible

## Testing Protocol

### 1. Verify 100Hz Operation
```bash
ros2 topic hz /drive
# Should show ~100Hz

ros2 topic echo --once /drive
# Check latency in timestamp
```

### 2. Check EKF Convergence
Add logging to print B_q every 50 iterations:
```python
self.get_logger().info(f'B_q = [{B_q[0]:.3f}, {B_q[1]:.3f}]')
```

**Expected**:
- B_q,v stabilizes to 0.5-2.0 (vehicle-dependent)
- B_q,ω stabilizes to -0.5 to 0.5
- Convergence within 5 seconds

### 3. Test QP Feasibility
Monitor QP failures:
```python
# In CBF_refactored.py, track success rate
if sol is None:
    self._qp_failures += 1
```

**Target**: <1% failure rate after 5s warmup

### 4. Safety Verification
- **Obstacle directly ahead**: Should brake smoothly, no collision
- **Obstacle at 45°**: Should steer + maintain speed
- **Obstacle at 90°**: Should ignore (not in collision course)

## References
1. "Safety via Control Barrier Functions Synthesized from Ultra-Local Models" (source paper)
2. Fliess, M., & Join, C. (2013). "Model-free control"
3. Williams, C. K., & Rasmussen, C. E. (2006). "Gaussian processes for machine learning"
