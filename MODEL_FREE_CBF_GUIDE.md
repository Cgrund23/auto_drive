# Model-Free CBF Implementation Guide

## Overview

This implementation follows the paper **"Safety via Control Barrier Functions Synthesized from Ultra-Local Models"** by Luis A. Duffaut Espinosa and Colin Grund (CDC 2026).

The key innovation is constructing safety certificates directly from sensor measurements without an explicit plant model, using:
1. **GP-based barrier learning** from LiDAR data
2. **MIMO Ultra-Local Models (ULMs)** for estimating safety dynamics
3. **Extended Kalman Filters (EKFs)** for online parameter estimation

---

## Architecture

### 1. Safety Output ULM-EKF (`SafetyULM_EKF`)

**Purpose**: Estimate the second-order MIMO ULM for the safety output q = h(p):

```
q̈ = F_q + B_q @ u
```

**State Vector**: ξ_q = [q, q̇, F_q, B_q,v, B_q,ω]^T

- **q**: Barrier value (distance to nearest obstacle)
- **q̇**: Barrier time derivative
- **F_q**: Lumped disturbance (absorbs drift, nonlinearities)
- **B_q**: Input sensitivity vector [B_q,v, B_q,ω]

**Measurements**:
1. **q_k** from GP posterior mean (Section II-E)
2. **q̇_k** from GP gradient and position velocity: q̇ = ∇h^T ṗ

**Dynamics** (Euler discretization):
```
q_{k+1} = q_k + T_s q̇_k + (T_s²/2)(F_q + B_q @ u)
q̇_{k+1} = q̇_k + T_s(F_q + B_q @ u)
F_{q,k+1} = F_q,k + w_F
B_{q,k+1} = B_q,k + w_B
```

### 2. Position ULM-EKF (`PositionULM_EKF`)

**Purpose**: Estimate first-order ULM for position to provide velocity measurements:

```
ṗ = F_p + B_p @ u
```

**State Vector**: [p_x, p_y, F_p,x, F_p,y, B_p,x,v, B_p,x,ω, B_p,y,v, B_p,y,ω]^T

- **p**: Position [x, y]
- **F_p**: Position drift
- **B_p**: 2×2 input influence matrix

**Measurement**: Position from odometry

### 3. GP Barrier Function (`ModelFreeCBF`)

**Posterior Mean** (Eq. 4):
```
h(p) = 1 + k(p, P)^T K^{-1}(Y - 1)
```
where:
- P: Training points (LiDAR obstacle detections)
- Y: Labels (-1 for obstacles)
- Shifted to make default (far from obstacles) = +1

**Gradient** (Eq. 5):
```
∇h(p) = Σ_j α_j k(p, p_j) (p_j - p) / ℓ²
```
where α = K^{-1}(Y - 1)

**Posterior Variance** (Eq. 6):
```
σ²_GP(p) = k(p,p) - k(p,P) K^{-1} k(P,p)
```

---

## HOCBF Constraint (Section III-C)

The key result from the paper: for a second-order safety output (relative degree r=2), the HOCBF condition is:

```
B̂_q,k u_k ≥ -F̂_q,k - (λ_0 + λ_1)q̇̂_k - λ_0λ_1q̂_k + σ_k
```

where:
- **λ_0, λ_1 > 0**: HOCBF parameters (typically both = 1.0)
- **σ_k**: Safety margin accounting for estimation uncertainty

### Safety Margin (Eq. 13-14)

```
σ̄²_{η,k} = max_{u ∈ U} ℓ^T_{η,k}(u) P^q_k ℓ_{η,k}(u)
σ_k = c_q σ̄_{η,k} + σ_{k,1} + σ_{k,2}
```

where:
- **ℓ_{η,k}(u) = [λ_0λ_1, λ_0+λ_1, 1, u^T]^T** (Eq. 12)
- **c_q**: Confidence quantile (e.g., 2 for 2-sigma ≈ 95% confidence)
- **σ_{k,1}**: ULM approximation residual (set to 0 if affine)
- **σ_{k,2}**: Discretization margin

---

## Control Flow

### Main Loop (in `lidar_callback`)

1. **Extract LiDAR data** → obstacle points P

2. **Predict EKFs**:
   ```python
   self.safety_ekf.predict(u_prev)
   self.position_ekf.predict(u_prev)
   ```

3. **Compute measurements**:
   - GP barrier: `q_meas, σ²_GP = cbf.get_barrier_and_variance(p)`
   - GP gradient: `grad_h = cbf.get_gradient(p)`
   - Position velocity: `ṗ = F_p + B_p @ u_prev`
   - Barrier derivative: `q̇_meas = grad_h^T ṗ`

4. **Update EKFs**:
   ```python
   self.safety_ekf.update_q(q_meas, R_q=σ²_GP)
   self.safety_ekf.update_qdot(q̇_meas, R_qdot)
   ```

5. **Get estimates**:
   ```python
   q̂, q̇̂, F̂_q, B̂_q, P = self.safety_ekf.get_estimates()
   ```

6. **Solve CLF-CBF QP**:
   ```
   minimize    ||u - u_ref||²
   subject to  B̂_q u ≥ r_k  (CBF constraint)
               u_min ≤ u ≤ u_max
   ```
   where `r_k = -F̂_q - (λ_0+λ_1)q̇̂ - λ_0λ_1q̂ + σ_k`

7. **Send command** and update `u_prev`

---

## Key Differences from Original Implementation

### Original (`cbf_Node.py` + `CBF.py`)
- ❌ First-order discrete-time CBF: `a_v v + a_ω ω ≥ b`
- ❌ Single-step barrier constraint: `q_{k+1} ≥ 0`
- ❌ No ULM parameter estimation (placeholder only)
- ❌ Direct gradient computation without dynamics estimation

### Refactored (Model-Free)
- ✅ Second-order HOCBF with proper λ_0, λ_1 parameters
- ✅ Full MIMO ULM-EKF for safety dynamics
- ✅ Separate position ULM for velocity estimation
- ✅ Safety margin from EKF covariance
- ✅ Proper GP gradient + velocity product for q̇ measurement
- ✅ Matches paper formulation exactly

---

## Tuning Parameters

### HOCBF Parameters
```python
self.lambda_0 = 1.0  # Higher → more conservative
self.lambda_1 = 1.0  # Higher → more conservative
```

**Rule of thumb**: Start with λ_0 = λ_1 = 1.0, increase if safety violations occur.

### Safety Margin Confidence
```python
self.c_q = 2.0  # 2-sigma ≈ 95% confidence
```

**Options**:
- c_q = 1.0: 68% confidence (less conservative)
- c_q = 2.0: 95% confidence (moderate)
- c_q = 3.0: 99.7% confidence (very conservative)

### GP Hyperparameters
```python
self.length_scale = 0.4  # Smoothness of barrier
self.sigma_f = 1.0       # Signal variance
```

**Guidelines**:
- Smaller length_scale → sharper boundaries (fit more precisely)
- Larger length_scale → smoother boundaries (more conservative)

### EKF Process Noise
In `SafetyULM_EKF.__init__`:
```python
Q_diag = [1e-6, 1e-5, 1e-3, 1e-3]  # [q, q̇, F_q, B_q]
```

**Tuning**:
- Increase Q[2] (F_q) if drift changes rapidly
- Increase Q[3:] (B_q) if robot dynamics vary significantly

### Measurement Noise
```python
R_q = σ²_GP  # Adaptive from GP uncertainty
R_qdot = ...  # Computed from gradient norm and velocity uncertainty
```

---

## Verification

### Feasibility Check (Lemma 1)
```python
feasible = cbf.check_feasibility(q̂, q̇̂, F̂_q, B̂_q, σ_k)
```

If infeasible, consider:
1. Decreasing c_q (less conservative margin)
2. Increasing control input bounds
3. Reducing λ_0, λ_1 (less aggressive barrier enforcement)

### Safety Guarantees (Theorem 1)

For affine safety dynamics (q̈ = F_q + B_q u), if:
- EKF errors bounded: |ΔF_q| ≤ F̄, |ΔB_q| ≤ B̄
- λ_0 T_s < 1, λ_1 T_s < 1
- Margin δ ≥ F̄ + (λ_0+λ_1)q̄̇ + λ_0λ_1q̄ + B̄ sup_u ||u||

Then: **q(p_k) ≥ 0 for all k** (sampling-time safety)

### Probabilistic Safety (Theorem 2)

For non-affine dynamics, per-step violation probability:
```
P(violation) ≤ δ_q = 1 - Φ(c_q)
```

Examples:
- c_q = 2 → δ_q ≈ 2.3%
- c_q = 3 → δ_q ≈ 0.13%

---

## Usage

### Quick Start
```python
# Replace imports in your launch file
from auto_drive.cbf_Node_refactored import ControllerNode

# Or run directly
ros2 run auto_drive cbf_Node_refactored
```

### Monitoring
Key logged values:
- **q̂**: Should stay > 0 (safe)
- **q̇̂**: Negative when approaching obstacles
- **F̂_q**: Estimated drift/disturbance
- **B̂_q**: Input sensitivity [B_v, B_ω]

### Debugging
If QP fails:
1. Check `q̂` value (very negative → already in collision)
2. Verify B̂_q signs (should be physically reasonable)
3. Check feasibility: `cbf.check_feasibility(...)`
4. Increase σ_k or reduce λ parameters

---

## References

1. **Paper**: Duffaut Espinosa & Grund, "Safety via Control Barrier Functions Synthesized from Ultra-Local Models", CDC 2026
2. **Section III-B**: EKF measurement model
3. **Section III-C**: HOCBF constraint formulation
4. **Section IV**: Safety guarantees (Theorems 1-2)
5. **Section V**: Numerical example (differential drive)

---

## Next Steps

1. **Test in simulation** with known obstacles first
2. **Tune GP length_scale** by visualizing learned barriers
3. **Adjust λ parameters** if too conservative/aggressive
4. **Monitor EKF convergence** (check P matrix trace over time)
5. **Validate safety** by logging barrier violations

For questions or issues, refer to the paper sections noted above or file an issue in the repository.
