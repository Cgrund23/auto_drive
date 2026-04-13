# Model-Free CBF for F1Tenth - Refactored Implementation

## Overview

This directory contains a **refactored implementation** of Control Barrier Functions for the F1Tenth platform, following the paper:

> **"Safety via Control Barrier Functions Synthesized from Ultra-Local Models"**  
> Luis A. Duffaut Espinosa and Colin Grund, CDC 2026

The implementation constructs safety certificates directly from LiDAR measurements without requiring an explicit plant model.

---

## Quick Start

### Installation
```bash
cd ~/f1tenth_ws/src/auto_drive
pip install cupy-cuda11x  # or cupy-cuda12x depending on your CUDA version
pip install qpsolvers[clarabel]
```

### Running
```bash
# Build
cd ~/f1tenth_ws
colcon build --packages-select auto_drive

# Launch
ros2 run auto_drive cbf_Node_refactored
```

### Testing
```bash
python3 auto_drive/test_model_free_cbf.py
```

---

## Files

### Core Implementation
- **`cbf_Node_refactored.py`**: ROS2 node with EKFs and control loop
- **`CBF_refactored.py`**: GP barrier functions and HOCBF QP solver

### Documentation
- **`MODEL_FREE_CBF_GUIDE.md`**: Detailed implementation guide
- **`IMPLEMENTATION_COMPARISON.md`**: Original vs refactored comparison

### Testing
- **`test_model_free_cbf.py`**: Unit tests and visualization

### Original Files (for reference)
- `cbf_Node.py`: Original first-order CBF implementation
- `CBF.py`: Original barrier functions

---

## Key Features

### 1. Model-Free Operation
- No explicit robot model required
- Learns dynamics online via MIMO ULMs
- Estimates drift and input sensitivity (F_q, B_q)

### 2. Gaussian Process Barriers
- Learns obstacle boundaries from LiDAR
- Adaptive safety margin from GP uncertainty
- Smooth, differentiable barriers

### 3. Extended Kalman Filters
- **Safety EKF**: Estimates [q, q̇, F_q, B_q]
- **Position EKF**: Estimates [p, ṗ, F_p, B_p]
- Fuses GP measurements with odometry

### 4. High-Order CBF (HOCBF)
- Second-order formulation for position-based barriers
- Constraint: `B̂_q u ≥ -F̂_q - (λ_0+λ_1)q̇̂ - λ_0λ_1q̂ + σ_k`
- Formal safety guarantees (Theorems 1-2 in paper)

---

## Theory Summary

### Ultra-Local Model (ULM)
For safety output q = h(p):
```
q̈ = F_q + B_q @ u
```
where:
- **F_q**: Lumped disturbance (drift, nonlinearities)
- **B_q**: Input sensitivity [B_v, B_ω]

### Measurements
1. **q** from GP posterior mean:
   ```
   q = 1 + k(p, P)^T K^{-1}(Y - 1)
   ```

2. **q̇** from GP gradient + velocity:
   ```
   q̇ = ∇h^T ṗ
   ```

### HOCBF Constraint
```
B̂_q u ≥ -F̂_q - (λ_0 + λ_1)q̇̂ - λ_0 λ_1 q̂ + σ_k
```

### Safety Margin
```
σ_k = c_q √(ℓ^T P ℓ)
```
where ℓ = [λ_0λ_1, λ_0+λ_1, 1, u^T]^T

---

## Tuning Parameters

### Critical Parameters
```python
# HOCBF gains
lambda_0 = 1.0  # Higher → more conservative
lambda_1 = 1.0  # Higher → more conservative

# Safety margin confidence
c_q = 2.0  # 2-sigma ≈ 95% confidence

# GP hyperparameters
length_scale = 0.4  # Barrier smoothness
sigma_f = 1.0       # Signal variance
```

### EKF Tuning
See `SafetyULM_EKF.__init__` and `PositionULM_EKF.__init__`:
- `Q`: Process noise (how fast parameters change)
- `R`: Measurement noise (sensor uncertainty)

---

## Validation

### Check Barrier Values
```bash
ros2 topic echo /cbf_debug
```
Look for:
- **q̂ > 0**: Safe
- **q̂ < 0**: Collision imminent
- **q̇̂ < 0**: Approaching obstacle

### Verify EKF Convergence
Monitor covariance matrix trace:
```python
P_trace = cp.trace(P)
```
Should decrease over time as estimates improve.

### Test Feasibility
```python
feasible = cbf.check_feasibility(q̂, q̇̂, F̂_q, B̂_q, σ_k)
```
If False:
- Decrease c_q (less conservative margin)
- Increase control bounds
- Reduce λ_0, λ_1

---

## Troubleshooting

### QP Infeasible
**Symptoms**: QP solver returns None, robot stops

**Solutions**:
1. Check if q̂ is very negative (already in collision)
2. Verify B̂_q has reasonable values (not NaN)
3. Reduce safety margin: `c_q = 1.5`
4. Increase control bounds: `u_max = [3.0, 1.2]`

### EKF Divergence
**Symptoms**: Estimates grow unbounded, P trace increases

**Solutions**:
1. Increase measurement frequency (reduce dt)
2. Tune process noise Q (likely too large)
3. Check for sensor dropouts
4. Verify persistent excitation (u must vary)

### GP Overfitting
**Symptoms**: Barrier has sharp discontinuities, oscillations

**Solutions**:
1. Increase length_scale (smoother barriers)
2. Downsample LiDAR points more aggressively
3. Add noise variance: `K + σ²_N I`

---

## Comparison to Original

| Feature | Original | Refactored |
|---------|----------|------------|
| CBF Order | 1st (discrete) | 2nd (HOCBF) |
| Dynamics | Assumed known | Learned (ULM) |
| Safety Margin | None | Adaptive (EKF) |
| Guarantees | Heuristic | Formal (Theorems) |
| Computation | ~10 ms | ~20 ms |

**When to use refactored**:
- ✅ Unknown or uncertain robot model
- ✅ Need formal safety proofs
- ✅ Complex environments (drift, terrain)
- ✅ Research / publication

**When to use original**:
- ✅ Quick prototyping
- ✅ Simple environments
- ✅ Known accurate model
- ✅ Minimal tuning time

---

## Citations

```bibtex
@inproceedings{duffaut2026safety,
  title={Safety via Control Barrier Functions Synthesized from Ultra-Local Models},
  author={Duffaut Espinosa, Luis A. and Grund, Colin},
  booktitle={IEEE Conference on Decision and Control (CDC)},
  year={2026}
}
```

---

## Support

For questions:
1. Check `MODEL_FREE_CBF_GUIDE.md` for detailed explanations
2. Run `test_model_free_cbf.py` to verify installation
3. Compare with original in `IMPLEMENTATION_COMPARISON.md`

---

## License

Match the license of the auto_drive package.

---

## Contributors

- Colin Grund (original implementation)
- Luis A. Duffaut Espinosa (paper / refactoring guidance)

**Last Updated**: 2026-04-13
