# Model-Free CBF Architecture Diagram

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                         F1Tenth Robot                            │
│  ┌────────────┐         ┌────────────┐         ┌──────────┐    │
│  │   LiDAR    │         │  Odometry  │         │ Motor    │    │
│  │  Sensor    │         │  (IMU)     │         │ Control  │    │
│  └─────┬──────┘         └──────┬─────┘         └────▲─────┘    │
└────────┼───────────────────────┼────────────────────┼──────────┘
         │ ranges, angles        │ position           │ u_safe
         │                       │                     │
    ┌────▼───────────────────────▼─────────────────────┴────┐
    │           ROS2 Controller Node                        │
    │         (cbf_Node_refactored.py)                      │
    └──────────────────────────────────────────────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        │                       │                       │
        │                       │                       │
    ┌───▼────────┐     ┌────────▼───────┐     ┌────────▼───────┐
    │ Position   │     │   Safety       │     │    CBF         │
    │ ULM-EKF    │     │   ULM-EKF      │     │  Barrier       │
    │            │     │                │     │  Function      │
    │ State:     │     │ State:         │     │                │
    │ [p, ṗ,     │     │ [q, q̇,        │     │ GP-based       │
    │  F_p, B_p] │     │  F_q, B_q]     │     │ barrier        │
    └────┬───────┘     └────────┬───────┘     └────────┬───────┘
         │                      │                      │
         │ ṗ estimate           │ (q̂, q̇̂, F̂_q, B̂_q)   │ (q, σ²_GP, ∇h)
         │                      │                      │
         └──────────────────────┼──────────────────────┘
                                │
                       ┌────────▼────────┐
                       │   QP Solver     │
                       │  (HOCBF + CLF)  │
                       │                 │
                       │ minimize        │
                       │  ||u - u_ref||² │
                       │ subject to:     │
                       │  B̂_q u ≥ r_k    │
                       └────────┬────────┘
                                │
                                ▼
                            u_safe [v, ω]
```

---

## Data Flow Diagram

```
LiDAR Scan
    │
    ├─► Parse ranges/angles
    │
    ├─► Extract obstacle points P = [(x₁,y₁), ..., (xₙ,yₙ)]
    │
    └─► Build GP kernel K = k(P, P)
         │
         ├─► Compute barrier: q = h(p) = 1 + k(p,P)ᵀ K⁻¹(Y-1)
         │                                            │
         │                                            └─► Safety EKF
         │                                                measurement 1
         ├─► Compute gradient: ∇h = Σ αⱼ k(p,pⱼ)(pⱼ-p)/ℓ²
         │                       │
         │                       └─────────┐
         │                                 │
         └─► Compute variance: σ²_GP       │
                                           │
Odometry                                   │
    │                                      │
    └─► Position p = [x, y]                │
         │                                 │
         └─► Position EKF update           │
              │                            │
              └─► Estimate ṗ = F̂_p + B̂_p u│
                   │                       │
                   └─────────────────┐     │
                                     │     │
                    ┌────────────────▼─────▼─┐
                    │ Compute q̇ = ∇hᵀ ṗ     │
                    └────────┬────────────────┘
                             │
                             └─► Safety EKF measurement 2
                                  │
                    ┌─────────────▼────────────┐
                    │ EKF State Update         │
                    │ ξ = [q, q̇, F_q, B_q]ᵀ  │
                    └─────────────┬────────────┘
                                  │
                    ┌─────────────▼────────────┐
                    │ Extract estimates:       │
                    │ q̂, q̇̂, F̂_q, B̂_q, P      │
                    └─────────────┬────────────┘
                                  │
                    ┌─────────────▼────────────┐
                    │ Compute safety margin:   │
                    │ σₖ = cᵩ √(ℓᵀ P ℓ)       │
                    └─────────────┬────────────┘
                                  │
                    ┌─────────────▼────────────┐
                    │ Build HOCBF constraint:  │
                    │ B̂_q u ≥ rₖ              │
                    │ where rₖ = -F̂_q -       │
                    │   (λ₀+λ₁)q̇̂ - λ₀λ₁q̂ + σₖ│
                    └─────────────┬────────────┘
                                  │
                    ┌─────────────▼────────────┐
                    │ Solve QP:                │
                    │ min ||u - u_ref||²       │
                    │ s.t. B̂_q u ≥ rₖ         │
                    │      uₘᵢₙ ≤ u ≤ uₘₐₓ     │
                    └─────────────┬────────────┘
                                  │
                                  ▼
                             u_safe = [v, ω]
                                  │
                                  └─► Send to motor controller
```

---

## EKF Update Cycle

### Safety ULM-EKF

```
        Predict                    Update (q)              Update (q̇)
   ┌──────────────┐           ┌──────────────┐       ┌──────────────┐
   │              │           │              │       │              │
   │ q₊ = q + Tₛq̇│           │ K = P Hᵀ/S   │       │ K = P Hᵀ/S   │
   │   + Tₛ²/2 q̈ │  ────────►│              │──────►│              │
   │              │           │ x₊ = x + K·y │       │ x₊ = x + K·y │
   │ q̇₊ = q̇ + Tₛq̈│           │              │       │              │
   │              │           │ P₊=(I-KH)P   │       │ P₊=(I-KH)P   │
   │ F_q₊ = F_q   │           │              │       │              │
   │              │           └──────────────┘       └──────────────┘
   │ B_q₊ = B_q   │                 ▲                       ▲
   │              │                 │                       │
   │ P₊ = APAᵀ+Q │                 │                       │
   └──────────────┘                 │                       │
                              z = q_meas              z = q̇_meas
                              R = σ²_GP               R = σ²_q̇
```

---

## Component Responsibilities

### 1. ModelFreeCBF (`CBF_refactored.py`)
**Responsibilities**:
- Maintain GP model of obstacles
- Compute barrier h(p) and gradient ∇h
- Provide posterior variance σ²_GP
- Formulate HOCBF constraint
- Solve CLF-CBF QP

**Inputs**:
- LiDAR ranges/angles
- Query point p
- EKF estimates (q̂, q̇̂, F̂_q, B̂_q, P)
- Reference control u_ref

**Outputs**:
- Barrier value q
- Gradient ∇h
- Variance σ²_GP
- Safe control u_safe

---

### 2. SafetyULM_EKF (`cbf_Node_refactored.py`)
**Responsibilities**:
- Estimate safety dynamics parameters
- Fuse GP measurements with dynamics
- Provide covariance for safety margin

**State**: ξ = [q, q̇, F_q, B_q,v, B_q,ω]ᵀ

**Measurements**:
- q from GP: h(p)
- q̇ from: ∇h(p)ᵀ ṗ

**Outputs**:
- Estimates: q̂, q̇̂, F̂_q, B̂_q
- Covariance: P

---

### 3. PositionULM_EKF (`cbf_Node_refactored.py`)
**Responsibilities**:
- Estimate position velocity ṗ
- Provide velocity for q̇ measurement

**State**: [p_x, p_y, F_p,x, F_p,y, B_p (2×2)]

**Measurement**: p from odometry

**Outputs**:
- Position estimate p̂
- Velocity estimate ṗ = F̂_p + B̂_p u
- Input influence matrix B̂_p

---

## Timing Diagram

```
Time ─────────────────────────────────────────►

         ┌─────┐         ┌─────┐         ┌─────┐
LiDAR    │  k  │         │ k+1 │         │ k+2 │
         └──┬──┘         └──┬──┘         └──┬──┘
            │               │               │
            ├─► Parse       ├─► Parse       ├─► Parse
            │               │               │
Odom     ───┴───────────────┴───────────────┴──────
            │               │               │
Predict     ├─► Safety EKF  ├─► Safety EKF  ├─►
            │   Position    │   Position    │
            │               │               │
Measure     ├─► q, q̇        ├─► q, q̇        ├─►
            │               │               │
Update      ├─► EKF update  ├─► EKF update  ├─►
            │               │               │
QP Solve    ├─► u_safe      ├─► u_safe      ├─►
            │               │               │
Control  ───┴───────────────┴───────────────┴──────
         u[k]            u[k+1]          u[k+2]

         ◄─ Tₛ = 50ms ──►◄─ Tₛ = 50ms ──►
```

---

## Mathematical Pipeline

```
┌─────────────────────────────────────────────────────────┐
│                    GP Barrier Learning                   │
│                                                          │
│  Training: P = obstacle points, Y = -1 (obstacle label) │
│  Kernel: K = σ_f² exp(-||pᵢ - pⱼ||²/(2ℓ²))              │
│  Posterior: h(p) = 1 + k(p,P)ᵀ K⁻¹(Y-1)                 │
│  Gradient: ∇h = Σⱼ αⱼ k(p,pⱼ)(pⱼ-p)/ℓ²                 │
│  Variance: σ²(p) = k(p,p) - k(p,P)K⁻¹k(P,p)            │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│                  ULM Dynamics Estimation                 │
│                                                          │
│  Safety ULM: q̈ = F_q + B_q @ u                         │
│  State: ξ_q = [q, q̇, F_q, B_q,v, B_q,ω]ᵀ              │
│  Meas 1: q_k = h(p_k)  [noise: σ²_GP]                  │
│  Meas 2: q̇_k = ∇h(p_k)ᵀ ṗ_k  [noise: computed]        │
│                                                          │
│  Position ULM: ṗ = F_p + B_p @ u                        │
│  State: ξ_p = [p, F_p, B_p]ᵀ                            │
│  Meas: p_k from odometry                                 │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│             HOCBF Constraint Synthesis                   │
│                                                          │
│  Safety margin: σₖ = cᵩ √(ℓᵀ P ℓ)                       │
│    where ℓ = [λ₀λ₁, λ₀+λ₁, 1, uᵀ]ᵀ                      │
│                                                          │
│  HOCBF: B̂_q u ≥ rₖ                                      │
│    where rₖ = -F̂_q - (λ₀+λ₁)q̇̂ - λ₀λ₁q̂ + σₖ            │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│                    CLF-CBF QP                            │
│                                                          │
│  minimize    ½||u - u_ref||²_W                          │
│  subject to  B̂_q u ≥ rₖ          (safety)              │
│              u_min ≤ u ≤ u_max    (actuator limits)     │
│              (optional CLF)                              │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
                    u_safe = [v, ω]
```

---

## Legend

**Symbols**:
- `q`: Safety barrier value
- `q̇, q̈`: First and second time derivatives
- `F_q`: Lumped disturbance
- `B_q`: Input sensitivity vector
- `P`: EKF covariance matrix
- `σ_k`: Safety margin
- `λ₀, λ₁`: HOCBF parameters
- `c_q`: Confidence quantile
- `ℓ`: Margin computation vector
- `u`: Control input [v, ω]

**Operators**:
- `@`: Matrix multiplication
- `ᵀ`: Transpose
- `⁻¹`: Matrix inverse
- `∇`: Gradient
- `√`: Square root

---

## Implementation Notes

1. **All CuPy**: Core computations use CuPy for GPU acceleration
2. **QP Solver**: Convert to NumPy for qpsolvers (CPU-based)
3. **ROS2**: Callbacks handle sensor data, publish commands
4. **Sampling Time**: T_s = 0.05s (20 Hz typical)

For detailed equations, see the paper (CDC 2026).
