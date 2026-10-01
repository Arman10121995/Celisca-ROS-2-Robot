# Localization Methods Tutorial

## Overview

This document describes the five localization methods implemented for R7.3, covering wheel odometry, probabilistic methods, scan matching, and visual localization.

## R7.3 Target Methods

| Method | Description | Implementation | Status |
|--------|-------------|----------------|---------|
| Wheel Dead Reckoning | Odometry integration for pose estimation | `dead_reckoning` | ✅ Integrated |
| AMCL | Adaptive Monte Carlo Localization | `amcl` | ✅ Integrated |
| ICP Scan Matching | Iterative Closest Point laser scan matching | `icp_localization` | ✅ Integrated |
| NDT Registration | Normal Distributions Transform registration | `ndt_localization` | ✅ Integrated |
| RGB-D SLAM | Visual SLAM-based localization | `rgbd_slam_localization` | ✅ Integrated |

## Implementation Details

### 1. Wheel Dead Reckoning (`dead_reckoning`)

- **Algorithm**: Integration of velocity commands into pose
- **Input**: Velocity commands (vx, wz)
- **Output**: Estimated pose (x, y, θ)
- **Mathematical Basis**: Kinematic integration

**Equations**:
```
# For zero angular velocity:
xₙ₊₁ = xₙ + vx·Δt·cos(θₙ)
yₙ₊₁ = yₙ + vx·Δt·sin(θₙ)
θₙ₊₁ = θₙ

# For non-zero angular velocity:
R = vx/wz
xₙ₊₁ = xₙ + R·(sin(θₙ + wz·Δt) - sin(θₙ))
yₙ₊₁ = yₙ - R·(cos(θₙ + wz·Δt) - cos(θₙ))
θₙ₊₁ = θₙ + wz·Δt
```

**Characteristics**:
- ✅ No external sensors required
- ❌ Unbounded error accumulation
- ❌ No correction mechanism
- ❌ Sensitive to wheel slippage

### 2. AMCL - Adaptive Monte Carlo Localization (`amcl`)

- **Algorithm**: Particle filter for probabilistic localization
- **Input**: Odometry `/odom/ground_truth`, LaserScan `/scan`
- **Output**: Pose estimate `/amcl_pose` (PoseWithCovarianceStamped)
- **Mathematical Basis**: Bayesian filtering with particle representation

**Algorithm**:
```
1. Prediction Step:
   - Sample new particles based on motion model + noise
   - Propagate particles using odometry data

2. Measurement Update:
   - Calculate measurement likelihood for each particle
   - Update particle weights based on scan matching

3. Resampling:
   - Systematically resample particles based on weights
   - Maintain particle diversity

4. Pose Estimation:
   - Compute weighted mean of particle positions
   - Estimate covariance from particle spread
```

**Equations**:
```
# Particle motion model:
xᵢₙ₊₁ = xᵢₙ + (vx + w₁)·Δt·cos(θᵢₙ + w₃·Δt)
yᵢₙ₊₁ = yᵢₙ + (vx + w₁)·Δt·sin(θᵢₙ + w₃·Δt)
θᵢₙ₊₁ = θᵢₙ + wz + w₃

# Measurement likelihood (simplified):
wᵢ = Π L(z|xᵢ) where L is laser scan likelihood
```

**Parameters**:
- `num_particles`: Number of particles (default: 100)
- `motion_noise`: Noise parameters for motion model
- `measurement_noise`: Noise parameters for sensor model

### 3. ICP Scan Matching (`icp_localization`)

- **Algorithm**: Iterative Closest Point for laser scan registration
- **Input**: LaserScan `/scan`, OccupancyGrid `/map`
- **Output**: Pose estimate `/icp_pose`
- **Mathematical Basis**: Least-squares point set registration

**Algorithm**:
```
For max_iterations:
    1. Transform current scan points to map frame
    2. Find closest points in reference scan
    3. Calculate transformation that minimizes error
    4. Apply transformation and check convergence
    5. If converged or max iterations reached, return estimate
```

**Equations**:
```
# Point transformation:
p_map = R·p_scan + t

# Error minimization:
minimize Σ ||p_map - p_ref||² over R, t
```

**Parameters**:
- `max_iterations`: Maximum ICP iterations (default: 10)
- `distance_threshold`: Maximum distance for point matching
- `convergence_threshold`: Stopping criterion

### 4. NDT Registration (`ndt_localization`)

- **Algorithm**: Normal Distributions Transform for scan matching
- **Input**: LaserScan `/scan`
- **Output**: Pose estimate `/ndt_pose`
- **Mathematical Basis**: Probabilistic point set registration

**Algorithm**:
```
1. Build NDT grid from reference scan
2. For each iteration:
   a. Transform current scan using current pose estimate
   b. Compute score and gradients using NDT
   c. Update pose using gradient descent
   d. Check convergence
```

**Equations**:
```
# NDT score for point p in cell with normal distribution N(μ, Σ):
S(p) = exp(-½·(p - μ)ᵀ·Σ⁻¹·(p - μ))

# Gradient for pose optimization:
∇θ S = ∂S/∂θ, ∇t S = ∂S/∂t
```

**Parameters**:
- `resolution`: Grid cell size (default: 0.5m)
- `max_iterations`: Maximum optimization iterations
- `convergence_threshold`: Stopping criterion

### 5. RGB-D SLAM Localization (`rgbd_slam_localization`)

- **Algorithm**: Visual feature-based localization
- **Input**: RGB-D data (simulated)
- **Output**: Pose estimate `/rgbd_slam_pose`
- **Mathematical Basis**: Feature matching and pose estimation

**Algorithm**:
```
1. Extract features from current RGB-D frame
2. Match features to map features
3. Estimate camera pose using PnP or similar method
4. Fuse with existing pose estimate
```

**Characteristics**:
- ✅ 3D localization
- ✅ Rich feature information
- ❌ Computationally intensive
- ❌ Requires good lighting/visibility

## Sensor/Map Assumptions and Strata

### Strata Definition

| Stratum | Sensor Type | Typical Configuration | Method Suitability |
|---------|-------------|------------------------|--------------------|
| Odometry | Wheel encoders, IMU | Position + velocity | Dead Reckoning, EKF |
| LiDAR-Map | 2D Laser + Map | Occupancy grid | AMCL, ICP, NDT |
| Visual | RGB-D Camera | Feature points | RGB-D SLAM |
| Hybrid | Multiple sensors | Sensor fusion | All methods |

### Group Characteristics

1. **Odometry Stratum**:
   - Sensors: Wheel odometry, IMU
   - Methods: Dead Reckoning, EKF variants
   - Strengths: High frequency, low latency
   - Weaknesses: Drift over time, no absolute reference

2. **LiDAR-Map Stratum**:
   - Sensors: 2D/3D LiDAR + pre-built map
   - Methods: AMCL, ICP, NDT
   - Strengths: Absolute positioning, environment-based
   - Weaknesses: Map dependency, computational cost

3. **Visual Stratum**:
   - Sensors: RGB-D cameras
   - Methods: RGB-D SLAM
   - Strengths: 3D information, rich features
   - Weaknesses: Lighting dependent, texture dependency

## Performance Metrics

### Evaluation Criteria

| Metric | Description | Importance |
|--------|-------------|------------|
| ATE (Absolute Trajectory Error) | Difference from ground truth trajectory | Critical |
| RPE (Relative Pose Error) | Frame-to-frame error accumulation | High |
| Convergence Rate | Time to achieve stable localization | Medium |
| Initialization Success | Ability to initialize from unknown pose | High |
| Relocalization Success | Ability to recover after tracking loss | High |
| Computational Cost | CPU/memory usage | Medium |
| Real-time Performance | Ability to run at sensor frequency | Critical |

### Measurement Protocol

1. **ATE/RPE Measurement**:
   ```python
   from robot_lab_benchmark.metrics import calculate_ate, calculate_rpe
   ate = calculate_ate(estimated_trajectory, ground_truth_trajectory)
   rpe = calculate_rpe(estimated_trajectory, ground_truth_trajectory)
   ```

2. **Convergence Measurement**:
   ```python
   # Track error over time
   convergence_time = first_time_error_below_threshold(estimated_pose, ground_truth, threshold=0.1)
   ```

## Usage Examples

### Running Individual Localization Methods

```bash
# Run wheel dead reckoning
ros2 run robot_lab_algorithms dead_reckoning

# Run AMCL particle filter
ros2 run robot_lab_algorithms amcl

# Run ICP scan matching
ros2 run robot_lab_algorithms icp_localization

# Run NDT registration
ros2 run robot_lab_algorithms ndt_localization

# Run RGB-D SLAM localization
ros2 run robot_lab_algorithms rgbd_slam_localization
```

### Configuration Files

```yaml
# Example AMCL configuration
amcl_node:
  ros__parameters:
    num_particles: 200
    motion_noise: [0.2, 0.2, 0.1]  # [x, y, theta]
    measurement_noise: [0.3, 0.3]    # [x, y]
    odom_topic: "/odom/ground_truth"
    scan_topic: "/scan"
    output_topic: "/amcl_pose"
```

## Initialization and Relocalization Protocol

### Initialization

1. **Known Pose**: Start with particle distribution centered at known pose
2. **Unknown Pose**: Distribute particles uniformly across map
3. **Partial Knowledge**: Distribute particles in likely regions

### Relocalization

1. **Detection**: Monitor error metrics and innovation covariance
2. **Triggering**: Initiate relocalization when error exceeds threshold
3. **Recovery**: Reinitialize particles in likely regions
4. **Verification**: Confirm successful relocalization before resuming

## Limitations and Future Work

- **Computational Efficiency**: Particle filters can be optimized with KLD sampling
- **Robustness**: Current implementations are basic; production systems need extensive tuning
- **Multi-Floor**: All methods assume 2D planar environments
- **Dynamic Environments**: Most methods assume static environments
- **Sensor Fusion**: Individual methods don't yet support multi-sensor fusion
- **Map Updates**: Methods don't handle changing environments

## References

- [AMCL ROS Implementation](https://docs.ros.org/en/noetic/api/amcl/html/)
- [ICP Algorithm](https://ieeexplore.ieee.org/document/166641)
- [NDT for Robot Localization](https://ieeexplore.ieee.org/document/1573803)
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) - For RGB-D SLAM comparison