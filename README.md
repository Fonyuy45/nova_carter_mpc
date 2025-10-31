# Nova Carter MPC Controller

Model Predictive Control implementation for the NVIDIA Nova Carter differential drive robot.

## Project Overview

This project implements a kinematic MPC controller for trajectory tracking on the Nova Carter AMR platform, progressing from MATLAB simulation to ROS 2 integration with Isaac Sim.

**Status:** Phase 0 - Kinematic Model Implementation ✓

## Development Phases

- [x] **Phase 0:** Kinematic model and foundations
- [ ] **Phase 1:** MPC controller design (MATLAB)
- [ ] **Phase 2:** Simulation and validation
- [ ] **Phase 3:** ROS 2 integration
- [ ] **Phase 4:** Isaac Sim deployment
- [ ] **Phase 5:** Hardware testing

## Robot Specifications

- **Platform:** Segway RMP Lite 220 with NVIDIA Jetson AGX Orin
- **Wheel radius:** 0.140 m
- **Track width:** 0.414 m
- **Max speed:** 3.0 m/s
- **Max angular rate:** 2.0 rad/s

## Project Structure
```
nova_carter_mpc/
├── params/         # Robot parameters and constants
├── models/         # Kinematic and dynamic models
├── trajectory/     # Reference trajectory generation
├── controllers/    # MPC and control algorithms
├── tests/          # Unit tests and validation
└── simulations/    # Full simulation scripts
```

## Getting Started

### Prerequisites

- MATLAB R2021b or later
- Optimization Toolbox (for MPC - Phase 1+)
- (Optional) Control System Toolbox

### Setup

1. Clone or download this repository
2. Open MATLAB and navigate to the project root
3. Run `setup_project` to initialize paths
4. Run `tests/test_kinematic_model` to verify installation
```matlab
>> setup_project
>> cd tests
>> test_kinematic_model
```

## Current Implementation (Phase 0)

### Kinematic Model (Option A)

**State:** x = [x, y, θ]ᵀ  
**Input:** u = [v, ω]ᵀ  

**Dynamics:**
```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
θ̇ = ω
```

### Validated Features

✓ Forward/inverse kinematics  
✓ Wheel velocity conversions  
✓ Discrete-time integration (Euler & RK4)  
✓ Reference trajectory generation (circle, figure-8, line)  
✓ Constraint checking  

## Next Steps

- [ ] Implement nonlinear MPC controller
- [ ] Add cost function tuning utilities
- [ ] Implement state estimation (EKF)
- [ ] Add obstacle avoidance
- [ ] ROS 2 bridge integration

## Author

[Your Name]  
[LinkedIn/GitHub links]

## References

- [Nova Carter Documentation](https://developer.nvidia.com/isaac/nova-carter)
- [Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS)

## License

MIT License (or whatever you choose)