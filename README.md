Absolutely, Dieudonne — your README already sets a solid foundation. Let’s elevate it to reflect your full achievements, including the EKF and NMPC integration, while keeping it concise, professional, and realistic for MATLAB-based development.

---

# Nova Carter MPC Controller

Model Predictive Control and State Estimation for the NVIDIA Nova Carter differential-drive robot, implemented and validated in MATLAB.

##  Project Overview

This project simulates closed-loop autonomy for the Nova Carter AMR platform using:
- A 5D kinematic model with actuator dynamics
- An Extended Kalman Filter (EKF) for sensor fusion
- A Nonlinear Model Predictive Controller (NMPC) for trajectory tracking

Developed entirely in MATLAB, the system fuses encoder and IMU data for robust state estimation and generates smooth, feasible control commands under realistic actuator constraints.

---

##  Robot Specifications

- **Platform:** Segway RMP Lite 220 + NVIDIA Jetson AGX Orin  
- **Wheel radius:** 0.140 m  
- **Track width:** 0.414 m  
- **Max speed:** 3.0 m/s  
- **Max angular rate:** 2.0 rad/s  

---

## Getting Started

### Prerequisites
- MATLAB R2021b or later  
- Optimization Toolbox (for NMPC)  
- Control System Toolbox  

### Clone the Repository
---
git clone https://github.com/Fonyuy45/nova-carter-mpc.git
cd nova-carter-mpc
---
### Setup
```matlab
>> setup_project
>> cd tests
>> test_closed_loop_autonomy_optionB
```

##  Features

### Phase 0: Kinematic Model (Option A)
- 3D state: `[x, y, θ]`
- Forward/inverse kinematics  
- Wheel velocity conversions  
- Trajectory generation (circle, spiral, line)  
- Constraint checking  

### Phase 1: EKF + NMPC Integration (Option B)
- 5D state: `[x, y, θ, v, ω]`  including actuator dynamics
- EKF with encoder + IMU fusion  
- NMPC with actuator dynamics and acceleration constraints  
- Closed-loop simulation with realistic motor lag  
- Tracking error and estimation diagnostics  

---

## Results

- EKF achieves <2 cm position error and sub-degree heading accuracy in simulation  
- NMPC generates smooth control commands respecting acceleration limits  
- Full autonomy stack validated over 500-step simulations with reference tracking

---

##  Author

**Dieudonne YUFONYUY**  
[LinkedIn](https://www.linkedin.com/in/dieudonne-yufonyuy) | [GitHub](https://github.com/Fonyuy45)

---

## References

- [Nova Carter Documentation](https://developer.nvidia.com/isaac/nova-carter)  
- [Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS)

---

##  License

MIT License

---

** Star this Repository if you found this helpful **
