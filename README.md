# Nova Carter MPC Controller

Model Predictive Control and State Estimation for the NVIDIA Nova Carter differential-drive robot, implemented and validated in MATLAB.

## Tracking Rectangular Trajectory with curved corners


<img width="1344" height="898" alt="image" src="https://github.com/user-attachments/assets/2366af41-09da-45c5-9a44-917314317f27" />

<img width="1644" height="884" alt="image" src="https://github.com/user-attachments/assets/bf3766b3-b30e-4742-b9f6-eed4c3ca141c" />


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

---
### Setup
```matlab
>> git clone https://github.com/Fonyuy45/nova_carter_mpc
>> cd nova-carter-mpc

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

### Star this Repository if you found this helpful 
