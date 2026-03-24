# ELE3005M Robotics & Automation - Assessment 1

This project provides a comprehensive solution for the ELE3005M Robotics & Automation Assessment 1. It features a high-fidelity robotics simulation engine built with TypeScript and React, capable of solving complex kinematics and trajectory planning problems for a 6-DOF industrial manipulator.

## 🚀 Features

- **Dynamic Robot Configuration**: Automatically configures link lengths and kinematics sequences based on Student ID.
- **Forward Kinematics (FK)**: Matrix-based transformation chain for non-standard Rx-Ry-Rx arm and Rz-Rx-Rz wrist.
- **Inverse Kinematics (IK)**: Hybrid solver using numerical Jacobian-based IK for the arm and analytical spherical wrist IK.
- **Jacobian Analysis**: Real-time calculation of the 6x6 geometric Jacobian matrix.
- **Trajectory Planning**:
  - **Rectangular Path**: Constant linear velocity (0.1 m/s) with corner point interpolation.
  - **Time-Optimal Motion**: Joint-space trajectory with velocity (1 rad/s) and acceleration (1 rad/s²) constraints.
  - **Spiral Path**: Complex 3D spiral following (0.2 m/s) with 5.5 turns along the X-axis.
- **Interactive Dashboard**: Real-time plotting of joint angles, Cartesian positions, and velocities.

## 🛠️ Tech Stack

- **Framework**: React 18 with Vite
- **Language**: TypeScript
- **Mathematics**: Math.js for matrix operations and linear algebra
- **Visualization**: Recharts for high-performance data plotting
- **Styling**: Tailwind CSS (Apple-inspired design)

## 📂 Project Structure

- `src/services/RoboticsSolver.ts`: The core kinematics engine.
- `src/App.tsx`: The interactive dashboard and trajectory runner.
- `src/types.ts`: Shared type definitions.

## 📖 How to Use

1. **Enter Student ID**: The app defaults to `27047194`. Changing this will recalculate the robot's physical properties.
2. **Select Task**: Use the tabs to switch between the Rectangular, Time-Optimal, and Spiral tasks.
3. **Analyze Plots**: View the resulting joint and Cartesian data in the interactive charts.

---

*Developed for the ELE3005M Assessment 1.*
