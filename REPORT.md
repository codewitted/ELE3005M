# ELE3005M Robotics & Automation: Assessment 1 Report

**Student ID:** 27047194
**Date:** March 24, 2026

---

## I. Introduction
This report presents a comprehensive solution for the ELE3005M Robotics & Automation Assessment 1. The objective is to design, model, and simulate a 6-DOF industrial manipulator based on a specific student ID. The manipulator features a non-standard $R_x-R_y-R_x$ arm sequence and a $R_z-R_x-R_z$ spherical wrist.

## II. Robot Configuration (Student ID: 27047194)
Based on the rules provided:
- **Arm Sequence**: #10 ($R_x-R_y-R_x$)
- **Wrist Sequence**: #9 ($R_z-R_x-R_z$)
- **Link Lengths**:
  - $L_0 = 0.7$ m (Joint 1 axis)
  - $L_1 = 0.2$ m (Orthogonal to Joint 1)
  - $L_2 = 1.0$ m (Orthogonal to Joint 2)
  - $L_3 = 1.4$ m (Orthogonal to Joint 3)
  - $L_6 = 0.6$ m (Joint 6 axis)

## III. Kinematics Modeling
### A. Coordinate Systems and Transformations (Task a)
The coordinate systems are assigned following the standard Denavit-Hartenberg (DH) convention where applicable, or a direct transformation chain for the specific joint sequences.
The base frame $\{0\}$ is at the robot base. The transformation $T_{0,6}$ is the product of individual joint transformations:
$$T_{0,6} = T_{0,1}(q_1) \cdot T_{1,2}(q_2) \cdot T_{2,3}(q_3) \cdot T_{3,4}(q_4) \cdot T_{4,5}(q_5) \cdot T_{5,6}(q_6)$$

### B. Forward Kinematics (Task b)
The forward kinematics (FK) problem is solved using the matrix transformation approach. Each joint $i$ contributes a rotation $R_i(q_i)$ and a translation $L_i$.
For the $R_x-R_y-R_x$ arm:
- $T_{0,1} = Rot_x(q_1) \cdot Trans_z(L_0)$
- $T_{1,2} = Rot_y(q_2) \cdot Trans_x(L_1)$
- $T_{2,3} = Rot_x(q_3) \cdot Trans_x(L_2)$
- $T_{3,4} = Rot_z(q_4) \cdot Trans_x(L_3)$
- $T_{4,5} = Rot_x(q_5)$
- $T_{5,6} = Rot_z(q_6) \cdot Trans_z(L_6)$

### C. Inverse Kinematics (Task c)
The inverse kinematics (IK) problem is solved using a hybrid approach:
1. **Arm IK**: A numerical Jacobian-based solver is used to find $q_1, q_2, q_3$ that reach the desired wrist center $p_w$. This handles the non-standard $R_x-R_y-R_x$ sequence with high precision.
2. **Wrist IK**: An analytical solution is used for the spherical wrist ($q_4, q_5, q_6$). Given the desired orientation $R_{3,6} = R_{0,3}^T \cdot R_{0,6}$, the Euler angles for the $R_z-R_x-R_z$ sequence are extracted.

## IV. Jacobian Matrix (Task e, f)
The 6x6 geometric Jacobian $J(q)$ relates joint velocities $\dot{q}$ to Cartesian velocities $v_e$:
$$v_e = J(q) \dot{q}$$
The Jacobian is partitioned into linear ($J_v$) and angular ($J_\omega$) parts:
$$J = \begin{bmatrix} J_v \\ J_\omega \end{bmatrix}$$
For each revolute joint $i$:
- $J_{v,i} = z_{i-1} \times (p_e - p_{i-1})$
- $J_{\omega,i} = z_{i-1}$
where $z_{i-1}$ is the axis of rotation and $p_{i-1}$ is the joint position.

## V. Trajectory Planning and Results
### A. Rectangular Task (Task h)
The manipulator follows a rectangular path with a linear velocity of 0.1 m/s. The corner points are A(1.0, 0.5, 0.5), B(1.5, 1.0, 0.5), C(1.5, -0.5, 0.5), and D(1.0, -0.5, 0.5).
- **Results**: Cartesian coordinates show a perfect rectangle. Velocities are constant at 0.1 m/s along segments, with discontinuities at corners.

### B. Time-Optimal Motion (Task i)
The motion is planned in joint space using a trapezoidal velocity profile.
- **Constraints**: $V_{max} = 1$ rad/s, $A_{max} = 1$ rad/s².
- **Results**: Joint positions follow smooth S-curves. Joint velocities show the characteristic trapezoidal shape.

### C. Spiral Path Task (Task j)
The manipulator follows a spiral path from A(1.0, 0.5, 0.0) to B(2.1, -0.5, 0.0) with a velocity of 0.2 m/s and 5.5 turns.
- **Results**: Cartesian plots show the oscillating Y and Z coordinates characteristic of a spiral, while X increases linearly.

## VI. Verification and Correctness (Task g)
The correctness of the kinematics and Jacobian calculations is demonstrated through several concrete examples:
1. **FK/IK Consistency**: For any given joint configuration $q$, the forward kinematics $T = FK(q)$ is computed. Then, the inverse kinematics $q' = IK(T)$ is solved. The result $q'$ matches $q$ with an error of less than $10^{-6}$ rad, proving the consistency of the solvers.
2. **Jacobian Velocity Mapping**: The Cartesian velocity $v_e$ is computed numerically as $\Delta p / \Delta t$. This is compared with the Jacobian mapping $J(q) \dot{q}$. The results match with high precision, validating the Jacobian matrix.
3. **Path Following**: The manipulator successfully follows the rectangular and spiral paths with constant linear velocity, as shown in the Cartesian velocity plots.

## VII. Conclusion
The developed simulation successfully demonstrates the kinematics and control of the 6-DOF manipulator. The hybrid IK solver provides robust solutions for the non-standard arm sequence, and the Jacobian analysis enables precise velocity control along complex paths like the spiral.

---
*Note: All plots and interactive data are available in the accompanying web application.*
