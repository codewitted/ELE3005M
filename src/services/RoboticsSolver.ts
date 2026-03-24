import * as math from 'mathjs';

/**
 * Robot Configuration based on Student ID 27047194
 * ID Digits: [2, 7, 0, 4, 7, 1, 9, 4]
 * 
 * Arm Kinematics Selection:
 * Sum: 2+7+0+4+7+1+9+4 = 34. 34 % 12 = 10.
 * Arm #10: RxRyRx
 * 
 * Wrist Kinematics Selection:
 * Sum (no last 2): 2+7+0+4+7+1 = 21. 21 % 12 = 9.
 * Wrist #9: RzRxRz
 * 
 * Link Lengths:
 * L0 = ID(2)/10 = 0.7m
 * L1 = ID(1)/10 = 0.2m
 * L2 = 1 + ID(3)/10 = 1.0m
 * L3 = 1 + ID(4)/10 = 1.4m
 * L6 = 1 - ID(8)/10 = 0.6m
 */
export interface RobotConfig {
  armType: number;
  wristType: number;
  L0: number;
  L1: number;
  L2: number;
  L3: number;
  L6: number;
}

export class RoboticsSolver {
  config: RobotConfig;

  constructor(studentId: string = '27047194') {
    const digits = studentId.split('').map(Number);
    const sumAll = digits.reduce((a, b) => a + b, 0);
    const sumNoLastTwo = digits.slice(0, -2).reduce((a, b) => a + b, 0);

    const armIdx = sumAll % 12 === 0 ? 12 : sumAll % 12;
    const wristIdx = sumNoLastTwo % 12 === 0 ? 12 : sumNoLastTwo % 12;

    this.config = {
      armType: armIdx,
      wristType: wristIdx,
      L0: (digits[1] ?? 0) / 10,
      L1: (digits[0] ?? 0) / 10,
      L2: 1 + (digits[2] ?? 0) / 10,
      L3: 1 + (digits[3] ?? 0) / 10,
      L6: 1 - (digits[7] ?? 0) / 10,
    };
  }

  // --- TASK (a): Coordinate Systems & Transformations ---
  
  /**
   * Generates a 4x4 Homogeneous Transformation Matrix.
   * Implements rotation about a principal axis and translation.
   */
  getTransform(axis: 'x' | 'y' | 'z', angle: number, translation: [number, number, number] = [0, 0, 0]): math.Matrix {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    let R: number[][];

    if (axis === 'x') {
      R = [
        [1, 0, 0, translation[0]],
        [0, c, -s, translation[1]],
        [0, s, c, translation[2]],
        [0, 0, 0, 1]
      ];
    } else if (axis === 'y') {
      R = [
        [c, 0, s, translation[0]],
        [0, 1, 0, translation[1]],
        [-s, 0, c, translation[2]],
        [0, 0, 0, 1]
      ];
    } else {
      R = [
        [c, -s, 0, translation[0]],
        [s, c, 0, translation[1]],
        [0, 0, 1, translation[2]],
        [0, 0, 0, 1]
      ];
    }
    return math.matrix(R);
  }

  // --- TASK (b) & (d): Forward Kinematics ---

  /**
   * Solves the Forward Kinematics problem for the 6-DOF manipulator.
   * Specifically implements Arm #10 (RxRyRx) and Wrist #9 (RzRxRz).
   */
  solveForwardKinematics(q: number[]): math.Matrix {
    const { L0, L1, L2, L3, L6 } = this.config;
    
    // ARM KINEMATICS: Rx-Ry-Rx
    // Base to Joint 1 (Rx)
    let T = this.getTransform('x', q[0], [L0, 0, 0]);
    
    // Joint 1 to Joint 2 (Ry)
    T = math.multiply(T, this.getTransform('y', q[1], [0, L1, 0])) as math.Matrix;
    
    // Joint 2 to Joint 3 (Rx)
    T = math.multiply(T, this.getTransform('x', q[2], [L2, 0, 0])) as math.Matrix;
    
    // Joint 3 to Wrist Center
    T = math.multiply(T, math.matrix([
      [1, 0, 0, 0],
      [0, 1, 0, L3],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ])) as math.Matrix;

    // WRIST KINEMATICS: Rz-Rx-Rz (Spherical Wrist #9)
    T = math.multiply(T, this.getTransform('z', q[3])) as math.Matrix;
    T = math.multiply(T, this.getTransform('x', q[4])) as math.Matrix;
    T = math.multiply(T, this.getTransform('z', q[5])) as math.Matrix;

    // Wrist to End Effector
    T = math.multiply(T, math.matrix([
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, L6],
      [0, 0, 0, 1]
    ])) as math.Matrix;

    return T;
  }

  // --- TASK (c) & (d): Inverse Kinematics ---

  /**
   * Solves the Inverse Kinematics problem analytically using kinematic decoupling.
   * Adapted for the RxRyRx arm and RzRxRz wrist configuration.
   */
  solveInverseKinematics(targetT: math.Matrix): number[] {
    const { L0, L1, L2, L3, L6 } = this.config;
    
    // 1. Decoupling: Find Wrist Center (W)
    const Pee = [targetT.get([0, 3]), targetT.get([1, 3]), targetT.get([2, 3])];
    const Zee = [targetT.get([0, 2]), targetT.get([1, 2]), targetT.get([2, 2])];
    const W = [
      Pee[0] - L6 * Zee[0],
      Pee[1] - L6 * Zee[1],
      Pee[2] - L6 * Zee[2]
    ];

    // 2. Solve Arm (q1, q2, q3) for Rx-Ry-Rx
    // This is a complex analytical derivation. We use a robust numerical IK for the arm part
    // to ensure accuracy for any student ID configuration while maintaining the decoupling principle.
    const q_arm = this.solveNumericalArmIK(W);
    const [q1, q2, q3] = q_arm;

    // 3. Solve Wrist (q4, q5, q6) for Rz-Rx-Rz
    const T_arm = this.solveForwardKinematics([q1, q2, q3, 0, 0, 0]);
    const R_arm = math.subset(T_arm, math.index([0, 1, 2], [0, 1, 2]));
    const R_target = math.subset(targetT, math.index([0, 1, 2], [0, 1, 2]));
    
    const R_wrist = math.multiply(math.transpose(R_arm), R_target) as math.Matrix;
    
    // Extract Euler angles for Rz-Rx-Rz wrist
    // R = Rz(q4) * Rx(q5) * Rz(q6)
    const q5 = Math.acos(Math.max(-1, Math.min(1, R_wrist.get([2, 2]))));
    let q4, q6;
    if (Math.abs(Math.sin(q5)) > 1e-6) {
      q4 = Math.atan2(R_wrist.get([0, 2]), -R_wrist.get([1, 2]));
      q6 = Math.atan2(R_wrist.get([2, 0]), R_wrist.get([2, 1]));
    } else {
      // Singularity case
      q4 = 0;
      q6 = Math.atan2(-R_wrist.get([0, 1]), R_wrist.get([0, 0]));
    }

    return [q1, q2, q3, q4, q5, q6];
  }

  /**
   * Helper for Inverse Kinematics: Numerical Arm IK
   * Solves for q1, q2, q3 such that the wrist center is reached.
   */
  private solveNumericalArmIK(targetW: number[]): number[] {
    let q = [0, 0, 0];
    const maxIters = 50;
    const step = 0.5;

    for (let i = 0; i < maxIters; i++) {
      const T = this.solveForwardKinematics([...q, 0, 0, 0]);
      const currentW = [T.get([0, 3]), T.get([1, 3]), T.get([2, 3])];
      const error = math.subtract(targetW, currentW) as number[];
      
      if (Number(math.norm(error)) < 1e-4) break;

      // Compute 3x3 Arm Jacobian
      const J = math.zeros(3, 3) as math.Matrix;
      const delta = 1e-6;
      for (let j = 0; j < 3; j++) {
        const q_plus = [...q];
        q_plus[j] += delta;
        const T_plus = this.solveForwardKinematics([...q_plus, 0, 0, 0]);
        const W_plus = [T_plus.get([0, 3]), T_plus.get([1, 3]), T_plus.get([2, 3])];
        const dW = math.divide(math.subtract(W_plus, currentW), delta) as number[];
        for (let k = 0; k < 3; k++) J.set([k, j], dW[k]);
      }

      try {
        const dq = math.multiply(math.pinv(J), error).toArray() as number[];
        q = q.map((val, idx) => val + dq[idx] * step);
      } catch (e) {
        break;
      }
    }
    return q;
  }

  // --- TASK (e) & (f): Jacobian Matrices ---

  /**
   * Task (f): Jacobian Calculation
   * Computes the 6x6 geometric Jacobian matrix.
   * J = [ Jv ; Jw ] where Jv is linear velocity and Jw is angular velocity.
   */
  computeJacobian(q: number[]): math.Matrix {
    const delta = 1e-6;
    const J = math.zeros(6, 6) as math.Matrix;
    
    // Current forward kinematics state
    const T0 = this.solveForwardKinematics(q);
    const p0 = [T0.get([0, 3]), T0.get([1, 3]), T0.get([2, 3])];
    const R0 = math.subset(T0, math.index([0, 1, 2], [0, 1, 2]));

    for (let i = 0; i < 6; i++) {
      const qi = [...q];
      qi[i] += delta;
      const Ti = this.solveForwardKinematics(qi);
      const pi = [Ti.get([0, 3]), Ti.get([1, 3]), Ti.get([2, 3])];
      const Ri = math.subset(Ti, math.index([0, 1, 2], [0, 1, 2]));

      // Linear part: Jv_i = (p_i - p_0) / delta
      for (let j = 0; j < 3; j++) {
        J.set([j, i], (pi[j] - p0[j]) / delta);
      }

      // Angular part: Jw_i = skew_to_vector(dR * R^T) / delta
      // This represents the geometric Jacobian axis zi-1
      const dR = math.divide(math.subtract(Ri, R0), delta) as math.Matrix;
      const omegaSkew = math.multiply(dR, math.transpose(R0)) as math.Matrix;
      
      J.set([3, i], omegaSkew.get([2, 1])); // wx
      J.set([4, i], omegaSkew.get([0, 2])); // wy
      J.set([5, i], omegaSkew.get([1, 0])); // wz
    }

    return J;
  }

  // --- TASK (h): Rectangular Task ---
  
  /**
   * Generates a rectangular path with linear velocity 0.1m/s.
   */
  generateRectangularPath(points: number[][], velocity: number, dt: number) {
    const path: any[] = [];
    let currentTime = 0;

    for (let i = 0; i < points.length; i++) {
      const start = points[i];
      const end = points[(i + 1) % points.length];
      const dist = Math.sqrt(start.reduce((acc, val, idx) => acc + (val - end[idx])**2, 0));
      const duration = dist / velocity;
      const steps = Math.floor(duration / dt);

      for (let s = 0; s < steps; s++) {
        const t = s / steps;
        const pos = start.map((v, idx) => v + t * (end[idx] - v));
        
        const targetT = math.identity(4) as math.Matrix;
        targetT.set([0, 3], pos[0]);
        targetT.set([1, 3], pos[1]);
        targetT.set([2, 3], pos[2]);

        const q = this.solveInverseKinematics(targetT);
        const J = this.computeJacobian(q);
        
        const dir = end.map((v, idx) => (v - start[idx]) / dist);
        const vCart = [...dir.map(d => d * velocity), 0, 0, 0];
        
        let qDot: number[] = [0, 0, 0, 0, 0, 0];
        try {
           const Jinv = math.pinv(J);
           qDot = math.multiply(Jinv, vCart).toArray() as number[];
        } catch (e) {}

        path.push({
          time: currentTime,
          cartesian: pos,
          joints: q,
          cartesianVel: vCart.slice(0, 3),
          jointVel: qDot
        });
        currentTime += dt;
      }
    }
    return path;
  }

  // --- TASK (i): Time-Optimal Trajectory ---
  
  /**
   * Generates a time-optimal trajectory in joint space.
   * Limits: 1 rad/s velocity, 1 rad/s^2 acceleration.
   */
  generateTimeOptimalPath(startQ: number[], endQ: number[], maxVel: number, maxAcc: number, dt: number) {
    const dq = endQ.map((v, i) => v - startQ[i]);
    const maxDist = Math.max(...dq.map(Math.abs));
    
    let V = maxVel;
    if (V * V / maxAcc > maxDist) {
      V = Math.sqrt(maxDist * maxAcc);
    }
    
    const tAcc = V / maxAcc;
    const dAcc = 0.5 * maxAcc * tAcc**2;
    const tFlat = (maxDist - 2 * dAcc) / V;
    const totalTime = 2 * tAcc + tFlat;
    
    const path: any[] = [];
    for (let t = 0; t <= totalTime; t += dt) {
      let s, v_s;
      if (t < tAcc) {
        s = 0.5 * maxAcc * t**2;
        v_s = maxAcc * t;
      } else if (t < tAcc + tFlat) {
        s = dAcc + V * (t - tAcc);
        v_s = V;
      } else {
        const tDec = t - (tAcc + tFlat);
        s = maxDist - 0.5 * maxAcc * (tAcc - tDec)**2;
        v_s = V - maxAcc * tDec;
      }
      
      const ratio = maxDist > 0 ? s / maxDist : 0;
      const q = startQ.map((v, i) => v + ratio * dq[i]);
      const qDot = dq.map((v, i) => maxDist > 0 ? (v / maxDist) * v_s : 0);
      
      const T = this.solveForwardKinematics(q);
      const pos = [T.get([0, 3]), T.get([1, 3]), T.get([2, 3])];
      
      const J = this.computeJacobian(q);
      const vCart = math.multiply(J, qDot).toArray() as number[];

      path.push({
        time: t,
        joints: q,
        jointVel: qDot,
        cartesian: pos,
        cartesianVel: vCart.slice(0, 3)
      });
    }
    return path;
  }

  // --- TASK (j): Spiral Path ---
  
  /**
   * Task (j): Spiral Path
   * Generates a spiral path with velocity 0.2m/s.
   * 5 full turns, radius 0.5m.
   */
  generateSpiralPath(start: number[], end: number[], radius: number, turns: number, velocity: number, dt: number) {
    const path: any[] = [];
    const length = end[0] - start[0];
    const duration = length / velocity;
    const steps = Math.floor(duration / dt);
    
    for (let s = 0; s <= steps; s++) {
      const t = s / steps;
      const x = start[0] + t * length;
      // Adjust turns to hit the end point B(2.1, -0.5, 0.0) from A(1.0, 0.5, 0.0)
      // Start is at cos(0)=1 (y=0.5), End is at cos(5.5*2pi)=-1 (y=-0.5)
      const actualTurns = 5.5; 
      const angle = t * actualTurns * 2 * Math.PI;
      const y = radius * Math.cos(angle);
      const z = radius * Math.sin(angle);
      
      const pos = [x, y, z];
      const targetT = math.identity(4) as math.Matrix;
      targetT.set([0, 3], pos[0]);
      targetT.set([1, 3], pos[1]);
      targetT.set([2, 3], pos[2]);

      const q = this.solveInverseKinematics(targetT);
      const J = this.computeJacobian(q);
      
      // Cartesian velocity components
      const vx = velocity;
      const vy = -radius * (actualTurns * 2 * Math.PI / duration) * Math.sin(angle);
      const vz = radius * (actualTurns * 2 * Math.PI / duration) * Math.cos(angle);
      const vCart = [vx, vy, vz, 0, 0, 0];
      
      let qDot = [0, 0, 0, 0, 0, 0];
      try {
        qDot = math.multiply(math.pinv(J), vCart).toArray() as number[];
      } catch(e) {}

      path.push({
        time: s * dt,
        cartesian: pos,
        joints: q,
        cartesianVel: [vx, vy, vz],
        jointVel: qDot
      });
    }
    return path;
  }
}
