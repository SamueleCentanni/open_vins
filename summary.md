# Static Initializer — Detailed Explanation

This document explains the static initializer implemented in `ov_init/src/static/StaticInitializer.cpp`.
It contains explicit formulas, short plain-English explanations next to complex expressions, and a final section
highlighting the main differences with the dynamic initializer.

---

## 1. Purpose and high-level idea

The static initializer detects a (brief) stationary period and uses averaged IMU samples to initialize:

- the rotation from global (gravity-aligned) frame to the IMU frame;
- the IMU biases (gyroscope bias and accelerometer bias);
- a conservative initial covariance for the IMU state used by the VIO filter.

This works when the platform is (nearly) stationary for a short period so gravity can be observed and averaged.

---

## 2. Window selection and sample statistics

The code splits IMU data into two half-windows within the configured initialization time window $T_{\text{win}}$:

Let $t_{\text{newest}}$ be the timestamp of the newest IMU sample and consider the window
$$
t \in [t_{\text{newest}} - T_{\text{win}},\; t_{\text{newest}}].
$$

Define the two half-windows:

- newest half-window (1→0):
$$
W_{1\to0} = \{\,t: t \in (t_{\text{newest}} - 0.5 T_{\text{win}},\; t_{\text{newest}}]\,\}
$$
- previous half-window (2→1):
$$
W_{2\to1} = \{\,t: t \in (t_{\text{newest}} - T_{\text{win}},\; t_{\text{newest}} - 0.5 T_{\text{win}}]\,\}
$$

For each half-window the initializer computes sample means and sample standard deviations of the accelerometer and (for the older window) the gyroscope:

Mean (accelerometer) in a window $W$:
$$
\bar{\mathbf{a}} = \frac{1}{n}\sum_{k=1}^n \mathbf{a}_k
$$
Simple explanation: average of measured accelerations; if stationary it approximates $R^\top g + b_a$ (gravity rotated + accel bias).

Sample standard deviation (accelerometer):
$$
s_a = \sqrt{\frac{1}{n-1}\sum_{k=1}^n \|\mathbf{a}_k - \bar{\mathbf{a}}\|^2}
$$
Plain English: measures vibration / motion in the window; low $s_a$ means the device is approximately stationary.

The code enforces three checks:
1. Both half-windows must contain at least two samples.
2. The newest half-window standard deviation $s_{a,1\to0}$ must be above or below thresholds depending on `wait_for_jerk`.
3. The older half-window must be below threshold if waiting for a jerk (ensures the device transitioned from stationary to motion or vice versa).

Rationale: these checks ensure reliable estimation of gravity (from averaged acceleration) and reliable bias estimation.

---

## 3. Rotation recovery: estimate orientation that aligns gravity

From the older half-window $W_{2\to1}$ the code computes the mean accelerometer:
$$
\bar{\mathbf{a}}_{2\to1} = \frac{1}{n}\sum_{k\in W_{2\to1}} \mathbf{a}_k
$$

Define the normalized axis (estimated gravity direction in IMU frame):
$$
\mathbf{z} = \frac{\bar{\mathbf{a}}_{2\to1}}{\|\bar{\mathbf{a}}_{2\to1}\|}
$$

We want a rotation $R_{G\to I}$ (global → IMU) such that the gravity direction in the global frame (conventionally $\mathbf{g}_{G} = [0,0,g]^\top$) maps to the measured direction in the IMU:
$$
R_{G\to I}\,\mathbf{g}_G \approx \bar{\mathbf{a}}_{2\to1}
$$

Practical implementation: the code builds an orthonormal rotation matrix `Ro` using Gram–Schmidt with $\mathbf{z}$ as the first axis. Conceptually:

1. Take $\mathbf{z}$ as the estimated local z-axis (aligns with measured accel mean).  
2. Choose an arbitrary global x (or use a stable reference) and apply Gram–Schmidt to form orthonormal columns $[\mathbf{x},\mathbf{y},\mathbf{z}]$.  
3. Set $R_{G\to I} = [\mathbf{x},\mathbf{y},\mathbf{z}]$.

Short explanation of Gram–Schmidt (two-step variant):
Given target $\mathbf{z}$ and an arbitrary non-collinear vector $\mathbf{v}$:
$$
\mathbf{x} = \frac{\mathbf{v} - (\mathbf{v}\cdot\mathbf{z})\mathbf{z}}{\|\mathbf{v} - (\mathbf{v}\cdot\mathbf{z})\mathbf{z}\|},\qquad
\mathbf{y} = \mathbf{z} \times \mathbf{x}
$$

Why this works: it makes a right-handed orthonormal triad with $\mathbf{z}$ matching the measured gravity direction.

The quaternion is then computed: $\mathbf{q}_{G\to I} =\text{rot\_2\_quat}(R_{G\to I})$ (a compact orientation representation used in the state vector).

---

## 4. Bias estimation (gyroscope and accelerometer)

The code computes simple bias estimates from window averages:

- Gyroscope bias estimate (assumes mean rotational rate ≈ bias during stationary):
$$
\mathbf{b}_g = \bar{\boldsymbol{\omega}}_{2\to1} = \frac{1}{n}\sum_{k\in W_{2\to1}} \boldsymbol{\omega}_k
$$
Plain English: if the device is not rotating, the mean gyro reading equals the gyro bias.

- Accelerometer bias estimate: remove gravity (expressed in IMU frame) from the mean accelerometer:
$$
\mathbf{b}_a = \bar{\mathbf{a}}_{2\to1} - R_{G\to I}\,\mathbf{g}_G
$$

Explanation: the accelerometer mean equals $R_{G\to I}\,\mathbf{g}_G + \mathbf{b}_a$ when stationary (gravity plus bias); rearrange to solve for $\mathbf{b}_a$.

Note: $\mathbf{g}_G = [0,0,g]^\top$ and `params.gravity_mag` supplies $g$ (positive scalar magnitude). The code uses the quaternion->rotation mapping to compute $R_{G\to I}$.

---

## 5. IMU state vector layout and assignment

The initializer packs the IMU initial state into a 16-element vector `imu_state` that is written to `t_imu`:

Typically `imu_state` layout in this file (indices are zero-based):

| Block | Indices | Quantity |
|---:|:---:|---|
| quaternion (q_{G\to I}) | 0..3 | orientation (4 elements) |
| position | 4..6 | position (3 elements) — often left zero here |
| velocity | 7..9 | velocity (3 elements) — set to zero for static init |
| gyro bias (b_g) | 10..12 | gyroscope bias (3 elements) |
| accel bias (b_a) | 13..15 | accelerometer bias (3 elements) |

The code sets the quaternion to `q_GtoI` and biases to `b_g`, `b_a`. Velocity and position remain zero or as previously configured.

`t_imu->set_value(imu_state)` and `t_imu->set_fej(imu_state)` store the nominal value and the FEJ (first-estimate Jacobian) copy used by the filter.

---

## 6. Covariance initialization and gauge considerations

After filling the state the initializer sets a conservative diagonal covariance matrix:

Let $P$ be the covariance; the code sets (values in meters/radians):
$$
P = \sigma_0^2 I_{16},\quad \sigma_0 = 0.02
$$
and then adjusts blocks for specific components (example values used in the code):

$$
P_{q,q} = (0.02)^2 I_{3},\quad P_{p,p} = (0.05)^2 I_3,\quad P_{v,v} = (0.01)^2 I_3
$$

Short explanation: assign small uncertainty to orientation and larger to position, velocity depending on prior knowledge; these choices are heuristic and conservative.

Observability / gauge: VIO has 4 unobservable degrees of freedom (global yaw and global position). The comment in the code notes that the authors choose to fix yaw and position (set them as known) at startup to remove singularities. Fixing these degrees of freedom prevents singular covariance (non-invertible information matrix) when the filter starts.

---

## 7. Why this works, limitations, and practical notes

- Works when the platform is actually stationary or nearly stationary so gravity dominates the accelerometer mean.  
- Very cheap computationally (averages + Gram–Schmidt + simple covariance set).  
- It provides robust initial orientation and bias estimates but not accurate initial velocities or feature depths — these remain to be estimated by later visual-inertial processing.

Limitations:
- If the device is moving during the windows (significant acceleration or rotation), the averages will be biased and initialization may be incorrect. The code attempts to detect that using the sample standard deviation thresholds.
- There is no use of camera features or full preintegration; static initializer is not intended for fully dynamic start-up.

---

## 8. Key formulas (compact list) with short explanations

- Window mean (accelerometer):
$$\bar{\mathbf{a}} = \frac{1}{n}\sum_{k=1}^n \mathbf{a}_k$$
  (average measured acceleration; approximates rotated gravity + bias when stationary)

- Sample standard deviation (accelerometer):
$$s_a = \sqrt{\frac{1}{n-1}\sum_{k=1}^n \|\mathbf{a}_k - \bar{\mathbf{a}}\|^2}$$
  (used to detect motion / jerk)

- Gravity direction estimate (normalized):
$$\mathbf{z} = \frac{\bar{\mathbf{a}}}{\|\bar{\mathbf{a}}\|}$$
  (unit vector in IMU frame pointing along measured gravity)

- Accelerometer bias estimate:
$$\mathbf{b}_a = \bar{\mathbf{a}} - R_{G\to I}\,\mathbf{g}_G$$
  (remove gravity expressed in IMU frame from mean to get bias)

- Gyroscope bias estimate:
$$\mathbf{b}_g = \bar{\boldsymbol{\omega}}$$
  (mean measured angular rate over the window)

---

## 9. Comparison: Static vs Dynamic initializer (main differences)

1. Assumptions
  - Static: requires a stationary or near-stationary period so gravity can be observed by averaging.
  - Dynamic: assumes motion and uses both IMU preintegration and visual tracks to solve a linear system and then refine with nonlinear optimisation.

2. Inputs used
  - Static: only IMU samples (averages); camera information is not needed for the initial orientation/bias estimates.
  - Dynamic: requires both IMU and camera feature tracks; forms $A\mathbf{x}=\mathbf{b}$ using reprojection constraints and preintegrated IMU.

3. Outputs
  - Static: orientation (gravity-aligned), gyro bias, accel bias, conservative covariance; velocity and feature depths are left uninitialized or zero.
  - Dynamic: gravity direction, initial velocity, feature 3D positions, full state for multiple frames; often followed by Ceres-based MLE refinement and covariance estimation.

4. Computational complexity
  - Static: O(n) over the window to compute means and variances (very cheap).
  - Dynamic: significantly heavier — preintegration, assembly of a large linear system, root-finding (Dongsi polynomial), and a nonlinear optimisation (Ceres).

5. Robustness and applicability
  - Static: robust if and only if a genuine stationary interval exists; fails or biases when motion is present.
  - Dynamic: designed to initialize under motion (excited trajectories) but requires enough visual information (tracked features) and careful time alignment.

6. Use in pipeline
  - Static: fast fallback when a stationary period is available (quick warm start).  
  - Dynamic: main initializer for bootstrap from motion; produces richer initial guesses usable directly by the VIO estimator.

---

## 10. Suggested next actions (if you want more):

- I can insert these formulas and short comments directly into `StaticInitializer.cpp` as inline documentation.  
- I can add a compact unit-test / runnable snippet that demonstrates static initialization on a short recorded IMU segment.  
- I can annotate the code with exact indices for `imu_state` and link to where `t_imu` is consumed by the filter.

If you want any of the above, tell me which and I'll apply the changes.

# VIO Initialization Summary

**Reference paper:** https://tdongsi.github.io/download/pubs/2011_VIO_Init_TR.pdf

---

## 1. Data Collection & Validation

- Collect camera frames and IMU data within a fixed window, determined by:
  $\text{window} = t_{\text{newest\_cam}} - \Delta t_{\text{init\_window}}$
- Discard any IMU data collected before this window.
- Iterate through all tracked features and count the valid ones.
- Check whether there are enough valid features to proceed.
- Determine IMU biases — low effort since they are specified in the config file.
- Check whether the rotational excitation is sufficient.

---

## 2. IMU Pre-integration

Pre-integrate the IMU measurements between $I_0$ (the initial IMU reference frame) and each $I_i$ (the IMU frame at time $t_i$), and also between consecutive frames $I_i$ and $I_{i+1}$. This yields:

- $\alpha_{I_0 \to I_i}$ — how the position has changed (pre-integrated position increment)
- $R_{I_0 \to I_i}$ — how the rotation has changed
- $\beta_{I_0 \to I_i}$ — how the velocity has changed

---

## 3. Building the Linear System $Ax = b$

### 3.1 Projection matrix

Compute $H_{\text{proj}}$ to model how a 3D point projects into the 2D image. This is used to build the linear system $Ax = b$.

### 3.2 Unknown state vector $x$

The unknown vector $x$ contains:
- The **gravity direction** $g$ (then used to rotate the world frame so gravity points along $-z$)
- The **initial IMU velocity** $v_{I_0}$
- The **3D positions** of features $p_{f}$ (expressed in the initial IMU frame $I_0$)

### 3.3 Geometric observation model

A feature point $p_f$ observed from camera $C$ at time $i$ — denoted ${}^{C_i}p_f$ — can be expressed in the initial IMU frame $I_0$ as:

$${}^{C_i}p_f = R_{I \to C} \cdot R_{I_i \to I_0}^{\top} \cdot \left({}^{I_0}p_f - {}^{I_0}p_{I_i}\right) + p_{IC}$$

where:
- ${}^{I_0}p_f$ is the 3D position of the feature in frame $I_0$
- ${}^{I_0}p_{I_i}$ is the IMU position at time $i$ relative to $I_0$
- $p_{IC}$ is the IMU-camera translation (extrinsics, constant)

### 3.4 IMU kinematic model

Using the uniformly accelerated motion equations integrated from IMU data, the IMU position ${}^{I_0}p_{I_i}$ is:

$${}^{I_0}p_{I_i} = v_{I_0} \Delta t + \frac{1}{2} g \Delta t^2 + \alpha_{I_0 \to I_i}$$

Substituting into the observation model:

$${}^{C_i}p_f = R_{I \to C} \cdot R_{I_0 \to I_i}^{\top} \cdot \left({}^{I_0}p_f - v_{I_0} \Delta t - \frac{1}{2} g \Delta t^2 - \alpha_{I_0 \to I_i}\right) + p_{IC}$$

### 3.5 Building the $H_i$ matrix (the $A$ matrix)

In code: $Y = H_{\text{proj}} \cdot R_{I \to C} \cdot R_{I_0 \to I_k}$

The row block $H_i$ is assembled by reading off the coefficients of each unknown from the kinematic formula:

| Term | Column block | Expression |
|------|-------------|------------|
| Feature position ${}^{I_0}p_f$ | $H_i[\cdot,\, \text{pos}] = Y$ | The coefficient is simply $Y$ |
| Initial velocity $v_{I_0}$ | $H_i[\cdot,\, \text{vel}] = -\Delta t \cdot Y$ | From $-v_{I_0} \Delta t$ |
| Gravity $g$ | $H_i[\cdot,\, g] = \frac{1}{2} \Delta t^2 \cdot Y$ | From $-\frac{1}{2} g \Delta t^2$ |

### 3.6 Known term $b$

The right-hand side $b$ collects everything that is not an unknown:

$$b_i = Y \cdot \alpha_{I_0 \to I_k} - H_{\text{proj}} \cdot p_{IC}$$

- $Y \cdot \alpha_{I_0 \to I_k}$: the displacement measured by the accelerometer (pre-integrated $\alpha$)
- $H_{\text{proj}} \cdot p_{IC}$: the camera-IMU relative position (constant extrinsics)

---

## 4. Solving for Gravity

The system $Ax = b$ is split as:

$$A_1 x_1 + A_2 g = b$$

where $x_1$ groups the 3D feature positions and initial velocity (not solved for in this step).

### 4.1 Gravity-only solve (Dong-Si polynomial)

To isolate $g$, we marginalise out $x_1$ and solve only for the gravity vector. This leads to a **degree-6 polynomial** (the Dong-Si polynomial). Its roots are found via the **companion matrix** (whose eigenvalues are the polynomial roots, i.e. the $\lambda$ values). We then:
1. Keep only **real** roots $\lambda$.
2. Select the root whose corresponding gravity vector satisfies $\|\hat{g}\| \approx 9.81\ \text{m/s}^2$.

The reduced system is solved efficiently with a **Cholesky decomposition**.

---

## 5. Recovering Pose and Velocity

Once $g$ is known, we recover $x_1$ (feature positions and initial velocity) by solving the normal equations:

$$A_1^{\top} A_1 \, x_1 = A_1^{\top}(b - A_2 g)$$

The solution is:

$$x_1 = \underbrace{(A_1^{\top} A_1)^{-1} A_1^{\top} b}_{\text{bias-free term}} - \underbrace{(A_1^{\top} A_1)^{-1} A_1^{\top} A_2 \, g}_{\text{gravity correction}}$$

- **Bias-free term** $(A_1^{\top} A_1)^{-1} A_1^{\top} b$: the solution you would get if gravity had no effect — the projection of $b$ onto the column space of $A_1$.
- **Gravity correction** $-(A_1^{\top} A_1)^{-1} A_1^{\top} A_2 \, g$: since $g$ is now known, its contribution is subtracted to obtain the correct velocity and feature positions.

The full state vector $\hat{x}$ is assembled as:

$$\hat{x} = \begin{bmatrix} x_1 \\ g \end{bmatrix} = \begin{bmatrix} \text{3D feature positions} + v_{I_0} \\ g \end{bmatrix}$$

### 5.1 Post-processing

- Recover feature positions expressed in the first IMU frame to obtain the final $\hat{x}$.
- Remove **impossible features** (i.e., features predicted to lie behind the camera).
- Rotate all states to a **gravity-aligned global frame** using Gram-Schmidt orthonormalisation, so that $g$ points along $-z$.

---

## 6. Non-linear Refinement with CERES Solver

Up to this point the pipeline approximates a non-linear system as a linear one to obtain fast, rough estimates of $g$, $v_{I_0}$, and the feature positions. To achieve accurate results, the state is refined with a **non-linear Maximum Likelihood Estimation (MLE)** step using CERES Solver.

- $\hat{x}$ from the linear stage is used as the **initial guess**.
- A **dense Schur complement** strategy solves the factor graph (dense because the initialization window is small).
- A **prior on the first frame** must be added; without it the information matrix is singular ($\det = 0$) and cannot be inverted.
- CERES outputs the refined factor graph together with the **covariance matrix**, which encodes the solver's confidence in each estimated quantity.
