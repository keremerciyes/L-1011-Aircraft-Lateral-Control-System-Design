# L-1011 Aircraft Lateral Control System Design

## Description

This project focuses on applying state-space control methods to the lateral dynamics of an L-1011 Tristar aircraft during cruise conditions. The primary goal is to design and simulate controllers and observers to achieve desired flight characteristics using techniques like pole placement and observer design via the Sylvester equation.


1.  Designing static state-feedback controllers ($u=Kx$) to place the closed-loop system eigenvalues at specified locations.
2.  Designing a state-feedback controller with a specific structure for the gain matrix K.
3.  Designing a full-order state observer to estimate the aircraft's states using output measurements.
4.  Combining the state-feedback controller with the observer ($u=K\hat{x}$) for control using estimated states.
5.  (Intended) Designing low-order controllers to ensure satisfactory tracking of reference signals.
6.  Simulating the complete system (aircraft, controller, observer) in MATLAB/Simulink to verify the design.

## System Model

The lateral dynamics of the L-1011 aircraft are represented by the following linear time-invariant state-space model:

$\dot{x} = Ax + Bu$
$y = Cx$

Where:
* **State vector $x(t)$:**
    * $x_1$: bank angle ($\phi$, rad)
    * $x_2$: roll rate ($p$, rad/s)
    * $x_3$: yaw rate ($r$, rad/s)
    * $x_4$: sideslip angle ($\beta$, rad)
* **Input vector $u(t)$:**
    * $\delta_r$: rudder deflection (rad)
    * $\delta_a$: aileron deflection (rad)
* **Output vector $y(t)$:**
    * $y_1$: bank angle ($\phi$, rad)
    * $y_2$: roll rate ($p$, rad/s)

**System Matrices:**
$A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & -1.89 & 0.39 & -5.555 \\ 0 & -0.034 & -2.98 & 2.43 \\ 0.034 & -0.0011 & -0.99 & -0.21 \end{bmatrix}$

$B = \begin{bmatrix} 0 & 0 \\ 0.36 & -1.6 \\ -0.95 & -0.032 \\ 0.03 & 0 \end{bmatrix}$

$C = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$

**Open-Loop Eigenvalues:**
* $-1.4811 \pm 0.62915i$ (Dutch roll mode)
* $-2.016157$ (Roll subsidence mode)
* $-0.1016$ (Spiral mode)

## Tasks Implemented

### 1. State Feedback Controller Design (Pole Placement)

* **Objective:** Design a state-feedback gain matrix $K$ such that $u=Kx$ places the closed-loop eigenvalues ($eig(A-BK)$) at desired locations.
* **Desired Eigenvalues:**
    * $-2.02506 \pm 2.02094i$ (Desired Dutch roll mode)
    * $-3.4146 \pm 2.96854i$ (Desired roll mode)
* **Method:** Solved using pole placement by matching coefficients of the characteristic polynomial $det(sI - A + BK)$ to the desired characteristic polynomial.
* **Implementation:** Calculated gain matrices $K$ for both full static feedback (part a) and structured feedback (part b, where some elements of K are zero).

### 2. Full-Order Observer Design

* **Objective:** Design an observer to estimate the state vector $x$ based on the system inputs $u$ and outputs $y$. The observer takes the form $\dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x})$.
* **Method:** Designed using the Sylvester equation method ($XA - FX = GC$) to find the observer gain $L$.
* **Observer Poles:** The observer dynamics (eigenvalues of $A-LC$, which are related to the eigenvalues of $F$) were chosen to be significantly faster (approx. 10 times) than the desired closed-loop system poles to ensure quick convergence of the estimated states.
* **Implementation:** Calculated the observer gain matrix $L$.

### 3. Simulation

* **Objective:** Verify the performance of the observer and the state-feedback controller using estimated states ($u=-K\hat{x}$).
* **Environment:** Implemented using MATLAB and Simulink.
* **Setup:** A Simulink model was constructed including the aircraft state-space model, the full-order observer, and the state-feedback loop using the estimated states $\hat{x}$. Step inputs were likely used as reference signals.
* **Results:** Simulation plots show the estimated states converging to the actual system states. The solution notes that the estimation appears unrealistically perfect, potentially indicating an issue in the observer implementation or simulation setup within the solution file. The implementation of the low-order tracking controllers (task d) is not explicitly detailed in the solution's simulation section.

## Files

* (Likely) `.m` file: MATLAB script containing the system matrices, controller design calculations (pole placement), observer design calculations (Sylvester method), and potentially code to run the Simulink simulation.
* (Likely) `.slx` file: Simulink model implementing the aircraft dynamics, observer, and controller.

## Tools Used

* MATLAB
* Simulink
* MATLAB Control System Toolbox™ (for functions like `ss`, `eig`, `coeffs`, `vpasolve`, `rank`, state-space block in Simulink etc.)