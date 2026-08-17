# ME 314 Final Project: Dice-in-a-Box Impact Dynamics Simulation

A planar, constrained multi-body dynamics simulation built for Northwestern's ME 314 (Dynamics of Mechanical Systems). The project models a dice (a stand-in for the course's "jack") bouncing and colliding inside a rotating box, with the full equations of motion, contact constraints, and impact-update law derived symbolically from first principles rather than pulled from a pre-built physics engine.

**Author:** Tomasz Krzeminski
**Collaborator:** Michael Jenz
**Course:** ME 314, Spring 2024

## Overview

This is the course's "default" final project: a two-rigid-body planar system (a box and a jack, modeled here as a dice) interacting entirely through contact. The system is described by six generalized coordinates,

```
q = [x_b, y_b, θ_b, x_j, y_j, θ_j]
```

corresponding to the planar pose of the box and the planar pose of the dice. The full nonlinear equations of motion, the sixteen dice-corner-to-box-wall contact constraints, and the impact law governing collisions are all derived symbolically in SymPy, then numerically integrated and animated.

## What the notebook does

The notebook (`Tomasz_Krzeminski_ME314_Final_Project.ipynb`) is organized into the following sections, in order:

| Section | Description |
|---|---|
| **Helper Functions** | `spec_euclid` (builds a planar SE(3) homogeneous transform from θ, x, y), `hat`/`unhat` (convert between twist vectors and their matrix form), and `inverse` (closed-form homogeneous transform inverse). Used throughout the rest of the notebook. |
| **Initializing q** | Defines the symbolic variables, the generalized coordinate vector `q` and its time derivatives, and the numeric parameter substitutions (box side length, box mass, dice side length, dice mass, gravity). |
| **Transformations** | Builds the homogeneous transformations from the world frame to the box and dice center-of-mass frames, and from those to each of the box's four wall frames and the dice's four corner frames. Also defines the inertia matrices for the box and dice. |
| **Creating Euler-Lagrange Equations** | Computes body velocities for the dice and each box wall, forms the kinetic and potential energy expressions, builds the Lagrangian, and solves the Euler-Lagrange equations (with an external force and torque applied to the box). |
| **Phi Constraints for Dice** | Derives the sixteen scalar constraint functions (`phi_bj`) - the y-distance from each dice corner to the box's top/bottom walls and the x-distance to the left/right walls - that keep the dice inside the box. |
| **Impact Substitution Symbols** | Creates dummy "before" and "after" symbols for every coordinate and velocity, used to express the impact law across a collision. |
| **Impact Update Equations** | Builds the momentum-jump / Hamiltonian-conservation impact equations for all sixteen constraints, following the impact-update formalism from *Dynamics: From Calculus and Geometry of Motion*. |
| **Impact Update Condition** | A function that checks, given the current state, whether any of the sixteen constraints is active (i.e. a collision is occurring). |
| **Impact Update Function** | Given the current state and the active constraint index, solves the corresponding impact equation and returns the post-impact velocities. |
| **Box and Dice Simulation** | An RK4 integrator that advances the system, checking the impact condition at every step and applying the impact update whenever a collision is detected. Produces the full state trajectory and a plot of all six generalized coordinates over time. |
| **Animation** | A Plotly-based animation function (`animate_dice_in_box`) that renders the simulated trajectory as an interactive, playable animation of the box and dice. |

## Modeling approach

- **Kinematics:** Every frame transformation is a planar homogeneous transform built with `spec_euclid`. Body velocities are computed as \(g^{-1}\dot g\) and converted to twist vectors with the `unhat` operator.
- **Dynamics:** The kinetic energy is assembled from the body velocities and inertia matrices of the box's four walls and the dice; the potential energy is `mgh` for each. The Lagrangian `L = KE - V` is differentiated symbolically to produce the Euler-Lagrange equations.
- **External forces:** A small constant upward force is applied to the box (just enough to keep it framed and in contact with the dice), and a constant torque is applied to the box's rotational coordinate to drive repeated collisions.
- **Constraints & impacts:** The sixteen `phi` constraints (four dice corners × four box walls) define when a collision occurs. When a constraint is (near) zero, the corresponding impact equation is solved for the post-impact velocities, conserving momentum and the Hamiltonian across the collision.

## Requirements

```
sympy
numpy
matplotlib
plotly
ipython
```

## Running it

This notebook was written and run in Google Colab, but works in any standard Jupyter environment:

1. Clone the repository:
   ```
   git clone https://github.com/Tomekkrzem/ME314-Final-Project.git
   ```
2. Install the dependencies:
   ```
   pip install sympy numpy matplotlib plotly ipython
   ```
3. Open `Tomasz_Krzeminski_ME314_Final_Project.ipynb` in Jupyter, JupyterLab, or Google Colab, and run all cells in order. The last cell, `animate_dice_in_box(traj, T=10)`, renders the interactive animation.

Note: the symbolic derivation (particularly simplifying the sixteen impact equations) is computationally heavy and can take several minutes to run.

## Results

Over a 17-second, 1,700-step simulation, the dice and box exchange momentum at every collision as expected: the lighter dice absorbs more of each impact and rotates noticeably faster, while the heavier box only recoils slightly. Position, velocity, and the impact-triggered velocity discontinuities all match the specified initial conditions, with no artifacts in the resulting animation.

## Full write-up

A complete write-up, including the system drawing, the derived transformation matrices, and a discussion of the Euler-Lagrange setup, constraints, external forces, and impact update laws, is included as a PDF in this repository (or in-line as markdown cells in the notebook).
