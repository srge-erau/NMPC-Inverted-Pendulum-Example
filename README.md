# Inverted Pendulum Stabilization using NMPC

The following code requires the installation of CasADi. It solves the problem of controlling an inverted pendulum to stay in its upright position at \theta = \pi (an inherently unstable position) while withstanding randomized external contact forces at the tip and halfway point of the pendulum. The x_ref variable can be changed to keep the pendulum at a different desired angle. The code is broken down into three distinct sections: Problem Setup, MPC Setup, and Plotting. This README can serve as a small introduction to NMPC modeling using the CasADi library. Further resources are located near the bottom.

## Problem Setup
The pendulum was assigned arbitrary conditions of 1 kg mass, 1 m length, and a 0.1 damping coefficient. The prediction horizon used for the NMPC simulation was set at 20 steps; this is how far the controller will look into the future for its predictions. Each step was set at 0.05 seconds, leading to the overall prediction time being 1 second long. Note that the symbolic variables were defined using CasADi's MX and sym functions - this is because CasADi requires its own type of symbolic expressions and will not work correctly if they are defined otherwise.

The external forces were defined to act in two positions: 1 and 0.5 m from the pendulum's attachment (the tip and halfway point on the mass). These can be changed or expanded. The torque is calculated with the standard $ /tau = d * F_ext $ equation. The dynamics of the pendulum are defined by the standard inverted pendulum nonlinear second-order differential equation (Sourced from Controls classes). It can be seen as follows:

$$
\ddot{\theta} = \frac{1}{m\ell^2} \left( \tau + \tau_{\text{ext}} - b\dot{\theta} - mg\ell \sin(\theta) \right)
$$

Note that a function of this equation is created in the code using CasADi, with inputs as x, u, and the external forces, and outputs as the state derivatives (angular velocity and acceleration). This allows for the equation to be used and manipulated in different ways as needed. The function is then used for integration in order to simulate the state after one time step, in order to tell the controller how the system evolves over time. Integration was at first attempted with CasADi's integrator, but the simulation was too slow this way. Instead, an RK4 integrator was created to integrate manually. ChatGPT was used to help with this setup. See https://www.youtube.com/watch?v=vNoFdtcPFdk for further help.

(EXPLAIN RK4 INTEGRATION)


## NMPC Setup
Next, NMPC is finally set up in the code. For some background information: MPC is an optimization control method

(EXPLAIN HOW MPC WORKS)


A cost matrix is created to penalize the deviation from the state as well as the control effort, represented by Q and R, respectively. The state xost matrix, Q, is set up so that different penalization is given to the angle and angular velocity error. These can be changed as needed for the objective. 

The x_ref representes the reference or target state, in this case a \pi position and a 0 rad/s rotation. It serves as the desired track.

CasADi is then used to create an optimization problem object. CasADi's opti is used to optimize 


## Resources 
