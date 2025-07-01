# Inverted Pendulum Stabilization using NMPC

The following code requires the installation of CasADi. It solves the problem of controlling an inverted pendulum to stay in its upright position at $\theta = \pi$ (an inherently unstable position) while withstanding randomized external contact forces at the tip and halfway point of the pendulum. The x_ref variable can be changed to keep the pendulum at a different desired angle. The code is broken down into three distinct sections: Problem Setup, MPC Setup, and Plotting. This README can serve as a small introduction to NMPC modeling using the CasADi library. Further resources are located near the bottom.

## Problem Setup
The pendulum was assigned arbitrary conditions of 1 kg mass, 1 m length, and a 0.1 damping coefficient. The prediction horizon used for the NMPC simulation was set at 20 steps; this is how far the controller will look into the future for its predictions. Each step was set at 0.05 seconds, leading to the overall prediction time being 1 second long. Note that the symbolic variables were defined using CasADi's MX and sym functions - this is because CasADi requires its own type of symbolic expressions and will not work correctly if they are defined otherwise.

The external forces were defined to act in two positions: 1 and 0.5 m from the pendulum's attachment (the tip and halfway point on the mass). These can be changed or expanded. The torque is calculated with the standard $ /tau = d * F_ext $ equation. The dynamics of the pendulum are defined by the standard inverted pendulum nonlinear second-order differential equation (Sourced from Controls classes). It can be seen as follows:

$$
\ddot{\theta} = \frac{1}{m\ell^2} \left( \tau + \tau_{\text{ext}} - b\dot{\theta} - mg\ell \sin(\theta) \right)
$$

Note that a function of this equation is created in the code using CasADi, with inputs as x, u, and the external forces, and outputs as the state derivatives (angular velocity and acceleration). This allows for the equation to be used and manipulated in different ways as needed. The function is then used for integration in order to simulate the state after one time step, in order to tell the controller how the system evolves over time. Integration was at first attempted with CasADi's integrator, but the simulation was too slow this way. Instead, an RK4 integrator was created to integrate manually. ChatGPT was used to help with this setup. See https://www.youtube.com/watch?v=vNoFdtcPFdk for further help. The RK4 integration approximates a solution to the ODE, estimating the next state by averaging slope estimates.

## NMPC Setup
Next, NMPC is finally set up in the code. For some background information, Model Predictive Control (also known as Receding Horizon Control) is an optimization-based control method. At each time step, the controller uses a given model of the system to predict its future behavior over a prediction horizon, computing a sequence of control actions that minimizes a given cost function while respecting system constraints. The controller then implements only the first optimized control input, repeating the process at the next time step with the new updated state information. This process is represented in the attached code. Up to this point, a reference model has been created using the pendulum's nonlinear equations of motion, and a prediction horizon has been defined. Next, the cost function, constraints, and simulation are created.

The cost function is formulated by first creating cost matrices to penalize the deviation from the state as well as the control effort, represented by Q and R, respectively. The state cost matrix, Q, is set up so that different penalization amounts are given to the angle and angular velocity error. These can be changed as needed for the objective. For the purposes of this example, a higher cost is given to deviation from the position compared to deviation from the angular velocity, as the position is the main objective. The weight for the control effort is low, as the $\pi$ position is inherently unstable and will therefore require more control. The x_ref represents this reference or target state.

CasADi's opti class is then introduced, used to define and solve optimization problems. A sequence of future states, control efforts, and external forces is defined. In a for loop, we then define the predicted state, control input, and state error at time step k. Finally, the cost function is defined as follows:

$$
J = \sum_{k=0}^{N-1} \left[ (x_k - x_{\text{ref}})^\top Q (x_k - x_{\text{ref}}) + u_k^\top R u_k \right]
$$

As mentioned above, the state is penalized for deviating from its reference. Only the magnitude of the control effort is penalized.

The constraints are then defined to ensure that the simulation is physically valid. The first defined constraint states that the first column of the predicted state trajectory equals the current state, ensuring that predictions are accurate and contemporary. The next defined constraint ensures that the system dynamics act on every predicted step. Another constraint is then put on the control inputs, ensuring that it remains within -5 to 5 Nm bounds, imitating a real actuator. Finally, IPOPT (Interior Point OPTimizer) is included by solving the optimization at every time step (minimizing the cost, solving constraints, and returning the optimal X and U). It essentially does the heavy lifting for NMPC. To summarize, IPOPT takes the current state and known external forces, solves the nonlinear optimization problem, and gives the first control input. 

## Simulation and Plotting

Next, the simulation is set up in the code for both the controlled and uncontrolled systems. At each time step, a random external force vector is generated to simulate disturbances acting on the pendulum. The current and external forces are optimized, ensuring IPOPT receives updated inputs. IPOPT is then put to work, solving the NMPC problem. 

Currently, the control system works, keeping the pendulum at the desired location with the desired velocity, with some oscillation present due to the randomized forces. The control effort reaches the bounds often but does not go past them. Future work includes changing the NMPC prediction, as currently it assumes that the external force is constant, even though it is randomized at every time step. The pendulum model may also be changed so that it is on a moving cart rather than stationary.

## Resources 
- https://www.youtube.com/watch?v=vNoFdtcPFdk
- https://web.casadi.org/python-api/
- https://www.youtube.com/watch?v=CNwV5GbTEGM
- https://www.youtube.com/watch?v=XaD8Lngfkzk
- https://arxiv.org/pdf/2408.07382
- https://ieeexplore.ieee.org/document/8619952
- https://www.youtube.com/watch?v=NkeqFtKH4Yo
