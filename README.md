# Backstepping control method on a two-degree linear/nonlinear system
This is the realization of Backstepping control method on any systems - linear and nonlinear.

Platform: MATLAB.

In the script **Backstepping_two_variables.m** at its end you can DIY your own dynamic model of system, just remind yourself that the model should have the following form:

$$
\begin{cases}
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= f \left( x, t \right) + g \left( x, t \right) u
\end{aligned}
\end{cases}
$$

The document contains two files: Backstepping_two_variables.m and ode4.m. You can run the first one to check the control results. The 2nd one is Runge-Kutta method and you can actually use it in any of your own projects.
