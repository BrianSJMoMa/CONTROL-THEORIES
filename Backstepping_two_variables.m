% This script works for simulation of a 2-degree derivative function with 
% following model:
% Model:
% dot_x1 = x2
% dot_x2 = fx + g*u
% where fx, g is the function of t, x. For example
% dx1 = x2
% dx2 = x1*x2 + 2*e^x1*u
% respectively fx = x1*x2, g = 2*e^x1

clc;
clear variables;



% x = [x1 x2]', u is scalar

T = 10;    % simulation time T
dt = 0.01;
t = 0 : dt : T;
t = t';     % column vector
x1_desired = 0.5;       % DESIRED x1
x1d = x1_desired*ones(size(t));

% initial conditions
x10 = 1;
x20 = 1;
x0 = [x10;
    x20];       % x is column vector

%% PARAMETERS of Backstepping method
a1 = 3;
a2 = 20;


%% Solve the ode function by using Runge-Kutta method
% [t, y] = ode45(odefun, tspan, y0)
for i = 1 : length(t)-1
    % Calculate the derivative of x1d at i-th step
    dot_x1d = (x1d(i+1) - x1d(i)) / dt;
    dot_x1d_total(i, :) = dot_x1d;
    if i <= 2
        ddot_x1d = dot_x1d;
    else
        ddot_x1d = (dot_x1d_total(i) - dot_x1d_total(i-1)) / dt;
    end
    ddot_x1d_total(i, :) = ddot_x1d;

    % calculate u of i-th step
    x1 = x0(1);
    x2 = x0(2);
    % calculate error e1 and e2
    e1 = x1d(i) - x1;
    x2v = dot_x1d + a1*e1;
    e2 = x2 - x2v;

    %% Calculate fx
    [fx, g] = func_fx(x0);
    
    %% Calculate control signal u
    u = ( fx + ddot_x1d - a1*(e2+a1*e1) + e1 - a2*e2 ) / g;

    %% Calculate states x = [x1 x2]' by Runge-Kutta
    x = ode4(@func1, [t(i) t(i+1)], x0, u);

    % Save the data to draw figures
    x_total(:, i) = x(end, :)';     % Append the calculated states x to the total states
    % Attention: the gotten x here is raw vector, and by running ode4 there
    % will generate 2 raws at one time. First raw is the initial conditon
    % of i-th step, the 2nd raw is the calculated x.
    % That's why we save only the second raw.
    u_total(i, :) = u;

    % Refresh initial contion for next step
    x0 = x(end, :);


end

x1 = x_total(1, :);
x2 = x_total(2, :);
t = t(1:length(t)-1);

%% Draw
figure(1);
clf;

subplot(2, 1, 1);
plot(t, x1, 'linewidth', 2);
hold on;
plot(t, x2, 'linewidth', 2);
hold on;
plot(t, x1d(1:length(t)), '--', 'linewidth', 2);
grid on;
grid minor;
title('$x_1$ and $x_2$', 'interpreter', 'latex');
legend('$x_1$', '$x_2$', '$x_{1d}$', 'interpreter', 'latex');
xlabel('Time, s', 'interpreter', 'latex');
ylabel('Amplitude', 'interpreter', 'latex');
set(gca, 'fontname', 'times new roman', 'fontsize', 25);


subplot(2, 1, 2);
plot(t, u_total, 'linewidth', 2);
grid on;
grid minor;
title('Backstepping Control $u$', 'interpreter', 'latex');
xlabel('Time, s', 'interpreter', 'latex');
ylabel('Control Signal', 'interpreter', 'latex');
set(gca, 'fontname', 'times new roman', 'fontsize', 25);

%% System model
function [dx, u] = func1(t, x0, u)
    % dx1 = x2;
    % dx2 = fx + g*u;
    
    x1 = x0(1);
    x2 = x0(2);

    [fx, g] = func_fx(x0);

    dx1 = x2;
    dx2 = fx + g*u;

    % Backstepping method: u = -x1*x2 + ddot_x1d - a1*(e2+a1*e1) + e1 - a2*e2

    dx = [dx1;
        dx2];
end


%% Calculate fx and g in dot_x2 = fx + g*u
function [fx, g] = func_fx(x)
    x1 = x(1);
    x2 = x(2);
    
    % YOU CAN DIY YOUR OWN fx AND g HERE FOR ANY LINEAR AND NONLINEAR SYSTEM HERE
    fx = x1;
    g = 1;
end