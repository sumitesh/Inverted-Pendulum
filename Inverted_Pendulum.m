% Inverted Pendulum Problem

% Parameters
M = [0.1 1 5];       % Mass of the cart (kg)
m = 20.0;       % Mass of the pendulum (kg)
l = 0.5;       % Length of the pendulum (m)
g = 9.81;      % Gravitational acceleration (m/s^2)
dt = 0.01;     % Time step (s)
t_max = 10.0;  % Simulation time (s)

for i = 1:3
% Initial conditions
x = 0.0;          % Initial position of the cart (m)
x_o = 0.0;        % Initial velocity of the cart (m/s)
x_oo = 0.0;       % Acceleration of the cart (m/s^2)
theta = 0.5;      % Initial angle of the pendulum (rad)
theta_o = 0.0;    % Initial angular velocity of the pendulum (rad/s)
theta_oo = 0.0;   % Angular acceleration of the pendulum (rad/s^2)
U = 0.0;          % Control force applied to the cart (N)
t = 0.0;          % Time variable

% Prepare to store the results
time_data = [];
x_data = [];
theta_data = [];
x_o_data = [];
theta_o_data = [];

% Solver 
while t < t_max
    % Calculating the denominator in the equations of motion
    denominator = M(i) + m*sin(theta)^2;
    
    % Calculating accelerations
    x_oo = (U + m*sin(theta)*(l*theta_o^2 + g*cos(theta))) / denominator;
    theta_oo = (-U*cos(theta) - m*l*theta_o^2*cos(theta)*sin(theta) - (M(i) + m)*g*sin(theta)) / (l*denominator);
    
    % Updating velocities
    x_o = x_o + x_oo*dt;
    theta_o = theta_o + theta_oo*dt;
    
    % Updating positions
    x = x + x_o*dt;
    theta = theta + theta_o*dt;
    
    % Storing output in vectors
    time_data = [time_data, t];
    x_data = [x_data, x];
    theta_data = [theta_data, theta];
    x_o_data = [x_o_data, x_o];
    theta_o_data = [theta_o_data, theta_o];
    
    % Update time
    t = t + dt;
end

% Display results
results = table(time_data', x_data', theta_data', x_o_data', theta_o_data','VariableNames', {'Time', 'Cart Position', 'Pendulum Angle', 'Car tVelocity', 'Pendulum Angular Velocity'});
disp(results);

% Plotting the results
figure;
subplot(2,1,1);
plot(time_data, x_data);
xlabel('Time (s)');
ylabel('Cart Position (m)');
title('Cart Position vs. Time');

subplot(2,1,2);
plot(time_data, theta_data);
xlabel('Time (s)');
ylabel('Pendulum Angle (rad)');
title('Pendulum Angle vs. Time');
end