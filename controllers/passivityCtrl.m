%% Passivity Based Controller
%% RBE 502 Fall 2018
%% Homework 5
%% November 21, 2018

function [ dx ] = passivityCtrl( t,x, system, params  )
%params should include: a1, a2 (trajectory parameters), lambda and kv,
%m1,m2,I1,I2,l1,l2,r1,r2 (system parameters);
%This function is taking the system and the controller parameters as input
%and computes the Passivity based controller that should be fed to the
%system and based on that it gets the first order state vector to be sloved
%by the ode45 Function.

% Getting parameters of the 2-link manipulator system
% a1 is the coefficients of the trajectory generated for theta1
a1 = params{1};
% a2 is the coefficients of the trajectory generated for theta2
a2 = params{2};

% Defining Lambda as a positive definite, symmetric, diagonal matrix
% with arbitrarily chosen values along the diagonal: 
Lambda = params{3};

% Defining K_v as a positive definite, symmetric, diagonal matrix
% with arbitrarily chosen values along the diagonal: 
K_v = params{4};

% The previous q_double_dot is saved as a parameter
q_double_dot_previous = params{5};


% q_dot comes from the current state variable
q_dot = x(3:4);

vec_t = [1; t; t^2; t^3]; % cubic polynomials
theta_d= [a1'*vec_t; a2'*vec_t];
%ref = [ref,theta_d];
% compute the velocity and acceleration in both theta 1 and theta2.
a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
a1_acc = [2*a1(3), 6*a1(4),0,0 ];
a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
a2_acc = [2*a2(3), 6*a2(4),0,0 ];

% compute the desired trajectory (assuming 3rd order polynomials for trajectories)
dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];
ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t];
theta= x(1:2,1);
dtheta= x(3:4,1);

% Measurements of the 2-link arm
I1 = system{1};
I2 = system{2};
m1 = system{3};
m2 = system{4};
l1 = system{5};
l2 = system{6};
r1 = system{7};
r2 = system{8};
g = system{9};

a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;
% the actual dynamic model of the system:
Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
Gmat =  [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
m2*g*r2*cos(x(1)+x(2))];
invM = inv(Mmat);
invMC = invM*Cmat;

% TODO: compute the control input for the system, which
% should provide the torques

% use the computed torque and state space model to compute
% the increment in state vector.
%TODO: compute dx = f(x,u) hint dx(1)=x(3); dx(2)= x(4); the rest
%of which depends on the dynamic model of the robot.

% Computing the error between the current position, velocity, and
% acceleration with the desired position, velocity, and acceleration
e = theta - theta_d;
e_dot = dtheta - dtheta_d;
e_double_dot = q_double_dot_previous - ddtheta_d;

% Set r = sigma = e_dot + Lambda*e
r = e_dot + Lambda*e;
r_dot = e_double_dot + Lambda*e_dot;

% Setting inputs to a and v for the controller
a = q_double_dot_previous - r_dot;
v = q_dot - r;

% Input controller for Passivity-based controller
u = Mmat*a + Cmat*v + Gmat - K_v*r;

% From state-space representation, dx = [q_dot q_double_dot]^T
dx = zeros(4,1);

% q_dot can be taken from the current state variable x
dx(1:2) = q_dot;

% q_double_dot comes from the dynamics of the arm and setting the torque as
% the input controller u for Passivity-based controller
q_double_dot = invM * (u - Cmat*q_dot - Gmat);
dx(3:4) = q_double_dot;

% Saving the current value of q_double_dot as a parameter to be used again
% the the next call of this ode function for passivity control
params{5} = q_double_dot;

%disp("Previous q_double_dot: ");
%disp(params{3});

end

