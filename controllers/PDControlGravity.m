%% PD with Gravity Compensation Controller
%% RBE 502 Fall 2018
%% Homework 5
%% November 21, 2018

function dx = PDControlGravity(t,x,measurements,params)

% Getting K_P and K_D as a positive definite, symmetric, diagonal matrix
% from the parameters:
K_P = params{1};
K_D = params{2};

% Getting the final desired state
xf = params{3};

% Measurements of the 2-link arm
I1 = measurements{1};
I2 = measurements{2};
m1 = measurements{3};
m2 = measurements{4};
l1 = measurements{5};
l2 = measurements{6};
r1 = measurements{7};
r2 = measurements{8};
g = measurements{9};

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

 
% Setting the K gain matrix
K = [-1*K_P -1*K_D];


% Error vector is the difference between current state and desired final
% state. So, e = [(q - q_desired) (q_dot - q_dot_desired)]^T
e = x' - xf;


% Input controller u for a PD controller for set point tracking and gravity compensation is
% u = -K_P(q - q_desired)-K_D(q_dot - q_dot_desired) + N(q) 
%   = [-K_P -K_D]*e + N(q)
u = (K*e') + Gmat;


% From state-space representation, dx = [q_dot q_double_dot]^T
dx = zeros(4,1);

% q_dot can be taken from the current state variable x
q_dot = x(3:4);
dx(1:2) = q_dot;

% q_double_dot comes from the dynamics of the arm and setting the torque as
% the input controller u for PD Control with gravity compensation
q_double_dot = invM * (u - Cmat*q_dot - Gmat);
dx(3:4) = q_double_dot;

end