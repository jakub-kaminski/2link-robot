%% Inverse Dynamics Controller
%% RBE 502 Fall 2018
%% Homework 5
%% November 21, 2018

function dxdt = inverseDC(t,x,system,params_inverseDC)
%params should include: a1, a2 (trajectory parameters), kp and kd,
%m1,m2,I1,I2,l1,l2,r1,r2 (system parameters);
%This function is taking the system and the controller parameters as input
%and computes the inverse dynamics controller that should be fed to the
%system and based on that it gets the first order state vector to be sloved
%by the ode45 Function.

%% Parameters of the trajectory and the controller
% a1 is the coefficients of the trajectory generated for theta1
a1 = params_inverseDC{1};
% a2 is the coefficients of the trajectory generated for theta2
a2 = params_inverseDC{2};

% Defining Lambda as a positive definite, symmetric, diagonal matrix
% with arbitrarily chosen values along the diagonal: 
kp = params_inverseDC{3};

% Defining Kd as a positive definite, symmetric, diagonal matrix
% with arbitrarily chosen values along the diagonal: 
kd = params_inverseDC{4};


%% Desired Trajectory setting up
vec_t = [1; t; t^2; t^3];          % cubic polynomials
theta_d= [a1'*vec_t; a2'*vec_t];   % Desired Position 
       
        
        
% compute the coffecients velocity and acceleration in both theta 1 and theta2.
 a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
 a1_acc = [2*a1(3), 6*a1(4),0,0 ];
 a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
 a2_acc = [2*a2(3), 6*a2(4),0,0 ];
        
       
 % compute the desired trajectory (assuming 3rd order polynomials for trajectories)
 dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];   %Desired Velocity
 ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t];  %Desired Acceleration
        
 %% Two Link Manipulator System Dynamic Model
 
 % Measurements of the 2-link arm
I1 = system{1};
I2 = system{2};
m1 = system{3};
m2 = system{4};
l1 = system{5};
l2 = system{6};
r1 = system{7};
r2 = system{8};
g =  system{9};

a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;
        
 % the actual dynamic model of the system:
 M = [a+2*b*cos(x(2)), d+b*cos(x(2)); d+b*cos(x(2)), d]; %Inertia Matrix
 C = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0]; %Coriolos Materix
 G = [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
        m2*g*r2*cos(x(1)+x(2))];
        
invM = inv(M);
invMC= inv(M)*C;
        

%% defining the controller

 e=[x(1)-theta_d(1);x(2)-theta_d(2)];         %position error vector
 e_dot=[x(3)-dtheta_d(1);x(4)-dtheta_d(2)];   %Velocity error vector
 aq=ddtheta_d-kp*e-kd*e_dot;                  %Virtual Controller
 u=M*aq+C*[x(3);x(4)]+G;                      %Original Controller
 xdd=invM*u-invMC*[x(3);x(4)]-invM*G;
        
% Defining the first order state vector.
dxdt=zeros(4,1);   
dxdt(1)=x(3);
dxdt(2)=x(4);
dxdt(3)= xdd(1,:);
dxdt(4)= xdd(2,:);


end

