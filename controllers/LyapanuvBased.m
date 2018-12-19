%% Lyapanuv Based Controller
%% RBE 502 Fall 2018
%% Homework 5
%% November 21, 2018

function dydt = LyapanuvBased(t,y,system,params_LyapanuvCtrl)
%params should include: a1, a2 (trajectory parameters), lambda and Kd
%m1,m2,I1,I2,l1,l2,r1,r2 (system parameters);
%This function is taking the system and the controller parameters as input
%and computes the Lyapanuv based controller that should be fed to the
%system and based on that it gets the first order state vector to be sloved
%by the ode45 Function.


%% Parameters of the trajectory and the controller
% a1 is the coefficients of the trajectory generated for theta1
a1 = params_LyapanuvCtrl{1};
% a2 is the coefficients of the trajectory generated for theta2
a2 = params_LyapanuvCtrl{2};

% Defining Lambda as a positive definite, symmetric, diagonal matrix
% with arbitrarily chosen values along the diagonal: 
lambda_L = params_LyapanuvCtrl{3};

% Defining Kd as a positive definite, symmetric, diagonal matrix
% with arbitrarily chosen values along the diagonal: 
Kd = params_LyapanuvCtrl{4};
       
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
g = system{9};

a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;
        
 % the actual dynamic model of the system:
 M = [a+2*b*cos(y(2)), d+b*cos(y(2)); d+b*cos(y(2)), d] %Inertia Matrix
 C = [-b*sin(y(2))*y(4), -b*sin(y(2))*(y(3)+y(4)); b*sin(y(2))*y(3),0] %Coriolos Materix
 G = [m1*g*r1*cos(y(1))+m2*g*(l1*cos(y(1))+r2*cos(y(1)+y(2)));
        m2*g*r2*cos(y(1)+y(2))];
        
invM = inv(M);
invMC= inv(M)*C;
              
        

        
%% Defining the controller

e=[y(1)-theta_d(1);y(2)-theta_d(2)];         %position error vector
e_dot=[y(3)-dtheta_d(1);y(4)-dtheta_d(2)];   %Velocity error vector
                
ksi_d=dtheta_d - lambda_L*(e)                %ksi dot parameter
ksi_dd = ddtheta_d-(lambda_L*e_dot)          %ksi double dot parameter    
sigma = e_dot+(lambda_L*e);                  %Sigma Paramter

u=M*(ksi_dd)+C*(ksi_d)-Kd*(sigma)+G          %Controller Definition

ydd=invM*u-invMC*[y(3);y(4)]-invM*G          %Feeding the controller into the 2nd order system/
        
 % Defining the first order state vector.
       
dydt=zeros(4,1);   %initialization of first order ode vector%
dydt(1)=y(3);
dydt(2)=y(4);
dydt(3)= ydd(1,:);
dydt(4)= ydd(2,:);


end

