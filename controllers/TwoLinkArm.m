%% HW5 Code for implementing 5 different controllers on Two Link Manipulator
%% By: Mostafa Atalla, Jakub Kaminski and Calvin He
%% RBE 502 Robotics Control course by Prof. Jie Fu

clc
clear all;
close all;

%% Initialize for Set Point Tracking %%
%Initialize PD Controller with a gravity compensation%
x0_s=[4, 2, 2, 2];                 %Setting initial conditions for the state vector
xf_s=[0 0 0 0];                    %final state for set point tracking control
kp_s=[150 0;0 150];                %proportional gain for the set point tracking controllers
kd_s=[100 0;0 100] ;               %dervative gain for the set point tracking controllers

%Initialize Iterative Learning Controller%
x0_i=[-0.5, 0.2, 0.1, 0.1];
tf_i = 100;


%% Initialization for Trajectory Tracking%% 
% Setting the initial,final conditions and the time span.
x0=[30 30 2 1];           %Setting initial conditions for the state vector
x0_traj=[5 1 0.3 0.1];    %Initial condition for the trajectory
xf = [50,40, 5, 5];       %Final state desired
tf=10;                    %Final time 

% Inverse Dynamics Controller Input
kp=[50 0;0 1];            %Proportional gain vector
kd=[100 0;0 1];           %Dervative gain vector


% Lyapanv Based Controller Inputs
lambda_L=[15 0;0 15];       %Lambda Square positive definite matrix for lyapanuv Based Controller.
Kd_Lyap=[15 0;0 15];        %Kd matrix.


% Passivity Based Controller Inputs
lambda_P=[25 0;0 25];       %Lambda Square positive definite matrix for lyapanuv Based Controller.
Kv=[25 0;0 25];             %Kv matrix.

%% Link Properties

I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1; 
g=9.8;
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

system = {I1,I2,m1,m2,l1,l2,r1,r2,g};  %System List


%% Implement the PD+ GRAVITY COMPENSATION control for set point tracking.
params_PDControlGravity = {kp_s, kd_s, xf_s};
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,W] = ode45(@(t,w) PDControlGravity(t,w,system,params_PDControlGravity),[0 tf],x0_s, options);


figure(1);
plot(T, W(:,1),'r-');
title('Theta1 Convergence under PD with gravity compensation Controller')
xlabel('Time')
ylabel('Theta1')
hold on
grid on
% 
figure(2);
plot(T, W(:,2),'r--');
title('Theta2 Convergence under PD with gravity compensation Controller')
xlabel('Time')
ylabel('Theta2')
hold on
grid on


%% Implement the iterative learning control (assume no knowledge about the dynamic model).
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,P] = ode45(@(t,p) ILControl(t,p),[0 tf_i], x0_i, options);

figure(3);
plot(T, P(:,1),'r-');
title('Theta1 Convergence under Iterative Learning controller')
xlabel('Time')
ylabel('Theta1')
hold on
grid on

figure(4);
plot(T, P(:,2),'r--');
title('Theta2 Convergence under Iterative Learning controller')
xlabel('Time')
ylabel('Theta2')
hold on
grid on


%% GENERATE TRAJECTORY USING TwoLinkArmTraj matlab file.
nofigure=2;
[a1] = TwoLinkArmTraj(x0_traj(1),x0_traj(3), xf(1),xf(3),tf, nofigure) %Trajectory Generation for the first Link
[a2] = TwoLinkArmTraj(x0_traj(2),x0_traj(4), xf(2),xf(4),tf, nofigure) %Trajectory Generation for the second Link



%% Implement the inverse dynamic control  
params_inverseDC = {a1, a2, kp,kd};
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) inverseDC(t,x,system,params_inverseDC),[0 tf],x0, options);

figure(5);
hold on
plot(T, X(:,1),'k-');


figure(6);
hold on
plot(T, X(:,2),'k--');

%% Implement the lyapunov-based control  

params_LyapanuvCtrl = {a1, a2, lambda_L, Kd_Lyap};


options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,Y] = ode45(@(t,y) LyapanuvBased(t,y,system,params_LyapanuvCtrl),[0 tf],x0, options);

figure(5);
hold on
plot(T, Y(:,1),'r-');

figure(6);
hold on
plot(T, Y(:,2),'r--');




%% Implement the passivity-based control  
q_double_dot_previous = [0,0]';
system = {I1,I2,m1,m2,l1,l2,r1,r2,g};
params_passivityCtrl = {a1, a2, lambda_P, Kv, q_double_dot_previous};

options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,Z] = ode45(@(t,z) passivityCtrl(t,z,system,params_passivityCtrl),[0 tf],x0, options);

figure(5);
hold on
plot(T, Z(:,1),'m-');
legend('Trajectory','Inverse Dyanmics','Lyapanuv Based','Passivity Based')
title('Theta1 Convergence under Inverse Dynamics, Lyapanuv Based and Passivity Based Controllers')
xlabel('Time')
ylabel('Theta1 (Degrees)')

figure(6);
hold on
plot(T, Z(:,2),'m--');
legend('Trajectory','Inverse Dyanmics','Lyapanuv Based','Passivity Based')
title('Theta2 Convergence under Inverse Dynamics, Lyapanuv Based and Passivity Based Controllers')
xlabel('Time')
ylabel('Theta2 (Degrees)')

