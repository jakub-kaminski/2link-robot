%% Iterative Learning Controller
%% RBE 502 Fall 2018
%% Homework 5
%% November 21, 2018

function dxdt = ILControl(t,x)
    % *************** VARIABLE DEFINITION ***************
    persistent Compensation % Variable to store the iteratively estimated gravity term
    if isempty(Compensation) % Fill with zeros initially, this happends one time per program execution
        Compensation = zeros(2,1);
    end

    % System properties
    I1=10;  I2 = 10;
    m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
    g = 9.8;

    % we compute the parameters in the dynamic model
    a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
    b = m2*l1*r2;
    d = I2+ m2*r2^2;

    symx= sym('symx',[4,1]); 

    M = [a+2*b*cos(x(2)), d+b*cos(x(2));
        d+b*cos(x(2)), d];
    C = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];

    invM = inv(M);
    invMC= inv(M)*C;

    % PD controller gains
    Kp1 = 30.4; Kp2 = 30; Kd1 = 145; Kd2 = 145; % works ok

    % *************** COMPUTATIONS ***************

    A_Mat = [0 0 1 0;
             0 0 0 1;
             0 0 0 0;
             0 0 0 0];

    A_Mat(3:4,3:4) = -double(invMC);
    %A_Mat(3:4,3:4) = double(invMC);

    Gain_Mat = [Kp1  0   Kd1  0;
                 0   Kp2  0   Kd2];

    % Joint torques from PD controller
    U_Mat = -Gain_Mat * x;

    B_Mat = [0 0;
             0 0;
             invM];

    % The torque resulting from the gravity
    G = [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
         m2*g*r2*cos(x(1)+x(2))];

    tresh = 0.0001; % using to check joint velocities
    beta = 0.10; % Condition: 0 < beta < 0.5

    U_Net_Mat = (1/beta)*U_Mat + Compensation - G; % Include compensation and gravity

    % Update the known gravity term if joint velocities are zero
    if(abs(x(3)) < tresh && abs(x(4)) < tresh)
        Compensation =  -(1/beta)*[Kp1; Kp2].*[x(1); x(2)] + Compensation;
        U_Net_Mat = Compensation - G;
    end

    dxdt = zeros(4,1); 
    dxdt = A_Mat*x + B_Mat*U_Net_Mat; % update new states
end
