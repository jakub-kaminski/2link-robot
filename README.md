# 2link-robot 
Matlab animations: 2-dof arm control for a predefined trajectory using different controllers

Animation by Jakub Kaminski 

Controllers code by Mostafa Atalla, Calvin He and Jakub Kaminski

### Animations generated can be found on YouTube:
Passivity-based Controller and reference trajectory: https://youtu.be/ciuWzi5VB5Q

Inverse-Dynamics Controller: https://youtu.be/AQJjr3sGdbM

Lapunov-based Controller: https://youtu.be/83hr36adFTA

Passivity-based Controller: https://youtu.be/UTVANNz7S3U

### Instructions to run animations:
### Clone repository:
    git clone https://github.com/jakub-kaminski/2link-robot

### Run MATLAB code (Passivity based controller and the reference trajectory):
    matlab animation_robot_ref.m

### Or alternatively, run (inverse dynamics controller by default):
    matlab animation_robot.m
    
### Or the same code in the MATLAB console:
    animation_robot

### To run simulation for a different controller (in this case passivity-based control), please uncomment lines of animation_robot.m containing:
    %th1 = Z(:,1)';
    %th2 = Z(:,2)';
    %th1_dot = Z(:,3)';
    %th2_dot = Z(:,4)';


### Project parts in progress:
Fixing matlab quiver scalling range 

Generating user friendly option menu for choosing desired controller

Simulating desired trajectory using 2-nd, semi-transparent robot
