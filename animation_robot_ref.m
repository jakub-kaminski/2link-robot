clear
close all
clc

figure

% initializing quiver graphics object
quivers = quiver3([0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [1.0; 1.0], 1.00, 'Color','b', 'LineWidth', 2.0);

% set z offset for arm if needed
[link1, verts1] = linkGeneration();
[link2, verts2] = linkGeneration();

% set z offset for desired arm trajectory
[link1d, verts1d] = linkGeneration();
[link2d, verts2d] = linkGeneration();
link1d.FaceAlpha = 0.2; 
link2d.FaceAlpha = 0.2; 

%provide an offset for verts 2
z_offset = -0.1;
verts2 = bsxfun(@plus, verts2, [0.0, 0.0, z_offset]);
verts2d = bsxfun(@plus, verts2d, [0.0, 0.0, z_offset]);

xlim([-2.25 2.25])
ylim([-2.25 2.25])
zlim([-2 2])

%set(gca,'xtick',[])
set(gca,'xticklabel',[])
%set(gca,'ytick',[])
set(gca,'yticklabel',[])
%set(gca,'ztick',[])
set(gca,'zticklabel',[])

camlight
campos([0.0 0.0 2]);
%camorbit(25,50,'camera')
camorbit(30,0,'camera')

axis equal

traj1 = animatedline('Marker','none', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
traj2 = animatedline('Marker','none', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);

load joint_data2.mat;
% Data loaded are:
% W: PD controller with gravity
% P: Iterative learning controller
% X: Inverse dynamics controller
% Y: Lapunov-based controller
% Z: Passiviy-based controller

% X: Desired trajectory (pure inverse dynamics controller)
th1d = X(:,1)';
th2d = X(:,2)';
th1d_dot = X(:,3)';
th2d_dot = X(:,4)';

% Y: Lapunov-based controller
%th1 = Y(:,1)';
%th2 = Y(:,2)';
%th1_dot = Y(:,3)';
%th2_dot = Y(:,4)';
%T = Ty;

% Z: Passiviy-based controller
th1 = Z(:,1)';
th2 = Z(:,2)';
th1_dot = Z(:,3)';
th2_dot = Z(:,4)';
T = Tz;

% Please note: the number of ODE simulation steps over the certain time range
% varies for inverse-dynamics (reference) and other controllers.
% Thus, the iterations with the closest timestamps are matched

i_ref = 1; % index of the current valid reference timestamp
           % this index will be updated to match the controller simulation
           % time as close as possible

for i=1:size(th1,2)
    % solve forward kinematics for the given theta angles
    % frame 1: located at joint1 axis
    % frame 2: located at joint2 axis
    [fr1, pos1, fr2, pos2] = planar_fk2(th1(i), th2(i));

    % apply the transformation on the graphics objects
    linkTransform(link1, verts1, fr1)
    linkTransform(link2, verts2, fr2)

    % update history of the joint positions
    addpoints(traj1, pos1(1), pos1(2), pos1(3));
    addpoints(traj2, pos2(1), pos2(2), pos2(3)+z_offset);

    % TODO: test different scaling factors
    %vec_w1 = sign(th1_dot(i))*0.1 + th1_dot(i)*0.010;
    %vec_w2 = sign(th2_dot(i))*0.1 + th2_dot(i)*0.010;
    
    % joint angular velocity vectors magnitude
    vec_w1 = sign(th1_dot(i))*0.1 + th1_dot(i);
    vec_w2 = sign(th2_dot(i))*0.1 + th2_dot(i);
    
    % updating magnitude of the graphics vector objects
    set(quivers, 'xdata', [0; pos1(1)],'ydata', [0; pos1(2)], 'wdata', [vec_w1; vec_w2]);

    axis equal
    xlim([-2.25 2.25])
    ylim([-2.25 2.25])
    zlim([-1 1])
    
    xlim([-0.75 2.25])
    ylim([-0.75 2.25])
    zlim([-1 1])

    if i==1 % special case when the first datapoint is considered
        [fr1d, pos1d, fr2d, pos2d] = planar_fk2(th1d(i), th2d(i));
        linkTransform(link1d, verts1d, fr1d)
        linkTransform(link2d, verts2d, fr2d)
        drawnow;
        % pause(10)
    elseif T(i) > Tx(i_ref)
        % since ODE number of iteration of controller and reference is not equal
        % time is used to match the nearest corresponding robot position
        % sets between the reference trajectory and the controller
        while(T(i) > Tx(i_ref))
            i_ref = i_ref + 1; % update the ref time if needed
                               % so it is always only slightly larger than
                               % the current controller simulation time
        end
    end
    % calculate the current desired robot pose using forward kinematics
    [fr1d, pos1d, fr2d, pos2d] = planar_fk2(th1d(i_ref), th2d(i_ref));
    
    % apply the transformation on the graphics objects
    linkTransform(link1d, verts1d, fr1d)
    linkTransform(link2d, verts2d, fr2d)
    drawnow;
    % T(i)
    % Tx(i_ref)
    % pause(1)
end
