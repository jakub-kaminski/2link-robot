clear
close all
clc

figure

%t1 = quiver3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.25, 'Color', [1.0, 0.65, 0.0], 'LineWidth', 2.0);
quivers = quiver3(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.25, 'Color','b', 'LineWidth', 2.0);

% set z offset for arm if needed
[link1, verts1] = linkGeneration();
[link2, verts2] = linkGeneration();

%provide an offset for verts 2
z_offset = -0.1;
verts2 = bsxfun(@plus, verts2, [0.0, 0.0, z_offset]);

xlim([-2.25 2.25])
ylim([-2.25 2.25])
zlim([-1 1])

%set(gca,'xtick',[])
set(gca,'xticklabel',[])
%set(gca,'ytick',[])
set(gca,'yticklabel',[])
%set(gca,'ztick',[])
set(gca,'zticklabel',[])

camlight
campos([0.0 0.0 2]);
camorbit(5,20,'camera')

axis equal

traj1 = animatedline('Marker','none', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
traj2 = animatedline('Marker','none', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);

%torques

th1 = linspace(1,90,90);
th2 = linspace(1,90,90);
th1
size(th1,1)

for i=1:size(th1,2)
    [fr1, pos1, fr2, pos2] = planar_fk2(th1(i), th2(i));

    linkTransform(link1, verts1, fr1)
    linkTransform(link2, verts2, fr2)

    addpoints(traj1, pos1(1), pos1(2), pos1(3));
    addpoints(traj2, pos2(1), pos2(2), pos2(3)+z_offset);

    %set(t1, 'wdata', 0.05 + i/45 ); % positon of joint 1 is fixed
    set(quivers, 'xdata', pos1(1),'ydata', pos1(2), 'wdata', 0.05 + i/45);

    axis equal
    xlim([-2.25 2.25])
    ylim([-2.25 2.25])
    zlim([-1 1])

    drawnow;
    %axis equal
    %xlim([-3 3])
    %ylim([-3 3])
    %zlim([-3 3])
end

%pause(33)
