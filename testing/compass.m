clc; close all;clear all;
t = 0: (2*pi) / 100: 2*pi; %Creating points
a = cos(t) + sin(t)*i;     %Finding the points 
for i=1:size(a,2)
  compass(a(i));
  drawnow;
end
