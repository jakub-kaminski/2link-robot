clear
close all

[X,Y] = meshgrid(-2:.2:2);
Z = X.*exp(-X.^2 - Y.^2);
[DX,DY] = gradient(Z,.2,.2);
h = quiver(X,Y,DX,DY);
xlim([-2.5 2.5])
for t=1:-0.1:-1
Z = X.*exp((-X.^2 - Y.^2)*t);
[DX,DY] = gradient(Z,.2,.2);
set(h,'udata',DX,'vdata',DY)
pause(0.1)
end
