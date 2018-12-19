close all
clear

x = linspace(0,8);
y = sin(x);
figure
h = plot(x,y);

h.XDataSource = 'x';
h.YDataSource = 'y';

for i=1:100
    y = sin(x.*(i/100.0));
    refreshdata
    drawnow
end
