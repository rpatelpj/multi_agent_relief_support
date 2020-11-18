%% Base Case: 1 robot, 1 sink

xlim = 1.6;
ylim = 1.0;
k = 10;
slope = 100;
step = .05;
x = (-xlim/k):step/k:(xlim/k);
y = (-ylim/k):step/k:(ylim/k);
% y = x';
h = 0; k = 0;
[x, y] = meshgrid(x, y);
E = slope.*((x - h).^2 + (y - k).^2);

[Ex, Ey] = gradient(-E);
figure
contour(x, y, E)
hold on
quiver(x, y, Ex, Ey)
hold off

mask = E < 2.49;
values = E(mask) - 2.49; %2.49 was eyeballed off the graph

xvals = x(mask);
yvals = y(mask);
figure
plot3(xvals, yvals, values, '*')
hold on