function circles = circle(x, y, r, c)
% circles = circle(x, y, r, c) plots a circle with center (x, y), radius r,
% and using color c.

hold on;
theta = 0:pi/50:2*pi;
xCircle = r * cos(theta) + x;
yCircle = r * sin(theta) + y;
circles = plot(xCircle, yCircle);
fill(xCircle, yCircle, c)
axis equal

end