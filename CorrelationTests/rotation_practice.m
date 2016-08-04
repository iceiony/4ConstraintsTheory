[X1,Y1,Z1] = load_surface('./Surfaces/surface_square.csv');
points = [X1(:) Y1(:) Z1(:)];

meansB = repmat(mean(points),length(points),1);
points = (rotz(120*pi/180) * roty(0*pi/180) * (points - meansB)')'  + meansB;

a = points; 
a = a - repmat(mean(a),length(a),1);
[u,s,v] = svd(a);

% v = rotz(120*pi/180) * roty(0*pi/180) * v;

figure();
hold on;

x = [0 0 0 ; 1.2 0 0];
y = [0 0 0 ; 0 1.2 0];
z = [0 0 0 ; 0 0 1.2];

plot3(x(:,1),x(:,2),x(:,3),'k');
plot3(y(:,1),y(:,2),y(:,3),'k');
plot3(z(:,1),z(:,2),z(:,3),'k');
text(1.1,0,0,'X');
text(0,1.1,0,'Y');
text(0,0,1.1,'Z');

plot3([0 v(1)],[0 v(2)],[0 v(3)],'b');
plot3([0 v(4)],[0 v(5)],[0 v(6)],'b');
plot3([0 v(7)],[0 v(8)],[0 v(9)],'b');
text(v(1),v(2),v(3),'X','color','blue');
text(v(4),v(5),v(6),'Y','color','blue');
text(v(7),v(8),v(9),'Z','color','blue');

%rotate by each axis


%align 1st component to x-axis
vx = v(:,1);
alpha = mod(atan2(vx(2),vx(1)),2*pi);
beta  = mod(-asin(vx(3)),2*pi);
v = roty(-beta) * rotz(-alpha)  * v;

%align 2nd component to y-axis
vy = v(:,2);
alpha = mod(atan2(vy(3),vy(2)),2*pi);
v = rotx(-alpha) * v;

plot3([0 v(1)],[0 v(2)],[0 v(3)],'r');
plot3([0 v(4)],[0 v(5)],[0 v(6)],'r');
plot3([0 v(7)],[0 v(8)],[0 v(9)],'r');
text(v(1),v(2),v(3),'X','color','red');
text(v(4),v(5),v(6),'Y','color','red');
text(v(7),v(8),v(9),'Z','color','red');
