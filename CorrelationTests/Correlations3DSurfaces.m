[X1,Y1,Z1] = load_surface('./Surfaces/surface_pyramid.csv');
[X2,Y2,Z2] = load_surface('./Surfaces/surface_square.csv');

figure();
subplot(1,2,1);
surf(X1,Y1,Z1,'FaceColor','red','EdgeColor','none');
camlight left;lighting phong;

subplot(1,2,2);
surf(X2,Y2,Z2,'FaceColor','red','EdgeColor','none');
camlight left;lighting phong;