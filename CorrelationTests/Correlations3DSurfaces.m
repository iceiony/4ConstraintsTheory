[X1,Y1,Z1] = load_surface('./Surfaces/surface_square.csv');

points = [X1(:) Y1(:) Z1(:)];

meansB = repmat(mean(points),length(points),1);
points = (rotz(60*pi/180) * roty(60*pi/180) * rotx(60*pi/180) * (points - meansB)')'  + meansB;

VIEW_DIM = sqrt(length(points));
X2 = reshape(points(:,1),VIEW_DIM,VIEW_DIM);
Y2 = reshape(points(:,2),VIEW_DIM,VIEW_DIM);
Z2 = reshape(points(:,3),VIEW_DIM,VIEW_DIM);

[fullCorr,angleX,angleY,angleZ] = corr_dim3( [X1(:) Y1(:) Z1(:)], [X2(:) Y2(:) Z2(:)] );


figure();
subplot(1,2,1);
hold on;
plot_pca([X1(:),Y1(:),Z1(:)]);
surf(X1,Y1,Z1,'FaceColor','red','EdgeColor','none');
camlight left;lighting phong;
title(sprintf('Correlation: %f', fullCorr));


subplot(1,2,2);
hold on;
plot_pca([X2(:),Y2(:),Z2(:)]);
surf(X2,Y2,Z2,'FaceColor','red','EdgeColor','none');
camlight left;lighting phong;
title(sprintf('%0.2f %0.2f %0.2f', angleX * 180/pi, angleY*180/pi,angleZ*180/pi));