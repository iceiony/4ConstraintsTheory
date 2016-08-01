points = load('surface.csv');
VIEW_DIM = sqrt(length(points));

%adjust coordinates to matlab's rendering 
Y = reshape(points(:,1),VIEW_DIM,VIEW_DIM);
Z = reshape(points(:,2),VIEW_DIM,VIEW_DIM);
X = reshape(points(:,3),VIEW_DIM,VIEW_DIM);

Z(Z(:)<1) = nan;
% Z(isnan(Z(:))) = min(Z(:));

figure();
hold on;
axis([min(X(:)) max(X(:)) min(Y(:)) max(Y(:)) min(Z(:))-0.3 max(Z(:))]);
% surf(X,Y,Z);
surf(X,Y,Z,'FaceColor','red','EdgeColor','none')
camlight left;
lighting phong;