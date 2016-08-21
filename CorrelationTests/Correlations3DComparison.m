n = 8;
m = 8;

pca_corr = zeros(n,m);
non_pca_corr = zeros(n,m);
% matrix_corr = zeros(n,m);

for i = 1:n
    [X1,Y1,Z1] = load_surface(['./Surfaces/surface_obj' num2str(i) '.csv']);
    for j=1:m
        [X2,Y2,Z2] = load_surface(['./Surfaces/surface_tool' num2str(j) '.csv']);
        
        [fullCorr,~,~,~] = corr_dim3( [X1(:) Y1(:) Z1(:)], [X2(:) Y2(:) Z2(:)] );      
        pca_corr(i,j) = fullCorr;
        
        non_pca_corr(i,j) = prod([corr(X1(:),X2(:)) corr(Y1(:),Y2(:)) corr(Z1(:),Z2(:))]);
        
        fullCorr = corr_matrix(X1,X2) * corr_matrix(Y1,Y2) * corr_matrix(Z1,Z2);
        matrix_corr(i,j) = fullCorr;
    end
end

disp('Complete!');

figure();

subplot(2,1,1);
imagesc(pca_corr);
hold on;
p = rectangle('position',[.6,.6,3.8,3.8],...
    'curvature',0.1,...
    'edgecolor','red',...
    'linewidth',2);
    
colorbar;
ylabel('object');
xlabel('tool');
title('PCA based correlation');

subplot(2,1,2);
imagesc(non_pca_corr);
hold on;
p = rectangle('position',[.6,.6,3.8,3.8],...
    'curvature',0.1,...
    'edgecolor','red',...
    'linewidth',2);
colorbar;
ylabel('object');
xlabel('tool');
title('NON-PCA correlation');
% 
% subplot(1,3,3);
% imagesc(matrix_corr);
% title('NON-PCA(Matrix) Correlation');
