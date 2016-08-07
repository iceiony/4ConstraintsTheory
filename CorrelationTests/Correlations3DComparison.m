n = 20;
m = 14;

pca_corr = zeros(n,m);
non_pca_corr = zeros(n,m);
matrix_corr = zeros(n,m);

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
subplot(1,3,1);
imagesc(pca_corr);
title('PCA based correlation');

subplot(1,3,2);
imagesc(non_pca_corr);
title('NON-PCA correlation');

subplot(1,3,3);
imagesc(matrix_corr);
title('NON-PCA(Matrix) Correlation');
