function plot_3d_mesh( sample , sample_size )
X1 = reshape(sample(:,1),sample_size);
Y1 = reshape(sample(:,2),sample_size);
Z1 = reshape(sample(:,3),sample_size);

hold on;

plot_pca([X1(:),Y1(:),Z1(:)]);
surf(X1,Y1,Z1,'FaceColor','red','EdgeColor','none');
camlight left;lighting phong;

end

