function [c,p,full_corr,angle_diff] = corr_multi_dim(x,y)
%Calculates correlations between two multi-dimensional random variables.
%The correlation matrix returned is the value of Pearson correlation on 
%each combination of columns of X and Y
c = zeros(size(x,2),size(y,2));
p = zeros(size(x,2),size(y,2));

for i=1:size(x,2)
    for j=1:size(y,2)
        [c(i,j),p(i,j)] = corr(x(:,i),y(:,j));
    end
end

%correlate on pca
x = x - repmat(mean(x),length(x),1);
[u,s,v] = svd(x);
angle_x = atan2(v(2),v(1));
rot = [cos(angle_x) -sin(angle_x) ; sin(angle_x) cos(angle_x)];
x_rot = x * rot;

y = y - repmat(mean(y),length(y),1);
[u,s,v] = svd(y);
angle_y = atan2(v(2),v(1));
rot = [cos(angle_y) -sin(angle_y) ; sin(angle_y) cos(angle_y)];
y_rot = y * rot;

x_corr = corr(x_rot(:,1),y_rot(:,1));
y_corr = corr(x_rot(:,2),y_rot(:,2));
full_corr = ( abs(x_corr) + abs(y_corr) ) / 2;

angle_diff = angle_x - angle_y;