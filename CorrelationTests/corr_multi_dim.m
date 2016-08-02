function [c,p,full_corr,angle_diff] = corr_multi_dim(a,b)
%Calculates correlations between two multi-dimensional random variables.
%The correlation matrix returned is the value of Pearson correlation on 
%each combination of columns of X and Y.
c = zeros(size(a,2),size(b,2));
p = zeros(size(a,2),size(b,2));

for i=1:size(a,2)
    for j=1:size(b,2)
        [c(i,j),p(i,j)] = corr(a(:,i),b(:,j));
    end
end

%correlate on pca
a = a - repmat(mean(a),length(a),1);
[u,s,v] = svd(a);
angle_a = atan2(v(2),v(1));
rot = [cos(angle_a) -sin(angle_a) ; sin(angle_a) cos(angle_a)];
a_rot = a * rot;

b = b - repmat(mean(b),length(b),1);
[u,s,v] = svd(b);
angle_b = atan2(v(2),v(1));
rot = [cos(angle_b) -sin(angle_b) ; sin(angle_b) cos(angle_b)];
b_rot = b * rot;

x_corr = corr(a_rot(:,1),b_rot(:,1));
y_corr = corr(a_rot(:,2),b_rot(:,2));
full_corr = ( abs(x_corr) + abs(y_corr) ) / 2;

angle_diff = angle_a - angle_b;