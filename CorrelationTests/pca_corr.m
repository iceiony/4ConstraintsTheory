clf;
hold on;
axis([-4 4 -4 4]);

D = b - repmat(mean(b),length(b),1);

plot(D(:,1),D(:,2),'xb');
% p = plot_gaussian_ellipsoid(mean(D),c*3);
% set(p,'color','blue');
% 
% [U,S,V]=svd(D);
% D3=V(:,2)*V(:,2)'*D';
% D3=U(:,2)*D2;
% plot(D3(1,:),D3(2,:),'g.');


% [v,l] = eig(cov(D));

[u,s,v] = svd(D);
v = [-v(:,1) v(:,2)];

plot(v(1,2)'*[-3,3],v(2,2)'*[-3 3],'b')
plot(v(1,1)'*[-3,3],v(1,2)'*[-3 3],'b')
% 
% 
D_rot = D * v';
plot(D_rot(:,1),D_rot(:,2),'r.')
% plot_gaussian_ellipsoid(mean(D_rot),cov(D_rot)*3);