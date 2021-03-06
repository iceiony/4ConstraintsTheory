n = 6; %number of points
a = zeros(n,2);
b = zeros(n,2);

fig1 = figure();
set(fig1,'position',[200 200 1200 500]);

subplot(1,2,2);
hold on;
axis([-9 17 -2 2]);

subplot(1,2,1);
hold on;
axis([-2 2 -2 2]);

%get object 1 surface 
for i = 1:n 
    t = text(-1.9,1.9,['Object 1 surface ' num2str(n-i+1) ' points'],'color','blue');
    a(i,:) = ginput(1);
    plot(a(i,1),a(i,2),'xb');
    delete(t);
end

%get object 2 surface
for i = 1:n 
    t = text(-1.9,1.9,['Object 2 surface ' num2str(n-i+1) ' points'],'color','red');
    b(i,:) = ginput(1);
    plot(b(i,1),b(i,2),'xr');
    delete(t);
end

pause();

% b = a - repmat(mean(a),length(a),1) + repmat(mean(b),length(b),1);
bOrig = b;

for theta= 0 : 15*pi/180 : 20*380*pi/180
    clf(fig1); 
    
    %draw final surfaces 
    subplot(1,2,1);
    hold on;
    axis([-2 2 -2 2]);
    
    plot(a(:,1),a(:,2),'b','linewidth',2);
    plot(b(:,1),b(:,2),'r','linewidth',2);
    label_points(a,b);

    %display 2D correlation matrix
    [cor,pval,fullCor,angle_diff] = corr_dim2(a,b);
    
    disp = {};
    disp{1} = 'Correlation';
    disp{2} = [ 'X-X : ' num2str(cor(1,1))];
    disp{3} = [ 'X-Y : ' num2str(cor(1,2))];
    disp{4} = [ 'Y-X : ' num2str(cor(2,1))];
    disp{5} = [ 'Y-Y : ' num2str(cor(2,2))];
    text(-1.9,1.7,disp');
    
    disp = {};
    disp{1} = 'Pvalues: ';
    disp{2} = [ 'X-X : ' num2str(pval(1,1))];
    disp{3} = [ 'X-Y : ' num2str(pval(1,2))];
    disp{4} = [ 'Y-X : ' num2str(pval(2,1))];
    disp{5} = [ 'Y-Y : ' num2str(pval(2,2))];
    text(1.3,1.7,disp');
    
    %first item pca
    a_rot = plot_pca(a);
    plot(a_rot(:,1),a_rot(:,2)-1,'b.');
    
    %second item pca
    b_rot = plot_pca(b);    
    plot(b_rot(:,1),b_rot(:,2)-1,'r.');

    title(sprintf('Surface Similarity : %f \n Angle %f',fullCor,angle_diff*180/pi));

    %plot point clouds for correlation
    subplot(1,2,2);
    hold on;
    axis([-9 17 -3 3]);
    
    plot(a(:,1)-5,b(:,1),'x');
    plot(a(:,1),b(:,2),'x');
    plot(a(:,2)+5,b(:,1),'x');
    plot(a(:,2)+10,b(:,2),'x');
    text(-5,2,'X-X');
    text(0,2,'X-Y');
    text(5,2,'Y-X');
    text(10,2,'Y-Y');
    
    pause();

    %rotate in succession the Y variable to see how values change
    meansB = repmat(mean(bOrig),length(bOrig),1);
    rotMat = [ cos(theta) -sin(theta);sin(theta) cos(theta)];
    b = (bOrig - meansB) * rotMat + meansB;
end


