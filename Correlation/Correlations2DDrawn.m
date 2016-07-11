close all;
addpath('./stats');

n = 7; %number of points
a = zeros(n,2);
b = zeros(n,2);

fig = figure();
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

b = a - repmat(mean(a),length(a),1) + repmat(mean(b),length(b),1);
bOrig = b;

for theta= 0 : 15*pi/180 : 380*pi/180
    %draw final surfaces 
    if ishandle(fig) 
        clf(fig); 
        hold on;
        axis([-2 2 -2 2]);
    end;
    
    plot(a(:,1),a(:,2),'b');
    plot(b(:,1),b(:,2),'r');
    label_points(a,b);

    %display 2D correlation matrix
    corDim = corr_multi_dim(a,b);
    disp = {};
    disp{1} = [ 'X-X : ' num2str(corDim(1,1))];
    disp{2} = [ 'X-Y : ' num2str(corDim(1,2))];
    disp{3} = [ 'Y-X : ' num2str(corDim(2,1))];
    disp{4} = [ 'Y-Y : ' num2str(corDim(2,2))];
    text(-1.9,1.7,disp');

    
    corSelf(1) = corr_multi_dim(a(:,1),a(:,2));
    corSelf(2) = corr_multi_dim(b(:,1),b(:,2));
    
    
    fullCor = (nansum(abs(corDim(:))) - nansum(abs(corSelf))) * 2 / sum(~isnan(corDim(:)));
    
%     fullCor = nansum(abs(corDim(:))) * 2 / sum(~isnan(corDim(:)));
  
    title(sprintf('Surface Similarity : %f',fullCor));

    pause;

    %rotate in succession the Y variable to see how values change
    meansY = repmat(mean(bOrig),length(bOrig),1);
    rotMat = [ cos(theta) -sin(theta);sin(theta) cos(theta)];
    b = (bOrig - meansY) * rotMat + meansY;
end


