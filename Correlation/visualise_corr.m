function visualise_corr(a,b)
close all;

figure();
subplot(3,1,1);
hold on;
axis([-1 6 -1 6]);
plot(a(:,1),a(:,2),'linewidth',2);
plot(b(:,1),b(:,2));
label_points(a,b);

%you can't correlate unless the number of observations is the same
if length(a) == length(b)
    corDim = corr_multi_dim(a,b);
    
    disp = {};
    disp{1} = [ 'X-X : ' num2str(corDim(1,1))];
    disp{2} = [ 'X-Y : ' num2str(corDim(1,2))];
    disp{3} = [ 'Y-X : ' num2str(corDim(2,1))];
    disp{4} = [ 'Y-Y : ' num2str(corDim(2,2))];
    text(-.9,4,disp');
    
    fullCor = corDim(1) * corDim(4) + corDim(2) * corDim(3);
    title(sprintf('Full Correlation : %f',fullCor));
end

subplot(3,1,2);
hold on;
a = a';
b = b';
plot(a(:),'linewidth',2);
plot(b(:));

%correlate how xcorr does it
corX = xcorr(a(:),b(:));
corX = corX ./ sqrt(sum(a(:).^2) * sum(b(:).^2));
corCoef = corrcoef(a(:),b(:));
title(sprintf('xCorrelation : %f \n Correlation : %f', max(corX),corCoef(2)));

subplot(3,1,3);
hold on;
title(sprintf('2D Correlation : %f',corr2(a',b')));

[a,b] = canoncorr(a',b');
desc = { sprintf('%f %f\n',a) ; sprintf('%f %f\n',b)};
text(.025,0.45,desc,'fontsize',14);

pause;
end

