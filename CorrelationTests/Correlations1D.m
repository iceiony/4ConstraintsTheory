%have them random and uniformely distributed
x = rand(1,100);
y = rand(1,100);
coef = corrcoef(x,y);

figure();
subplot(2,1,1);
plot(x,y,'x');
title(sprintf('Correlation %f', coef(2)));

subplot(2,1,2);
title(sprintf('X-Correlation %f', max(xcorr(x,y,'coeff'))));
hold on;
plot(x);
plot(y);

%have them in the same direction
x = (1:100) + rand(1,100)*2;
y = (1:100) + rand(1,100)*5;
coef = corrcoef(x,y);

figure();
subplot(2,1,1);
plot(x,y,'x');
title(sprintf('Correlation %f', coef(2)));

subplot(2,1,2);
title(sprintf('X-Correlation %f', max(xcorr(x,y,'coeff'))));
hold on;
plot(x);
plot(y);

%have them in reverse
x = (1:100) + rand(1,100)*2;
y = (100:-1:1) + rand(1,100)*5;
coef = corrcoef(x,y);

figure();
subplot(2,1,1);
plot(x,y,'x');
title(sprintf('Correlation %f', coef(2)));

subplot(2,1,2);
title(sprintf('X-Correlation %f', max(xcorr(x,y,'coeff'))));
hold on;
plot(x);
plot(y);


%permute both of them by the same random permutation
p = randperm(100);
x = (1:100)+ rand(1,100)*10;
y = (100:-1:1) + rand(1,100)*2;
coef = corrcoef(x(p),y(p));

figure();
subplot(3,1,1);
hold on;
title(sprintf('Correlation %f', coef(2)));
plot(x,y,'x');
plot(x(p),y(p),'or');

subplot(2,1,2);
title(sprintf('X-Correlation %f', max(xcorr(x,y,'coeff'))));
hold on;
plot(x(p));
plot(y(p));

%permute only the first
p = randperm(100);
x = (1:100)+ rand(1,100)*10;
y = (100:-1:1) + rand(1,100)*2;
coef = corrcoef(x(p),y);

figure();
subplot(3,1,1);
hold on;
title(sprintf('Correlation %f', coef(2)));
plot(x,y,'x');
plot(x(p),y,'or');

subplot(2,1,2);
title(sprintf('X-Correlation %f', max(xcorr(x,y,'coeff'))));
hold on;
plot(x(p));
plot(y);

%reverse until half and then flat for one
x = [ (1:50) 51*ones(1,50)];
y = (100:-1:1);
coef = corrcoef(x,y);

figure();
subplot(3,1,1);
hold on;
title(sprintf('Correlation %f', coef(2)));
plot(x,y,'x');
plot(x,y,'or');

subplot(2,1,2);
title(sprintf('X-Correlation %f', max(xcorr(x,y,'coeff'))));
hold on;
plot(x);
plot(y);

%reverse until half then same order
n = 50;
x = [ (1:n) (n:-1:2*n-99)];
y = (100:-1:1);
coef = corrcoef(x,y);

figure();
subplot(3,1,1);
hold on;
title(sprintf('Correlation %f', coef(2)));
plot(x,y,'x');

subplot(2,1,2);
title(sprintf('X-Correlation %f', max(xcorr(x,y,'coeff'))));
hold on;
plot(x,'linewidth',2);
plot(y);


%reverse then flat then reverse
x = [ (1:25) 25*ones(1,50) (25:-1:1) ];
y = [ (25:-1:1) ones(1,50) (1:25) ];
coef = corrcoef(x,y);

figure();
subplot(3,1,1);
hold on;
title(sprintf('Correlation %f', coef(2)));
plot(x,y,'x');

subplot(2,1,2);
title(sprintf('X-Correlation %f', max(xcorr(x,y,'coeff'))));
hold on;
plot(x,'linewidth',2);
plot(y);
