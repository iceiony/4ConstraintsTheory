figure();

subplot(2,2,1);
hold on;
axis([-.5 3 -.5 3]);

plot(a(:,1),a(:,2),'b','linewidth',2);
b = ([1 0 1.4; 0 1 1.25; 0 0 1] * a')';
plot(b(:,1),b(:,2),'k-.','linewidth',2);

legend('original','translated');
title('Translation')
%-----------

subplot(2,2,2);
hold on;

axis([-.5 3 -.5 3]);
plot(a(:,1),a(:,2),'b','linewidth',2);
b = ([2 0 0; 0 2.5 0; 0 0 1] * a')';
plot(b(:,1),b(:,2),'k-.','linewidth',2);

legend('original','scaled');
title('Scaling');

%-----------

subplot(2,2,3);
hold on;

axis([-.5 2 -.5 2]);
plot(a(:,1),a(:,2),'b','linewidth',2);
rot = rotz(60 * pi /180);

b = (rot * a')';
plot(b(:,1),b(:,2),'k-.','linewidth',2);

legend('original','rotated');
title('Rotation');

%-----------

subplot(2,2,4);
hold on;

axis([-.5 2 -.5 2]);
plot(a(:,1),a(:,2),'b','linewidth',2);

b = ([1 1 0; 0 1 0; 0 0 1] * a')';
plot(b(:,1),b(:,2),'k-.','linewidth',2);

legend('original','sheared');
title('Shearing');


