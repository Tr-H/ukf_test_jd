clear all
close all
clc
real_p = readtable('real_predict.csv');
real_u = readtable('real_update.csv');
t_p = real_p.timestamp - min([real_p.timestamp;real_u.timestamp]);
t_u = real_u.timestamp - min([real_p.timestamp;real_u.timestamp]);
figure(1)
subplot(3,1,1)
plot(t_p, real_p.phi,'.-');
hold on
plot(t_u, real_u.phi,'.-');
legend('ukf','vio');
title('phi')
hold off
subplot(3,1,2)
plot(t_p, real_p.theta,'.-');
hold on
plot(t_u, real_u.theta,'.-');
legend('ukf','vio');
title('theta')
hold off
subplot(3,1,3)
plot(t_p, real_p.psi,'.-');
hold on
plot(t_u, real_u.psi,'.-');
legend('ukf','vio');
title('psi')
hold off

figure(2)
subplot(3,1,1)
plot(t_p, real_p.vx,'.-');
hold on
plot(t_u, real_u.vx,'.-');
legend('ukf','vio');
title('vx')
hold off
subplot(3,1,2)
plot(t_p, real_p.vy,'.-');
hold on
plot(t_u, real_u.vy,'.-');
legend('ukf','vio');
title('vy')
hold off
subplot(3,1,3)
plot(t_p, real_p.vz,'.-');
hold on
plot(t_u, real_u.vz,'.-');
legend('ukf','vio');
title('vz')
hold off

figure(3)
subplot(3,1,1)
plot(t_p, real_p.x,'.-');
hold on
plot(t_u, real_u.x,'.-');
legend('ukf','vio');
title('x')
hold off
subplot(3,1,2)
plot(t_p, real_p.y,'.-');
title('y')
hold on
plot(t_u, real_u.y,'.-');
legend('ukf','vio');
hold off
subplot(3,1,3)
plot(t_p, real_p.z,'.-');
hold on
plot(t_u, real_u.z,'.-');
legend('ukf','vio');
title('z')
hold off

figure(4)
subplot(3,1,1)
plot(t_p, real_p.ax,'.-');
hold on
% plot(t_u, real_u.ax,'.-');
legend('ukf');
title('ax')
hold off
subplot(3,1,2)
plot(t_p, real_p.ay,'.-');
hold on
% plot(t_u, real_u.vy,'.-');
legend('ukf');
title('ay')
hold off
subplot(3,1,3)
plot(t_p, real_p.az,'.-');
hold on
% plot(t_u, real_u.vz,'.-');
legend('ukf');
title('az')
hold off


% figure(5)
% subplot(3,1,1)
% plot(t_p, real_p.bias_ax,'.-');
% hold on
% % plot(t_u, real_u.ax,'.-');
% legend('ukf');
% title('ax')
% hold off
% subplot(3,1,2)
% plot(t_p, real_p.bias_ay,'.-');
% hold on
% % plot(t_u, real_u.vy,'.-');
% legend('ukf');
% title('ay')
% hold off
% subplot(3,1,3)z
% plot(t_p, real_p.bias_az,'.-');
% hold on
% % plot(t_u, real_u.vz,'.-');
% legend('ukf');
% title('az')
% hold off
% 
% 
% figure(6)
% subplot(3,1,1)
% plot(t_p, real_p.bias_wx,'.-');
% hold on
% % plot(t_u, real_u.ax,'.-');
% legend('ukf');
% title('wx')
% hold off
% subplot(3,1,2)
% plot(t_p, real_p.bias_wy,'.-');
% hold on
% % plot(t_u, real_u.vy,'.-');
% legend('ukf');
% title('wy')
% hold off
% subplot(3,1,3)
% plot(t_p, real_p.bias_wz,'.-');
% hold on
% % plot(t_u, real_u.vz,'.-');
% legend('ukf');
% title('waz')
% hold off