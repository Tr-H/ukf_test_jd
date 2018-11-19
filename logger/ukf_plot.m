clear all
clc
predict = readtable('predict.csv');
u = readtable('u.csv');
ukf = readtable('ukf.csv');
m = readtable('m.csv');
figure(1)
plot(predict.vx);
hold on
plot(ukf.vx);
% plot(predict.vz);
% plot(ukf.vz)
legend('predict','ukf','predict','ukf');
hold off

figure(2)
plot(u.wx)

figure(3)
plot(m.z)