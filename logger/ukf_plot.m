clear all
clc
predict = readtable('predict.csv');
u = readtable('u.csv');
ukf = readtable('ukf.csv');
m = readtable('m.csv');
figure(1)
plot(predict.z);
hold on
plot(ukf.z);
plot(predict.vz);
plot(ukf.vz)
legend('predict','ukf','predict','ukf');
hold off

figure(2)
plot(u.az)

figure(3)
plot(m.z)