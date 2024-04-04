clear all
close all
clc


t = linspace(0,1,100);
ro = 3.5;
eps = 15;
beta = 0.04;
h = ro*exp(-eps.*t).*t.^(-beta);
figure
plot(t,h)
hold on 

x = 0.17; % metti limite a 0.2
h = ro*exp(-eps*x)*x^(-beta);
plot(x,h,'*r')

ro = 0.1;
eps = 1;
beta = 0.9;
h = ro*exp(-eps.*(t)).*(t).^(-beta);
figure
plot(t,h)
hold on 
x = 0.8;
h = ro*exp(-eps*x)*x^(-beta);
plot(x,h,'*r')

ro = 0.15;
eps = 1;
beta = 0.8;
h = ro*exp(-eps.*t).*t.^(-beta);
figure
plot(t,h)
hold on 
x = 0.6;
h = ro*exp(-eps*x)*x^(-beta);
plot(x,h,'*r')
