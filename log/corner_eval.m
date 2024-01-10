clear all
close all
clc
% position of 4 points on the E-E frame
dim_ee = 0.02;

vec_1 = [-dim_ee/2, -dim_ee/2, 0.0, 1.0]';
vec_2 = [dim_ee/2, -dim_ee/2, 0.0, 1.0]';
vec_3 = [dim_ee/2,  dim_ee/2, 0.0, 1.0]';
vec_4 = [-dim_ee/2,  dim_ee/2, 0.0, 1.0]';

T_cb_temp = [eye(3), [0.085,0,0.06]';[0 ,0,0,1]];           % Transformation from base_link frame to camera_temp frame

% R_temp = roty(-pi/2);
% T_cb_temp = [R_temp, [0.085,0,0.06]';[0 ,0,0,1]];           % Transformation from base_link frame to camera_temp frame

T_cb = [0 0 1 0;-1 0 0 0;0 -1 0 0 ; 0 0 0 1];               % Transformation from camera_temp frame to real camera_frame
% T = [eul2rotm([-1.5648,-0.00842267,1.96025]), [0.279861   0.0204094 -0.00404568]';[0 0 0 1]]

% E-E position w.r.t. /base_link
p = [0.390, 0.020, -0.052]';
rpy = [-1.572, 0.000, 1.571];
T = [ eul2rotm(rpy),    p;
      0.0, 0.0, 0.0, 1.0];
theta = [0.0, -0.5076663999999997, 0.9217161000000007, 0.0018848999999998561, -0.4139999999999999, 0.0018848999999998561];

% 4 co-planar point of E-E frame described in /base_link frame
T1=T*vec_1
T2=T*vec_2
T3=T*vec_3
T4=T*vec_4

% 4 co-planar point of E-E frame described in /base_link frame
T1_c = (inv(T_cb)*inv(T_cb_temp))*T1
T2_c = (inv(T_cb)*inv(T_cb_temp))*T2
T3_c = (inv(T_cb)*inv(T_cb_temp))*T3
T4_c = (inv(T_cb)*inv(T_cb_temp))*T4

figure
plot(T(2,4),T(3,4),'*k')
hold on

plot(T1(2),T1(3),'*b')

plot(T2(2),T2(3),'*r')

plot(T3(2),T3(3),'*g')

plot(T4(2),T4(3),'*y')

legend('ref','c1','c2','c3','c4')
grid on
axis equal

figure
plot3(0,0,0,'*k')
hold on

plot3(T(1,4),T(2,4),T(3,4),'*k')
hold on

plot3(T1(1),T1(2),T1(3),'*b')

plot3(T2(1),T2(2),T2(3),'*r')

plot3(T3(1),T3(2),T3(3),'*g')

plot3(T4(1),T4(2),T4(3),'*y')

legend('origin','ref','c1','c2','c3','c4')
grid on
axis equal

omega = [476.703,0,400.5;0,476.703,300.5;0,0,1];
PI = [1 0 0 0; 0 1 0 0; 0 0 1 0];
c1_px = omega*PI*T1_c
c2_px = omega*PI*T2_c
c3_px = omega*PI*T3_c
c4_px = omega*PI*T4_c
c1_px=c1_px/c1_px(3)
c2_px=c2_px/c2_px(3)
c3_px=c3_px/c3_px(3)
c4_px=c4_px/c4_px(3)
c_mat = [c1_px, c2_px, c3_px, c4_px];
c_mat_norm = inv(omega)*c_mat;
s = [c_mat_norm(1,1), c_mat_norm(2,1),c_mat_norm(1,2), c_mat_norm(2,2),c_mat_norm(1,3), c_mat_norm(2,3),c_mat_norm(1,4), c_mat_norm(2,4)];

