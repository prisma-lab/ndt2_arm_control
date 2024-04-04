%% Haptic device data
pos_falcon = importdata("p_novint.txt",',');
t = linspace(0,length(pos_falcon)/100,length(pos_falcon));
figure
plot(t,pos_falcon(1,:),'-k',LineWidth=2)
hold on
plot(t,pos_falcon(2,:),'--k',LineWidth=2)
plot(t,pos_falcon(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('x','y','z')
xlabel('t [s]')
ylabel('p [m]')
grid on

v_falcon = importdata("v_novint.txt",',');
t = linspace(0,length(v_falcon)/100,length(v_falcon));
figure
plot(t,v_falcon(1,:),'-k',LineWidth=2)
hold on
plot(t,v_falcon(2,:),'--k',LineWidth=2)
plot(t,v_falcon(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('vx','vy','vz')
xlabel('t [s]')
ylabel('v [m/s]')
grid on

vs_falcon = importdata("vs_novint.txt",',');
t = linspace(0,length(vs_falcon)/100,length(vs_falcon));
figure
plot(t,vs_falcon(1,:),LineWidth=2)
hold on
plot(t,vs_falcon(2,:),LineWidth=2)
plot(t,vs_falcon(3,:),LineWidth=2)
plot(t,vs_falcon(4,:),LineWidth=2)
plot(t,vs_falcon(5,:),LineWidth=2)
plot(t,vs_falcon(6,:),LineWidth=2)
plot(t,vs_falcon(7,:),LineWidth=2)
plot(t,vs_falcon(8,:),LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('vx1','vy1','vx2','vy2','vx3','vy3','vx4','vy4')
xlabel('t [s]')
ylabel('v [pxl/s]')
grid on

f_falcon = importdata("f_novint.txt",',');
t = linspace(0,length(f_falcon)/100,length(f_falcon));
figure
plot(t,f_falcon(1,:),'-k',LineWidth=2)
hold on
plot(t,f_falcon(2,:),'--k',LineWidth=2)
plot(t,f_falcon(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('fx','fy','fz')
xlabel('t [s]')
ylabel('F [N]')
grid on


%% arm device data
pos_arm = importdata("p_arm.txt",',');
t = linspace(0,length(pos_arm)/100,length(pos_arm));
figure
plot(t,pos_arm(1,:),'-k',LineWidth=2)
hold on
plot(t,pos_arm(2,:),'--k',LineWidth=2)
plot(t,pos_arm(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('x','y','z')
xlabel('t [s]')
ylabel('p [m]')
grid on

v_arm = importdata("v_arm.txt",',');
t = linspace(0,length(v_arm)/100,length(v_arm));
figure
plot(t,v_arm(1,:),'-k',LineWidth=2)
hold on
plot(t,v_arm(2,:),'--k',LineWidth=2)
plot(t,v_arm(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('vx','vy','vz')
xlabel('t [s]')
ylabel('v [m/s]')
grid on

rpy_arm = importdata("rpy_arm.txt",',');
t = linspace(0,length(rpy_arm)/100,length(rpy_arm));
figure
plot(t,rpy_arm(1,:),'-k',LineWidth=2)
hold on
plot(t,rpy_arm(2,:),'--k',LineWidth=2)
plot(t,rpy_arm(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('r','p','y')
xlabel('t [s]')
ylabel('\theta [rad]')
grid on

w_arm = importdata("w_arm.txt",',');
t = linspace(0,length(w_arm)/100,length(w_arm));
figure
plot(t,w_arm(1,:),'-k',LineWidth=2)
hold on
plot(t,w_arm(2,:),'--k',LineWidth=2)
plot(t,w_arm(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('wx','wy','wz')
xlabel('t [s]')
ylabel('w [rad/s]')
grid on

s_arm = importdata("ps_arm.txt",',');
t = linspace(0,length(s_arm)/100,length(s_arm));
figure
plot(t,s_arm(1,:),LineWidth=2)
hold on
plot(t,s_arm(2,:),LineWidth=2)
plot(t,s_arm(3,:),LineWidth=2)
plot(t,s_arm(4,:),LineWidth=2)
plot(t,s_arm(5,:),LineWidth=2)
plot(t,s_arm(6,:),LineWidth=2)
plot(t,s_arm(7,:),LineWidth=2)
plot(t,s_arm(8,:),LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('px1','py1','px2','py2','px3','py3','px4','py4')
xlabel('t [s]')
ylabel('p [pxl]')
grid on

sd_arm = importdata("ps_des_arm.txt",',');
t = linspace(0,length(sd_arm)/100,length(sd_arm));
figure
plot(t,sd_arm(1,:),LineWidth=2)
hold on
plot(t,sd_arm(2,:),LineWidth=2)
plot(t,sd_arm(3,:),LineWidth=2)
plot(t,sd_arm(4,:),LineWidth=2)
plot(t,sd_arm(5,:),LineWidth=2)
plot(t,sd_arm(6,:),LineWidth=2)
plot(t,sd_arm(7,:),LineWidth=2)
plot(t,sd_arm(8,:),LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('px1','py1','px2','py2','px3','py3','px4','py4')
xlabel('t [s]')
ylabel('p [pxl]')
grid on

vs_arm = importdata("vs_arm.txt",',');
t = linspace(0,length(vs_arm)/100,length(vs_arm));
figure
plot(t,vs_arm(1,:),LineWidth=2)
hold on
plot(t,vs_arm(2,:),LineWidth=2)
plot(t,vs_arm(3,:),LineWidth=2)
plot(t,vs_arm(4,:),LineWidth=2)
plot(t,vs_arm(5,:),LineWidth=2)
plot(t,vs_arm(6,:),LineWidth=2)
plot(t,vs_arm(7,:),LineWidth=2)
plot(t,vs_arm(8,:),LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('vx1','vy1','vx2','vy2','vx3','vy3','vx4','vy4')
xlabel('t [s]')
ylabel('v [pxl/s]')
grid on

f_arm = importdata("f_arm.txt",',');
t = linspace(0,length(f_arm)/100,length(f_arm));
figure
plot(t,f_arm(1,:),'-k',LineWidth=2)
hold on
plot(t,f_arm(2,:),'--k',LineWidth=2)
plot(t,f_arm(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('fx','fy','fz')
xlabel('t [s]')
ylabel('F [N]')
grid on

fd_arm = importdata("f_des_arm.txt",',');
t = linspace(0,length(fd_arm)/100,length(fd_arm));
figure
plot(t,fd_arm,'-k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('f_des')
xlabel('t [s]')
ylabel('F [N]')
grid on

%% uav device data
pos_uav = importdata("p_uav.txt",',');
t = linspace(0,length(pos_uav)/100,length(pos_uav));
figure
plot(t,pos_uav(1,:),'-k',LineWidth=2)
hold on
plot(t,pos_uav(2,:),'--k',LineWidth=2)
plot(t,pos_uav(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('x','y','z')
xlabel('t [s]')
ylabel('p [m]')
grid on

v_uav = importdata("v_uav.txt",',');
t = linspace(0,length(v_uav)/100,length(v_uav));
figure
plot(t,v_uav(1,:),'-k',LineWidth=2)
hold on
plot(t,v_uav(2,:),'--k',LineWidth=2)
plot(t,v_uav(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('vx','vy','vz')
xlabel('t [s]')
ylabel('v [m/s]')
grid on

rpy_uav = importdata("rpy_uav.txt",',');
t = linspace(0,length(rpy_uav)/100,length(rpy_uav));
figure
plot(t,rpy_uav(1,:),'-k',LineWidth=2)
hold on
plot(t,rpy_uav(2,:),'--k',LineWidth=2)
plot(t,rpy_uav(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('r','p','y')
xlabel('t [s]')
ylabel('\theta [rad]')
grid on

w_uav = importdata("w_uav.txt",',');
t = linspace(0,length(w_uav)/100,length(w_uav));
figure
plot(t,w_uav(1,:),'-k',LineWidth=2)
hold on
plot(t,w_uav(2,:),'--k',LineWidth=2)
plot(t,w_uav(3,:),':k',LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('wx','wy','wz')
xlabel('t [s]')
ylabel('w [rad/s]')
grid on

s_uav = importdata("ps_uav.txt",',');
t = linspace(0,length(s_uav)/100,length(s_uav));
figure
plot(t,s_uav(1,:),LineWidth=2)
hold on
plot(t,s_uav(2,:),LineWidth=2)
plot(t,s_uav(3,:),LineWidth=2)
plot(t,s_uav(4,:),LineWidth=2)
plot(t,s_uav(5,:),LineWidth=2)
plot(t,s_uav(6,:),LineWidth=2)
plot(t,s_uav(7,:),LineWidth=2)
plot(t,s_uav(8,:),LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('px1','py1','px2','py2','px3','py3','px4','py4')
xlabel('t [s]')
ylabel('p [pxl]')
grid on

sd_uav = importdata("ps_des_uav.txt",',');
t = linspace(0,length(sd_uav)/100,length(sd_uav));
figure
plot(t,sd_uav(1,:),LineWidth=2)
hold on
plot(t,sd_uav(2,:),LineWidth=2)
plot(t,sd_uav(3,:),LineWidth=2)
plot(t,sd_uav(4,:),LineWidth=2)
plot(t,sd_uav(5,:),LineWidth=2)
plot(t,sd_uav(6,:),LineWidth=2)
plot(t,sd_uav(7,:),LineWidth=2)
plot(t,sd_uav(8,:),LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('px1','py1','px2','py2','px3','py3','px4','py4')
xlabel('t [s]')
ylabel('p [pxl]')
grid on

vs_uav = importdata("vs_uav.txt",',');
t = linspace(0,length(vs_uav)/100,length(vs_uav));
figure
plot(t,vs_uav(1,:),LineWidth=2)
hold on
plot(t,vs_uav(2,:),LineWidth=2)
plot(t,vs_uav(3,:),LineWidth=2)
plot(t,vs_uav(4,:),LineWidth=2)
plot(t,vs_uav(5,:),LineWidth=2)
plot(t,vs_uav(6,:),LineWidth=2)
plot(t,vs_uav(7,:),LineWidth=2)
plot(t,vs_uav(8,:),LineWidth=2)
set(gca,'fontsize',18)
% title('Linear position error')
legend('vx1','vy1','vx2','vy2','vx3','vy3','vx4','vy4')
xlabel('t [s]')
ylabel('v [pxl/s]')
grid on