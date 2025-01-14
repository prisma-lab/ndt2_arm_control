clear all
close all
clc

% pos = importdata("ep.txt",',');
% t = linspace(0,length(pos)/100,length(pos));
% figure
% plot(t,pos(:,1),'-k',LineWidth=2)
% hold on
% plot(t,pos(:,2),'--k',LineWidth=2)
% plot(t,pos(:,3),':k',LineWidth=2)
% set(gca,'fontsize',18)
% % title('Linear position error')
% legend('e_x','e_y','e_z')
% xlabel('t [s]')
% ylabel('e_p [m]')
% grid on
% 
% lin_vel = importdata("ev.txt",',');
% figure
% plot(t,lin_vel(:,1),'-k',LineWidth=2)
% hold on
% plot(t,lin_vel(:,2),'--k',LineWidth=2)
% plot(t,lin_vel(:,3),':k',LineWidth=2)
% set(gca,'fontsize',18)
% % title('Linear velocity error')
% legend('e_{xd}','e_{yd}','e_{zd}')
% xlabel('t [s]')
% ylabel('e_v [m/s]')
% grid on 
% 
% rpy = importdata("erpy.txt",',');
% figure
% plot(t,rpy(:,1),'-k',LineWidth=2)
% hold on
% plot(t,rpy(:,2),'--k',LineWidth=2)
% % title('Angular position error')
% plot(t,rpy(:,3),':k',LineWidth=2)
% set(gca,'fontsize',18)
% legend('r','p','y')
% xlabel('t [s]')
% ylabel('e_o [rad]')
% grid on
% 
% w = importdata("ew.txt",',');
% figure
% plot(t,w(:,1),'-k',LineWidth=2)
% hold on
% plot(t,w(:,2),'--k',LineWidth=2)
% % title('Angular velocity error')
% plot(t,w(:,3),':k',LineWidth=2)
% legend('e_w_x_d','e_w_y_d','e_w_z_d') 
% set(gca,'fontsize',18)
% xlabel('t [s]')
% ylabel('e_w [rad/s]')
% grid on
% 
% tau = importdata("tau.txt",',');
% t = linspace(0,length(tau)/100,length(tau));
% figure
% plot(t,tau)
% set(gca,'fontsize',18)
% xlabel('t [s]')
% grid on

f = importdata("f_ext.txt",',');
f(end+1,:) = f(end,:);
f(end+1,:) = f(end,:);
t = linspace(0,length(f)/100,length(f));
figure
plot(t,f)
set(gca,'fontsize',18)
legend('f_x','f_y','f_z')
xlabel('t [s]')
grid on

% e_s_x = importdata("esx.txt",',');
% e_s_y = importdata("esy.txt",',');
% e_s = [e_s_x(:,1),e_s_y(:,1), e_s_x(:,2),e_s_y(:,2), e_s_x(:,3),e_s_y(:,3), e_s_x(:,4),e_s_y(:,4)];
% figure
% t = linspace(0,length(e_s)/100,length(e_s));
% plot(t,e_s(:,1))
% hold on
% plot(t,e_s(:,2),'LineWidth',1.5)
% plot(t,e_s(:,3),'LineWidth',1.5)
% plot(t,e_s(:,4),'LineWidth',1.5)
% plot(t,e_s(:,5),'LineWidth',1.5)
% plot(t,e_s(:,6),'LineWidth',1.5)
% plot(t,e_s(:,7),'LineWidth',1.5)
% plot(t,e_s(:,8),'LineWidth',1.5)
% grid on 
% legend('c1_x','c1_y','c2_x','c2_y','c3_x','c3_y','c4_x','c4_y')

e_pr = importdata("epreg.txt",',');
e_vr = importdata("evreg.txt",',');
t = linspace(0,length(e_pr)/100,length(e_pr));

figure
plot(t,e_pr,'LineWidth',1.5)
legend('$\theta_1$','$\theta_2$','$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$','Interpreter','latex')
figure
plot(t,e_vr,'LineWidth',1.5)
legend('$\theta_1$','$\theta_2$','$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$','Interpreter','latex')

% %% low pass filter f_ext
% alpha = 0.1;
% f_ext_filtered = [0 0 0];
% f_ext_filtered_old = [0 0 0];
% output = [];
% for i=1:length(f)
%     f_ext = f(i,:);
%     f_ext_filtered = alpha * f_ext + (1.0 - alpha) * f_ext_filtered_old;
%     f_ext_filtered_old = f_ext_filtered;
%     output(i,:) = f_ext_filtered;
% end
% figure
% plot(t,output)
% grid on
% legend('f_x','f_y','f_z')
% xlabel('t [s]')
% set(gca,'fontsize',18)

