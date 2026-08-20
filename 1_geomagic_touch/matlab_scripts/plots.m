clear;
close all;
clc;
 
load simu
q = q_pos;
t = time;

%% -----Position errors---------------------------------------------------------
figure()
subplot(3,1,1)
plot(t,q(:,1)*180/pi,'b-','LineWidth',0.90)
set(gca,'FontSize',12)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',18);
ylabel('$${e}_1[^\circ]$$','Interpreter','Latex','FontSize',19)
grid on

subplot(3,1,2)
plot(t,q(:,2)*180/pi,'b-','LineWidth',0.90)
set(gca,'FontSize',12)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',18);
ylabel('$${e}_2[^\circ]$$','Interpreter','Latex','FontSize',19)
grid on

subplot(3,1,3)
plot(t,q(:,3)*180/pi,'b-','LineWidth',0.90)
set(gca,'FontSize',12)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',18);
ylabel('$${e}_3[^\circ]$$','Interpreter','Latex','FontSize',19)
grid on
sgtitle('Position errors');

%% -----------Parameter estimation errors---------------------------

figure('Position', [10, 50, 750, 1000]);
subplot(4,3,1)
plot(t,theta_esti(:,1),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_1[\rm{kg\cdot m^2}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,2)
plot(t,theta_esti(:,2),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_2[\rm{kg \cdot m^2}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,3)
plot(t,theta_esti(:,3),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_3[\rm{kg \cdot m^2}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,4)
plot(t,theta_esti(:,4),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_4[\rm{kg \cdot m^2}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,5)
plot(t,theta_esti(:,5),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_5[\rm{kg \cdot m^2}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,6)
plot(t,theta_esti(:,6),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_6[\rm{kg/}  \rm{s}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,7)
plot(t,theta_esti(:,7),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_7[\rm{kg}/ \rm{s}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,8)
plot(t,theta_esti(:,8),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_8[\rm{kg}/ \rm{s}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,9)
plot(t,theta_esti(:,9),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_9[\rm{kg} \cdot \rm{m}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

subplot(4,3,10)
plot(t,theta_esti(:,10),'b-','LineWidth',0.75)
set(gca,'FontSize',11)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$$\tilde{\theta}_{10}[\rm{kg} \cdot \rm{m}]$$','Interpreter','Latex','FontSize',15)
ylim padded
grid on

sgtitle('Parameter estimation errors');

%% -------Phim^2 and phi^2
figure()
subplot(2,1,1)
plot(t,phi_m2(:),'b-','LineWidth',0.75)
set(gca,'FontSize',12)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',19);
ylabel('$$\phi_{m}^2$$','Interpreter','Latex','FontSize',19) 
ylim([0 0.045])
grid on

subplot(2,1,2)
plot(t,phi_2(:),'b-','LineWidth',0.75)
set(gca,'FontSize',12)
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',19);
ylabel('$$\phi^2$$','Interpreter','Latex','FontSize',19)
grid on
