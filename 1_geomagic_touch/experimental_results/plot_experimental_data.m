close all
clear all
clc

g = load('expData.m');

g = g(1:1001,:);

t = g(:,1);
vpi_est1 = g(:,2);
vpi_est2 = g(:,3);
vpi_est3 = g(:,4);
vpi_est4 = g(:,5);
vpi_est5 = g(:,6);
vpi_est6 = g(:,7);
vpi_est7 = g(:,8);
vpi_est8 = g(:,9);
vpi_est9 = g(:,10);
vpi_est10 = g(:,11);
eth1 = g(:,12);
eth2 = g(:,13);
eth3 = g(:,14);
eth4 = g(:,15);
eth5 = g(:,16);
eth6 = g(:,17);
eth7 =g(:,18);
eth8 =g(:,19);
eth9 =g(:,20);
eth10 = g(:,21);
rankYf=g(:,22);
Phi2 = g(:,23)/1e25;
Phi2_m = g(:,24)/1e25;
zeta = g(:,25);
q1 = g(:,26)*180/pi;
q2 = g(:,27)*180/pi;
q3 = g(:,28)*180/pi;

%% ------------- Joint angular positions ----------------
figure()
sgtitle('Joint angular positions')

subplot(3,1,1)
plot(t,q1,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$q_1 \; [^\circ]$','Interpreter','latex','FontSize',18)
ylim([-55 60])

subplot(3,1,2)
plot(t,q2,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$q_2 \; [^\circ]$','Interpreter','latex','FontSize',18)

subplot(3,1,3)
plot(t,q3,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',18);
ylabel('$q_3 \; [^\circ]$','Interpreter','latex','FontSize',18)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% --------------- Estimated parameters ----------------
figure('WindowState', 'maximized')
sgtitle('Estimated parameters')

subplot(4,3,1)
plot(t,vpi_est1,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$\hat{\theta}_1 \; [kg \cdot m^2]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est1)+0.0008,sprintf('$$\\hat{\\theta}_1$$ = %.6f',vpi_est1(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est1) max(vpi_est1)+0.0021])

subplot(4,3,2)
plot(t,vpi_est2,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$\hat{\theta}_2 \; [kg \cdot m^2]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est2)+0.004,sprintf('$$\\hat{\\theta}_2$$ = %.6f',vpi_est2(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est2) max(vpi_est2)+0.01])

subplot(4,3,3)
plot(t,vpi_est3,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$\hat{\theta}_3 \; [kg \cdot m^2]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est3)+0.006,sprintf('$$\\hat{\\theta}_3$$ = %.6f',vpi_est3(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est3) max(vpi_est3)+0.016])

subplot(4,3,4)
plot(t,vpi_est4,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$\hat{\theta}_4 \; [kg \cdot m^2]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est4)+0.0009,sprintf('$$\\hat{\\theta}_4$$ = %.6f',vpi_est4(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est4) max(vpi_est4)+0.0023])

subplot(4,3,5)
plot(t,vpi_est5,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$\hat{\theta}_5 \; [kg \cdot m^2]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est5)+0.0004,sprintf('$$\\hat{\\theta}_5$$ = %.6f',vpi_est5(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est5) max(vpi_est5)+0.001])

subplot(4,3,6)
plot(t,vpi_est6,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$\hat{\theta}_6 \; [kg/s]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est6)+0.002,sprintf('$$\\hat{\\theta}_6$$ = %.6f',vpi_est6(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est6) max(vpi_est6)+0.005])

subplot(4,3,7)
plot(t,vpi_est7,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$\hat{\theta}_7 \; [kg/s]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est7)+0.013,sprintf('$$\\hat{\\theta}_7$$ = %.6f',vpi_est7(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est7) max(vpi_est7)+0.035])

subplot(4,3,8)
plot(t,vpi_est8,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$\hat{\theta}_8 \; [kg/s]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est8)+0.0015,sprintf('$$\\hat{\\theta}_8$$ = %.6f',vpi_est8(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est8) max(vpi_est8)+0.0051])

subplot(4,3,9)
plot(t,vpi_est9,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$\hat{\theta}_9 \; [kg \cdot m]$','Interpreter','latex','FontSize',15)
text(0.9,max(vpi_est9)+0.0013,sprintf('$$\\hat{\\theta}_9$$ = %.6f',vpi_est9(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est9) max(vpi_est9)+0.0033])

subplot(4,3,10)
plot(t,vpi_est10,'LineWidth', 1.2)
set(gca,'FontWeight','bold')
grid
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',15);
ylabel('$\hat{\theta}_{10} \; [kg \cdot m]$','Interpreter','latex','FontSize',15)
text(0.8,max(vpi_est10)+0.0017,sprintf('$$\\hat{\\theta}_{10}$$ = %.6f',vpi_est10(end)),'Interpreter', 'latex', 'fontsize', 10);
ylim([min(vpi_est10) max(vpi_est10)+0.0045])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% -------Phim^2 and phi^2-------------------------------------------------
figure()
subplot(2,1,1)
plot(t,Phi2_m,'b-','LineWidth',1.2)
set(gca,'FontWeight','bold')
grid
ylabel('$$\phi_{m}^2$$','Interpreter','Latex','FontSize',18) 
ylim([0 0.045])

subplot(2,1,2)
plot(t,Phi2,'b-','LineWidth',1.2)
set(gca,'FontWeight','bold')
grid
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',18);
ylabel('$$\phi^2$$','Interpreter','Latex','FontSize',18)
text(1.285,1.0e-9,'\leftarrow')
text(0.35,1.0e-9,'\phi^2 = \eta_m = 1.0 \times 10^{-9} \rightarrow')
