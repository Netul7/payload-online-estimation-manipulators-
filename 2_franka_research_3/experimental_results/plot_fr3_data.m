clc
clear
close all

data = readmatrix('exp_data_FR3.txt');
data = data(1:3281,:);

t = data(:, 1);
q = data(:, 2:8);
theta = data(:, 54:63);
phi = data(:,64)./(1e25);
phim = data(:,65)./(1e25);

% PARAMETERS OF THE PAYLOAD:
link_8_mass = 0.73;   %kg; mass of the payload
link_8_CoM = [-0.01 0 0.03]';  % m; center of mass

%Inertia tensor elements with respect its center mass frame 
link_8_MoI = [0.001 0.0025 0.0017];  % kg*m^2 original
link_8_PoI = [0 0 0];  % kg*m^2  original

%Inertia tensor with respect its center mass frame 
Ig =[link_8_MoI(1), link_8_PoI(1), link_8_PoI(2);
   link_8_PoI(1), link_8_MoI(2), link_8_PoI(3);
   link_8_PoI(2), link_8_PoI(3), link_8_MoI(3)];

%Inertia tensor with respect the 7th link coordinate frame 
Ji = Ig + link_8_mass*(link_8_CoM'*link_8_CoM*eye(3)-link_8_CoM*link_8_CoM');

%Inertia tensor elements with respect the 7th link coordinate frame 
link_8_MoI = [Ji(1) Ji(5) Ji(9)];
link_8_PoI = [Ji(2) Ji(3) Ji(6)];

thetaL=[link_8_mass;link_8_mass*link_8_CoM(1);link_8_mass*link_8_CoM(2);link_8_mass*link_8_CoM(3);link_8_MoI(1);link_8_PoI(1);link_8_PoI(2);link_8_MoI(2);link_8_PoI(3);link_8_MoI(3)];

%% -----Positions---------------------------------------------------------
figure()
for i=1:7
    subplot(2,4,i)
    plot(t,q(:,i)*180/pi,'LineWidth',1.2)
    set(gca,'FontWeight','bold')
    xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',18);
    ylabeltext = sprintf('${q}_{%d} [^{\\circ}]$',i);
    ylabel(ylabeltext ,'Interpreter','Latex','FontSize',18)
end

%% ----------- Parameter estimation error ---------------------------
err_param = zeros(length(t),10);
for i = 1:10
err_param(:,i) = theta(:,i) - thetaL(i)*ones(length(t),1);
end

figure('WindowState', 'maximized')
for i=1:10
    subplot(4,3,i)
    plot(t,err_param(:,i),'LineWidth',1.5)
    set(gca,'FontWeight','bold')
    grid on
    xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',14);
    if (i==1)
        ylabeltext = sprintf('$\\tilde{\\theta}_{L{%d}}$ [{kg}]',i);
        ylabel(ylabeltext ,'Interpreter','Latex','FontSize',14)
    elseif (i>1) && (i<5)
        ylabeltext = sprintf('$\\tilde{\\theta}_{L{%d}}$ [{kg $\\cdot$ m}]',i);
        ylabel(ylabeltext ,'Interpreter','Latex','FontSize',14)
    elseif (i==7) % It was added to adjust the text
        ylabeltext = sprintf('$\\tilde{\\theta}_{L{%d}}$ [{kg $\\cdot$ m$^2$}]',i);
        ylabel(ylabeltext ,'Interpreter','Latex','FontSize',14)
    else
        ylabeltext = sprintf('$\\tilde{\\theta}_{L{%d}}$ [{kg $\\cdot$ m$^2$}]',i);
        ylabel(ylabeltext ,'Interpreter','Latex','FontSize',14)
    end
    yl = ylim;   % It obtains the current limits of the Y axis
    margen = 0.09 * (yl(2) - yl(1));  % 9% of the upper margin
    ylim([yl(1) - margen, yl(2) + margen]);   % It applies the new limit
end

%% -------Phim^2 and phi^2-------------------------------------------------
figure()
subplot(2,1,1)
plot(t,phim.^2,'LineWidth',1.2)
set(gca,'FontWeight','bold')
grid on
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',18);
ylabel('$$\phi_{m}^2$$','Interpreter','Latex','FontSize',18) 
ylim([0 4.2e-4])

subplot(2,1,2)
plot(t,phi.^2,'LineWidth',1.2)
set(gca,'FontWeight','bold')
grid on
xlabel('$$t {\rm [s]}$$','Interpreter','Latex','FontSize',18);
ylabel('$$\phi^2$$','Interpreter','Latex','FontSize',18)
text(2.58,2.5e-8,'\leftarrow')
text(1.06,2.5e-8,'\phi^2 = \eta_m = 2.5 \times 10^{-8} \rightarrow')
