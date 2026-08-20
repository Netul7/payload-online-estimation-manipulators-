clc;
clear;

global T qf dqf ddqf dqd ddqd
global n g0 fe taue dtaue Lambda Ke Kv Kp lambe lambs T_max
global lmb_p a2 a3 a4 b2 b3 b4 p Gamma lmb_t eta_m Phi_d e_t_min
global vpi_r
global vpi cont tau_p tau_pa tau_pb tau_pc dtau_pc e_t f_t dvpi signo Y_p1 Y_pa Y_pb Y_pc dY_pc
global Yap
global time q_pos theta_esti phi_m2 phi_2

%Sample time
T = 1e-5;

%Initial position;
qi = [45; -30; -100]; %Degrees
qi=qi.*(pi/180);    %Radians

%final desired position
qf=[-45; 60; -10];  %Degrees
qf=qf.*(pi/180);    %Radians

%final velocity and acceleration
dqf=[0; 0; 0];
ddqf=[0; 0; 0];

%Initial velocity and acceleration 
dqd=[0; 0; 0];
ddqd=[0; 0; 0];

%%%% Control law parameters %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n=3;    %degrees of Freedom (DoF)
g0 = 9.81; %gravity

fe = [0;0;0]; %preallocating for speed
taue = [0;0;0]; %preallocating for speed
dtaue = [0;0;0]; %preallocating for speed

%Control gains:
Lambda = 20*eye(n);
lambe = [2.813; 2.813; 2.813]; 
lambs = [2; 2; 2];
Ke=0.8*eye(n);
Kv =1*[0.01; 0.04; 0.01];
Kv = diag(Kv);
Kp = 0.01*eye(n); 


T_max = 200; %for saturating the control input torque

%%% Adaptive law parameters %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p=10; %number of parameters to estimate

Gamma = 0.1*[0.14; 0.1; 0.59; 0.0044; 0.17; 0.0385; 1.41; 0.31; 0.81; 0.695];
Gamma = diag(Gamma);
lmb_t=2;   % the i-th element for i=1,...,p

e_t_min = 1e-10*[1;1;1;0.01;1;1;1;1;1;1]; %This defines a threshold where the sign of the error is zero

e_t=zeros(p,1); %preallocating for speed
f_t=zeros(p,1); %preallocating for speed
dvpi=zeros(p,1); %preallocating for speed
signo=zeros(p,1); %preallocating for speed

%%% Robot +  payload parameters %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% used to compute the estimated paramater errors

vpi1 = 0.00349359;
vpi2 = 0.00248272;
vpi3 = 0.00459359;
vpi4 = 0.00006993;
vpi5 = 0.00465868;
vpi6 = 0.00076823;
vpi7 = 0.03526735;
vpi8 = 0.00744473;
vpi9 = 0.02191225;
vpi10 =0.01712223;


vpi_r = [vpi1; vpi2; vpi3; vpi4; vpi5; vpi6; vpi7; vpi8; vpi9; vpi10];

%%% Parameters for the MDREM algorithm %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lmb_p=1; %used in the first-order filter to avoid using acceleration 


%these are used in the filters to construct the exteded regressor
b2=1.35;
b3=3.46;
b4=5;
a2 = 9;
a3=a2;
a4=a3;

eta_m=0.0001; %appears in the convergence condition of the MDREM
Phi_d=0.2;

cont = 0;

vpi=zeros(10,1); %initial conditions of the estimated parameters

%These are the initial conditions for the filters of the regressor and input
%torque vector:
tau_p = zeros(3,1);
tau_pa = zeros(3,1);
tau_pb = zeros(3,1);
tau_pc = zeros(1,1);
dtau_pc = zeros(1,1);

Y_p1 = zeros(3,10);
Y_pa = zeros(3,10);
Y_pb = zeros(3,10);
Y_pc = zeros(1,10);
dY_pc = zeros(1,10);

Yap = zeros(3,10);

%these variables are for storing simulation data for plotting 
time = 0;
q_pos = zeros(1,3); %joint positions
theta_esti = zeros(1,10); % estimated parameters
phi_m2 = 0; %squared determinant of the MDREM regressor
phi_2 = 0; %squared determinant of the extended regressor