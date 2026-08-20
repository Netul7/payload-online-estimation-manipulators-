function out = controladorGT(in)

global tau e Yf Tf e_est Phi2 Phi_m Phi2_m qd dqr ddqr Tep %for the function outputs
global T qf dqf ddqf dqd ddqd
global n g0 fe taue dtaue Lambda Ke Kv Kp lambe lambs T_max
global lmb_p a2 a3 a4 b2 b3 b4 p Gamma lmb_t eta_m Phi_d e_t_min
global vpi_r
global vpi cont tau_p tau_pa tau_pb tau_pc dtau_pc e_t f_t dvpi signo Y_p1 Y_pa Y_pb Y_pc dY_pc
global Yap
global time q_pos theta_esti phi_m2 phi_2

%%%% Inputs %%%%
%%%%%%%%%%%%%%%%%%

t = in(1); %time

%joint acceleration is not required

% first derivative of the generalized coordinate (joint angular velocity)
dq1 = in(2);
dq2 = in(3);
dq3 = in(4);

dq = [dq1; dq2; dq3];

% generalized coordinate (joint angular position)
q1 = in(5);
q2 = in(6);
q3 = in(7);

q = [q1; q2; q3];

t_actual = T*cont;

if t == t_actual

cont = cont + 1;

% parameter estimation error

e_est=vpi-vpi_r;

% trigonometric expressions
s2=sin(q2);
s3=sin(q3);
s23=sin(q2+q3);
c2=cos(q2);
c3=cos(q3);
c23=cos(q2+q3);

% desired position (constant)
qd = qf;
dqd = dqf;
ddqd = ddqf;

%% The following variables are related to terms in the implementation of the control law

% position error and its derivative  
e=q-qd;
de=dq-dqd;

%this term appears in the reference velocity and sliding surface:
for i = 1:n
    fe(i) = abs(e(i))^(lambe(i)*tanh(e(i)*e(i)));
    taue(i) = sign(e(i))*(tanh(abs(e(i))^(1/2))/tanh(1))*abs(e(i))^((1/2)*(1-2*tanh(e(i)*e(i))/tanh(1)))*fe(i);
end

%reference velocity
dqr=dqd - Lambda*e - Ke*taue;
dqr1=dqr(1);
dqr2=dqr(2);
dqr3=dqr(3);

%this term appears in the reference acceleration
for i = 1:n
    %there is an error when e(i)=0 in ln(e(i))=-inf, that's why this
    %if-statement
    dtaue(i) = ( ((2*lambe(i)*e(i)*de(i))/((cosh(e(i)*e(i)))^2))*(log(abs(e(i)))) - ((2*e(i)*de(i))/(tanh(1)*(cosh(e(i)*e(i)))^2))*(log(abs(e(i)))) + (lambe(i)*tanh(e(i)*e(i)) + (1/2)*(1 - 2*tanh(e(i)*e(i))/tanh(1)))*(de(i)/e(i)) )*taue(i) + ( (abs(e(i))^(lambe(i)*tanh(e(i)*e(i)) + (1/2)*(1 - 2*tanh(e(i)*e(i))/tanh(1))))/(2*(abs(e(i))^(1/2))*tanh(1)*(cosh(abs(e(i))^(1/2)))^2) )*de(i);
    if e(i) == 0
        dtaue(i)=0;
    end
end
%reference acceleration
ddqr=ddqd - Lambda*de - Ke*dtaue;
ddqr1=ddqr(1);
ddqr2=ddqr(2);
ddqr3=ddqr(3);

s=de + Lambda*e + Ke*taue;  %sliding surface
%% 


% Regressor
y11 = c2 * c2 * ddqr1 - c2 * s2 * dq2 * dqr1 - c2 * s2 * dq1 * dqr2;
y12 = 2*c2 * c23 * ddqr1 - 0.5 * c2 * s23 * ( (dq2 + dq3) * dqr1 + (dqr2 + dqr3) * dq1 ) - 0.5 * s2 * c23 * (dq2*dqr1 + dq1*dqr2) ;%%%
y13 = c23 * c23 * ddqr1 - c23 * s23 * ( (dq2 + dq3) * dqr1 + dq1 * dqr3 )  - s23 * c23 * dq1 * dqr2;
y14 = s23 * s23 * ddqr1 + c23 * s23 * ( (dq2 + dq3) * dqr1 + dq1 * dqr3 )  + s23 * c23 * dq1 * dqr2;
y15 = 0;
y16 = dqr1;
y17 = 0;
y18 = 0;
y19 = 0;
y110 = 0;

y21 = ddqr2 + c2 * s2 * dq1 * dqr1;
y22 = 2 * c3 * ddqr2 + c3 * ddqr3 + (1 / 2) * (s2 * c23 + c2 * s23) * dq1 * dqr1 - s3 * dqr2 * dq3 - s3 *( dq2 + dq3 ) * dqr3;
y23 = s23 * c23 * dq1 * dqr1;
y24 = - s23 * c23 * dq1 * dqr1;
y25 = ddqr2 + ddqr3;
y26 = 0;
y27 = dqr2;
y28 = 0;
y29 = g0 * c2;
y210 = g0 * c23;

y31 = 0;
y32 = c3 * ddqr2 + (1 / 2) * c2 * s23 * dq1 * dqr1 + s3 * dq2 * dqr2; %%%% En el libro hay error en el primer termino, debe ser c3 en lugar de c2
y33 = c23 * s23 * dq1 * dqr1;
y34 = - c23 * s23 * dq1 * dqr1;
y35 = ddqr2 + ddqr3;
y36 = 0;
y37 = 0;
y38 = dqr3;
y39 = 0;
y310 = g0 * c23;

Ya = [y11, y12, y13, y14, y15, y16, y17, y18, y19, y110; y21, y22, y23, y24, y25, y26, y27, y28, y29, y210; y31, y32, y33, y34, y35, y36, y37, y38, y39, y310];


% robust control law

tau_s = sign(s).*abs(s).^(lambs.*tanh(s.^2));
tau = Ya*vpi-Kv*tau_s-Kp*norm(s,2)*s;

% torque saturation
for h = 1:n
    if abs(tau(h)) > T_max
        tau(h) = T_max*sign(tau(h));
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% MDREM %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%constructing the extended regressor and extended input torque vector from
%filtered versions 
Yf = [Y_p1; Y_pa; Y_pb; Y_pc];
Tf = [tau_p; tau_pa; tau_pb; tau_pc];


zeta=1; %variable introduced in the MDREM method
Phi=det(Yf); %determinant of the extended regressor 
Phi2=Phi*Phi;

if Phi2 < eta_m
    zeta = 1;
elseif Phi2 >= eta_m
    zeta = (Phi_d/abs(Phi))^(1/p);
end


Ym=zeta*Yf; %MDREM extended regressor
Tm=zeta*Tf; %MDREM extended input vector

Tep=adjoint(Ym)*Tm; %to use function adjoint() symbolic math toolbox is required
Phi_m=det(Ym); %determinant of the MDREM extended regressor
Phi2_m = Phi_m*Phi_m;

%% The following variables are related to terms of the adaptive law
for k = 1:p
    e_t(k)=Phi_m*(Phi_m*vpi(k)-Tep(k)); %this is a kind of a metric of the parameter estimation error
    f_t(k)=(abs(e_t(k))^(lmb_t*tanh(e_t(k)*e_t(k) )));

    signo(k) = sign(e_t(k));
    if abs(e_t(k)) < e_t_min(k)
        signo(k) = 0;
    end
    dvpi(k)=-Gamma(k,k)*signo(k)*f_t(k); %dynamics of the parameter estimation error
end

for i = 1:p
    vpi(i) = vpi(i) + dvpi(i)*T; %estimated parameters
end
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Filtered input %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%applying a first-order filter
dtau_p = -lmb_p*tau_p + lmb_p*tau;
tau_p = tau_p + dtau_p*T;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Extended input %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%A second filter is applied to the already filtered input vector 
%LTI first-order filters are chosen as stable operators to compute the
%filtered input vectors that make up the extended input vector 

% stable operator 1
dtau_pa = -b2*tau_pa + a2*tau_p;
tau_pa = tau_pa + dtau_pa*T;

% stable operator 2
dtau_pb = -b3*tau_pb + a3*tau_p;
tau_pb = tau_pb + dtau_pb*T;

% stable operator 3
dtau_pc(1) = -b4*tau_pc(1) + a4*tau_p(1);
tau_pc = tau_pc + dtau_pc*T;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Filtered regressor %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%applying a first-order filter to avoid joint acceleration 
%note that before this implementation, integration by parts must be performed

yfa11 = c2*c2*dq1;
yfa12 = 2*c2*c23*dq1;
yfa13 = c23*c23*dq1;
yfa14 = s23*s23*dq1;

yfa21 = dq2;
yfa22 = 2*c3*dq2 + c3*dq3;
yfa25 = dq2 + dq3;

yfa32 = c3*dq2;
yfa35 = yfa25;

Yfa = [yfa11, yfa12, yfa13, yfa14, 0, 0, 0, 0, 0, 0; yfa21, yfa22, 0, 0, yfa25, 0, 0, 0, 0, 0; 0, yfa32, 0, 0, yfa35, 0, 0, 0, 0, 0];

Y_p1 = Yfa + Yap;

%We defined an auxiliar regressor
 
ya11 = -lmb_p*c2*c2*dq1;
ya12 = -2*lmb_p*c2*c23*dq1 + s2*c23*dq1*dq2 + c2*s23*(dq2+dq3)*dq1;
ya13 = -lmb_p*c23*c23*dq1;
ya14 = -lmb_p*s23*s23*dq1;
ya16 = dq1;

ya21 = -lmb_p*dq2 + c2*s2*dq1*dq1;
ya22 = -2*lmb_p*c3*dq2 - lmb_p*c3*dq3 + 0.5*(s2*c23 + c2*s23)*dq1*dq1;
ya23 = s23*c23*dq1*dq1;
ya24 = - s23*c23*dq1*dq1;
ya25 = - lmb_p* (dq2 + dq3);
ya27 = dq2;
ya29 = g0*c2;
ya210 = g0*c23;

ya32 = - lmb_p*c3*dq2 + s3*dq2*dq3 + 0.5*c2*s23*dq1*dq1 + s3*dq2*dq2;
ya33 = c23*s23*dq1*dq1;
ya34 = - c23*s23*dq1*dq1;
ya35 = ya25;
ya38 =  dq3;
ya310 = g0*c23;

Yaux = [ya11, ya12, ya13, ya14, 0, ya16, 0, 0, 0, 0; ya21, ya22, ya23, ya24, ya25, 0, ya27, 0, ya29, ya210; 0, ya32, ya33, ya34, ya35, 0, 0, ya38, 0, ya310];

Yap_p = -lmb_p*Yap + lmb_p*Yaux; %the actual filter
Yap = Yap + Yap_p*T; %the previous line filter is integrated

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Extended regressor %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%similar to the filtered input, a second filter is applied to the already filtered regressor
%the exact same LTI first-order filters are applied to the filtered
%regressor in the same order to make up the extended regressor

%stable operator 1
dY_pa = -b2*Y_pa + a2*Y_p1;
Y_pa = Y_pa + dY_pa*T;

%stable operator 2
dY_pb = -b3*Y_pb + a3*Y_p1;
Y_pb = Y_pb + dY_pb*T;

%stable operator 3
dY_pc(1,:) = -b4*Y_pc(1,:) + a4*Y_p1(1,:);
Y_pc = Y_pc + dY_pc*T;

%% ----------------------------------------
%these variables are for storing simulation data for plotting
time(cont) = t;
q_pos(cont,:) = e;
theta_esti(cont,:) = e_est;
phi_m2(cont) = Phi2_m;
phi_2(cont) = Phi2;

end

%%%%%% output %%%%%%%
out = [tau; e; e_est; Phi2; Phi2_m; qd]; %dim=[3, 3, 10, 1, 1, 3] = 21 

 % The following saves the data in a .mat file to process and plot it in 
 % another matlab file

if t == 1
    time = time';
    phi_m2  = phi_m2';
    phi_2 = phi_2';
    save simu time q_pos theta_esti phi_m2 phi_2
end
