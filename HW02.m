%% Parameter Setting:
m = 1575;
Iz = 2875;
lf = 1.2;
lr = 1.6;
Cf = 19000;
Cr = 33000;
Vx = 15;
%% Dynamic system
A = [-(2*Cf+2*Cr)/m/Vx, 0, -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx, 0;
0, 0, 1, 0;
-(2*Cf*lf-2*Cr*lr)/Iz/Vx, 0, -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx, 0; 
1, Vx, 0, 0];
B = [2*Cf/m 0 2*Cf*lf/Iz 0]';
C = [0 0 0 1; 0 1 0 0];
vehicle = ss(A,B,C,0);
%% Sampling Time and Simulation Time 
T = 15;% simulation duration
Ts=0.1;% Sampling time
time = 0:0.1:T; % simulation time
% Discrete-time model
Gd = c2d(vehicle,Ts);
Ad = Gd.A;
Bd = Gd.B;
C = Gd.C;
%% Load Trajectory Data 
load Ref.mat
%% Define data for MPC controller 
N = 5;
Q = [100 0; 0 100];
Qf = [100 0; 0 100];
R = 0.01;
%%
yalmip('clear')
%%
nx = 4;%# state
nu = 1;%# input
no = 2;%# output
u = sdpvar(repmat(nu,1,150),repmat(1,1,150));
x = sdpvar(repmat(nx,1,151),repmat(1,1,151));
y = sdpvar(repmat(no,1,150),repmat(1,1,150));
r = sdpvar(repmat(no,1,150),repmat(1,1,150));
objective1 = 0;
objectivef = 0;
constraints = [];
syms X Y U;
X = zeros(4,150);
U = zeros(1,145);
X(:,1) = 0;

for j=1:1:150-N
    if j == 1
%% Design your MPC controller using YALMIP toolbox
%% Note that here you need to get a function in terms of the following:
objective = (C*x{1+5}-r{1+5})'*Qf*(C*x{1+5}-r{1+5});
    for m = 1:5
objective = objective + (C*x{m}-r{m})'*Q*(C*x{m}-r{m})+u{m}'*R*u{m};
constraints = [constraints,x{m+1} == Ad*x{m} + Bd*u{m}];
constraints = [constraints,-0.05<=u{m+1}-u{m}<=0.05];
constraints = [constraints,-0.1<=u{m}<=0.1];
constraints = [constraints,-0.5 <= C*x{m}-r{m} <= 0.5];
    end
    end
Controller = optimizer(constraints,objective,[],{x{1},r{1},r{2},r{3},r{4},r{5},r{6}},u{1});%%%    
uout = Controller{{X(:,j),Yd(:,j),Yd(:,j+1),Yd(:,j+2),Yd(:,j+3),Yd(:,j+4),Yd(:,j+5)}};
X(:,j+1) = Ad*X(:,j)+Bd*uout;
%%Y(:,j) =C*X(:,j);????????????????????????????????
U(:,j) = uout;
end
Y = C*X;%%????????????????????????
figure(1);
plot(Y(1,1:145),'b-','linewidth',3); 
hold on;
plot(Yd(1,1:145),'r-','linewidth',3); 
legend('actual trajectory','reference');
axis tight