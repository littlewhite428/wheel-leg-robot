m = 3.0 + 2 * (0.017303 + 0.016649+0.005); % 车身重 + 两条腿重 + 两个髋关节
l = 0.13; % 质心到转轴距离
M_w  = 0.09293; % 单个轮子重
I_w = 7.0742E-05; % 轮子绕转轴转动惯量
I_p = 0.0035 + m * l^2; % 车身到转轴转动惯量 平行轴定理 
r = 0.04; % 车轮半径
g = 9.8; % 重力加速度
D = 0.22; % 轮距

m11 = (m+2*M_w)*r + 2*(I_w)/r;
m12 = m*l*r;
m21 = m*l;
m22 = I_p + m*l^2;
m31 = (M_w+I_w*r^-2)*D;
G = m*g*l;
delta = m11*m22 - m21*m12;

% state = [a,a_dot,x_dot,pusai_dot] 
% u = [t_r t_l]
Ts = 0.01
A = [0 1 0 0;m11*G/delta 0 0 0;-m12*G/delta 0 0 0;0 0 0 0];
B = [0 0;-(m11+m21)/delta -(m11+m21)/delta;(m12+m22)/delta (m12+m22)/delta;m31^-1 -m31^-1];
C=[0 0 1 0;0 0 0 1];
D=[0 0;0 0;];
sys1 = ss(A,B,C,D);
rank(ctrb(A,B))

% LQR
% Q=diag([50,150,30,10]);R=diag([1,1]);
% [K,S_tmp,E_tmp] = lqr(sys1.A,sys1.B,Q,R)

% sys_d = c2d(sys1,Ts)
% dlqr(sys_d.A,sys_d.B,Q,R)

% 内模控制器
A_c = [0 0;0 0];
b_c = [1 0;0 1];
C_c = eye(2);
D_c = [0 0;0 0];
sys_c = ss(A_c,b_c,C_c,D_c);
sys_c_d = c2d(sys_c,Ts);

A_s = [A  zeros(4,2);-b_c*C A_c];
B_s = [B;-b_c*D];
Q_s = diag([50,10,20,10,50,20]);R_s=diag([1,1]);
[KK,S_tmp,E_tmp] = lqr(A_s,B_s,Q_s,R_s)

K=KK(:,1:4)
K_c = -KK(:,5:6)

%   -17.1384   -5.3649   -3.5415    1.1348
%   -17.1384   -5.3649   -3.5415   -1.1348
%    -2.5000    2.5000
%    -2.5000   -2.5000
