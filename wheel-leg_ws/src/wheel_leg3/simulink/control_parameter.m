m = 3.0 + 2 * 0.045518; % 车身重 + 两条腿重
l = 0.213; % 质心到转轴距离
M_w  = 0.09293; % 单个轮子重
I_w = 7.0742E-05; % 轮子绕转轴转动惯量
I_p = 0.003535057 + m * l^2; % 车身到转轴转动惯量 平行轴定理
r = 0.04; % 车轮半径
g = 9.8; % 重力加速度
D = 0.236; % 轮距

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
A = [0 1 0 0;m11*G/delta 0 0 0;-m12*G/delta 0 0 0;0 0 0 0]
B = [0 0;-(m11+m21)/delta -(m11+m21)/delta;(m12+m22)/delta (m12+m22)/delta;m31^-1 -m31^-1]

rank(ctrb(A,B))

[G,H] = c2d(A,B,Ts)
% LQR
Q=diag([50,100,20,10]);R=diag([1,1]);
[K,S_tmp,E_tmp] = lqr(A,B,Q,R)