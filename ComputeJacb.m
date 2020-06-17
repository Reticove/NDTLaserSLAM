function [A,B,e]=ComputeJab (z_ij,v_i,v_j)
zt_ij = v2t(z_ij);%向量转化为矩阵
vt_i = v2t(v_i);
vt_j = v2t(v_j);


f_ij=(vt_i\vt_j);% Xi的逆乘以Xj

theta_i = v_i(3);
ti = v_i(1:2);
tj = v_j(1:2);
dt_ij = (tj-ti)';

si = sin(theta_i);
ci = cos(theta_i);


A= [-ci, -si, [-si, ci]*dt_ij; si, -ci, [-ci, -si]*dt_ij; 0, 0, -1 ];
B =[  ci, si, 0           ; -si, ci, 0            ; 0, 0, 1 ];


e = t2v(zt_ij\ f_ij); % 误差向量
zt_ij(1:2,3) = 0;% 偏导A,B计算公式中只用了z的旋转矩阵R，所以把平移向量强制清0
A = zt_ij\A; % 乘以Z的转置，即Z的逆，这是由于旋转矩阵的关系
B = zt_ij\B;
end
