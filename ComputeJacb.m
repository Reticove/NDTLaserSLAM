function [A,B,e]=ComputeJab (z_ij,v_i,v_j)
zt_ij = v2t(z_ij);%����ת��Ϊ����
vt_i = v2t(v_i);
vt_j = v2t(v_j);


f_ij=(vt_i\vt_j);% Xi�������Xj

theta_i = v_i(3);
ti = v_i(1:2);
tj = v_j(1:2);
dt_ij = (tj-ti)';

si = sin(theta_i);
ci = cos(theta_i);


A= [-ci, -si, [-si, ci]*dt_ij; si, -ci, [-ci, -si]*dt_ij; 0, 0, -1 ];
B =[  ci, si, 0           ; -si, ci, 0            ; 0, 0, 1 ];


e = t2v(zt_ij\ f_ij); % �������
zt_ij(1:2,3) = 0;% ƫ��A,B���㹫ʽ��ֻ����z����ת����R�����԰�ƽ������ǿ����0
A = zt_ij\A; % ����Z��ת�ã���Z���棬����������ת����Ĺ�ϵ
B = zt_ij\B;
end
