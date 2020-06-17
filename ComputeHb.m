function [H,b]=ComputeHb(H,b,A,B,e,id_i,id_j,omega)
%Hij ¿é¾ØÕó
    b_i = -A' * omega * e;
    b_j = -B' * omega * e;
    H_ii=  A' * omega * A;
    H_ij=  A' * omega * B;
    H_jj=  B' * omega * B;
    
    %ÀÛ¼Ó
    H((id_i-1)*3+1:id_i*3,(id_i-1)*3+1:id_i*3) = ...
        H((id_i-1)*3+1:id_i*3,(id_i-1)*3+1:id_i*3)+ H_ii;
    H((id_j-1)*3+1:id_j*3,(id_j-1)*3+1:id_j*3) = ...
        H((id_j-1)*3+1:id_j*3,(id_j-1)*3+1:id_j*3) + H_jj;
    H((id_i-1)*3+1:id_i*3,(id_j-1)*3+1:id_j*3) = ...
        H((id_i-1)*3+1:id_i*3,(id_j-1)*3+1:id_j*3) + H_ij;
    H((id_j-1)*3+1:id_j*3,(id_i-1)*3+1:id_i*3) = ...
        H((id_j-1)*3+1:id_j*3,(id_i-1)*3+1:id_i*3) + H_ij';
    b((id_i-1)*3+1:id_i*3,1) = ...
        b((id_i-1)*3+1:id_i*3,1) + b_i;
    b((id_j-1)*3+1:id_j*3,1) = ...
        b((id_j-1)*3+1:id_j*3,1) + b_j;
