

n = length(traj.keyscans);
L = length(traj.connections);
k=0;
while k<4
    H = zeros(n*3,n*3);
    b = zeros(n*3,1);
    
    for i = 1:1:L
        for topose = 1:1:size(traj.connections(i).to,1)
            j = traj.connections(i).to(topose);
            z_ij = traj.connections(i).relativePose(topose,:);
            v_i = traj.keyscans(i).pose;
            v_j = traj.keyscans(j).pose;
            omega = traj.connections(i).covariance(3*topose-2:3*topose,:);

            
            % 雅克比矩阵计算 误差向量计算
            [A,B,e] =ComputeJacb(z_ij,v_i,v_j);

            % H,b计算
            i_ind = id2ind(i);
            j_ind = id2ind(j);
            [H,b] = ComputeHb(H,b,A,B,e,i_ind,j_ind,omega);
        end
    end
    H_sparse = sparse(H);
    
    %     dx=pinv(H) * b;
    dx=H_sparse \ b;
    dpose = reshape(dx, 3, n);
    for i_node = 1:1:n
        traj.keyscans(i_node).pose = traj.keyscans(i_node).pose ...
            + (dpose(:,i_node))';
    end
    for t = 1:1:n-1
        for s = t+1:1:n
            if ismember(s,traj.connections(t).to) > 0
                continue
            end
            dp = DiffPose(traj.keyscans(t).pose,traj.keyscans(s).pose);
            if norm(dp(1:2)) < 5
                %                 diffdis = 0;
                %                 for f2t = t+1:1:s
                %                     diffdis = traj.keyscans(f2t).diffDistance+diffdis;
                %                 end
                %                 if diffdis <20
                %                     continue
                %                 end
                traj = Addconnection(traj,scan,t,s);
            end
        end
    end
    k=k+1;
end
clear maptest
maptest = robotics.OccupancyGrid(90, 100, 20);
maptest.GridLocationInWorld = [-50 -35];
for i = 1:1:n
    insertRay(maptest, traj.keyscans(i).pose, scan(traj.keyscans(i).idx), double(30));
end
function ind = id2ind(id)
%ID2IND converts id to indices in H and b
ind = (3*(id-1)+1):(3*id);
end



