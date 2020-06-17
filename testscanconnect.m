maxmove = 0;
for i = 1:1:length(traj.keyscans)-4
    if (norm(traj.keyscans(i+4).pose(1:2) - traj.keyscans(i).pose(1:2))) > maxmove
        maxmove=(traj.keyscans(i+4).pose(1:2) - traj.keyscans(i).pose(1:2));
        t = i ;
    end
end