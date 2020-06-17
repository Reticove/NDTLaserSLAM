function traj = Addconnection(traj,scan,from,to)


[transform,stats] = matchScans(scan(traj.keyscans(to).idx),scan(traj.keyscans(from).idx), 'InitialPose', computeTransform(traj.keyscans(from).pose,traj.keyscans(to).pose));
if stats.Score < 900
    return
end



traj.connections(from).to = [traj.connections(from).to;to];
traj.connections(from).relativePose = [[traj.connections(from).relativePose];[transform]];
traj.connections(from).covariance = [traj.connections(from).covariance;[20 0 0;0 20 0;0 0 100]];
end