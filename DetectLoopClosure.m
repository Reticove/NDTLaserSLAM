function traj = DetectLoopClosure(traj, scan, currentScan, Tmax)
% Tmax=4;
% Remove recent keyscans
s = 0;
k = length(traj.keyscans);
while (k>1) && (s<20) 
    k = k-1;
    s = s + traj.keyscans(k+1).diffDistance;
end

% We have no intention to close a loop with an error larger than Tmax
dear = 0;
pose = traj.keyscans(end).pose;
for i = k : -1 : 1
    dp = DiffPose(traj.keyscans(i).pose, pose);
    if norm(dp(1:2)) < Tmax+1
        dear = i;
        break;
    end
end
if dear < 1
    return;
end

% ===== Now we try to detect a loop closure seriouly =====
% Extract a refrence map around current scan
dearidx = traj.keyscans(dear).idx;
refscan  = scan(dearidx);
[transform,stats] = matchScans(currentScan, refscan, 'SolverAlgorithm','trust-region','InitialPose', computeTransform(traj.keyscans(dear).pose,traj.keyscans(end).pose));

if stats.Score< 500
    return
end
c = length(traj.connections);
traj.connections(c+1).keyIdPair = [dear, length(traj.keyscans)];
traj.connections(c+1).relativePose = transform;
traj.connections(c+1).covariance = [100 0 0;0 100 0;0 0 10000];
traj.Looppose=[traj.Looppose;dearidx,traj.keyscans(end).idx];


end
        
    
    
    
    
    
    
    
    
    