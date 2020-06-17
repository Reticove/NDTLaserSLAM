%
function [map,traj] = AddAKeyScan(map,traj,pose,idx,transform)


% First, evaluate the pose & hits, make sure that there is no large error
lastKeyPose = traj.keyscans(end).pose;
dp = DiffPose(lastKeyPose, pose);
% if abs(dp(1)) > 0.5 || abs(dp(2)) > 0.5 || abs(dp(3)) > pi
%     disp('Oh no no no nobody but you : So Large Error!');
%     pose = lastKeyPose;
% end

% keyscans
k = length(traj.keyscans);
traj.keyscans(k+1).pose = pose;
traj.keyscans(k+1).idx = idx;
traj.keyscans(k+1).loopClosed = false;
traj.keyscans(k+1).loopTried = false;
traj.keyscans(k+1).diffDistance = norm(dp(1:2));

% connections 
% ToDo: Estimate the relative pose and covariance, they will be useful
% when we close a loop (pose graph optimization).
c = length(traj.connections);
traj.connections(c+1).from = k;
traj.connections(c+1).to = k+1;
traj.connections(c+1).relativePose = transform;
traj.connections(c+1).covariance = [20 0 0;0 20 0;0 0 100];
                                    


