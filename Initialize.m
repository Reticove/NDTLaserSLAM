function [map,tarj] = Initialize(map,tarj,pose,idx,currentScan)

% Points in world frame
insertRay(map, pose, currentScan, double(24));
poseList(1,:) = pose;
% Key scans' information
k = length(tarj.keyscans);
tarj.keyscans(k+1).pose = pose;
tarj.keyscans(k+1).idx = idx;
tarj.keyscans(k+1).loopClosed = false;
tarj.keyscans(k+1).loopTried = false;
tarj.keyscans(k+1).diffDistance = 0;