function needLoopTrial = TryLoopOrNot(traj)

% First of all, we don't want to detect loop closure too frequently.
%(At least 10 keyscans away from last trial)
trialInterval = 1;
for k = length(traj.keyscans)-1 : -1 : 1
    if ~traj.keyscans(k).loopTried
        trialInterval = trialInterval + 1;
    else
        break;
    end
end
if trialInterval < 10
    needLoopTrial = false;
    return;
end

% Then we don't want to detect loop closure after a newly closed loop
% (At least 10 meters need to be traveled since last closure)
traveledDistance = 0 ;
for k  = length(traj.keyscans) : -1 : 1
    if ~traj.keyscans(end).loopClosed
%         dT = DiffPose(traj.keyscans(k).pose, traj.keyscans(k+1).pose);
        traveledDistance = traveledDistance + traj.keyscans(k).diffDistance;
        if traveledDistance > 10
           break;
        end
     else
        break;
     end
end
if traveledDistance < 10
    needLoopTrial = false;
    return;
end
needLoopTrial = true;

    