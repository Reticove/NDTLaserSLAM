% for e=1:1:size(traj.Looppose,1)
close all
%     m = traj.Looppose(e,1);
% n = traj.Looppose(e,2);
m = 1000;
n = 1060;
referenceScan = scan(m);
currentScan = scan(n);

currScanCart = currentScan.Cartesian;
refScanCart = referenceScan.Cartesian;
figure(1)
plot(refScanCart(:,1), refScanCart(:,2), 'k.');
hold on
plot(currScanCart(:,1), currScanCart(:,2), 'r.');
legend('Reference laser scan', 'Current laser scan', 'Location', 'NorthWest');
%  transform = matchScans(currentScan, referenceScan, 'MaxIterations', 500);
% transform = matchScans(currentScan, referenceScan, 'SolverAlgorithm','fminunc', 'InitialPose', computeTransform(traj.keyscans(760).pose,traj.keyscans(780).pose));
[transform,stats] = matchScans(currentScan, referenceScan, 'SolverAlgorithm','trust-region', 'InitialPose', computeTransform(poseList(m,:),poseList(n,:)));
% transform = matchScans(currentScan, scan(traj.keyscans(end).idx), 'SolverAlgorithm','fminunc', 'InitialPose', transform);
% transform = matchScansGrid(currentScan, referenceScan);
transScan = transformScan(currentScan, transform);
figure(2)
plot(refScanCart(:,1), refScanCart(:,2), 'k.');
hold on
transScanCart = transScan.Cartesian;
plot(transScanCart(:,1), transScanCart(:,2), 'r.');
legend('Reference laser scan', 'Transformed current laser scan', 'Location', 'NorthWest');
figure(3)
scantrans = transformScan(referenceScan,-computeTransform(poseList(m,:),poseList(n,:)));
plot(currentScan);
hold on
plot(scantrans);

figure(4)
show(map);
hold on
plot(poseList(m,1),poseList(m,2),'bo');
plot(poseList(n,1),poseList(n,2),'ro');
stats
% end

