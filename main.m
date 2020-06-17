clear; close all; clc;

%cfig = figure('Position', [10,10,1280,1080]);
cfig = figure(1);

% Lidar parameters
lidar = SetLidarParameters();

% Map parameters

miniUpdated     = false;        %
miniUpdateDT    = 0.1;          % m
miniUpdateDR    = deg2rad(5);   % rad

% Load lidar data
lidar_data = load('dataset/horizental_lidar.mat');
N = size(lidar_data.timestamps, 1);
% Create an empty map
map = robotics.OccupancyGrid(90, 100, 20);
map.GridLocationInWorld = [-50 -35];

% map.points = [];
% map.connections = [];

traj.keyscans = [];
traj.connections = [];
traj.Looppose = [];
poseList = zeros(N,3);


% Graphoptimizer
Graphrange = 5;
numIter = 50;
cl = 5;

for scanIdx = 1 : 1 : N
    
    disp(['scan ', num2str(scanIdx)]);
    
    % Get current scan [x1,y1; x2,y2; ...]
    time = lidar_data.timestamps(scanIdx) * 1e-9;
    [ranges,angles] = ReadAScan(lidar_data, scanIdx, lidar, 40);
    scan(scanIdx) = lidarScan(ranges,angles);
    %     plot(scan(scanIdx));
    %       初始化
    if scanIdx == 1
        initialPose = [0 0 0];  % Typically, robot odometry is used to supply the initial estimate.
        [map,traj]=Initialize(map,traj,initialPose,scanIdx,scan(scanIdx));
        transform = initialPose;
        miniUpdated = true;
        continue;
    end
    currentScan = scan(scanIdx);
    referenceScan = scan(scanIdx-1);
    [transform,stats] = matchScans(currentScan, referenceScan, 'SolverAlgorithm','trust-region','InitialPose', transform);
    %             transform = matchScansGrid(currentScan, referenceScan);
    %     transform = matchScans(currentScan, referenceScan, 'MaxIterations', 100, 'InitialPose', transform);
    
    %     if stats.Score / currentScan.Count < 1.0
    %         disp(['Low scan match score for index ' num2str(scanIdx) '. Score = ' num2str(stats.Score) '.']);
    %     end
    %     %     transform相当于现位相对于原位的变换
    
    if scanIdx>2
        poseList(scanIdx,:) = poseTransform(poseList(scanIdx-1,:), transform);
        dp = abs(DiffPose(traj.keyscans(end).pose, poseList(scanIdx,:)));
        if dp(1)>miniUpdateDT || dp(2)>miniUpdateDT || dp(3)>miniUpdateDR
%                     if mod(scanIdx, 5) == 0
            %             transform = matchScans(currentScan, referenceScan, 'MaxIterations', 500, 'InitialPose', poseList(scanIdx,:));
%             if mod(length(traj.keyscans),5) == 0
%                 transform1= matchScans(scan(traj.keyscans(end).idx), scan(traj.keyscans(end-4).idx), 'SolverAlgorithm','trust-region', 'InitialPose', computeTransform(traj.keyscans(end-4).pose,traj.keyscans(end).pose));
%                 traj.keyscans(end).pose=poseTransform(traj.keyscans(end-4).pose,transform1);
%             end
            transform = matchScans(currentScan, scan(traj.keyscans(end).idx), 'SolverAlgorithm','trust-region','InitialPose', ...
                computeTransform(traj.keyscans(end).pose,poseList(scanIdx,:)));

            %             transform = matchScansGrid(currentScan, referenceScan);
%                       poseList(scanIdx,:) = poseTransform(poseList(scanIdx-1,:), transform);
            poseList(scanIdx,:) = poseTransform(traj.keyscans(end).pose, transform);
            [map,traj] = AddAKeyScan(map,traj,poseList(scanIdx,:),scanIdx,transform);
            
%             if length(traj.keyscans)>10
%                 from = length(traj.keyscans)-cl;
%                 to = length(traj.keyscans);
%                 traj = Addconnection(traj,scan,poseList,from,to);
%             end
            
%             poseList(scanIdx,:)=traj.keyscans(end).pose;
%             if TryLoopOrNot(traj)
%                 traj.keyscans(end).loopTried = true;
%                 traj = DetectLoopClosure(traj, scan, currentScan, 4);
% %                 poseList(scanIdx,:)=traj.keyscans(end).pose;
%             end
            insertRay(map, poseList(scanIdx,:), currentScan, double(30));
        end
    end
    
    
    
    

end
show(map);



