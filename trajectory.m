% show(maptest)
for i_node = 1:1:length(traj.keyscans)-1
    hold on
    plot(traj.keyscans(i_node).pose(1), traj.keyscans(i_node).pose(2), 'b.', 'DisplayName', 'Estimated robot position');
    j = 0 ;
    while j < size(traj.connections(i_node).to,1)
        j=j+1;
        j_node = traj.connections(i_node).to(j);
        hold on
        line([traj.keyscans(i_node).pose(1),traj.keyscans(j_node).pose(1)],...
            [traj.keyscans(i_node).pose(2),traj.keyscans(j_node).pose(2)]);
    end
end