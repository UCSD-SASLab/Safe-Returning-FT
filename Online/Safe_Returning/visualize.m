function [hV, boxMap]=visualize(obsMap, trueQuad, virt_x, trackErr, writerObj,...
    iter, boxMap, hV,vis, RpFlag)
% Local obstacles and true position
obsMap.plotLocal;
hold on
if RpFlag == 1
    plot3(trueQuad.x(1), trueQuad.x(5), trueQuad.x(9), 'c.')
else
plot3(trueQuad.x(1), trueQuad.x(5), trueQuad.x(9),'Marker',".", 'Color',"#0894FF")
end

hold on

% Virtual state
if iter == 1
    hV = plot3(virt_x(1,end), virt_x(2,end), virt_x(3,end), "pentagram",...
     'MarkerFaceColor', 'g', 'MarkerSize',15);
else
    hV.XData = virt_x(1);
    hV.YData = virt_x(2);
    hV.ZData = virt_x(3);

end

% Tracking error bound
left_bd = virt_x(1) - trackErr;
right_bd = virt_x(1) + trackErr;
back_bd = virt_x(2) - trackErr;
front_bd = virt_x(2) + trackErr;
bottom_bd = virt_x(3) - trackErr;
top_bd = virt_x(3) + trackErr;

left_surf = [left_bd, back_bd, bottom_bd; ...
    left_bd, back_bd, top_bd; ...
    left_bd, front_bd, top_bd; ...
    left_bd, front_bd, bottom_bd];

right_surf = [right_bd, back_bd, bottom_bd; ...
    right_bd, back_bd, top_bd; ...
    right_bd, front_bd, top_bd; ...
    right_bd, front_bd, bottom_bd];

back_surf = [left_bd, back_bd, bottom_bd; ...
    left_bd, back_bd, top_bd; ...
    right_bd, back_bd, top_bd; ...
    right_bd, back_bd, bottom_bd];

front_surf = [left_bd, front_bd, bottom_bd; ...
    left_bd, front_bd, top_bd; ...
    right_bd, front_bd, top_bd; ...
    right_bd, front_bd, bottom_bd];

bottom_surf = [left_bd, front_bd, bottom_bd; ...
    left_bd, back_bd, bottom_bd; ...
    right_bd, back_bd, bottom_bd; ...
    right_bd, front_bd, bottom_bd];

top_surf = [left_bd, front_bd, top_bd; ...
    left_bd, back_bd, top_bd; ...
    right_bd, back_bd, top_bd; ...
    right_bd, front_bd, top_bd];

boxShape = cat(3, left_surf, right_surf);
boxShape = cat(3, boxShape, back_surf);
boxShape = cat(3, boxShape, front_surf);
boxShape = cat(3, boxShape, bottom_surf);
boxShape = cat(3, boxShape, top_surf);

if iter == 1
    boxMap = ObstacleMapRRT(boxShape);
else
    boxMap.global_obs = boxShape;
end
boxMap.plotGlobal('b', '-');

drawnow
if vis == 1
    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
end
%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))
end