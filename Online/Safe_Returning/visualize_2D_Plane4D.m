function [hV, box_map]=visualize_2D_Plane4D(obs_map, box_map, trueQuad, virt_x, cTEB,...
    writerObj, iter, hV, showvideo , RpFlag)
% Local obstacles and true position
%         obs_disk.plotLocal();
%         hold on
if RpFlag == 1
    plot(trueQuad.x(1), trueQuad.x(2),'c.')
else
    plot(trueQuad.x(1), trueQuad.x(2),'Marker',".", 'Color',"#0894FF")
end
hold on

% Virtual state
        if iter ~= 1
            hV.XData = virt_x(1);
            hV.YData = virt_x(2);
        else
            hV = plot(virt_x(1,end), virt_x(2,end), "pentagram",...
                'MarkerFaceColor', 'g');
        end
        % Tracking error bound
            aug_x = [-cTEB(1), cTEB(1), cTEB(1), -cTEB(1)];
            aug_y = [-cTEB(2), -cTEB(2), cTEB(2), cTEB(2)];
            xv_aug = virt_x(1)*ones(1,4) + aug_x;
            yv_aug = virt_x(2)*ones(1,4) + aug_y;
            %       figure
            % aug_map = polyshape(reshape(mat2cell(xv_aug,ones(1,size(xv_aug,1))), 1, []),...
            %     reshape(mat2cell(yv_aug,ones(1,size(yv_aug,1))), 1, []));
            % plot(aug_map,'FaceAlpha',0,'EdgeColor','g',...
            %     'LineWidth', 2, 'LineStyle','-');
        if iter == 1
            box_map = ObstacleMapAstar_2D_rectangle(xv_aug, yv_aug);
        else
            box_map.xv = xv_aug;
            box_map.yv = yv_aug;
        end
        box_map.plotGlobal('g', '--', 1, 0)
        obs_map.plotPadded('#D41159', '--', 2)
        drawnow
%         delete(bound)

if showvideo  == 1
    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
end
%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))
end