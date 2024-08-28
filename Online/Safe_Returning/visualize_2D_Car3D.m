function [hV, hLS]=visualize_2D_Car3D(obs_map, trueQuad, virt_x, cTEB,...
    writerObj, iter, hV, hLS, showvideo , RpFlag, g, dataX, level)
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
        delete(hLS);
        [g2d,V2d] = proj(g,dataX,[0,0,1],'min');
        g2d.xs{1} = g2d.xs{1}+trueQuad.x(1);
        g2d.xs{2} = g2d.xs{2}+trueQuad.x(2);
        
        hLS = visSetIm(g2d,V2d,'b',level);
        [~, radius] = compute_level_set_radius_ZG(g, dataX, level,'separate');
        obs_map.plotPadded('#D41159', '--', 2, radius(1))
        drawnow
%         delete(bound)

if showvideo  == 1
    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
end
%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))
end