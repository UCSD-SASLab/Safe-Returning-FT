function hV=visualize_2D(obs_disk, trueQuad, virt_x, trackErr,...
    writerObj, iter, hV, showvideo , RpFlag)
% Local obstacles and true position
%         obs_disk.plotLocal();
%         hold on
if RpFlag == 1
    plot(trueQuad.x(1), trueQuad.x(5),'c.')
else
    plot(trueQuad.x(1), trueQuad.x(5),'Marker',".", 'Color',"#0894FF")
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
%  bound = viscircles([virt_x(1) virt_x(2)], TEB, 'Color','r');
        drawnow
%         delete(bound)

if showvideo  == 1
    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
end
%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))
end