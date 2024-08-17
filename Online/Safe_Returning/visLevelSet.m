function hV = visLevelSet(trueQuad, g,V ,...
    writerObj, hV, showvideo , JFlag)
% Local obstacles and true position
%         obs_disk.plotLocal();
%         hold on

delete(hV);
[g2d,V2d] = proj(g,V,[0,0,1,1],'min');
g2d.xs{1} = g2d.xs{1}+trueQuad.x(1);
g2d.xs{2} = g2d.xs{2}+trueQuad.x(2);
hV = visSetIm(g2d,V2d,'b',0.01);

if showvideo  == 1
    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
end
%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))
end