function h=plane(a,lengthleft,lengthright,obs)
% Draws a line with normal vectro 'a' and origin translation 'b'

    if ~iscolumn(obs)
        obs = obs';
    end
    p = null(a');
    x1 = obs+lengthright*p;
    x2 = obs-lengthleft*p;
    h = line([x1(1),x2(1)],[x1(2),x2(2)]);

end