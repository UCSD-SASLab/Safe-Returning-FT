function [obs] = Vert2grid(xv,yv)
    % convert obstacle defined by vertices to grid points. 
    % first ronuds to the nearest interger, then find the oqqupied grids 
    % Ex: 
    % if the obstacle is defined by xv = [1,3,3,1], yv = [1,1,2,2], then
    % obs = [1,1;2,1;3,1;2,1;2,2;3,2]

    xv_obs = round(xv);
    yv_obs = round(yv);

    xv_min = min(xv_obs);
    xv_max = max(xv_obs);
    yv_min = min(yv_obs);
    yv_max = max(yv_obs);

    x = [xv_min:1:xv_max];
    y = [yv_min:1:yv_max];

    obs = combvec(x,y);

    length_obs = size(obs,2);
    j = [];
    for i = 1 : length_obs
        d = p_poly_dist(obs(1,i),obs(2,i), xv,yv);
        if d <=0
            j = [j,i];
        end
    end

    obs = obs(:,j);

end