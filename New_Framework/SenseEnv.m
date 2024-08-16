function SenseMap = SenseEnv(x,y,senseRange,Map,TEB,dim)
    % x,y: current tracker location
    % Map: the grounnd truth map that is unknow to the system
    % TEB: 3 or 2-dim vector, specify size of TEB on x/y/(z) dims 

    if ~isrow(TEB)
        TEB = TEB';
    end

    if length(TEB) ~= dim
        error('dimsion of TEB no correct')
    end
        
   
    x = round(x); y = round(y);
    TEB = round(TEB);
    % Update sensed ground grids to 2
    % r_x = [-senseRange:senseRange];
    % r_y = rx;
    % r = r_x'.^2*r_y.^2;
    % circle_ind = find(r<=senseRange^2);
    sense_x = [ max(1,x-senseRange):min(Map.xMax,x+senseRange)];
    sense_y = [ max(1,y-senseRange):min(Map.yMax,x+senseRange)];
    g = Map.ground;
    g(sense_x,sense_y) = 2;

    % Update sensed obs grids to -2
    obs_x_ind_Max = find(Map.obs.grid(1,:) <= sense_x(end)); 
    obs_x_ind_Min = find(Map.obs.grid(1,:) >= sense_x(1));
    obs_x_ind = intersect(obs_x_ind_Max,obs_x_ind_Min);

    obs_y_ind_Max = find(Map.obs.grid(2,:) <= sense_y(end)); 
    obs_y_ind_Min = find(Map.obs.grid(2,:) >= sense_y(1));
    obs_y_ind = intersect(obs_y_ind_Max,obs_y_ind_Min);
    
    obs_ind = intersect(obs_x_ind,obs_y_ind);
    Obs_x = Map.obs.grid(1,obs_ind);
    Obs_y = Map.obs.grid(2,obs_ind);
    obs_grid =[Obs_x;Obs_y];
    num_obs = length(Obs_x);
    aug_obs_grid = [];
    g_aug = g;

    % first augment obs
    for i = 1 : num_obs 
        g(Obs_x(i),Obs_y(i)) = -2; % turns to -2 once sensed
        
        aug_obs_temp_xv = [Obs_x(i)-TEB(1),Obs_x(i)+TEB(1),...
                           Obs_x(i)+TEB(1),Obs_x(i)-TEB(1)];
        aug_obs_temp_yv = [Obs_y(i)-TEB(2),Obs_y(i)-TEB(2),...
                           Obs_y(i)+TEB(2),Obs_y(i)+TEB(2)];
        aug_obs_grid_temp = Vert2grid(aug_obs_temp_xv,aug_obs_temp_yv);
        aug_obs_grid = [aug_obs_grid,aug_obs_grid_temp];
        % if dim ==3
        %      aug_obs_temp_z = [Obs_z(3)-TEB(1):Obs_z(i)+TEB(3)];
        % end
    end
    aug_obs_grid = unique(aug_obs_grid','rows');
    aug_obs_grid = aug_obs_grid';
    
    % find sensed aug obs ind
    aug_obs_x_ind_Max = find(aug_obs_grid(1,:) <= sense_x(end)); 
    aug_obs_x_ind_Min = find(aug_obs_grid(1,:) >= sense_x(1));
    aug_obs_x_ind = intersect(aug_obs_x_ind_Max,aug_obs_x_ind_Min);

    aug_obs_y_ind_Max = find(aug_obs_grid(2,:) <= sense_y(end)); 
    aug_obs_y_ind_Min = find(aug_obs_grid(2,:) >= sense_y(1));
    aug_obs_y_ind = intersect(aug_obs_y_ind_Max,aug_obs_y_ind_Min);
    aug_obs_ind = intersect(aug_obs_x_ind,aug_obs_y_ind);

    aug_obs_grid = aug_obs_grid(:,aug_obs_ind);

    num_aug_obs = length(aug_obs_grid(1,:));
    for i = 1 : num_aug_obs 
        g_aug(aug_obs_grid(1,i),aug_obs_grid(2,i)) = -2; % turns to -2 once sensed
    end

    SenseMap.ground = g;
    SenseMap.aug_ground = g_aug;
    SenseMap.obs.grid = obs_grid;
    SenseMap.aug_obs.grid = aug_obs_grid;