classdef ObstacleMapAstar_2D_rectangle < handle

    properties
        xv; % x vertices of polygons, counter-clock wise, each row represents the same polygon
        yv; % y vertices of polygons, counter-clock wise, each row represents the same polygon
        local_obs; % indices of sensed obstacles, reprenting rows in xv and yv
        dx; % sTEB_x to be padded
        dr; % for 3D car whose TEB is a circle
        dy; % sTEB_y to be padded
        padded_obs; % indices of padded obstacles, reprenting rows in xv and yv
        num_obs; % the number of global obstacles
        seen_obs; % 1 x num_obs vector of obstacles seen so far, so that it isn't added again into local_obs
        indexx = 0; % counter for number of local obstacles added in

        % handle for the global obstacle plot
        hG

        % same for local obs
        hL

        % same for padded obs
        hP
        hP_O

    end

    methods
        %% Constructor.
        function self = ObstacleMapAstar_2D_rectangle(xv, yv)
            self.num_obs = size(xv,1);
            self.xv = xv;
            self.yv = yv;
            self.dx = zeros(self.num_obs, 4);
            self.dy = zeros(self.num_obs, 4);
            self.local_obs = [];
            self.padded_obs = [];
            self.seen_obs = false(1, self.num_obs);
        end

        %% SenseAndUpdate
        % sense for obstacles that are within sense_range of the point
        % then update local and padded obstacles.
        function new_sensed = sense_update(obj, point, sense_range, sTEB_X, sTEB_Y)
            % SenseAndUpdate(obj, point, sense_range, track_err)
            if ~iscolumn(point)
                point = point';
            end

            new_sensed = false;

            for i = find(~obj.seen_obs) % iterate over unseen global obs
                if obj.ableToSense(point, sense_range, i)
                    obj.seen_obs(i) = true; % mark as seen

                    % pad it and add it into the padded set
                    obj.dx(i,:) = [-sTEB_X, sTEB_X, sTEB_X, -sTEB_X];
                    obj.dy(i,:) = [-sTEB_Y, -sTEB_Y, sTEB_Y, sTEB_Y];
                    % add to local set
                    obj.padded_obs = [obj.padded_obs i];
                    obj.local_obs = [obj.local_obs i];
                    new_sensed = true;
                end
            end
            % repad seen obs with the new sTEB
            for i = find(obj.seen_obs) % iterate over seen global obs
                obj.dx(i,:) = [-sTEB_X, sTEB_X, sTEB_X, -sTEB_X];
                obj.dy(i,:) = [-sTEB_Y, -sTEB_Y, sTEB_Y, sTEB_Y];
            end
        end

        %% AbleToSense
        function sensed = ableToSense(obj, point, sense_ranges, obs_ind)
            % compute the max and min coordinates of obstacs
            x = obj.xv(obs_ind, :);
            y = obj.yv(obs_ind, :);
            [distance, ~, ~] = p_poly_dist(point(1),  point(2), ...
                x, y);
            if distance <= (sense_ranges)
                sensed = true;
            else
                sensed = false;
            end
        end


        %% ObstaclePlot
        function plotPadded(obj, color, linestyle, linewidth, dr)
            if nargin < 2
                color = '#D41159';
            end
            if nargin < 3
                linestyle = '--';
            end
            if nargin < 4
                linewidth = 3;
            end
            if nargin < 5
                dr = [];
            end
            if ~isempty(obj.hP)
                delete(obj.hP)
            end
            if ~isempty(obj.hP_O)
                delete(obj.hP_O)
            end
            seen = find(obj.seen_obs==1);
            if ~isempty(seen)
                if isempty(dr)
                    aug_x = obj.dx(1,:);
                    aug_y = obj.dy(2,:);
                    xv_aug = obj.xv(seen,:) + aug_x;
                    yv_aug = obj.yv(seen,:) + aug_y;
                    %       figure

                    aug_map = polyshape(reshape(mat2cell(xv_aug,ones(1,size(xv_aug,1))), 1, []),...
                        reshape(mat2cell(yv_aug,ones(1,size(yv_aug,1))), 1, []));
                    obj.hP = plot(aug_map,'FaceAlpha',0,'EdgeColor',color,...
                        'LineWidth', linewidth, 'LineStyle',linestyle);
                else
                    obj.hP_O = nan(1,length(seen));
                    for i = 1:length(seen)
                        center = [(obj.xv(seen(i),1) + obj.xv(seen(i),2))/2,...
                                    (obj.yv(seen(i),2) + obj.yv(seen(i),3))/2];
                        temp(i) = rectangle('Position',[center-dr, dr*2, dr*2],...
                            'Curvature',[1,1], 'FaceAlpha',0,'EdgeColor',color,...
                        'LineWidth', linewidth, 'LineStyle',linestyle);
                                        hold on
                        % if i == 1
                        %     obj.hP = temp(i);
                        % else
                        %     obj.hP = copyobj(obj.hP, temp(i));
                        % end
                          % obj.hP = copyobj(obj.hP, temp);              
                    end
                    obj.hP_O = temp;
                end

            end
        end
            function plotGlobal(obj, color, linestyle, linewidth, facealpha)
                if nargin < 2
                    color = '#D41159';
                end
                if nargin < 3
                    linestyle = 'none';
                end
                if nargin < 4
                    linewidth = 1;
                end
                if nargin < 5
                    facealpha = 0.7;
                end
                if ~isempty(obj.hG)
                    delete(obj.hG)
                end
                global_map = polyshape(reshape(mat2cell(obj.xv,ones(1,size(obj.xv,1))), 1, []),...
                    reshape(mat2cell(obj.yv,ones(1,size(obj.yv,1))), 1, []));
                obj.hG = plot(global_map,'FaceColor', color,'FaceAlpha',facealpha,'EdgeColor',color,...
                    'LineWidth', linewidth, 'LineStyle',linestyle);
                % obj.hL = gcf;
                % obj.hG = gcf;
            end
            function plotLocal(obj, sTEB_X, sTEB_Y)
                if ~isempty(obj.hL)
                    delete(obj.hL)
                end
                % Local obstacles
                xv_seen = self.xv(size(obj.local_obs),:);
                yv_seen = self.yv(size(obj.local_obs),:);
                local_map = polyshape(reshape(mat2cell(xv_seen,ones(1,size(xv_seen,1))), 1, []),...
                    reshape(mat2cell(yv_seen,ones(1,size(yv_seen,1))), 1, []));
                plot(local_map,'FaceColor', '#D41159','FaceAlpha',0.7,'EdgeColor','#D41159',...
                    'LineWidth', 1);
                obj.hL = gcf;
            end
            %
            %     function plotPadded(obj)
            %       if nargin < 2
            %         color = 'g';
            %       end
            %
            %       % Augmented obstacles
            %       coords = get_obs_coords_for_plot(obj.padded_obs);
            %
            %       if ~isempty(obj.hP)
            %         delete(obj.hP)
            %       end
            %
            %       obj.hP = fill3(coords{:}, color, 'FaceAlpha', 0.05, 'LineStyle', 'none');
        end

        % END OF METHODS
    end
