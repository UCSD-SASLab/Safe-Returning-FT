classdef ObstacleMapRRT_2D < handle

    properties
        center; % a three dimensional matrix of all the obstacles on the map. TODO: reformatting into cell etc...
        radius; % a three dimensional matrix of the obstacles that the drone has seen so far. Starts empty.
        local_obs
        dr
        padded_obs; % three dimensional matrix of the obstacles that the drone has seen so far PLUS trackErrBd
        num_obs; % the number of global obstacles
        seen_obs; % 1 x num_obs vector of obstacles seen so far, so that it isn't added again into local_obs
        indexx = 0; % counter for number of local obstacles added in

        % handle for the global obstacle plot
        hG

        % same for local obs
        hL

        % same for padded obs
        hP

    end

    methods
        %% Constructor.
        function self = ObstacleMapRRT_2D(center, r)
            self.num_obs = length(r);
            self.center = center;
            self.radius = r;
            self.dr = zeros(self.num_obs,1);
            self.local_obs = [];
            self.padded_obs = [];
            self.seen_obs = false(1, self.num_obs);
        end

        %% SenseAndUpdate
        % sense for obstacles that are within sense_range of the point
        % then update local and padded obstacles.
        function new_sensed = sense_update(obj, point, sense_range, track_err)
            % SenseAndUpdate(obj, point, sense_range, track_err)
            if ~iscolumn(point)
                point = point';
            end

            new_sensed = false;

            for i = find(~obj.seen_obs) % iterate over unseen global obs
                if obj.ableToSense(point, sense_range, i)
                    obj.seen_obs(i) = true; % mark as seen

                    % pad it and add it into the padded set
                    obj.dr(i) = track_err;
                    % add to local set
                    obj.padded_obs = [obj.padded_obs i];
                    obj.local_obs = [obj.local_obs i];
                    new_sensed = true;
                end
            end
            for i = find(obj.seen_obs) % iterate over seen global obs
                obj.dr(i) = track_err;
            end
        end

        %% AbleToSense
        function sensed = ableToSense(obj, point, sense_ranges, obs_ind)
            % compute the max and min coordinates of obstacs
            x = obj.center(obs_ind,1);
            y = obj.center(obs_ind,2);
            r = obj.radius(obs_ind);
            distance = norm(point-[x;y]);
            if distance <= (sense_ranges+r)
                sensed = true;
            else
                sensed = false;
            end
        end


        %% ObstaclePlot
        function plotGlobal(obj, color, linestyle)
            if nargin < 2
                color = 'k';
            end

            if nargin < 3
                linestyle = 'none';
            end
            if ~isempty(obj.hL)
                delete(obj.hL)
            end

            %       figure
            for i = 1:obj.num_obs
                rectangle('Position',[obj.center(i,:)-obj.radius(i), obj.radius(i)*2, obj.radius(i)*2],...
                    'Curvature',[1,1], 'FaceColor', 'k', 'EdgeColor', 'k');
                %                 hold on
            end
            obj.hG = gcf;
        end
        function plotLocal(obj, color)
            if nargin < 2
                color = 'r';
            end
            if ~isempty(obj.hL)
                delete(obj.hL)
            end
            % Local obstacles
            for i = 1:length(obj.local_obs)
                hold on
                rectangle('Position',[obj.center(i,:)-obj.radius(i), obj.radius(i)*2, obj.radius(i)*2],...
                    'Curvature',[1,1], 'FaceColor', 'r', 'EdgeColor', 'r');
                %                 hold on
            end
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
