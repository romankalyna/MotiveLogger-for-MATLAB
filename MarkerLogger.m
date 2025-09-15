function varargout = MarkerLogger(command, param)
% MARKERLOGGER Enhanced function to log/visualize OptiTrack data.
%
% Displays data in "My Robot" coordinate system:
%   My Robot X = Motive Z
%   My Robot Y = Motive X
%   My Robot Z = Motive Y
% RB2 is treated as the origin for visualization, positions in MILLIMETERS.
% Calculates and stores the distance between RB1 and RB2.

persistent marker_history_mm rb1_position_history_mm rb2_position_history_mm;
persistent rb1_orientation_history rb2_orientation_history; % Stores ORIGINAL Motive orientations
persistent rb1_robot_euler_history rb2_robot_euler_history; % Stores Euler angles for "My Robot" frame
persistent rb1_quaternion_history rb2_quaternion_history; % Stores ORIGINAL Motive quaternions
persistent last_raw_rb1_pos_m last_raw_rb2_pos_m;
persistent last_distance_m;

persistent currentIndex historyLength;
persistent h_fig h_plot h_subplot_rb1 h_subplot_rb2;
persistent isRb2OriginSet rb2_origin_m;
persistent markerColors rbColors;
persistent data_actually_logged_count; 
persistent C_rm; % Transformation matrix: Motive vector to Robot vector components

varargout = {};
NUM_MARKERS = 6; NUM_RB = 2; MARKERS_PER_RB = 3;
if nargin < 1; command = 'log'; end
if nargin < 2; param = []; end

% Transformation from Motive component basis to My Robot component basis
% If v_m = [mx; my; mz] are components in Motive's frame,
% then v_r = [rx; ry; rz] are components in My Robot's frame, where:
% rx (My Robot X) = mz (Motive Z)
% ry (My Robot Y) = mx (Motive X)
% rz (My Robot Z) = my (Motive Y)
% So, [rx;ry;rz] = C_rm * [mx;my;mz]
% C_rm = [[0 0 1]; [1 0 0]; [0 1 0]];
% This C_rm is also used to transform rotation matrices: R_robot = C_rm * R_motive * C_rm'

switch lower(command)
    case 'init'
        C_rm = [[0 0 1]; [1 0 0]; [0 1 0]]; % Define the transformation matrix

        historyLength = 500;
        markerColors = {[1 0 0],[0.8 0 0],[1 0.5 0.5],[0 0 1],[0 0 0.8],[0.5 0.5 1]};
        rbColors = {[1 0 0],[0 0 1]};

        marker_history_mm = NaN(historyLength, NUM_MARKERS, 3);
        rb1_position_history_mm = NaN(historyLength, 3);
        rb2_position_history_mm = NaN(historyLength, 3);
        rb1_orientation_history = NaN(historyLength, 3, 3); % Original Motive RMs
        rb2_orientation_history = NaN(historyLength, 3, 3); % Original Motive RMs
        rb1_robot_euler_history = NaN(historyLength, 3); % Euler for Robot frame
        rb2_robot_euler_history = NaN(historyLength, 3); % Euler for Robot frame
        rb1_quaternion_history = NaN(historyLength, 4);
        rb2_quaternion_history = NaN(historyLength, 4);
        
        last_raw_rb1_pos_m = [NaN, NaN, NaN]; last_raw_rb2_pos_m = [NaN, NaN, NaN];
        last_distance_m = NaN;
        currentIndex = 1; data_actually_logged_count = 0;
        isRb2OriginSet = false; rb2_origin_m = [NaN, NaN, NaN];

        existing_fig = findobj('Type', 'figure', 'Number', 2);
        if ~isempty(existing_fig); h_fig = figure(2); clf(h_fig);
        else; h_fig = figure(2); end
        screenSize = get(0, 'ScreenSize');
        set(h_fig, 'Name', 'Rigid Body Tracking (My Robot Coords, RB2 as Origin, mm)', ...
                  'Position', [screenSize(3)/2, 100, screenSize(3)/2-100, screenSize(4)-200], ...
                  'DeleteFcn', @figureClosedCallback);

        h_plot = subplot(2, 1, 1); view(3); grid on; daspect([1 1 1]); axis equal;
        xlabel('My Robot X (mm)'); ylabel('My Robot Y (mm)'); zlabel('My Robot Z (mm)'); % Updated Labels
        title('Positions Relative to RB2 (My Robot Coords, mm)');
        hold(h_plot, 'on');

        for i = 1:NUM_MARKERS; plot3(h_plot, NaN, NaN, NaN, 'Color', markerColors{i}, 'LineWidth', 1, 'Tag', sprintf('MarkerTraj%d', i)); end
        for i = 1:NUM_MARKERS; plot3(h_plot, NaN, NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerFaceColor', markerColors{i}, 'MarkerEdgeColor', 'k', 'Tag', sprintf('MarkerCurrent%d', i)); end
        plot3(h_plot, NaN, NaN, NaN, '-', 'Color', rbColors{1}, 'LineWidth', 1.5, 'Tag', 'RB1Frame');
        plot3(h_plot, NaN, NaN, NaN, '-', 'Color', rbColors{2}, 'LineWidth', 1.5, 'Tag', 'RB2Frame');
        plot3(h_plot, NaN, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', rbColors{1}, 'Tag', 'RB1Origin');
        plot3(h_plot, NaN, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', rbColors{2}, 'Tag', 'RB2Origin');
        plot3(h_plot, NaN, NaN, NaN, '-', 'Color', rbColors{1}, 'LineWidth', 1.5, 'Tag', 'RB1RobotAxisX'); % Tags reflect robot axes
        plot3(h_plot, NaN, NaN, NaN, '-', 'Color', rbColors{1}, 'LineWidth', 1.5, 'Tag', 'RB1RobotAxisY');
        plot3(h_plot, NaN, NaN, NaN, '-', 'Color', rbColors{1}, 'LineWidth', 1.5, 'Tag', 'RB1RobotAxisZ');
        plot3(h_plot, NaN, NaN, NaN, '-', 'Color', rbColors{2}, 'LineWidth', 1.5, 'Tag', 'RB2RobotAxisX');
        plot3(h_plot, NaN, NaN, NaN, '-', 'Color', rbColors{2}, 'LineWidth', 1.5, 'Tag', 'RB2RobotAxisY');
        plot3(h_plot, NaN, NaN, NaN, '-', 'Color', rbColors{2}, 'LineWidth', 1.5, 'Tag', 'RB2RobotAxisZ');
        legend(h_plot, 'off');

        h_subplot_rb1 = subplot(2, 2, 3); title('RB1 My Robot Orient (Euler Deg)'); xlabel('Sample'); ylabel('Angle (deg)'); grid on; hold(h_subplot_rb1, 'on'); % Updated Title
        plot(h_subplot_rb1, NaN, NaN, 'r-', 'LineWidth', 1, 'Tag', 'RB1Roll'); plot(h_subplot_rb1, NaN, NaN, 'g-', 'LineWidth', 1, 'Tag', 'RB1Pitch'); plot(h_subplot_rb1, NaN, NaN, 'b-', 'LineWidth', 1, 'Tag', 'RB1Yaw');
        legend(h_subplot_rb1, 'off');

        h_subplot_rb2 = subplot(2, 2, 4); title('RB2 My Robot Orient (Euler Deg)'); xlabel('Sample'); ylabel('Angle (deg)'); grid on; hold(h_subplot_rb2, 'on'); % Updated Title
        plot(h_subplot_rb2, NaN, NaN, 'm-', 'LineWidth', 1, 'Tag', 'RB2Roll'); plot(h_subplot_rb2, NaN, NaN, 'c-', 'LineWidth', 1, 'Tag', 'RB2Pitch'); plot(h_subplot_rb2, NaN, NaN, 'y-', 'LineWidth', 1, 'Tag', 'RB2Yaw');
        legend(h_subplot_rb2, 'off');
        fprintf('Marker logger initialized. Plotting in "My Robot" Coords. RB2 origin. Units mm.\n');

    case 'log'
        if isempty(marker_history_mm); fprintf('Logger not init.\n'); return; end
        if isempty(C_rm); C_rm = [[0 0 1]; [1 0 0]; [0 1 0]]; end % Ensure C_rm is defined

        [raw_marker_pos_m, current_raw_rb1_pos_m, current_raw_rb2_pos_m, R1_motive, R2_motive, Q1, Q2] = getMotive6MarkerPositions();
        
        last_raw_rb1_pos_m = current_raw_rb1_pos_m; last_raw_rb2_pos_m = current_raw_rb2_pos_m;
        if ~any(isnan(last_raw_rb1_pos_m(:))) && ~any(isnan(last_raw_rb2_pos_m(:))); last_distance_m = calculate_distance_subfunc(last_raw_rb1_pos_m, last_raw_rb2_pos_m);
        else; last_distance_m = NaN; end

        if ~isRb2OriginSet && ~any(isnan(last_raw_rb2_pos_m)); rb2_origin_m = last_raw_rb2_pos_m; isRb2OriginSet = true; fprintf('RB2 origin (Motive): [%.3f,%.3f,%.3f]m\n', rb2_origin_m); end
        
        current_markers_mm = NaN(NUM_MARKERS, 3); current_rb1_pos_mm = [NaN NaN NaN]; current_rb2_pos_mm = [NaN NaN NaN];
        if isRb2OriginSet
            if ~any(isnan(last_raw_rb1_pos_m)); current_rb1_pos_mm = (last_raw_rb1_pos_m - rb2_origin_m) * 1000; end
            if ~any(isnan(last_raw_rb2_pos_m)); current_rb2_pos_mm = (last_raw_rb2_pos_m - rb2_origin_m) * 1000; end
            for i = 1:NUM_MARKERS; if ~any(isnan(raw_marker_pos_m(i,:))); current_markers_mm(i,:) = (raw_marker_pos_m(i,:) - rb2_origin_m) * 1000; end; end
        end

        if ~any(isnan(current_rb1_pos_mm)) || ~any(isnan(current_rb2_pos_mm)); data_actually_logged_count = min(data_actually_logged_count + 1, historyLength); end

        marker_history_mm(currentIndex, :, :) = current_markers_mm; % Stored in Motive frame (relative to RB2)
        rb1_position_history_mm(currentIndex, :) = current_rb1_pos_mm; % Stored in Motive frame
        rb2_position_history_mm(currentIndex, :) = current_rb2_pos_mm; % Stored in Motive frame
        rb1_quaternion_history(currentIndex, :) = Q1; rb2_quaternion_history(currentIndex, :) = Q2;

        % Store original Motive orientation matrices
        if ~any(isnan(R1_motive),'all'); rb1_orientation_history(currentIndex,:,:) = R1_motive; R1_robot_frame = C_rm * R1_motive * C_rm'; rb1_robot_euler_history(currentIndex,:) = rotationMatrixToEulerAngles(R1_robot_frame);
        else; rb1_orientation_history(currentIndex,:,:) = NaN; rb1_robot_euler_history(currentIndex,:) = NaN; end
        if ~any(isnan(R2_motive),'all'); rb2_orientation_history(currentIndex,:,:) = R2_motive; R2_robot_frame = C_rm * R2_motive * C_rm'; rb2_robot_euler_history(currentIndex,:) = rotationMatrixToEulerAngles(R2_robot_frame);
        else; rb2_orientation_history(currentIndex,:,:) = NaN; rb2_robot_euler_history(currentIndex,:) = NaN; end
        
        currentIndex = mod(currentIndex, historyLength) + 1;

    case 'update'
        if isempty(marker_history_mm); fprintf('Logger not init.\n'); return; end
        if isempty(h_fig) || ~ishandle(h_fig); fprintf('Vis window closed.\n'); MarkerLogger('init'); return; end
        if isempty(C_rm); C_rm = [[0 0 1]; [1 0 0]; [0 1 0]]; end 
        figure(h_fig);

        latestValidPositions_mm_motive = NaN(NUM_MARKERS, 3);
        for i = 1:NUM_MARKERS
            h_traj = findobj(h_plot, 'Tag', sprintf('MarkerTraj%d', i));
            h_curr = findobj(h_plot, 'Tag', sprintf('MarkerCurrent%d', i));
            valid_data_indices = find(~all(isnan(squeeze(marker_history_mm(:, i, :))), 2));
            if ~isempty(valid_data_indices)
                traj_mm_motive = squeeze(marker_history_mm(valid_data_indices, i, :));
                if size(traj_mm_motive,2)~=3 && size(traj_mm_motive,1)==3; traj_mm_motive = traj_mm_motive'; end 
                if size(traj_mm_motive,1)==1 && size(traj_mm_motive,2)==3; traj_mm_motive = [traj_mm_motive; traj_mm_motive]; end
                
                % Permute for "My Robot" plot: [MotiveZ, MotiveX, MotiveY]
                if ~isempty(h_traj) && ~isempty(traj_mm_motive); set(h_traj, 'XData', traj_mm_motive(:,3), 'YData', traj_mm_motive(:,1), 'ZData', traj_mm_motive(:,2)); end

                latestValidPositions_mm_motive(i,:) = squeeze(marker_history_mm(valid_data_indices(end), i, :))';
                if ~isempty(h_curr) && ~any(isnan(latestValidPositions_mm_motive(i,:))); set(h_curr, 'XData', latestValidPositions_mm_motive(i,3), 'YData', latestValidPositions_mm_motive(i,1), 'ZData', latestValidPositions_mm_motive(i,2)); 
                elseif ~isempty(h_curr); set(h_curr, 'XData', NaN, 'YData', NaN, 'ZData', NaN); end
            else
                if ~isempty(h_traj); set(h_traj, 'XData', NaN, 'YData', NaN, 'ZData', NaN); end
                if ~isempty(h_curr); set(h_curr, 'XData', NaN, 'YData', NaN, 'ZData', NaN); end
            end
        end

        h_rb1_frame = findobj(h_plot, 'Tag', 'RB1Frame');
        if ~any(isnan(latestValidPositions_mm_motive(1:3,:)), 'all') && ~isempty(h_rb1_frame)
            p_motive = latestValidPositions_mm_motive(1:3,:); % [m1x m1y m1z; m2x m2y m2z; ...]
            set(h_rb1_frame, 'XData', [p_motive(1,3) p_motive(2,3) p_motive(3,3) p_motive(1,3)], ... % Motive Z for Robot X
                               'YData', [p_motive(1,1) p_motive(2,1) p_motive(3,1) p_motive(1,1)], ... % Motive X for Robot Y
                               'ZData', [p_motive(1,2) p_motive(2,2) p_motive(3,2) p_motive(1,2)]);   % Motive Y for Robot Z
        elseif ~isempty(h_rb1_frame); set(h_rb1_frame, 'XData', NaN, 'YData', NaN, 'ZData', NaN); end
        
        h_rb2_frame = findobj(h_plot, 'Tag', 'RB2Frame');
        if ~any(isnan(latestValidPositions_mm_motive(4:6,:)), 'all') && ~isempty(h_rb2_frame)
            p_motive = latestValidPositions_mm_motive(4:6,:);
            set(h_rb2_frame, 'XData', [p_motive(1,3) p_motive(2,3) p_motive(3,3) p_motive(1,3)], ...
                               'YData', [p_motive(1,1) p_motive(2,1) p_motive(3,1) p_motive(1,1)], ...
                               'ZData', [p_motive(1,2) p_motive(2,2) p_motive(3,2) p_motive(1,2)]);
        elseif ~isempty(h_rb2_frame); set(h_rb2_frame, 'XData', NaN, 'YData', NaN, 'ZData', NaN); end

        axis_length_mm = 100;
        % RB1 visuals
        h_rb1_origin_plot = findobj(h_plot, 'Tag', 'RB1Origin'); 
        h_rb1_robot_ax = findobj(h_plot, 'Tag', 'RB1RobotAxisX'); 
        h_rb1_robot_ay = findobj(h_plot, 'Tag', 'RB1RobotAxisY'); 
        h_rb1_robot_az = findobj(h_plot, 'Tag', 'RB1RobotAxisZ');
        valid_rb1_pos_idx = find(~any(isnan(rb1_position_history_mm), 2), 1, 'last');
        valid_rb1_orient_idx = find(~any(isnan(rb1_orientation_history(:,1,1))), 1, 'last'); % Using original Motive orientation
        if ~isempty(valid_rb1_pos_idx) && ~isempty(valid_rb1_orient_idx)
            pos_mm_motive = rb1_position_history_mm(valid_rb1_pos_idx, :); % [mx, my, mz]
            pos_plot_robot = [pos_mm_motive(3), pos_mm_motive(1), pos_mm_motive(2)]; % [RobotX, RobotY, RobotZ] for plotting
            R_motive = squeeze(rb1_orientation_history(valid_rb1_orient_idx,:,:)); % Original Motive RM

            if ~isempty(h_rb1_origin_plot); set(h_rb1_origin_plot, 'XData', pos_plot_robot(1), 'YData', pos_plot_robot(2), 'ZData', pos_plot_robot(3)); end
            
            % Motive's local axes in Motive frame (columns of R_motive)
            motive_local_X_in_motive_frame = R_motive(:,1); % This is Robot's Y-axis direction
            motive_local_Y_in_motive_frame = R_motive(:,2); % This is Robot's Z-axis direction
            motive_local_Z_in_motive_frame = R_motive(:,3); % This is Robot's X-axis direction

            % Transform these direction vectors for plotting in "My Robot" plot coordinates
            % dir_RobotX_plot = [motive_local_Z_in_motive_frame(3); motive_local_Z_in_motive_frame(1); motive_local_Z_in_motive_frame(2)];
            % dir_RobotY_plot = [motive_local_X_in_motive_frame(3); motive_local_X_in_motive_frame(1); motive_local_X_in_motive_frame(2)];
            % dir_RobotZ_plot = [motive_local_Y_in_motive_frame(3); motive_local_Y_in_motive_frame(1); motive_local_Y_in_motive_frame(2)];
            % Or more directly using C_rm on each column:
            dir_RobotX_plot = C_rm * motive_local_Z_in_motive_frame;
            dir_RobotY_plot = C_rm * motive_local_X_in_motive_frame;
            dir_RobotZ_plot = C_rm * motive_local_Y_in_motive_frame;


            if ~isempty(h_rb1_robot_ax); set(h_rb1_robot_ax, 'XData', [pos_plot_robot(1), pos_plot_robot(1)+axis_length_mm*dir_RobotX_plot(1)], 'YData', [pos_plot_robot(2), pos_plot_robot(2)+axis_length_mm*dir_RobotX_plot(2)], 'ZData', [pos_plot_robot(3), pos_plot_robot(3)+axis_length_mm*dir_RobotX_plot(3)]); end
            if ~isempty(h_rb1_robot_ay); set(h_rb1_robot_ay, 'XData', [pos_plot_robot(1), pos_plot_robot(1)+axis_length_mm*dir_RobotY_plot(1)], 'YData', [pos_plot_robot(2), pos_plot_robot(2)+axis_length_mm*dir_RobotY_plot(2)], 'ZData', [pos_plot_robot(3), pos_plot_robot(3)+axis_length_mm*dir_RobotY_plot(3)]); end
            if ~isempty(h_rb1_robot_az); set(h_rb1_robot_az, 'XData', [pos_plot_robot(1), pos_plot_robot(1)+axis_length_mm*dir_RobotZ_plot(1)], 'YData', [pos_plot_robot(2), pos_plot_robot(2)+axis_length_mm*dir_RobotZ_plot(2)], 'ZData', [pos_plot_robot(3), pos_plot_robot(3)+axis_length_mm*dir_RobotZ_plot(3)]); end
        else
            if ~isempty(h_rb1_origin_plot); set(h_rb1_origin_plot,'XData',NaN,'YData',NaN,'ZData',NaN);end; if ~isempty(h_rb1_robot_ax);set(h_rb1_robot_ax,'XData',NaN,'YData',NaN,'ZData',NaN);end; if ~isempty(h_rb1_robot_ay);set(h_rb1_robot_ay,'XData',NaN,'YData',NaN,'ZData',NaN);end; if ~isempty(h_rb1_robot_az);set(h_rb1_robot_az,'XData',NaN,'YData',NaN,'ZData',NaN);end
        end
        % RB2 visuals (similar logic)
        h_rb2_origin_plot = findobj(h_plot, 'Tag', 'RB2Origin'); 
        h_rb2_robot_ax = findobj(h_plot, 'Tag', 'RB2RobotAxisX'); 
        h_rb2_robot_ay = findobj(h_plot, 'Tag', 'RB2RobotAxisY'); 
        h_rb2_robot_az = findobj(h_plot, 'Tag', 'RB2RobotAxisZ');
        valid_rb2_pos_idx = find(~any(isnan(rb2_position_history_mm), 2), 1, 'last');
        valid_rb2_orient_idx = find(~any(isnan(rb2_orientation_history(:,1,1))), 1, 'last');
        if ~isempty(valid_rb2_pos_idx) && ~isempty(valid_rb2_orient_idx)
            pos_mm_motive = rb2_position_history_mm(valid_rb2_pos_idx, :);
            pos_plot_robot = [pos_mm_motive(3), pos_mm_motive(1), pos_mm_motive(2)];
            R_motive = squeeze(rb2_orientation_history(valid_rb2_orient_idx,:,:));
            if ~isempty(h_rb2_origin_plot); set(h_rb2_origin_plot, 'XData', pos_plot_robot(1), 'YData', pos_plot_robot(2), 'ZData', pos_plot_robot(3)); end
            
            dir_RobotX_plot = C_rm * R_motive(:,3); % Motive Z is Robot X
            dir_RobotY_plot = C_rm * R_motive(:,1); % Motive X is Robot Y
            dir_RobotZ_plot = C_rm * R_motive(:,2); % Motive Y is Robot Z

            if ~isempty(h_rb2_robot_ax); set(h_rb2_robot_ax, 'XData', [pos_plot_robot(1), pos_plot_robot(1)+axis_length_mm*dir_RobotX_plot(1)], 'YData', [pos_plot_robot(2), pos_plot_robot(2)+axis_length_mm*dir_RobotX_plot(2)], 'ZData', [pos_plot_robot(3), pos_plot_robot(3)+axis_length_mm*dir_RobotX_plot(3)]); end
            if ~isempty(h_rb2_robot_ay); set(h_rb2_robot_ay, 'XData', [pos_plot_robot(1), pos_plot_robot(1)+axis_length_mm*dir_RobotY_plot(1)], 'YData', [pos_plot_robot(2), pos_plot_robot(2)+axis_length_mm*dir_RobotY_plot(2)], 'ZData', [pos_plot_robot(3), pos_plot_robot(3)+axis_length_mm*dir_RobotY_plot(3)]); end
            if ~isempty(h_rb2_robot_az); set(h_rb2_robot_az, 'XData', [pos_plot_robot(1), pos_plot_robot(1)+axis_length_mm*dir_RobotZ_plot(1)], 'YData', [pos_plot_robot(2), pos_plot_robot(2)+axis_length_mm*dir_RobotZ_plot(2)], 'ZData', [pos_plot_robot(3), pos_plot_robot(3)+axis_length_mm*dir_RobotZ_plot(3)]); end
        else
             if ~isempty(h_rb2_origin_plot);set(h_rb2_origin_plot,'XData',NaN,'YData',NaN,'ZData',NaN);end; if ~isempty(h_rb2_robot_ax);set(h_rb2_robot_ax,'XData',NaN,'YData',NaN,'ZData',NaN);end; if ~isempty(h_rb2_robot_ay);set(h_rb2_robot_ay,'XData',NaN,'YData',NaN,'ZData',NaN);end; if ~isempty(h_rb2_robot_az);set(h_rb2_robot_az,'XData',NaN,'YData',NaN,'ZData',NaN);end
        end

        all_plotted_data_robot_coords = []; % Data for auto-scaling, now in robot plot coords
        valid_rb1_pos_indices = find(~any(isnan(rb1_position_history_mm),2));
        if ~isempty(valid_rb1_pos_indices)
            motive_coords = rb1_position_history_mm(valid_rb1_pos_indices,:);
            robot_plot_coords = [motive_coords(:,3), motive_coords(:,1), motive_coords(:,2)];
            all_plotted_data_robot_coords = [all_plotted_data_robot_coords; robot_plot_coords];
        end
        for i_marker_hist = 1:NUM_MARKERS
            valid_marker_indices = find(~all(isnan(squeeze(marker_history_mm(:,i_marker_hist,:))),2));
            if ~isempty(valid_marker_indices)
                motive_coords = squeeze(marker_history_mm(valid_marker_indices,i_marker_hist,:));
                if size(motive_coords,2)~=3 && size(motive_coords,1)==3; motive_coords = motive_coords'; end
                if size(motive_coords,2)==3
                    robot_plot_coords = [motive_coords(:,3), motive_coords(:,1), motive_coords(:,2)];
                    all_plotted_data_robot_coords = [all_plotted_data_robot_coords; robot_plot_coords];
                end
            end
        end
        if ~isempty(all_plotted_data_robot_coords)
            min_coords = min(all_plotted_data_robot_coords, [], 1) - axis_length_mm*0.2; max_coords = max(all_plotted_data_robot_coords, [], 1) + axis_length_mm*0.2;
             min_coords = min(min_coords, -axis_length_mm*0.1); max_coords = max(max_coords, axis_length_mm*0.1); % Ensure origin is somewhat visible
            center = (min_coords + max_coords)/2; max_range = max(max_coords - min_coords);
            if max_range < 2*axis_length_mm; max_range = 2*axis_length_mm; end % Ensure a minimum plot range
            xlim(h_plot, [center(1)-max_range/2, center(1)+max_range/2]); 
            ylim(h_plot, [center(2)-max_range/2, center(2)+max_range/2]); 
            zlim(h_plot, [center(3)-max_range/2, center(3)+max_range/2]);
        else
            xlim(h_plot, [-200 200]); ylim(h_plot, [-200 200]); zlim(h_plot, [-200 200]);
        end
        
        textStr = '';
        if ~isnan(last_distance_m); textStr = [textStr sprintf('Distance RB1-RB2: %.3f m (%.1f mm)\n\n', last_distance_m, last_distance_m*1000)];
        else; textStr = [textStr sprintf('Distance RB1-RB2: N/A\n\n')]; end
        if ~isempty(valid_rb1_pos_idx)
             pos_mm_motive_rb1_rel = rb1_position_history_mm(valid_rb1_pos_idx, :); % [mx, my, mz]
             textStr = [textStr sprintf('RB1 Pos (MyRobot XYZ): [%.1f, %.1f, %.1f] mm\n', pos_mm_motive_rb1_rel(3), pos_mm_motive_rb1_rel(1), pos_mm_motive_rb1_rel(2))]; % mz, mx, my
        end
        txtInfoObj = findobj(h_fig, 'Tag', 'InfoTextDisplay');
        if isempty(txtInfoObj); annotation(h_fig,'textbox',[0.02 0.75 0.25 0.2],'String',textStr,'EdgeColor','k','Tag','InfoTextDisplay','FitBoxToText','off','BackgroundColor','w','VerticalAlignment','top','HorizontalAlignment','left','FontSize',8);
        else; set(txtInfoObj, 'String', textStr); end

        % Update Orientation Plots (using rbX_robot_euler_history)
        subplot(h_subplot_rb1);
        h_r1_roll=findobj(h_subplot_rb1,'Tag','RB1Roll'); h_r1_pitch=findobj(h_subplot_rb1,'Tag','RB1Pitch'); h_r1_yaw=findobj(h_subplot_rb1,'Tag','RB1Yaw');
        valid_euler1 = ~any(isnan(rb1_robot_euler_history),2);
        if any(valid_euler1)
            indices = find(valid_euler1); angles = rb1_robot_euler_history(valid_euler1,:);
            if ~isempty(h_r1_roll); set(h_r1_roll, 'XData', indices, 'YData', angles(:,1));end
            if ~isempty(h_r1_pitch); set(h_r1_pitch, 'XData', indices, 'YData', angles(:,2));end
            if ~isempty(h_r1_yaw); set(h_r1_yaw, 'XData', indices, 'YData', angles(:,3));end
            xlim(h_subplot_rb1, [max(1, min(indices)-10), max(indices)+10]);
            if ~isempty(angles); ylim(h_subplot_rb1, [min(angles(:))-10, max(angles(:))+10]); end
        end
        subplot(h_subplot_rb2);
        h_r2_roll=findobj(h_subplot_rb2,'Tag','RB2Roll'); h_r2_pitch=findobj(h_subplot_rb2,'Tag','RB2Pitch'); h_r2_yaw=findobj(h_subplot_rb2,'Tag','RB2Yaw');
        valid_euler2 = ~any(isnan(rb2_robot_euler_history),2);
        if any(valid_euler2)
            indices = find(valid_euler2); angles = rb2_robot_euler_history(valid_euler2,:);
            if ~isempty(h_r2_roll); set(h_r2_roll, 'XData', indices, 'YData', angles(:,1)); end
            if ~isempty(h_r2_pitch); set(h_r2_pitch, 'XData', indices, 'YData', angles(:,2)); end
            if ~isempty(h_r2_yaw); set(h_r2_yaw, 'XData', indices, 'YData', angles(:,3)); end
            xlim(h_subplot_rb2, [max(1, min(indices)-10), max(indices)+10]);
            if ~isempty(angles); ylim(h_subplot_rb2, [min(angles(:))-10, max(angles(:))+10]); end
        end
        drawnow;

    case 'get_distance'
        varargout{1} = last_distance_m;

    case 'save'
        if isempty(marker_history_mm); fprintf('Logger not init.\n'); return; end
        if isempty(C_rm); C_rm = [[0 0 1]; [1 0 0]; [0 1 0]]; end 
        if isempty(param); filebase = sprintf('MarkerLogger_MyRobotCoords_mm_%s', datestr(now,'yyyymmdd_HHMMSS'));
        else; [~, filebase, ~] = fileparts(param); end

        full_csv_filename = sprintf('%s_full_data.csv', filebase);
        fid = -1; 
        try
            fid = fopen(full_csv_filename, 'w');
            if fid == -1; error('Failed to open file %s for writing.', full_csv_filename); end
            % CSV Header reflects "My Robot" coordinates for positions and Euler angles
            header_str = "SampleIdx,Time_s_approx," + ...
                         "RB1_MyRobotX_mm,RB1_MyRobotY_mm,RB1_MyRobotZ_mm," + ...
                         "RB1_qX_motive,RB1_qY_motive,RB1_qZ_motive,RB1_qW_motive," + ... % Quaternions are original Motive
                         "RB1_MyRobotRoll_deg,RB1_MyRobotPitch_deg,RB1_MyRobotYaw_deg," + ...
                         "RB2_MyRobotX_mm,RB2_MyRobotY_mm,RB2_MyRobotZ_mm," + ...
                         "RB2_qX_motive,RB2_qY_motive,RB2_qZ_motive,RB2_qW_motive," + ...
                         "RB2_MyRobotRoll_deg,RB2_MyRobotPitch_deg,RB2_MyRobotYaw_deg," + ...
                         "Dist_RB1_RB2_m_Current";
            for m=1:NUM_MARKERS; header_str = header_str + sprintf(",M%d_MyRobotX_mm,M%d_MyRobotY_mm,M%d_MyRobotZ_mm", m,m,m); end
            fprintf(fid, '%s\n', header_str);

            num_points_to_save = data_actually_logged_count;
            start_idx_in_buffer = 1;
            if num_points_to_save == historyLength; start_idx_in_buffer = currentIndex; end

            csv_sample_idx = 0;
            for k_offset = 0:num_points_to_save-1
                idx_to_save = mod(start_idx_in_buffer + k_offset -1, historyLength) + 1;
                
                pos_rb1_motive = rb1_position_history_mm(idx_to_save,:);
                pos_rb2_motive = rb2_position_history_mm(idx_to_save,:);

                if ~all(isnan(pos_rb1_motive)) || ~all(isnan(pos_rb2_motive))
                    csv_sample_idx = csv_sample_idx + 1;
                    fprintf(fid, '%d,%.3f,', csv_sample_idx, (csv_sample_idx-1)*0.1);
                    
                    % RB1 Data (Positions permuted for My Robot)
                    fprintf(fid, '%.3f,%.3f,%.3f,', pos_rb1_motive(3), pos_rb1_motive(1), pos_rb1_motive(2)); % mz,mx,my
                    fprintf(fid, '%.4f,%.4f,%.4f,%.4f,', rb1_quaternion_history(idx_to_save,:)); % Original Quaternions
                    fprintf(fid, '%.2f,%.2f,%.2f,', rb1_robot_euler_history(idx_to_save,:)); % Robot Euler
                    % RB2 Data
                    fprintf(fid, '%.3f,%.3f,%.3f,', pos_rb2_motive(3), pos_rb2_motive(1), pos_rb2_motive(2)); % mz,mx,my
                    fprintf(fid, '%.4f,%.4f,%.4f,%.4f,', rb2_quaternion_history(idx_to_save,:));
                    fprintf(fid, '%.2f,%.2f,%.2f,', rb2_robot_euler_history(idx_to_save,:));
                    
                    fprintf(fid, '%.4f', NaN); % Placeholder for historical distance

                    for m=1:NUM_MARKERS
                        marker_pos_motive = marker_history_mm(idx_to_save,m,:);
                        fprintf(fid, ',%.3f,%.3f,%.3f', marker_pos_motive(3), marker_pos_motive(1), marker_pos_motive(2)); % mz,mx,my
                    end
                    fprintf(fid, '\n');
                end
            end
            fclose(fid);
            fprintf('Saved data to %s (%d points written to CSV, MyRobot Coords)\n', full_csv_filename, csv_sample_idx);
        catch e_save
            fprintf('Error saving to CSV: %s\n', e_save.message);
            if fid ~= -1; fclose(fid); end
        end

    case 'close'
        clearvars -global h_fig_global 
        clear marker_history_mm rb1_position_history_mm rb2_position_history_mm;
        clear rb1_orientation_history rb2_orientation_history rb1_robot_euler_history rb2_robot_euler_history;
        clear rb1_quaternion_history rb2_quaternion_history;
        clear last_raw_rb1_pos_m last_raw_rb2_pos_m last_distance_m C_rm;
        clear currentIndex historyLength isRb2OriginSet rb2_origin_m data_actually_logged_count;
        if ~isempty(h_fig) && ishandle(h_fig); annotation(h_fig,'textbox',[0.1 0.92 0.8 0.05],'String','Logger closed. Window remains.','EdgeColor','none','HorizontalAlignment','center','BackgroundColor',[1 .8 .8]); end
        fprintf('Marker logger closed (visualization window remains open)\n');
    otherwise
        fprintf('Unknown command: %s\n', command);
end
end

function distance = calculate_distance_subfunc(pos1, pos2)
    if ~(isnumeric(pos1) && (isequal(size(pos1),[1,3]) || isequal(size(pos1),[3,1]))) || any(isnan(pos1(:))) || ~(isnumeric(pos2) && (isequal(size(pos2),[1,3]) || isequal(size(pos2),[3,1]))) || any(isnan(pos2(:)))
        distance = NaN; return;
    end
    diff_vec = reshape(pos1,1,3) - reshape(pos2,1,3);
    distance = norm(diff_vec);
end

function euler = rotationMatrixToEulerAngles(R) % Standard ZYX Euler angles
    if any(isnan(R),'all') || ~isequal(size(R),[3,3]); euler = [NaN NaN NaN]; return; end
    pitch = asin(-R(3,1)); % Pitch around Y'
    if abs(R(3,1)) < (1-1e-6) % Not at gimbal lock for pitch
        roll = atan2(R(3,2),R(3,3)); % Roll around X'' (new X)
        yaw = atan2(R(2,1),R(1,1));   % Yaw around Z (original Z)
    else % Gimbal lock
        yaw = 0; % Can set yaw to 0
        roll = atan2(-R(1,2),R(2,2)); % Roll is sum/difference of yaw and original roll
    end
    euler = [roll, pitch, yaw] * 180/pi;
end

function figureClosedCallback(~, ~)
    fprintf('MarkerLogger: Visualization window (Figure 2) was manually closed.\n');
end
