%%inv kinematics code matlab
%% PRPP Robot - Shortest-Path IK Simulation
% FK Model: X = q1 + q4*cos(q2), Y = q4*sin(q2), Z = z_offset + q3
% Inverse kinematics with minimum joint displacement

clear all; close all; clc;

%% ========== CONFIGURATION ==========
% Joint limits
Q1_MIN = 0.0;        % X prismatic [m]
Q1_MAX = 0.190;
Q2_MIN = -pi;        % Rotation [rad]
Q2_MAX = pi;
Q3_MIN = 0.0;        % Z prismatic [m]
Q3_MAX = 0.400;
Q4_MIN = 0.0;        % Radial prismatic [m]
Q4_MAX = 0.400;

% Workspace bounds (Cartesian)
X_MIN = -Q4_MAX;                    % -0.400 m
X_MAX = Q1_MAX + Q4_MAX;            % 0.590 m
Y_MIN = -Q4_MAX;                    % -0.400 m
Y_MAX = Q4_MAX;                     % 0.400 m
Z_OFFSET = 0.220;                   % Fixed Z offset [m]
Z_MIN = Z_OFFSET + Q3_MIN;          % 0.220 m
Z_MAX = Z_OFFSET + Q3_MAX;          % 0.620 m

fprintf('========================================\n');
fprintf('   PRPP ROBOT - SHORTEST-PATH IK\n');
fprintf('========================================\n');
fprintf('Forward Kinematics:\n');
fprintf('  X = q1 + q4·cos(q2)\n');
fprintf('  Y = q4·sin(q2)\n');
fprintf('  Z = %.3f + q3\n\n', Z_OFFSET);
fprintf('Workspace (Cartesian):\n');
fprintf('  X: [%.0f, %.0f] mm\n', X_MIN*1000, X_MAX*1000);
fprintf('  Y: [%.0f, %.0f] mm\n', Y_MIN*1000, Y_MAX*1000);
fprintf('  Z: [%.0f, %.0f] mm\n\n', Z_MIN*1000, Z_MAX*1000);
fprintf('Joint Limits:\n');
fprintf('  q1 (X):   [%.0f, %.0f] mm\n', Q1_MIN*1000, Q1_MAX*1000);
fprintf('  q2 (Rot): [%.0f, %.0f] deg\n', rad2deg(Q2_MIN), rad2deg(Q2_MAX));
fprintf('  q3 (Z):   [%.0f, %.0f] mm\n', Q3_MIN*1000, Q3_MAX*1000);
fprintf('  q4 (Rad): [%.0f, %.0f] mm\n', Q4_MIN*1000, Q4_MAX*1000);
fprintf('========================================\n\n');

%% ========== BUILD ROBOT ==========
robot = rigidBodyTree('DataFormat', 'row');

% J1: Prismatic along X
body1 = rigidBody('link1');
jnt1 = rigidBodyJoint('j1_prismatic_x', 'prismatic');
jnt1.JointAxis = [1 0 0];
jnt1.PositionLimits = [Q1_MIN, Q1_MAX];
setFixedTransform(jnt1, eye(4));
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% J2: Revolute about Z
body2 = rigidBody('link2');
jnt2 = rigidBodyJoint('j2_revolute_z', 'revolute');
jnt2.JointAxis = [0 0 1];
jnt2.PositionLimits = [Q2_MIN, Q2_MAX];
setFixedTransform(jnt2, eye(4));
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

% J3: Prismatic along Z (with offset)
body3 = rigidBody('link3');
jnt3 = rigidBodyJoint('j3_prismatic_z', 'prismatic');
jnt3.JointAxis = [0 0 1];
jnt3.PositionLimits = [Q3_MIN, Q3_MAX];
setFixedTransform(jnt3, trvec2tform([0 0 Z_OFFSET]));
body3.Joint = jnt3;
addBody(robot, body3, 'link2');

% J4: Prismatic along local X (radial in XY plane after rotation)
body4 = rigidBody('end_effector');
jnt4 = rigidBodyJoint('j4_prismatic_radial', 'prismatic');
jnt4.JointAxis = [1 0 0];  % Local X axis (rotated by q2)
jnt4.PositionLimits = [Q4_MIN, Q4_MAX];
setFixedTransform(jnt4, eye(4));
body4.Joint = jnt4;
addBody(robot, body4, 'link3');

fprintf('✓ Robot model built\n\n');

%% ========== VERIFY FORWARD KINEMATICS ==========
fprintf('Verifying FK model...\n');
q_test = [0.095, deg2rad(45), 0.200, 0.150];
T_test = getTransform(robot, q_test, 'end_effector');
pos_test = tform2trvec(T_test);

X_fk = q_test(1) + q_test(4)*cos(q_test(2));
Y_fk = q_test(4)*sin(q_test(2));
Z_fk = Z_OFFSET + q_test(3);

fk_error = norm([X_fk Y_fk Z_fk] - pos_test);
fprintf('  Test config: q=[%.3f, %.1f°, %.3f, %.3f]\n', ...
        q_test(1), rad2deg(q_test(2)), q_test(3), q_test(4));
fprintf('  Analytical FK: [%.4f, %.4f, %.4f]\n', X_fk, Y_fk, Z_fk);
fprintf('  rigidBodyTree: [%.4f, %.4f, %.4f]\n', pos_test);
fprintf('  Error: %.6f m\n', fk_error);

if fk_error > 0.001
    error('FK model mismatch! Check robot definition.');
else
    fprintf('  ✓ FK verified (error < 1mm)\n\n');
end

%% ========== CREATE SIMULATION UI ==========
fig = figure('Name', 'PRPP Robot - Shortest-Path IK', ...
             'Position', [50 50 1800 950], 'Color', [0.94 0.94 0.96], ...
             'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'figure');

% Main 3D view
ax_robot = axes('Position', [0.03 0.08 0.65 0.87]);
current_q = [0.095, 0, 0.200, 0.100];
show(robot, current_q, 'Parent', ax_robot, 'Frames', 'off', 'PreservePlot', false);
hold(ax_robot, 'on');

% Workspace boundaries
plot3(ax_robot, 0, 0, 0, 'p', 'MarkerSize', 50, 'LineWidth', 6, ...
      'MarkerEdgeColor', [0 0 0.6], 'MarkerFaceColor', 'yellow', 'Tag', 'workspace');
text(0, 0, 0.06, 'ORIGIN', 'Parent', ax_robot, 'FontSize', 12, ...
     'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
     'BackgroundColor', [1 1 1 0.9], 'Tag', 'workspace');

% Axes
plot3(ax_robot, [X_MIN X_MAX], [0 0], [0 0], 'r-', 'LineWidth', 5, 'Tag', 'workspace');
plot3(ax_robot, [0 0], [Y_MIN Y_MAX], [0 0], 'g-', 'LineWidth', 5, 'Tag', 'workspace');
plot3(ax_robot, [0 0], [0 0], [Z_MIN Z_MAX], 'b-', 'LineWidth', 5, 'Tag', 'workspace');
plot3(ax_robot, [0 0], [0 0], [0 Z_MIN], 'r--', 'LineWidth', 4, 'Tag', 'workspace');

% Coordinate labels
quiver3(0,0,0, 0.20,0,0, 'r', 'LineWidth', 6, 'MaxHeadSize', 0.3, 'Parent', ax_robot, 'Tag', 'workspace');
quiver3(0,0,0, 0,0.20,0, 'g', 'LineWidth', 6, 'MaxHeadSize', 0.3, 'Parent', ax_robot, 'Tag', 'workspace');
quiver3(0,0,0, 0,0,0.20, 'b', 'LineWidth', 6, 'MaxHeadSize', 0.3, 'Parent', ax_robot, 'Tag', 'workspace');
text(0.23, 0, 0, 'X', 'Parent', ax_robot, 'FontSize', 18, 'FontWeight', 'bold', 'Color', 'r', 'Tag', 'workspace');
text(0, 0.23, 0, 'Y', 'Parent', ax_robot, 'FontSize', 18, 'FontWeight', 'bold', 'Color', 'g', 'Tag', 'workspace');
text(0, 0, 0.23, 'Z', 'Parent', ax_robot, 'FontSize', 18, 'FontWeight', 'bold', 'Color', 'b', 'Tag', 'workspace');

axis(ax_robot, 'equal');
grid(ax_robot, 'on');
xlabel(ax_robot, 'X [m]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel(ax_robot, 'Y [m]', 'FontSize', 14, 'FontWeight', 'bold');
zlabel(ax_robot, 'Z [m]', 'FontSize', 14, 'FontWeight', 'bold');
title(ax_robot, 'PRPP Robot - Shortest-Path IK', 'FontSize', 18, 'FontWeight', 'bold');
view(ax_robot, 45, 25);
set(ax_robot, 'FontSize', 12, 'Color', [0.96 0.96 1]);
lighting(ax_robot, 'gouraud');
light_handle = camlight(ax_robot, 'headlight');

% End effector marker
ee_marker = plot3(ax_robot, 0, 0, 0, 'ro', 'MarkerSize', 25, 'LineWidth', 5, ...
    'MarkerFaceColor', [1 0.2 0.2], 'MarkerEdgeColor', [0.6 0 0], 'Tag', 'marker');

% Path trail
path_line = plot3(ax_robot, [], [], [], 'c-', 'LineWidth', 3.5, 'LineStyle', ':', 'Tag', 'path');

% Joint labels
txt_j1 = text(0,0,0,'', 'Parent', ax_robot, 'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 0.9 0.9 0.95], 'EdgeColor', 'r', 'Tag', 'label');
txt_j2 = text(0,0,0,'', 'Parent', ax_robot, 'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.9 0.9 1 0.95], 'EdgeColor', 'b', 'Tag', 'label');
txt_j3 = text(0,0,0,'', 'Parent', ax_robot, 'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.9 1 0.9 0.95], 'EdgeColor', [0 0.7 0], 'Tag', 'label');
txt_j4 = text(0,0,0,'', 'Parent', ax_robot, 'FontSize', 11, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 0.8 0.95], 'EdgeColor', [0.8 0.6 0], 'Tag', 'label');
txt_ee = text(0,0,0,'', 'Parent', ax_robot, 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 0.3 0.3 0.95], 'Color', 'w', 'EdgeColor', [0.8 0 0], 'LineWidth', 2.5, 'Tag', 'label');

% Status panel
panel = uipanel('Position', [0.70 0.08 0.28 0.87], 'BackgroundColor', [0.95 0.98 1], ...
    'BorderType', 'line', 'BorderWidth', 3, 'HighlightColor', [0 0.4 0.8], ...
    'Title', ' STATUS ', 'FontSize', 16, 'FontWeight', 'bold');
ax_status = axes('Parent', panel, 'Position', [0 0 1 1]);
axis(ax_status, 'off');

y_pos = 0.96;
dy = 0.045;

text(0.5, y_pos, 'SHORTEST-PATH IK', 'Parent', ax_status, 'FontSize', 18, ...
     'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', [0 0.3 0.7]);
y_pos = y_pos - 1.5*dy;

text(0.05, y_pos, 'END-EFFECTOR:', 'Parent', ax_status, 'FontSize', 12, 'FontWeight', 'bold');
y_pos = y_pos - dy;
txt_ee_pos = text(0.08, y_pos, '', 'Parent', ax_status, 'FontSize', 11, ...
    'FontName', 'Courier New', 'FontWeight', 'bold', 'Color', [0.6 0 0]);
y_pos = y_pos - 1.5*dy;

text(0.05, y_pos, 'JOINTS:', 'Parent', ax_status, 'FontSize', 12, 'FontWeight', 'bold');
y_pos = y_pos - dy;
txt_q1 = text(0.08, y_pos, '', 'Parent', ax_status, 'FontSize', 10, 'FontName', 'Courier New');
y_pos = y_pos - dy;
txt_q2 = text(0.08, y_pos, '', 'Parent', ax_status, 'FontSize', 10, 'FontName', 'Courier New');
y_pos = y_pos - dy;
txt_q3 = text(0.08, y_pos, '', 'Parent', ax_status, 'FontSize', 10, 'FontName', 'Courier New');
y_pos = y_pos - dy;
txt_q4 = text(0.08, y_pos, '', 'Parent', ax_status, 'FontSize', 10, 'FontName', 'Courier New');
y_pos = y_pos - 1.5*dy;

text(0.05, y_pos, 'JOINT DISPLACEMENT:', 'Parent', ax_status, 'FontSize', 12, 'FontWeight', 'bold');
y_pos = y_pos - dy;
txt_delta = text(0.08, y_pos, 'N/A', 'Parent', ax_status, 'FontSize', 10, 'FontName', 'Courier New');
y_pos = y_pos - 1.5*dy;

text(0.05, y_pos, 'ACCURACY:', 'Parent', ax_status, 'FontSize', 12, 'FontWeight', 'bold');
y_pos = y_pos - dy;
txt_error = text(0.08, y_pos, 'N/A', 'Parent', ax_status, 'FontSize', 11, 'FontWeight', 'bold');
y_pos = y_pos - 1.5*dy;

text(0.05, y_pos, 'STATUS:', 'Parent', ax_status, 'FontSize', 12, 'FontWeight', 'bold');
y_pos = y_pos - dy;
txt_motion = text(0.08, y_pos, 'Ready', 'Parent', ax_status, 'FontSize', 11, 'Color', [0 0.7 0]);

xlim(ax_status, [0 1]);
ylim(ax_status, [0 1]);

fprintf('✓ UI initialized\n\n');

%% ========== CONTROL LOOP ==========
fprintf('========================================\n');
fprintf('  READY FOR INPUT\n');
fprintf('========================================\n\n');

path_history = [];

while true
    fprintf('\n%s\n', repmat('=', 1, 60));
    fprintf('Enter target position (or press Enter to quit):\n');
    
    x_mm = input('  X (mm): ');
    if isempty(x_mm)
        fprintf('\n✓ Exiting.\n');
        break;
    end
    y_mm = input('  Y (mm): ');
    z_mm = input('  Z (mm): ');
    
    % Convert to meters
    X_target = x_mm / 1000;
    Y_target = y_mm / 1000;
    Z_target = z_mm / 1000;
    
    % Visual feedback
    set(txt_motion, 'String', 'Computing shortest-path IK...', 'Color', [0 0 0.8]);
    drawnow;
    
    % Solve IK with shortest-path
    [q_solution, valid, fk_error] = solve_ik_shortest_path(X_target, Y_target, Z_target, ...
                                                            robot, Z_OFFSET, ...
                                                            Q1_MIN, Q1_MAX, Q3_MIN, Q3_MAX, Q4_MAX, ...
                                                            current_q);
    
    if ~valid
        fprintf('\n✗ UNREACHABLE\n');
        set(txt_motion, 'String', 'Error: Target unreachable', 'Color', 'r');
        drawnow;
        continue;
    end
    
    % Compute joint displacement
    delta_q = q_solution - current_q;
    joint_distance = norm(delta_q);
    
    fprintf('\n✓ IK SOLVED (Shortest-Path)\n');
    fprintf('  q1 = %.3f m (%.1f mm)\n', q_solution(1), q_solution(1)*1000);
    fprintf('  q2 = %.3f rad (%.1f deg)\n', q_solution(2), rad2deg(q_solution(2)));
    fprintf('  q3 = %.3f m (%.1f mm)\n', q_solution(3), q_solution(3)*1000);
    fprintf('  q4 = %.3f m (%.1f mm)\n', q_solution(4), q_solution(4)*1000);
    fprintf('  FK error: %.3f mm\n', fk_error);
    fprintf('  Joint displacement: %.4f (Euclidean norm)\n', joint_distance);
    fprintf('  Δq: [%.3f, %.3f°, %.3f, %.3f]\n', ...
            delta_q(1)*1000, rad2deg(delta_q(2)), delta_q(3)*1000, delta_q(4)*1000);
    
    % Update delta display
    set(txt_delta, 'String', sprintf('Δq1:%+.1f Δq2:%+.1f° Δq3:%+.1f Δq4:%+.1f\nNorm: %.4f', ...
        delta_q(1)*1000, rad2deg(delta_q(2)), delta_q(3)*1000, delta_q(4)*1000, joint_distance));
    
    % Add target marker
    delete(findobj(ax_robot, 'Tag', 'target'));
    plot3(ax_robot, X_target, Y_target, Z_target, 'r*', 'MarkerSize', 45, ...
          'LineWidth', 6, 'Tag', 'target');
    
    % Sequential motion
    joint_names = {'J1 (X)', 'J2 (Rot)', 'J3 (Z)', 'J4 (Radial)'};
    q_temp = current_q;
    
    for j = 1:4
        if abs(q_solution(j) - current_q(j)) < 0.001
            continue;
        end
        
        fprintf('  Moving %s...\n', joint_names{j});
        set(txt_motion, 'String', sprintf('Moving %s', joint_names{j}), 'Color', [0.9 0.5 0]);
        
        % Animate
        for step = 1:25
            alpha = step / 25;
            q_temp(j) = (1-alpha)*current_q(j) + alpha*q_solution(j);
            
            % Clear robot visualization only
            children = get(ax_robot, 'Children');
            for i = 1:length(children)
                try
                    tag = get(children(i), 'Tag');
                    if isempty(tag) || (~strcmp(tag,'workspace') && ~strcmp(tag,'label') && ...
                       ~strcmp(tag,'marker') && ~strcmp(tag,'path') && ~strcmp(tag,'target'))
                        delete(children(i));
                    end
                end
            end
            
            % Redraw robot
            show(robot, q_temp, 'Parent', ax_robot, 'Frames', 'off', 'PreservePlot', true);
            lighting(ax_robot, 'gouraud');
            delete(light_handle);
            light_handle = camlight(ax_robot, 'headlight');
            
            % Update positions
            T1 = getTransform(robot, q_temp, 'link1');
            T2 = getTransform(robot, q_temp, 'link2');
            T3 = getTransform(robot, q_temp, 'link3');
            T_ee = getTransform(robot, q_temp, 'end_effector');
            
            p1 = tform2trvec(T1);
            p2 = tform2trvec(T2);
            p3 = tform2trvec(T3);
            p_ee = tform2trvec(T_ee);
            
            % Update labels
            set(txt_j1, 'Position', [p1(1), p1(2), p1(3)+0.03], ...
                'String', sprintf('q1: %.1fmm', q_temp(1)*1000));
            set(txt_j2, 'Position', [p2(1), p2(2), p2(3)+0.03], ...
                'String', sprintf('q2: %.1f°', rad2deg(q_temp(2))));
            set(txt_j3, 'Position', [p3(1), p3(2), p3(3)+0.03], ...
                'String', sprintf('q3: %.1fmm', q_temp(3)*1000));
            set(txt_j4, 'Position', [p_ee(1)-0.04, p_ee(2), p_ee(3)], ...
                'String', sprintf('q4: %.1fmm', q_temp(4)*1000));
            set(txt_ee, 'Position', [p_ee(1)+0.04, p_ee(2), p_ee(3)], ...
                'String', sprintf('EE\n[%.1f,%.1f,%.1f]', p_ee*1000));
            
            % Update marker
            set(ee_marker, 'XData', p_ee(1), 'YData', p_ee(2), 'ZData', p_ee(3));
            
            % Path trail
            path_history = [path_history; p_ee];
            if size(path_history,1) > 1
                set(path_line, 'XData', path_history(:,1), ...
                               'YData', path_history(:,2), ...
                               'ZData', path_history(:,3));
            end
            
            % Update status
            set(txt_ee_pos, 'String', sprintf('[%.1f, %.1f, %.1f] mm', p_ee*1000));
            set(txt_q1, 'String', sprintf('q1 = %7.1f mm', q_temp(1)*1000), 'Color', [0.8 0.2 0.2]);
            set(txt_q2, 'String', sprintf('q2 = %7.1f deg', rad2deg(q_temp(2))), 'Color', [0.2 0.2 0.8]);
            set(txt_q3, 'String', sprintf('q3 = %7.1f mm', q_temp(3)*1000), 'Color', [0 0.6 0]);
            set(txt_q4, 'String', sprintf('q4 = %7.1f mm', q_temp(4)*1000), 'Color', [0.7 0.5 0]);
            
            if fk_error < 1.0
                set(txt_error, 'String', sprintf('%.3f mm (Excellent)', fk_error), ...
                    'Color', [0 0.7 0]);
            else
                set(txt_error, 'String', sprintf('%.3f mm', fk_error), 'Color', [0.9 0.5 0]);
            end
            
            drawnow;
            pause(0.04);
        end
    end
    
    current_q = q_solution;
    set(txt_motion, 'String', 'Ready', 'Color', [0 0.7 0]);
    fprintf('  ✓ Complete\n');
end

%% ========== SHORTEST-PATH IK FUNCTION ==========

function [q, isValid, error_mm] = solve_ik_shortest_path(X, Y, Z, robot, z_offset, ...
                                                          q1_min, q1_max, q3_min, q3_max, q4_max, ...
                                                          q_current)
    % Shortest-path IK: find solution closest to current joint state
    % Treats all joints equally (Euclidean distance in joint space)
    
    % Step 1: Z axis (independent, only one solution)
    q3 = Z - z_offset;
    if q3 < q3_min || q3 > q3_max
        fprintf('  ✗ Z unreachable: q3=%.3f ∉ [%.3f,%.3f]\n', q3, q3_min, q3_max);
        q = q_current;
        isValid = false;
        error_mm = inf;
        return;
    end
    
    % Step 2: Check Y feasible
    if abs(Y) > q4_max
        fprintf('  ✗ |Y|=%.3f > q4_max=%.3f\n', abs(Y), q4_max);
        q = q_current;
        isValid = false;
        error_mm = inf;
        return;
    end
    
    % Step 3: Compute valid q1 interval
    s = sqrt(q4_max^2 - Y^2);
    q1_reach_min = X - s;
    q1_reach_max = X + s;
    q1_valid_min = max(q1_reach_min, q1_min);
    q1_valid_max = min(q1_reach_max, q1_max);
    
    if q1_valid_min > q1_valid_max
        fprintf('  ✗ X unreachable: no valid q1 interval\n');
        q = q_current;
        isValid = false;
        error_mm = inf;
        return;
    end
    
    % Step 4: Generate candidate q1 values
    q1_candidates = unique([
        q1_valid_min;
        q1_valid_max;
        max(q1_valid_min, min(X, q1_valid_max));
        max(q1_valid_min, min(q_current(1), q1_valid_max));
    ]);
    
    % Step 5: Build all IK candidates
    candidates = [];
    
    for i = 1:length(q1_candidates)
        q1_test = q1_candidates(i);
        
        % Compute q2 and q4 for this q1
        X_prime = X - q1_test;
        q4_test = sqrt(X_prime^2 + Y^2);
        q2_base = atan2(Y, X_prime);
        
        % Generate angle wrapping variants
        q2_variants = [
            q2_base;
            q2_base + 2*pi;
            q2_base - 2*pi;
        ];
        
        for j = 1:length(q2_variants)
            q2_test = q2_variants(j);
            q2_wrapped = wrapToPi(q2_test);
            
            q_candidate = [q1_test, q2_wrapped, q3, q4_test];
            
            if q1_test >= q1_min && q1_test <= q1_max && ...
               q4_test >= 0 && q4_test <= q4_max
                candidates = [candidates; q_candidate]; %#ok<AGROW>
            end
        end
    end
    
    % Remove duplicates
    candidates = uniquetol(candidates, 1e-6, 'ByRows', true);
    
    if isempty(candidates)
        fprintf('  ✗ No valid candidates\n');
        q = q_current;
        isValid = false;
        error_mm = inf;
        return;
    end
    
    % Step 6: Find candidate closest to current state
    best_q = [];
    min_distance = inf;
    
    for i = 1:size(candidates, 1)
        q_cand = candidates(i, :);
        
        % Wrap q2 to be closest to current q2
        q_cand(2) = wrap_angle_to_closest(q_cand(2), q_current(2));
        
        % Euclidean distance in joint space
        delta_q = q_cand - q_current;
        distance = norm(delta_q);
        
        if distance < min_distance
            min_distance = distance;
            best_q = q_cand;
        end
    end
    
    % Step 7: Verify FK
    T_ee = getTransform(robot, best_q, 'end_effector');
    pos_fk = tform2trvec(T_ee);
    error_mm = norm([X Y Z] - pos_fk) * 1000;
    
    if error_mm < 1.0
        q = best_q;
        isValid = true;
    else
        fprintf('  ⚠ FK error %.3fmm\n', error_mm);
        q = best_q;
        isValid = false;
    end
end

%% ========== HELPER FUNCTIONS ==========

function theta_out = wrap_angle_to_closest(theta, theta_ref)
    % Wrap theta to equivalent angle closest to theta_ref
    candidates = [
        theta;
        theta + 2*pi;
        theta - 2*pi;
        theta + 4*pi;
        theta - 4*pi;
    ];
    
    [~, idx] = min(abs(candidates - theta_ref));
    theta_out = candidates(idx);
    theta_out = wrapToPi(theta_out);
end
