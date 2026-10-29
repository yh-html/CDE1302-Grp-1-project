%% Fly the mission's real sweep path through the course's Simulink quadrotor
% Reads plan.json (written by export_plan.py from laptop/tag_mission.yaml), builds a
% stop-and-go reference like the mission asks for (sweep_reference.m), flies it through
% the course model PosControl_Sim_RPT, and reports tracking, timing and whether the
% camera model would have seen each of the six possible tag spots.
%
% This is the COURSE controller (position + attitude loops), not PX4 and not the ESP32:
% it answers "can a drone with this kind of controller follow this path, and how long
% does it take", not "will the real firmware work". Acceleration and yaw limits below
% are assumptions, not values read from the drone.
%
% Run from anywhere:  run('mini_project/simulink_sweep/run_sweep_sim.m')
% Set headless = false first to show the figures.

clearvars -except headless;
close all; clc;
if ~exist('headless', 'var'), headless = true; end

here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
results_dir = fullfile(here, 'results');
if ~exist(results_dir, 'dir'), mkdir(results_dir); end
addpath(here);
addpath(fullfile(root, 'trajectory', 'position_control'));
addpath(fullfile(root, 'trajectory', 'position_control', 'icon'));

old_visibility = get(groot, 'defaultFigureVisible');
if headless, set(groot, 'defaultFigureVisible', 'off'); end
visibility_cleanup = onCleanup(@() set(groot, 'defaultFigureVisible', old_visibility)); %#ok<NASGU>

plan = jsondecode(fileread(fullfile(here, 'plan.json')));
plan.start = plan.start(:).';
wp = plan.waypoints;
cam = plan.camera;

%% Assumptions (not measured on the drone)
opt = struct('tstep', 0.01, 'a_xy', 0.5, 'v_z', 0.3, 'a_z', 0.5, ...
    'w_yaw', 0.8, 'a_yaw', 2.0, 't_pad', 0.5);
arrival_radius = plan.arrival_radius_m;

%% Reference and simulation
ref = sweep_reference(plan, opt);
t_ref = ref.time;
V = ref.signals.values;
ref_pos = V(:, 1:3); ref_vel = V(:, 4:6); ref_yaw = V(:, 10);
fprintf('Reference: %d waypoints + return, %.1f m, %.1f s long.\n', ...
    size(wp, 1), plan.path_length_m, t_ref(end));

model = 'PosControl_Sim_RPT';
load_system(model);
try
    linked = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'off', 'Type', 'block');
    status = get_param(linked, 'LinkStatus');
    n_unresolved = sum(strcmp(status, 'unresolved'));
catch
    n_unresolved = 0;
end
if n_unresolved > 0
    warning('sweep:unresolved', '%d unresolved library link(s) in the model.', n_unresolved);
end
assignin('base', 'ref', ref);
set_param(model, 'StopTime', num2str(t_ref(end), '%.15g'));
lastwarn('', '');
sim_out = sim(model, 'ReturnWorkspaceOutputs', 'on');
[sim_t, sim_pos] = unpack_signal(get_var(sim_out, 'PosE'), 3);
[ang_t, sim_euler] = unpack_signal(get_var(sim_out, 'AngEuler'), 3);
sim_euler = interp1(ang_t, sim_euler, sim_t, 'linear', 'extrap');   % roll, pitch, yaw
sim_yaw = sim_euler(:, 3);

%% Tracking and timing
ref_pos_i = interp1(t_ref, ref_pos, sim_t, 'linear', 'extrap');
ref_vel_i = interp1(t_ref, ref_vel, sim_t, 'linear', 'extrap');
err_h = vecnorm(sim_pos(:, 1:2) - ref_pos_i(:, 1:2), 2, 2);
err_3d = vecnorm(sim_pos - ref_pos_i, 2, 2);
sim_speed = [0; vecnorm(diff(sim_pos(:, 1:2)), 2, 2) ./ diff(sim_t)];
sim_speed = movmean(sim_speed, 21);
ref_speed = vecnorm(ref_vel_i(:, 1:2), 2, 2);
tilt_deg = rad2deg(max(abs(sim_euler(:, 1)), abs(sim_euler(:, 2))));

% Mission events in the reference: takeoff command, landing command.
alt = plan.altitude_m;
t_takeoff_cmd = opt.t_pad;
idx_cruise = find(abs(ref_pos(:, 3) + alt) < 1e-9);
t_land_cmd = t_ref(idx_cruise(end)) - opt.t_pad;
mission_time = t_land_cmd - t_takeoff_cmd;

% Waypoint by waypoint: when the reference arrives, when the simulated drone is
% first within the mission's arrival radius, and how close it gets.
targets = [wp; plan.start];
n_t = size(targets, 1);
arrive = table('Size', [n_t 5], 'VariableTypes', repmat({'double'}, 1, 5), ...
    'VariableNames', {'waypoint', 't_ref_s', 't_in_radius_s', 'lag_s', 'closest_m'});
t_prev = t_takeoff_cmd + 5;     % start looking after takeoff
for k = 1:n_t
    d_ref = vecnorm(ref_pos(:, 1:2) - targets(k, :), 2, 2);
    ix = find(t_ref >= t_prev & d_ref < 1e-6, 1, 'first');
    t_arr = t_ref(ix);
    d_sim = vecnorm(sim_pos(:, 1:2) - targets(k, :), 2, 2);
    iy = find(sim_t >= t_prev & d_sim < arrival_radius, 1, 'first');
    t_in = NaN; if ~isempty(iy), t_in = sim_t(iy); end
    window = sim_t >= t_prev & sim_t <= t_arr + 5;
    arrive(k, :) = {k, t_arr, t_in, t_in - t_arr, min(d_sim(window))};
    t_prev = t_arr;
end

dt_speed = [diff(sim_t); 0];

%% Would the camera have seen each tag spot?  (same geometry as the mission code)
spots = plan.tag_spots;
n_s = size(spots, 1);
visible = false(numel(sim_t), n_s);
height = -sim_pos(:, 3);
for j = 1:n_s
    dx = spots(j, 1) - sim_pos(:, 1);
    dy = spots(j, 2) - sim_pos(:, 2);
    fwd = dx .* cos(sim_yaw) + dy .* sin(sim_yaw);
    right = -dx .* sin(sim_yaw) + dy .* cos(sim_yaw);
    p = deg2rad(cam.pitch_deg);
    a = fwd - cam.fwd_offset_m;
    b = height - cam.down_offset_m;
    tx = right - cam.right_offset_m;
    tz = cos(p) * a + sin(p) * b;
    ty = -sin(p) * a + cos(p) * b;
    rng = sqrt(tx.^2 + ty.^2 + tz.^2);
    elev = atan2d(height, hypot(fwd, right));
    visible(:, j) = height > 0.2 & tz > 0.1 & rng <= plan.detection.max_range_m & ...
        abs(atan2d(tx, tz)) <= cam.hfov_deg / 2 & abs(atan2d(ty, tz)) <= cam.vfov_deg / 2 & ...
        elev >= plan.detection.min_view_elevation_deg;
end
dt_s = [diff(sim_t); 0];
coverage = table((1:n_s).', spots(:, 1), spots(:, 2), ...
    arrayfun(@(j) any(visible(:, j)), 1:n_s).', ...
    arrayfun(@(j) first_time(sim_t, visible(:, j)), 1:n_s).', ...
    arrayfun(@(j) sum(dt_s(visible(:, j))), 1:n_s).', ...
    'VariableNames', {'spot', 'x_m', 'y_m', 'seen', 'first_seen_s', 'time_in_view_s'});

%% Summary
summary = struct();
summary.reference_duration_s = t_ref(end);
summary.mission_time_takeoff_to_landing_cmd_s = mission_time;
summary.max_horizontal_tracking_error_m = max(err_h);
summary.rms_horizontal_tracking_error_m = sqrt(mean(err_h.^2));
summary.max_3d_tracking_error_m = max(err_3d);
summary.max_sim_speed_mps = max(sim_speed);
summary.speed_overshoot_percent = 100 * (max(sim_speed) / plan.cruise_speed_mps - 1);
summary.seconds_above_110pct_of_cruise = sum(dt_speed(sim_speed > 1.1 * plan.cruise_speed_mps));
summary.max_ref_speed_mps = max(ref_speed);
summary.max_tilt_deg = max(tilt_deg);
summary.all_waypoints_within_arrival_radius = all(~isnan(arrive.t_in_radius_s));
summary.max_waypoint_lag_s = max(arrive.lag_s);
summary.max_closest_miss_m = max(arrive.closest_m);
summary.tag_spots_seen = sum(coverage.seen);
summary.tag_spots_total = n_s;
summary.assumptions = opt;
summary.arrival_radius_m = arrival_radius;
summary.simulink_warning = lastwarn;
summary.coverage = table2struct(coverage);
summary.waypoints = table2struct(arrive);
fid = fopen(fullfile(results_dir, 'sweep_summary.json'), 'w');
fprintf(fid, '%s', jsonencode(summary, 'PrettyPrint', true));
fclose(fid);
writetable(arrive, fullfile(results_dir, 'sweep_waypoints.csv'));
writetable(coverage, fullfile(results_dir, 'sweep_tag_coverage.csv'));

%% Figure 1: top view
f1 = figure('Color', 'w', 'Position', [100 100 760 820]);
ax = axes(f1); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'on');
sa = plan.search_area;
rectangle(ax, 'Position', [sa.min_y, sa.min_x, sa.max_y - sa.min_y, sa.max_x - sa.min_x], ...
    'EdgeColor', [0.4 0.4 0.4], 'LineStyle', '--', 'LineWidth', 1.2);
ls = plan.launch_square;
rectangle(ax, 'Position', [ls.min_y, ls.min_x, ls.max_y - ls.min_y, ls.max_x - ls.min_x], ...
    'EdgeColor', [0.1 0.55 0.1], 'FaceColor', [0.85 0.95 0.85], 'LineWidth', 1.2);
% Reference drawn wide and light underneath so it stays visible where the two agree.
h_ref = plot(ax, ref_pos(:, 2), ref_pos(:, 1), '-', 'Color', [0.55 0.75 0.95], 'LineWidth', 5);
h_sim = plot(ax, sim_pos(:, 2), sim_pos(:, 1), '-', 'Color', [0.85 0.33 0.1], 'LineWidth', 1.3);
h_wp = plot(ax, targets(:, 2), targets(:, 1), 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 6);
for k = 1:size(wp, 1)
    text(ax, wp(k, 2) + 0.06, wp(k, 1) + 0.06, sprintf('%d', k), 'FontSize', 9);
end
seen = coverage.seen;
h_seen = plot(ax, spots(seen, 2), spots(seen, 1), 'p', 'MarkerFaceColor', [0.1 0.7 0.2], ...
    'MarkerEdgeColor', 'k', 'MarkerSize', 15, 'LineWidth', 1);
h_unseen = plot(ax, spots(~seen, 2), spots(~seen, 1), 'rx', 'MarkerSize', 14, 'LineWidth', 2.5);
h_start = plot(ax, 0, 0, '^', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'MarkerSize', 10);
xlabel(ax, 'East, y (m)'); ylabel(ax, 'North, x (m)');
title(ax, sprintf('Sweep through the course quadrotor model: %d/%d tag spots in camera view', ...
    sum(seen), n_s));
handles = [h_ref, h_sim, h_wp, h_start]; labels = {'reference', 'simulated', 'waypoints', 'start'};
if any(seen), handles(end + 1) = h_seen(1); labels{end + 1} = 'tag spot seen'; end
if any(~seen), handles(end + 1) = h_unseen(1); labels{end + 1} = 'tag spot NOT seen'; end
legend(ax, handles, labels, 'Location', 'southoutside', 'NumColumns', 3, 'Box', 'off');
hide_toolbars(f1);
exportgraphics(f1, fullfile(results_dir, 'sweep_topview.png'), 'Resolution', 130);

%% Figure 2: time histories
f2 = figure('Color', 'w', 'Position', [100 100 1000 760]);
tl = tiledlayout(f2, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(tl); hold on; grid on;
plot(sim_t, ref_speed, '--', 'Color', [0.0 0.35 0.75]); plot(sim_t, sim_speed, 'Color', [0.85 0.33 0.1]);
xlabel('Time (s)'); ylabel('Horizontal speed (m/s)'); title('Speed'); legend('reference', 'simulated');
nexttile(tl); hold on; grid on;
plot(sim_t, err_h, 'Color', [0.85 0.33 0.1]); yline(arrival_radius, 'k:', 'arrival radius');
xlabel('Time (s)'); ylabel('Horizontal tracking error (m)'); title('Tracking error');
nexttile(tl); hold on; grid on;
plot(t_ref, -ref_pos(:, 3), '--', 'Color', [0.0 0.35 0.75]); plot(sim_t, -sim_pos(:, 3), 'Color', [0.85 0.33 0.1]);
xlabel('Time (s)'); ylabel('Altitude (m)'); title('Altitude');
nexttile(tl); hold on; grid on;
plot(t_ref, rad2deg(ref_yaw), '--', 'Color', [0.0 0.35 0.75]); plot(sim_t, rad2deg(sim_yaw), 'Color', [0.85 0.33 0.1]);
xlabel('Time (s)'); ylabel('Yaw (deg)'); title('Heading');
hide_toolbars(f2);
exportgraphics(f2, fullfile(results_dir, 'sweep_timehistory.png'), 'Resolution', 120);

%% Console report
fprintf('\n=== Sweep through the course quadrotor model (PosControl_Sim_RPT) ===\n');
fprintf('mission time, takeoff cmd to landing cmd : %.1f s (reference), hold + legs at %.2f m/s\n', ...
    mission_time, plan.cruise_speed_mps);
fprintf('max / rms horizontal tracking error      : %.3f / %.3f m\n', max(err_h), sqrt(mean(err_h.^2)));
fprintf('max simulated speed                      : %.3f m/s\n', max(sim_speed));
fprintf('max tilt (roll or pitch)                 : %.1f deg\n', max(tilt_deg));
fprintf('all waypoints inside the %.2f m radius   : %s (worst lag %.1f s, worst closest miss %.3f m)\n', ...
    arrival_radius, string(summary.all_waypoints_within_arrival_radius), ...
    max(arrive.lag_s), max(arrive.closest_m));
fprintf('tag spots seen by the camera model       : %d of %d\n', sum(coverage.seen), n_s);
disp(coverage);

%% Local functions
function value = get_var(sim_out, name)
    try, value = sim_out.get(name); catch, value = []; end
    if isempty(value) && evalin('base', sprintf("exist('%s','var')", name))
        value = evalin('base', name);
    end
    if isempty(value), error('Simulation output %s not found.', name); end
end

function [time, values] = unpack_signal(logged, ncol)
    if isa(logged, 'timeseries')
        time = logged.Time(:); values = squeeze(logged.Data);
    elseif isstruct(logged) && isfield(logged, 'time')
        time = logged.time(:); values = squeeze(logged.signals.values);
    else
        error('Unsupported logged-signal format.');
    end
    if size(values, 1) ~= numel(time) && size(values, 2) == numel(time), values = values.'; end
    values = values(:, 1:ncol);
end

function hide_toolbars(fig)
    for a = findall(fig, 'Type', 'axes').'
        try, a.Toolbar.Visible = 'off'; catch, end
    end
end

function t = first_time(time, flag)
    ix = find(flag, 1, 'first');
    if isempty(ix), t = NaN; else, t = time(ix); end
end
