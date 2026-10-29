function ref = sweep_reference(plan, opt)
%SWEEP_REFERENCE Reference trajectory for the tag-search mission, stop-and-go.
%   Mirrors what the mission asks of the drone: take off to the cruise altitude, wait
%   takeoff_wait_s, then for every waypoint turn in place to face it and fly a straight
%   leg at cruise speed (trapezoidal speed profile), return to the start, descend.
%   NED frame, origin at the launch point, z down. Output fields are the course
%   model's reference: time and 11 columns [pos vel acc yaw yawrate].
%
%   opt fields (all assumptions, none measured on the drone):
%     tstep   sample time (s), course model uses 0.01
%     a_xy    horizontal acceleration limit (m/s^2)
%     v_z     climb/descent speed (m/s), a_z vertical acceleration (m/s^2)
%     w_yaw   yaw-rate limit (rad/s), a_yaw yaw acceleration (rad/s^2)
%     t_pad   hold at the start and at the end (s)

    arguments
        plan struct
        opt struct
    end
    dt = opt.tstep;
    alt = plan.altitude_m;
    vmax = plan.cruise_speed_mps;

    t_all = []; p_all = []; v_all = []; a_all = []; yaw_all = []; yr_all = [];
    t0 = 0;
    pos = [0 0 0];
    yaw = 0;

    append(hold_segment(pos, yaw, opt.t_pad, dt));

    % Take off.
    append(line_segment(pos, [0 0 -alt], yaw, opt.v_z, opt.a_z, dt));
    wait = max(plan.takeoff_wait_s - t0, 0);
    if wait > 0, append(hold_segment(pos, yaw, wait, dt)); end

    % Sweep, then return to the start.
    targets = [plan.waypoints; plan.start(:).'];
    for k = 1:size(targets, 1)
        goal = [targets(k, 1), targets(k, 2), -alt];
        d = goal(1:2) - pos(1:2);
        if norm(d) > 1e-6
            heading = atan2(d(2), d(1));
            dpsi = atan2(sin(heading - yaw), cos(heading - yaw));   % shortest turn
            if abs(dpsi) > 1e-3
                append(turn_segment(pos, yaw, yaw + dpsi, opt.w_yaw, opt.a_yaw, dt));
            end
            append(line_segment(pos, goal, yaw, vmax, opt.a_xy, dt));
        end
    end

    % Land.
    append(hold_segment(pos, yaw, opt.t_pad, dt));
    append(line_segment(pos, [pos(1) pos(2) 0], yaw, opt.v_z, opt.a_z, dt));
    append(hold_segment(pos, yaw, opt.t_pad, dt));

    ref.time = t_all;
    ref.signals.values = [p_all, v_all, a_all, yaw_all, yr_all];
    ref.signals.dimensions = 11;

    function append(seg)
        % seg: struct with t (from 0), pos, vel, acc, yaw, yawrate over one phase
        t = seg.t(:);
        if ~isempty(t_all), keep = 2:numel(t); else, keep = 1:numel(t); end
        t_all = [t_all; t0 + t(keep)]; %#ok<AGROW>
        p_all = [p_all; seg.pos(keep, :)]; %#ok<AGROW>
        v_all = [v_all; seg.vel(keep, :)]; %#ok<AGROW>
        a_all = [a_all; seg.acc(keep, :)]; %#ok<AGROW>
        yaw_all = [yaw_all; seg.yaw(keep)]; %#ok<AGROW>
        yr_all = [yr_all; seg.yawrate(keep)]; %#ok<AGROW>
        t0 = t0 + t(end);
        pos = seg.pos(end, :);
        yaw = seg.yaw(end);
    end
end

function seg = hold_segment(pos, yaw, T, dt)
    t = (0:dt:T).';
    if t(end) < T, t(end + 1) = T; end
    n = numel(t);
    seg = struct('t', t, 'pos', repmat(pos, n, 1), 'vel', zeros(n, 3), ...
        'acc', zeros(n, 3), 'yaw', yaw * ones(n, 1), 'yawrate', zeros(n, 1));
end

function seg = line_segment(p0, p1, yaw, vmax, amax, dt)
    L = norm(p1 - p0);
    u = (p1 - p0) / max(L, eps);
    [t, s, v, a] = trap_profile(L, vmax, amax, dt);
    seg = struct('t', t, 'pos', p0 + s * u, 'vel', v * u, 'acc', a * u, ...
        'yaw', yaw * ones(numel(t), 1), 'yawrate', zeros(numel(t), 1));
end

function seg = turn_segment(pos, yaw0, yaw1, wmax, amax, dt)
    dpsi = yaw1 - yaw0;
    [t, s, w, ~] = trap_profile(abs(dpsi), wmax, amax, dt);
    n = numel(t);
    seg = struct('t', t, 'pos', repmat(pos, n, 1), 'vel', zeros(n, 3), ...
        'acc', zeros(n, 3), 'yaw', yaw0 + sign(dpsi) * s, 'yawrate', sign(dpsi) * w);
end

function [t, s, v, a] = trap_profile(L, vmax, amax, dt)
%TRAP_PROFILE Distance, speed and acceleration of a rest-to-rest trapezoidal move.
    if L <= 0
        t = 0; s = 0; v = 0; a = 0;
        return
    end
    if L >= vmax^2 / amax
        vpk = vmax;
        ta = vpk / amax;
        tc = (L - vpk^2 / amax) / vpk;
    else
        vpk = sqrt(L * amax);
        ta = vpk / amax;
        tc = 0;
    end
    T = 2 * ta + tc;
    t = (0:dt:T).';
    if t(end) < T, t(end + 1) = T; end
    s = zeros(size(t)); v = zeros(size(t)); a = zeros(size(t));
    for i = 1:numel(t)
        ti = t(i);
        if ti < ta
            a(i) = amax; v(i) = amax * ti; s(i) = 0.5 * amax * ti^2;
        elseif ti < ta + tc
            a(i) = 0; v(i) = vpk; s(i) = 0.5 * vpk * ta + vpk * (ti - ta);
        else
            td = ti - ta - tc;
            a(i) = -amax; v(i) = max(vpk - amax * td, 0);
            s(i) = 0.5 * vpk * ta + vpk * tc + vpk * td - 0.5 * amax * td^2;
        end
    end
    s(end) = L; v(end) = 0; a(end) = 0;
end
