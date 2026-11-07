function [s, v, a, terminal_time] = SolveMinTimeOPtimalControlProblem(path_length)
global params
threshold_s = (params.vehicle.v_max^2) / (2 * params.vehicle.a_max);
if (path_length > 2 * threshold_s)
    s_cruise = path_length - 2 * threshold_s;
    time_cruise = s_cruise / params.vehicle.v_max;
    time_slope = params.vehicle.v_max / params.vehicle.a_max;
    terminal_time = 2 * time_slope + time_cruise;
    Nfe = round(terminal_time / 0.001);
    time_line = linspace(0, terminal_time, Nfe);
    time_vec1 = time_line(find(time_line <= time_slope));
    time_vec3 = time_line(find(time_line > time_slope + time_cruise));
    time_vec1plus2 = time_line(find(time_line <= time_slope + time_cruise));
    time_vec2 = time_vec1plus2(find(time_vec1plus2 > time_slope));

    a = [ones(1,size(time_vec1,2)).* params.vehicle.a_max, zeros(1,size(time_vec2,2)), ones(1,size(time_vec3,2)).* -params.vehicle.a_max];

    v_part1 = time_vec1 .* params.vehicle.a_max;
    v_part2 = ones(1,size(time_vec2,2)) .* params.vehicle.v_max;
    v_part3 = params.vehicle.v_max + (time_vec3 - time_slope - time_cruise).* -params.vehicle.a_max;
    v = [v_part1, v_part2, v_part3];

    s_part1 = 0.5 * params.vehicle.a_max * (time_vec1.^2);
    s_part2 = 0.5 * params.vehicle.a_max * (time_slope^2) + params.vehicle.v_max * (time_vec2 - time_slope);
    s_part3 = 0.5 * params.vehicle.a_max * (time_slope^2) + params.vehicle.v_max * time_cruise + params.vehicle.v_max * (time_vec3 - time_slope - time_cruise) + 0.5 * -params.vehicle.a_max * ((time_vec3 - time_slope - time_cruise).^2);
    s = [s_part1, s_part2, s_part3];
else
    half_terminal_time = sqrt(path_length / params.vehicle.a_max);
    half_Nfe = round(half_terminal_time / 0.001);
    time_vec1 = linspace(0, half_terminal_time, half_Nfe);
    v_part1 = time_vec1 .* params.vehicle.a_max;
    v_part2 = v_part1(end) + time_vec1 .* -params.vehicle.a_max;
    a = [ones(1,half_Nfe).* params.vehicle.a_max, ones(1,half_Nfe).* -params.vehicle.a_max];
    s_part1 = 0.5 * params.vehicle.a_max * (time_vec1.^2);
    s_part2 = s_part1(end) + v_part1(end) .* time_vec1 + 0.5 * (-params.vehicle.a_max) * (time_vec1.^2);

    s = [s_part1, s_part2];
    v = [v_part1, v_part2];
    terminal_time = 2 * half_terminal_time;
end
end