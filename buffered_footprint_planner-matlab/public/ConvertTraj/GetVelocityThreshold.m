function threshold = GetVelocityThreshold(v)
global params

v = abs(v);  % 支持倒车：用速度幅值
v = min(v, params.vehicle.v_max);

if (v < 1e-6)
    threshold = -1;   % 或者 0，看你希望停住时是否触发切分
else
    threshold = params.ef.max_dt * params.nlp.threshold_rate;
end
end