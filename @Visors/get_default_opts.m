function opts = get_default_opts(obj)
opts.calc_q_det_flag = 0; % if 1, introduces fictitious measurements 
opts.corrupt_measurements = 1; % if 1, corrupts measurements from truth
opts.est_q = @obj.calc_q_stat; % function handle for which attitude determination function to call
end