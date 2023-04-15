import CSV;
import Dierckx;

function read_human_traj_files(means_filepath::String, vars_filepath::String; n_human_joints::Int=11, offset::AbstractArray=[0., 0., 0.])
    # read in human trajectory means
    means_reader = CSV.File(means_filepath);
    n_timesteps = length(means_reader)
    human_traj = zeros(3,n_human_joints,n_timesteps)
    head_traj = zeros(3, n_timesteps)
    curr_timestep = 1;
    head_symbols = CSV.Symbol.(["headx", "heady", "headz"])
    goal_symbols = CSV.Symbol.(["right_palmx", "right_palmy", "right_palmz"])
    human_goal = zeros(3)
    for row in means_reader
        i = 1;
        while i < n_human_joints
            human_traj[:,i,curr_timestep] .= [row[3*(i-1) + x] + offset[x] for x=1:3];
            i += 1;
        end
        head_traj[:,curr_timestep] = [row[s] for s in head_symbols];
        human_goal = [row[s] for s in goal_symbols];

        curr_timestep += 1;
    end
    human_traj = SArray{Tuple{3,n_human_joints,n_timesteps}}(human_traj);
    head_traj = SArray{Tuple{3,n_timesteps}}(head_traj);
    human_goal = SVector{3}(human_goal)

    # read in human trajectory vars
    vars_reader = CSV.File(vars_filepath, header=false);
    human_vars_traj = zeros(3,3,n_human_joints,n_timesteps);
    curr_timestep = 1;
    for row in vars_reader
        for curr_joint = 1:n_human_joints
            for matr_row = 1:3
                idx_start = (curr_joint-1)*9 + (matr_row-1)*3
                human_vars_traj[matr_row,:,curr_joint,curr_timestep] = [row[idx_start+i] for i=1:3];
            end
        end
        curr_timestep += 1
    end

    human_vars_traj = SArray{Tuple{3,3,n_human_joints,n_timesteps}}(human_vars_traj);

    return human_traj, head_traj, human_vars_traj, human_goal
end

function get_nominal_traj(start::AbstractVector, goal::AbstractVector, n_timesteps::Int)
    @assert length(start) == length(goal);
    ret_val = zeros(length(start), n_timesteps);
    for i = 1:length(start)
        ret_val[i,:] = collect(range(start[i], goal[i], length=n_timesteps));
    end
    SMatrix{length(start), n_timesteps}(ret_val)
end

function resample_1d_traj(traj::AbstractVector, new_n_timesteps::Int)
    spl = Dierckx.Spline1D(collect(0:1/(length(traj)-1:1)), traj);
    return map(x -> Dierckx.evaluate(spl, x), collect(0:1/(new_n_timesteps-1):1));
end

function resample_human_traj(traj::AbstractArray, new_n_timesteps::Int)
    @assert new_n_timesteps > 2
    traj_dims = collect(size(traj))
    if length(traj_dims) == 1
        return resample_1d_traj(traj, new_n_timesteps)
    end
    old_n_timesteps = traj_dims[end]
    intermediate_dims = traj_dims[1:end-1]
    spline_arr = Array{Dierckx.Spline1D, length(intermediate_dims)}(undef, intermediate_dims...);
    
    for i = 1:length(spline_arr)
        y = Float64[];
        for t = i:length(spline_arr):length(traj)
            append!(y, traj[t])
        end
        spline_arr[i] = Dierckx.Spline1D(collect(0:1/(old_n_timesteps-1):1), y)
    end

    ret_val = Array{Float64, length(traj_dims)}(undef, intermediate_dims..., new_n_timesteps)
    for i = 1:length(spline_arr)
        t_scaled = 0.
        for t = i:length(spline_arr):length(ret_val)
            ret_val[t] = Dierckx.evaluate(spline_arr[i], t_scaled);
            t_scaled += 1/(new_n_timesteps - 1);
        end
    end

    return SArray{Tuple{intermediate_dims...,new_n_timesteps}}(ret_val);
end