"""
End effector constraint struct (used to ensure above table)
n: state dimensionality
n_joints: number of robot joints 
lower_bounds: lower bounds for (x,y,z) eef position
upper_bounds: upper bounds for (x,y,z) eef position
fk: fk function (joint state)->cartesian position
"""
RobotDynamics.@autodiff struct PosEECons <: TO.StateConstraint
    n::Int # state dimension
    n_joints::Int
    lower_bounds::SArray{Tuple{3}, Float64}
    upper_bounds::SArray{Tuple{3}, Float64}
    fk::Function
end

RobotDynamics.state_dim(con::PosEECons) = con.n
TO.sense(::PosEECons) = TO.Inequality()
RobotDynamics.output_dim(con::PosEECons) = 6*con.n_joints

function RobotDynamics.evaluate(cons::PosEECons, x::AbstractVector{T}) where {T}
    xyz = cons.fk(x)
    ans = T[]

    for i = 1:cons.n_joints
        joint_pos = xyz[:,i]
        append!(ans, cons.lower_bounds - joint_pos)
        append!(ans, joint_pos - cons.upper_bounds)
    end

    return SArray{Tuple{6*cons.n_joints}, T}(ans)
end

function RobotDynamics.evaluate(cons::PosEECons, z::RobotDynamics.AbstractKnotPoint)
    return RobotDynamics.evaluate(cons, RobotDynamics.getstate(z, RobotDynamics.get_data(z)));
end

function RobotDynamics.evaluate!(cons::PosEECons, c, x::AbstractVector{T}) where T
    xyz = cons.fk(x)
    for i = 1:cons.n_joints
        xyz_t = xyz[:,i]
        c[6*i - 5:6*i] = [cons.lower_bounds - xyz_t; xyz_t - cons.upper_bounds];
    end
    return nothing;
end

function RobotDynamics.jacobian!(cons::PosEECons, J, y, x::AbstractVector)
    ForwardDiff.jacobian!(J, (w, z) -> RobotDynamics.evaluate!(cons, w, z), y, x)#, cons.cfg);
    return nothing;
end

function RobotDynamics.jacobian!(cons::PosEECons, J, y, z::RobotDynamics.AbstractKnotPoint)
    RobotDynamics.jacobian!(cons, J, y, RobotDynamics.getstate(z, RobotDynamics.get_data(z)));
    return nothing;
end