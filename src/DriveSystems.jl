# Author: Seymur Kafkas 040170911

module DriveSystems

struct ACSupply
    v_phase::Number
    f::Number
end

struct VFControllerSettings
    v_rated::Number
    f_base::Number
end

struct InductionMotorParams
    rs::Number # Resistance Input
    p_pair::Integer # Pole Pairs
    f_base::Number    # Frequency Base for reactance values ( assume a linear relationship (X= jwl))
    r1::Number # Stator Resistance
    x1_base::Number # Stator Base Reactance
    r2::Number # Rotor Referred Resistance (at standstill)
    x2_base::Number # Rotor Base Referred Reactance
    rfe::Number #Core loss Resistance
    xm_base::Number # Magnetization Branch Base Reactance
    n_n::Number # Nominal Speed (RPM)
end

mutable struct InductionMotorWithSupply
    rs::Number # Resistance Input
    p_pair::Integer # Pole Pairs
    f_base::Number    # Frequency Base for reactance values ( assume a linear relationship (X= jwl))
    r1::Number # Stator Resistance
    x1_base::Number # Stator Base Reactance
    x1::Number # Stator Reactance
    r2::Number # Rotor Referred Resistance (at standstill)
    x2_base::Number # Rotor Base Referred Reactance
    x2::Number #Rotor Referred  Reactance
    rfe::Number #Core loss Resistance
    xm_base::Number # Magnetization Branch Base Reactance
    xm::Number #  Magnetization Branch Reactance
    n_n::Number # Nominal Speed (RPM)
    n_sync::Number #Synchronous Speed
    v_phase::Number # Phase Voltage    
    f::Number #Frequency
end

function make_motor_with_supply(params::InductionMotorParams, supply::ACSupply)

    v_phase = supply.v_phase
    f = supply.f

    n_sync = 60 * f / params.p_pair # Synchronous Speed (RPM)
    reactance_scaling_factor = f / params.f_base
    x1 = params.x1_base * reactance_scaling_factor
    x2 = params.x2_base * reactance_scaling_factor
    xm = params.xm_base * reactance_scaling_factor

    motor = InductionMotorWithSupply(
        params.rs,
        params.p_pair,
        params.f_base,
        params.r1,
        params.x1_base,
        x1,
        params.r2,
        params.x2_base,
        x2,
        params.rfe,
        params.xm_base,
        xm,
        n_n,
        n_sync,
        v_phase,
        f
    )

    return motor
end


function update_motor!(motor::InductionMotorWithSupply, new_f::Number, new_v_phase::Number)

    motor.n_sync = 60 * new_f / motor.p_pair
    reactance_scaling_factor = new_f / motor.f_base
    motor.x1 = motor.x1_base * reactance_scaling_factor
    motor.x2 = motor.x2_base * reactance_scaling_factor
    motor.xm = motor.xm_base * reactance_scaling_factor
    motor.v_phase = new_v_phase
    motor.f = new_f
end


function make_torque_for_slip_function(motor::InductionMotorWithSupply)
    z_eq = (motor.rfe * im * motor.xm) / (motor.rfe + im * motor.xm) #Magnetising Branch and Core Loss Branch 
    v_th = motor.v_phase * abs((z_eq) / (z_eq + im * motor.x1 + motor.r1)) # Thevenin Z and V
    z_th = (z_eq * (im * motor.x1 + motor.r1)) / (z_eq + (im * motor.x1 + motor.r1))
    w_sync = 2 * motor.n_sync * pi / 60
    r_th = real(z_th)
    x_th = imag(z_th)

    function torque_for_slip(s::Number)
        pAirGap = (3 * v_th^2 * (motor.r2 + motor.rs) / s) / (((r_th + (motor.r2 + motor.rs) / s)^2 + (x_th + motor.x2)^2))
        t_ind = pAirGap / w_sync
        return t_ind
    end

    return torque_for_slip
end

function calculate_pullout_torque(motor::InductionMotorWithSupply)
    z_eq = (motor.rfe * im * motor.xm) / (motor.rfe + im * motor.xm) #Magnetising Branch and Core Loss Branch 
    v_th = motor.v_phase * abs((z_eq) / (z_eq + im * motor.x1 + motor.r1)) # Thevenin Z and V
    z_th = (z_eq * (im * motor.x1 + motor.r1)) / (z_eq + (im * motor.x1 + motor.r1))
    w_sync = 2 * motor.n_sync * pi / 60
    r_th = real(z_th)
    x_th = imag(z_th)
    pullout_torque = 3 * v_th^2 / (2 * w_sync * (r_th + sqrt(r_th^2 + (x_th + motor.x2)^2)))
    return pullout_torque
end


function calculate_rated_torque(motor::InductionMotorWithSupply)
    torque_speed_function = make_torque_for_speed_function(motor)
    nominal_speed = motor.n_n
    rated_torque = torque_speed_function(nominal_speed)
    return rated_torque
end


function make_torque_for_speed_function(motor::InductionMotorWithSupply)
    torque_for_slip_function = make_torque_for_slip_function(motor)

    function torque_for_speed(n_m)
        s = (1 - n_m / motor.n_sync)
        torque = torque_for_slip_function(s)
        return torque
    end

    return torque_for_speed
end


function make_torque_for_speed_function_angular(motor::InductionMotorWithSupply)
    torque_for_slip_function = make_torque_for_slip_function(motor)
    w_sync = 2 * motor.n_sync * pi / 60

    function torque_for_speed(w_m)
        s = (1 - w_m / w_sync)
        torque = torque_for_slip_function(s)
        return torque
    end

    return torque_for_speed
end


rs = 0 # Resistance Input
p_pair = 1 # Pole Pairs
f = 50 #Input Frequency in Hz
f_base = 50    # Frequency Base for reactance values ( assume a linear relationship (X= jwl))
r1 = 0.4 # Stator Resistance
x1_base = 1.5 # Stator Base Reactance
r2 = 0.5 # Rotor Referred (wrt Stator) Resistance
x2_base = 2 # Rotor Base Referred (wrt Stator) Reactance
rfe = 400 #Core loss Resistance
xm_base = 250 # Magnetization Branch Base Reactance
v_phase = 440 # Phase to Neutral Voltage
n_n = 2920 # Nominal Speed (RPM)

default_induction_motor_params = InductionMotorParams(rs, p_pair,
    f_base,
    r1,
    x1_base,
    r2,
    x2_base,
    rfe,
    xm_base,
    n_n)


using Plots

function make_one_dimensional_range(lower, upper, delta)

    length::Integer = floor((upper - lower) / delta)
    result = collect(range(lower, upper, length=length))
    return result
end

function plot_function(x_lower, x_upper, x_delta, func, append::Bool)
    domain_values = make_one_dimensional_range(x_lower, x_upper, x_delta)
    range_values = map(func, domain_values)

    if append
        return plot!(domain_values, range_values)
    else
        return plot(domain_values, range_values)
    end
end


function calculate_v_for_f_for_controller(controller_settings::VFControllerSettings, frequency::Number)
    if frequency <= controller_settings.f_base
        return frequency * controller_settings.v_rated / controller_settings.f_base
    else
        return controller_settings.v_rated
    end
end


function set_motor_inputs_with_controller!(motor::InductionMotorWithSupply, controller_settings::VFControllerSettings, frequency_new::Number)
    v_phase_new = calculate_v_for_f_for_controller(controller_settings, frequency_new)
    update_motor!(motor, frequency_new, v_phase_new)
end

function get_pullout_torque_for_frequency_function(motor, controller)
    function pullout_torque_function(frequency)
        v_new = calculate_v_for_f_for_controller(controller, frequency)
        set_motor_inputs_with_controller!(motor, controller, frequency)
        pullout_torque = calculate_pullout_torque(motor)
        return pullout_torque
    end

    return pullout_torque_function
end

function part_1()
    delta_n = 10
    delta_f = 10

    f_lower = 10
    f_higher = 100

    n_lower = 0
    n_upper = 9000

    controller = VFControllerSettings(440, 50)
    default_supply_values = ACSupply(440, 50)
    motor = make_motor_with_supply(default_induction_motor_params, default_supply_values)

    frequency_values = range(f_lower, f_higher, step=delta_f)

    for f in frequency_values
        println(f)
        set_motor_inputs_with_controller!(motor, controller, f)
        torque_speed_function = make_torque_for_speed_function(motor)
        current_cumulative_plot = plot_function(n_lower, n_upper, delta_n, torque_speed_function, true)
        display(current_cumulative_plot)
        #num = readline()
    end

    num = readline()
end

function get_load_square_torque_speed_function(square_coefficient, constant_coefficient)
    function torque_speed_function(speed)
        if speed >= 0
            return square_coefficient * speed^2 + constant_coefficient
        else
            return -square_coefficient * speed^2 - constant_coefficient
        end
    end

    return torque_speed_function
end

function convert_rpm_function_to_angular(func)
    function angular_func(w_in_rad_per_second)
        n_in_rpm = w_in_rad_per_second * 60 / (2 * pi)
        result = func(n_in_rpm)
        return result
    end
    return angular_func
end

function angular_speed_to_rpm(w_in_rad_per_second)
    n_in_rpm = w_in_rad_per_second * 60 / (2 * pi)
    return n_in_rpm
end

function part_2()
    n_lower = -3000
    n_upper = 6000

    default_supply_values = ACSupply(440, 50)
    motor::InductionMotorWithSupply = make_motor_with_supply(default_induction_motor_params, default_supply_values)
    rated_torque = calculate_rated_torque(motor)

    load_torque_at_zero_speed = 5
    data = [motor.n_n^2 1; 0 1]
    values = [rated_torque; load_torque_at_zero_speed]
    load_parameters = inv(data) * values
    load_torque_function = get_load_square_torque_speed_function(load_parameters[1], load_parameters[2])
    load_torque_curve = plot_function(n_lower, n_upper, 1, load_torque_function, false)
    display(load_torque_curve)
    num = readline()
end


function part_3()
    n_lower = -3000
    n_upper = 6000
    delta_n = 5

    delta_f = 10
    f_lower = 10
    f_higher = 100

    default_supply_values = ACSupply(440, 50)
    controller = VFControllerSettings(440, 50)
    motor::InductionMotorWithSupply = make_motor_with_supply(default_induction_motor_params, default_supply_values)

    rated_torque = calculate_rated_torque(motor)
    load_torque_at_zero_speed = 5
    data = [motor.n_n^2 1; 0 1]
    values = [rated_torque; load_torque_at_zero_speed]
    load_parameters = inv(data) * values
    load_torque_function = get_load_square_torque_speed_function(load_parameters[1], load_parameters[2])
    torque_curve = plot_function(n_lower, n_upper, 1, load_torque_function, true)

    frequency_values = range(f_lower, f_higher, step=delta_f)
    current_cumulative_plot = nothing

    for f in frequency_values
        set_motor_inputs_with_controller!(motor, controller, f)
        torque_speed_function = make_torque_for_speed_function(motor)
        current_cumulative_plot = plot_function(n_lower, n_upper, delta_n, torque_speed_function, true)
    end

    display(current_cumulative_plot)
    num = readline()

end


function part_4()
    f_lower = 10
    f_higher = 100
    delta_f = 1
    frequency_values = range(f_lower, f_higher, step=delta_f)

    controller = VFControllerSettings(440, 50)
    default_supply_values = ACSupply(440, 50)
    motor = make_motor_with_supply(default_induction_motor_params, default_supply_values)

    pullout_torque_for_frequency_function = get_pullout_torque_for_frequency_function(motor, controller)
    pullout_values_for_frequencies = map(pullout_torque_for_frequency_function, frequency_values)
    curve = plot(frequency_values, pullout_values_for_frequencies)
    display(curve)
    num = readline()
end


using DynamicalSystems

function part_5()
    frequency = 50
    v_supply = 440

    controller = VFControllerSettings(440, 50)
    default_supply_values = ACSupply(440, 50)
    motor = make_motor_with_supply(default_induction_motor_params, default_supply_values)

    #Calculate the Load Torque Parameters
    rated_torque = calculate_rated_torque(motor)
    load_torque_at_zero_speed = 5
    data = [motor.n_n^2 1; 0 1]
    values = [rated_torque; load_torque_at_zero_speed]
    load_parameters = inv(data) * values

    load_torque_function_rpm = get_load_square_torque_speed_function(load_parameters[1], load_parameters[2])
    load_torque_function = convert_rpm_function_to_angular(load_torque_function_rpm)
    motor_torque_function = make_torque_for_speed_function_angular(motor)

    j_eq = 1.25

    function motor_load_dynamics(state_vector, parameters, time)
        w = state_vector[1]
        dw = (1 / j_eq) * (motor_torque_function(w) - load_torque_function(w))
        return SVector{2}(dw, 0)
    end

    initial_state = SVector{2}(0.0, 0.0) #Start at standstill (0 velocity), the second state is a dummy variable
    motor_load_system = ContinuousDynamicalSystem(motor_load_dynamics, initial_state, nothing)
    total_simulation_time_in_s = 10
    record_state_delta_t = 0.04 # Time step for recording the state data (Not the time step used in simulation)
    tr = trajectory(motor_load_system, total_simulation_time_in_s, save_idxs=[1], Δt=record_state_delta_t)
    time_array = collect(range(0, total_simulation_time_in_s, length(tr.data)))
    angular_speeds_array = tr.data[:, 1]

    function unroll(x)
        return x[1]
    end

    angular_speeds_array = map(unroll, angular_speeds_array)
    rpm_speeds_array = map(angular_speed_to_rpm, angular_speeds_array)

    res = plot(time_array, rpm_speeds_array)

    display(res)
    input = readline()
end

part_5()


end