#The following piece of code is adapted from Electric Machinery Fundamentals
#by Stephen Chapman
# Author: Seymur Kafkas 040170911

module demo

struct ACSupply
    v_phase::Float64
    f::Float64
end


struct VFControllerSettings
    v_rated::Float64
    f_base::Float64
end

struct InductionMotorParams
    rs::Float64 # Resistance Input
    p_pair::Int # Pole Pairs
    f_base::Float64    # Frequency Base for reactance values ( assume a linear relationship (X= jwl))
    r1::Float64 # Stator Resistance
    x1_base::Float64 # Stator Base Reactance
    r2::Float64 # Rotor Referred Resistance (at standstill)
    x2_base::Float64 # Rotor Base Referred Reactance
    rfe::Float64 #Core loss Resistance
    xm_base::Float64 # Magnetization Branch Base Reactance
    n_n::Float64 # Nominal Speed (RPM)
end

mutable struct InductionMotorWithSupply
    rs::Float64 # Resistance Input
    p_pair::Int # Pole Pairs
    f_base::Float64    # Frequency Base for reactance values ( assume a linear relationship (X= jwl))
    r1::Float64 # Stator Resistance
    x1_base::Float64 # Stator Base Reactance
    x1::Float64 # Stator Reactance
    r2::Float64 # Rotor Referred Resistance (at standstill)
    x2_base::Float64 # Rotor Base Referred Reactance
    x2::Float64 #Rotor Referred  Reactance
    rfe::Float64 #Core loss Resistance
    xm_base::Float64 # Magnetization Branch Base Reactance
    xm::Float64 #  Magnetization Branch Reactance
    n_n::Float64 # Nominal Speed (RPM)
    n_sync::Float64 #Synchronous Speed
    v_phase::Float64 # Phase Voltage    
    f::Float64 #Frequency
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


function update_motor!(motor::InductionMotorWithSupply, new_f::Float64, new_v_phase::Float64)

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
    r_th = real(z_th);
    x_th = imag(z_th);
    
    function torque_for_slip(s::Float64)
        pAirGap = (3 * v_th^2 * (motor.r2 + motor.rs) / s) / (((r_th + (motor.r2 + motor.rs) / s)^2 + (x_th + motor.x2)^2))
        t_ind = pAirGap / w_sync
        return t_ind
    end

    return torque_for_slip
end

function calculate_pullout_torque(motor::InductionMotorWithSupply)
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


rs = 0; # Resistance Input
p_pair = 1 # Pole Pairs
f = 50 #Input Frequency in Hz
f_base = 50    # Frequency Base for reactance values ( assume a linear relationship (X= jwl))
r1 = 0.4; # Stator Resistance
x1_base = 1.5; # Stator Base Reactance
r2 = 0.5; # Rotor Referred (wrt Stator) Resistance
x2_base = 2; # Rotor Base Referred (wrt Stator) Reactance
rfe = 400; #Core loss Resistance
xm_base = 250; # Magnetization Branch Base Reactance
v_phase = 440; # Phase to Neutral Voltage
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

#default_supply_values = ACSupply(440 / sqrt(3) , 50)
#motor_with_supply = make_motor_with_supply(default_induction_motor_params,default_supply_values)
#torque_slip_function = make_torque_for_slip_function(motor_with_supply)
#torque_speed_function = make_torque_for_speed_function(motor_with_supply)

using Plots

function make_one_dimensional_range(lower , upper , delta)

    length =  Int(floor((upper - lower)  / delta))
    result = collect(range(lower, upper, length=length))
    return result
end

function plot_function(x_lower, x_upper, x_delta, func, append:: Bool)
    domain_values = make_one_dimensional_range(x_lower,x_upper, x_delta)
    range_values = map(func, domain_values)

    if append
        return plot!(domain_values,range_values)
    else
        return plot(domain_values,range_values)
    end
end

#x = 1:10; y = rand(10); # These are the plotting data
#x= make_one_dimensional_range(15,25,2)
#y = rand(length(x))

#display(plot(x, y))


s_lower = 0 
s_upper = 1
s_delta = 0.01

#torque_slip_plot = plot_function(s_lower,s_upper,s_delta,torque_slip_function)

#torque_speed_plot = plot_function(n_lower, n_upper , n_delta,torque_speed_function)
#display(torque_speed_plot)
#num = readline()

function calculate_v_for_f_for_controller( controller_settings::VFControllerSettings, frequency::Float64)
    if frequency <= controller_settings.f_base
        return frequency * controller_settings.v_rated / controller_settings.f_base
    else
        return controller_settings.v_rated
    end
end


function set_motor_inputs_with_controller!(motor::InductionMotorWithSupply, controller_settings:: VFControllerSettings , frequency_new::Float64)
    v_phase_new = calculate_v_for_f_for_controller(controller_settings,frequency_new)
    update_motor!(motor, frequency_new,v_phase_new)
end


function part_1()
    delta_n = 10
    delta_f = 10

    f_lower = 10
    f_higher = 100

    n_lower = 0
    n_upper = 9000

    controller = VFControllerSettings(440, 50)
    default_supply_values = ACSupply(440 , 50)
    motor = make_motor_with_supply(default_induction_motor_params,default_supply_values)

    frequency_values = range(f_lower, f_higher , step = delta_f)


    for f in frequency_values
        println(f)
        set_motor_inputs_with_controller!(motor,controller,convert(Float64,f))
        torque_speed_function = make_torque_for_speed_function(motor)
        current_cumulative_plot = plot_function(n_lower, n_upper,delta_n,torque_speed_function,true)
        display(current_cumulative_plot)
        #num = readline()
    end

        num = readline()
end




function test()
    f_lower = 10
    f_higher = 100
    delta_f = 1

    frequency_values = range(f_lower, f_higher , step = delta_f)

    controller = VFControllerSettings(440, 50)

    function bla(controller)
        function ann(f)
            return calculate_v_for_f_for_controller(controller,convert(Float64,f))
        end
        
        return ann
    end


    out = map(bla(controller), frequency_values)
    curve = plot(frequency_values,out)
    display(curve)
    num = readline()
end



part_1()
#test()

function part_2()
end



end