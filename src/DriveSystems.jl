#The following piece of code is adapted from Electric Machinery Fundamentals
#by Stephen Chapman
# Author: Seymur Kafkas 040170911

module demo

struct ACSupply
    v_phase::Float64
    f::Float64
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
p_pair = 2 # Pole Pairs
f = 50 #Input Frequency in Hz
f_base = 50    # Frequency Base for reactance values ( assume a linear relationship (X= jwl))
r1 = 0.075; # Stator Resistance
x1_base = 0.17; # Stator Base Reactance
r2 = 0.065; # Rotor Referred (wrt Stator) Resistance
x2_base = 0.17; # Rotor Base Referred (wrt Stator) Reactance
rfe = 150; #Core loss Resistance
xm_base = 7.2; # Magnetization Branch Base Reactance
v_phase = 440 / sqrt(3); # Phase Voltage
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

default_supply_values = ACSupply(440 / sqrt(3) , 50)
motor_with_supply = make_motor_with_supply(default_induction_motor_params,default_supply_values)
torque_slip_function = make_torque_for_slip_function(motor_with_supply)

using Plots

function make_one_dimensional_range(lower , upper , delta)

    length =  Int(floor((upper - lower)  / delta))
    result = collect(range(lower, upper, length=length))
    return result
end

function plot_function(x_lower, x_upper, x_delta, func)
    domain_values = make_one_dimensional_range(x_lower,x_upper, x_delta)
    range_values = map(func, domain_values)
    return plot(domain_values,range_values)
end

#x = 1:10; y = rand(10); # These are the plotting data
#x= make_one_dimensional_range(15,25,2)
#y = rand(length(x))

#display(plot(x, y))


s_lower = 0 
s_upper = 1
s_delta = 0.01


torque_speed_plot = plot_function(s_lower,s_upper,s_delta,torque_slip_function)
display(torque_speed_plot)
num = readline()


function part_1()
    v_per_f_factor = 12
    delta_n = 10
    delta_f = 10
    f_lower = 10
    f_higher = 100
end



function part_2()
end

#  = (1 - nm / n_sync)    Mechanical speed in RPM
end