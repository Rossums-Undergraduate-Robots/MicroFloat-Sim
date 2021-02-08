%% Setup
temp_coeff = @(x) (-5.734e-06.*x.^2 + 3.167e-05.*x + 0.9999)./0.997102576811705; %normalized at 25C
pressure_coeff = @(x) (0.002944949571663.*x + 1.023429331929961e+03)./(1.023472610614371e+03); %normalized at 14.696 psi
salinity_coeff = @(x) (7.530909090909034.*x + 9.971418181818185e+02)./(9.971418181818185e+02); %normalized at 0% salinity
water_density = @(P, T, S) water_kgm3_base*temp_coeff(T)*pressure_coeff(P)*salinity_coeff(S); %approximate formula
depth_2_pressure = @(d) (1023.6*9.80665*d); %approximate pressure

% conversion factors
in_2_mm = 25.4;
in_2_m = 0.0254;
psi_2_Pa = 6894.76;
Pa_2_psi = 1/psi_2_Pa;
lbf_2_N = 4.44822;

% physical properties
water_kgm3_base = 997.17; %at 25 C, 1 atm, 0% salinity in kg/m^3 
atm_psi = 14.6959; %atmospheric temp in psi

%% USER DEFINED VARIABLES
% ocean properties
salt_level_percent = 3.5; %in percent
temp_C = 5; %in celcius
depth = 1000; %in meters

% vehicle properties
D_in = 4;
L_in = 36;

% ballast properties
ballast_D_in = 3;
ballast_L_in = 4;
ballast_h = -1; %[-1, 1]

% actuator properties
actuator_power_W = 50;
internal_air_volume_in3 = 0.5*pi*(D_in^2/4)*L_in; %at ballast half open

%% Calculated Values
% ocean water density
pressure_psi = @(h) depth_2_pressure(h).*Pa_2_psi + atm_psi; %in psi
water_kgm3 = water_density(pressure_psi(depth), temp_C, salt_level_percent);
fprintf('pressure at %dm depth: %d psi \n', depth, pressure_psi(depth));

%vehicle calculated properties
V_in3 = pi*(D_in^2/4)*L_in;
ballast_L_m = ballast_L_in * in_2_m;

% ballast properties
ballast_A_in2 = pi*ballast_D_in^2/4;
ballast_V_in3 = ballast_A_in2*ballast_L_in;

mass_of_float_kg = water_density(depth_2_pressure(500)*Pa_2_psi, 5, 3.5)*(V_in3)*in_2_m^3;

internal_pressure_psi = @(db_V) internal_air_volume_in3.*atm_psi./(internal_air_volume_in3+db_V);
force_required_lbf = @(h, b_A, db_V) b_A.*(pressure_psi(h)-internal_pressure_psi(db_V)); %force on actuator
force_required_N = @(h, b_A, db_V) force_required_lbf(h, b_A, db_V).*lbf_2_N; %force on actuator
energy_required_J = @(V1, V2, d, b_A) trapz(linspace(V1, V2, 100).*in_2_m.^3, (pressure_psi(d)-internal_pressure_psi(linspace(V1, V2, 100).*in_2_m.^3-V1.*in_2_m.^3)).*psi_2_Pa);
seconds_2_move = @(V1, V2, d, b_A) abs(energy_required_J(V1, V2, d, b_A)./actuator_power_W); %in seconds
x_2_V = @(x) x.*ballast_V_in3./2;
boyancy_force_N = @(b_V, d) 9.80665.*(water_density(pressure_psi(d), temp_C, salt_level_percent).*((V_in3+b_V).*in_2_m^3)-mass_of_float_kg);
accel_ms2 = @(b_V, d) boyancy_force_N(b_V, d)./mass_of_float_kg;
drag_approx = @(V, d) 1.5.*water_density(pressure_psi(d), temp_C, salt_level_percent).*pi.*(D_in^2/4).*in_2_m.^2.*V.^2./2;
steady_state_vel = @(b_V, d) sqrt(abs(8.*mass_of_float_kg.*accel_ms2(b_V, d)./(1.5.*water_density(pressure_psi(d), temp_C, salt_level_percent).*pi.*(D_in.*in_2_m).^2)));

%% Print results for user input
fprintf('internal_pressure: %d psi \n', internal_pressure_psi(x_2_V(ballast_h)));
fprintf('mass of float: %d kg \n', mass_of_float_kg);
fprintf('force on actuator: %d lbf \n', force_required_lbf(depth, ballast_A_in2, x_2_V(ballast_h)));
fprintf('min time for half actuator extension: %d s \n', seconds_2_move(0,x_2_V(1),depth, ballast_A_in2));
fprintf('float acceleration: %d m/s^2 \n', accel_ms2(x_2_V(ballast_h), depth));
fprintf("steady state velocity ~= %d m/s\n", steady_state_vel(x_2_V(ballast_h), depth));

%% Plot steady state velocity vs time
b_V = 0:60;
v_ss = steady_state_vel(b_V/2, 1000);
figure(1);
plot(b_V, v_ss);
xlabel('Ballast Displacement Volume [in^3]');
ylabel('Steady State Velocity [m/s]');
title('Steady State Velocity Analysis (4"x36" 1000m)');

%% Plot steady state energy vs volume
b_V = 0:60;
E = zeros(length(b_V), 1);
for V = b_V
    E(V+1) = energy_required_J(0, V./2, 1000, V./4);
end
figure(2);
plot(b_V, E);
xlabel('Ballast Displacement Volume [in^3]');
ylabel('Energy Needed [J]');
title('Energy requirement for half extension [1000m]');

%% Float Sim
time_step = 0.01;
y = zeros(1, 8000);
time = zeros(1, length(y));
v = zeros(1, length(y));
a = zeros(1, length(y));
start_x = -0.05;
end_x = -1;
b_A = ballast_A_in2;

time_to_actuate = seconds_2_move(x_2_V(start_x), x_2_V(end_x), depth, b_A);
if (time_to_actuate <= length(y)*time_step)
    b_x = ones(1, length(y)).*end_x;
    b_x(1:floor(time_to_actuate/time_step)+1) = start_x + (0:time_step:time_to_actuate).*(end_x-start_x)./time_to_actuate;
else
    b_x = linspace(start_x, end_x*length(y)*time_step/time_to_actuate, length(y));
end

for t = 2:length(y)
    boyancy_accel = accel_ms2(x_2_V(b_x(t)), depth);
    a(t-1) = boyancy_accel - sign(boyancy_accel)*drag_approx(v(t-1), depth)/mass_of_float_kg;
    time(t) = time(t-1)+time_step;
    v(t) = v(t-1) + a(t-1)*time_step;
    y(t) = y(t-1) + v(t-1)*time_step + 0.5*a(t-1)*time_step^2;
end

figure(3);

subplot(1, 3, 1);
plot(time, a);
xlim([0,80]);
xlabel('Time (s)');
ylabel('Acceleration [m/s^2]');

subplot(1, 3, 2);
plot(time, v);
xlim([0,80]);
xlabel('Time (s)');
ylabel('Velocity [m/s]');
title('Vehicle Dynamics [1000m, 3"x4" ballast, 4"x36" frame]');

subplot(1, 3, 3);
plot(time, y);
xlim([0,80]);
xlabel('Time (s)');
ylabel('Position [m]');