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
depth = 750; %in meters

% vehicle properties
D_in = 4;
L_in = 36;

% ballast properties
ballast_D_in = 2;
ballast_L_in = 4;
ballast_h = 0;

% actuator properties
actuator_power_W = 35;
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
internal_pressure_psi = @(ext) internal_air_volume_in3.*atm_psi./(internal_air_volume_in3-(0.5-ext).*ballast_V_in3);
fprintf('internal_pressure: %d psi \n', internal_pressure_psi(ballast_h));

mass_of_float_kg = water_density(depth_2_pressure(500)*Pa_2_psi, 10, 3.5)*(V_in3+ballast_V_in3/2)*in_2_m^3;
fprintf('mass of float: %d kg \n', mass_of_float_kg);
force_required_lbf = @(h, x) ballast_A_in2.*(pressure_psi(h)-internal_pressure_psi(x)); %force on actuator
force_required_N = @(h, x) force_required_lbf(h, x).*lbf_2_N; %force on actuator
fprintf('force on actuator: %d lbf \n', force_required_lbf(depth, ballast_h));

energy_required_J = @(x1, x2, d) trapz(linspace(x1.*ballast_L_m, x2.*ballast_L_m, 100), force_required_N(d, linspace(x1, x2, 100)));
seconds_2_move = @(x1, x2, d) energy_required_J(x1, x2, d)./actuator_power_W; %in seconds

fprintf('min time to move from 0 to 0.5: %d s \n', seconds_2_move(0,0.5,depth));

boyancy_force_N = @(x, d) 9.80665.*(water_kgm3.*((V_in3+x.*ballast_V_in3).*in_2_m^3)-mass_of_float_kg);
accel_ms2 = @(x, d) boyancy_force_N(x, d)/mass_of_float_kg;
fprintf('float acceleration: %d m/s^2 \n', accel_ms2(ballast_h, depth));

%% Drag approximation
drag_approx = @(V) 1.5*water_kgm3*pi*(D_in^2/4)*in_2_m^2*V^2/2;
time_step = 0.01;
y = zeros(1, 1000);
time = zeros(1, length(y));
v = zeros(1, length(y));
a = zeros(1, length(y));
for t = 2:length(y)
    boyancy_accel = accel_ms2(ballast_h, depth);
    a(t-1) = boyancy_accel - sign(boyancy_accel)*drag_approx(v(t-1))/mass_of_float_kg;
    time(t) = time(t-1)+time_step;
    v(t) = v(t-1) + a(t-1)*time_step;
    y(t) = y(t-1) + v(t-1)*time_step + 0.5*a(t-1)*time_step^2;
end

fprintf("max velocity ~= %d\n", v(end));
plot(time, y);