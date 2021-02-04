% %source: https://www.engineeringtoolbox.com/water-density-specific-weight-d_595.html
% TV_x = [0.100000000000000;1;4;10;15;20;25;30;35];
% TV_y = [0.999849500000000;0.999901700000000;0.999974900000000;0.999700000000000;0.999102600000000;0.998206700000000;0.997047000000000;0.995648800000000;0.994032600000000];
% plot (TV_x, TV_y);
% title("Temp vs. Specific Volume");
% [bl, gof] = fit(TV_x, TV_y, 'poly2');
% hold on;
% sim_x = 0:35;
% plot (sim_x, bl(sim_x));

% %source: https://www.omnicalculator.com/physics/water-density
% PV_x = [14.5; 100; 200; 300; 400; 500; 700; 900; 1100; 1300; 1500];
% PV_y = [1023.47; 1023.72; 1024.02; 1024.31; 1024.61; 1024.9; 1025.5; 1026.08; 1026.67; 1027.26; 1027.84 ];
% plot (PV_x, PV_y);
% title("Pressure vs. Specific Volume");
% [bl, gof] = fit(PV_x, PV_y, 'poly1');
% hold on;
% sim_x = 0:1500;
% ylim([1023, 1028]);
% plot (sim_x, bl(sim_x));

%source: https://www.omnicalculator.com/physics/water-density
saltV_x = [0; 0.5; 1; 1.5; 2; 2.5; 3.0; 3.5; 4; 4.5; 5];
saltV_y = [997.17; 1000.94; 1004.68; 1008.43; 1012.18; 1015.93; 1019.7; 1023.47; 1027.25; 1031.05; 1034.86];
plot (saltV_x, saltV_y);
title("Salinity vs. Specific Volume");
[bl, gof] = fit(saltV_x, saltV_y, 'poly1');
hold on;
sim_x = 0:0.1:5;
plot (sim_x, bl(sim_x));
