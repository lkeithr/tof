% Plot data downloaded from CLI

% Load files
cd 'C:\Users\RUMBAUGHLK\OneDrive - Grove City College\Documents\Research\Time of flight camera\Software\Espros Python\tof\data'
fnA = '20240517-141424_12MHz26200us_amp_data.csv';
fnD = '20240517-141424_12MHz26200us_dist_data.csv';
A = csvread(fnA);
D = csvread(fnD);

% Normalize amplitude data to saturation limits
AdB = 10*log10(A);

% Threshold distance data
Athresh = -10;
D(A<Athresh) = -1;

% Set colorscaling for both plots
Dmin = 150; % cm, based on known scene
Dmax = 250; % cm, based on known scene
Amin = 0;
Amax = max(AdB(:));

% Plot data
figure;
subplot(121);
imagesc(AdB,[Amin, Amax]);
colormap(gca,'turbo');
colorbar;
axis image;
title('Amplitude image (dB)'); 
subplot(122);
imagesc(D,[Dmin,Dmax]);
colorbar;
colormap(gca,'turbo');
axis image;
title('Depth image (cm)');
