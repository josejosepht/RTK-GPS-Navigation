% Clearing previous data and closing all figures
close all;
clear all;

% Loading rosbag files
bag1 = rosbag("lab2_open_stat.bag");
bag2 = rosbag("lab2_open_moving.bag");
bag3 = rosbag("lab2_obstructed_stat2.bag");
bag4 = rosbag("lab2_obstructed_moving.bag");

% Selecting specific topic from the rosbag files
bsel1 = select(bag1, "Topic", "full_gps");
bsel2 = select(bag2, "Topic", "full_gps");
bsel3 = select(bag3, "Topic", "full_gps");
bsel4 = select(bag4, "Topic", "full_gps");

% Reading messages from selected topics in a structured format
msgStruct1 = readMessages(bsel1, 'DataFormat', 'struct');
msgStruct2 = readMessages(bsel2, 'DataFormat', 'struct');
msgStruct3 = readMessages(bsel3, 'DataFormat', 'struct');
msgStruct4 = readMessages(bsel4, 'DataFormat', 'struct');

% Extracting relevant data fields from the messages
utm_easting_open_stat = cellfun(@(m) double(m.UtmEasting), msgStruct1);
utm_easting_open_mov = cellfun(@(m) double(m.UtmEasting), msgStruct2);
utm_easting_obs_stat = cellfun(@(m) double(m.UtmEasting), msgStruct3);
utm_easting_obs_mov = cellfun(@(m) double(m.UtmEasting), msgStruct4);
utm_northing_open_stat = cellfun(@(m) double(m.UtmNorthing), msgStruct1);
utm_northing_open_mov = cellfun(@(m) double(m.UtmNorthing), msgStruct2);
utm_northing_obs_stat = cellfun(@(m) double(m.UtmNorthing), msgStruct3);
utm_northing_obs_mov = cellfun(@(m) double(m.UtmNorthing), msgStruct4);
alt_open_stat = cellfun(@(m) double(m.Altitude), msgStruct1);
alt_open_mov = cellfun(@(m) double(m.Altitude), msgStruct2);
alt_obs_stat = cellfun(@(m) double(m.Altitude), msgStruct3);
alt_obs_mov = cellfun(@(m) double(m.Altitude), msgStruct4);
gps_quality_open_stat = cellfun(@(m) double(m.GpsQuality), msgStruct1);
gps_quality_open_mov = cellfun(@(m) double(m.GpsQuality), msgStruct2);
gps_quality_obs_stat = cellfun(@(m) double(m.GpsQuality), msgStruct3);
gps_quality_obs_mov = cellfun(@(m) double(m.GpsQuality), msgStruct4);

% Plotting UTM data for stationary points in open and partially occluded areas
subplot(2,1,1);
plot(utm_easting_open_stat-min(utm_easting_open_stat), utm_northing_open_stat-min(utm_northing_open_stat), 'r+');
title("RTK GPS Data Plot for a stationary point's readings in an open area");
xlabel("UTM Easting (metres)");
ylabel("UTM Northing (metres)");

subplot(2,1,2);
plot(utm_easting_obs_stat-min(utm_easting_obs_stat), utm_northing_obs_stat-min(utm_northing_obs_stat), 'r+');
title("RTK GPS Data Plot for a stationary point's readings in a partially occluded area");
xlabel("UTM Easting (metres)");
ylabel("UTM Northing (metres)");

figure
subplot(2,2,1)
plot(utm_easting_open_stat-min(utm_easting_open_stat),'r+')
hold on
p=polyfit([1:length(utm_easting_open_stat)],utm_easting_open_stat-min(utm_easting_open_stat),1);
f_utm_east_stat=polyval(p,[1:length(utm_easting_open_stat)]);
plot(f_utm_east_stat,'b')
legend("UTM Easting for open area stationary point data","Best fit line for UTM Easting for open area stationary point data")
hold off
title("Stationary point UTM Easting Readings in the open space")
xlabel("Time (seconds)")
ylabel("UTM Easting (metres)")
subplot(2,2,2)
plot(utm_easting_obs_stat-min(utm_easting_obs_stat),'r+')
title("Stationary point UTM Easting Readings in the partially occluded space")
xlabel("Time (seconds)")
ylabel("UTM Easting (metres)") 

subplot(2,2,3)
plot(utm_northing_open_stat-min(utm_northing_open_stat),'r+')
hold on
p=polyfit([1:length(utm_northing_open_stat)],utm_northing_open_stat-min(utm_northing_open_stat),1);
f_utm_east_stat=polyval(p,[1:length(utm_northing_open_stat)]);
plot(f_utm_east_stat,'b')
legend("UTM Northing for open area stationary point data","Best fit line for UTM Northing for open area stationary point data")
hold off
title("Stationary point UTM Northing Readings in the open space")
xlabel("Time (seconds)")
ylabel("UTM Easting (metres)")
subplot(2,2,4)
plot(utm_northing_obs_stat-min(utm_northing_obs_stat),'r+')
title("Stationary point UTM Northing Readings in the partially occluded space")
xlabel("Time (seconds)")
ylabel("UTM Easting (metres)")

figure
subplot(2,1,1)
x = utm_easting_open_mov-min(utm_easting_open_mov);
y = utm_northing_open_mov-min(utm_northing_open_mov);
mux = mean(x);
muy = mean(y);
[Theta,R] = cart2pol(x- mux,y-muy);
[Theta,ind] = sort(Theta);
R = R(ind);
n = numel(x);
M = 6;
A = [ones(n,1),sin(Theta*(1:M)),cos(Theta*(1:M))];
coeffs = A\R;
Rhat = A*coeffs;
hold on
[xhat,yhat] = pol2cart(Theta,Rhat);
l1=xhat + mux;
k1=yhat + muy;
plot(x,y,'b+',l1,k1,'.r')
l1=circshift(l1,-327);
k1=circshift(k1,-322);
title("RTK GPS Data Plot for a structured path points' readings in an open area")
xlabel("UTM Easting (metres)")
ylabel("UTM Northing (metres)")
legend("UTM Data for open area structured path points","Best fit discrete curve for Data for open area structured path points")
for i=1:397
    err_utm_easting_open_mov(i)=abs(utm_easting_open_mov(i)-min(utm_easting_open_mov)-l1(i));
    err_utm_northing_open_mov(i)=abs(utm_northing_open_mov(i)-min(utm_northing_open_mov)-k1(i));
    ++i;
end
err_utm_open_mov=((err_utm_easting_open_mov.^(2))+(err_utm_northing_open_mov.^(2))).^(0.5);

subplot(2,1,2)
x = utm_easting_obs_mov-min(utm_easting_obs_mov);
y = utm_northing_obs_mov-min(utm_northing_obs_mov);
mux = mean(x);
muy = mean(y);
[Theta,R] = cart2pol(x- mux,y-muy);
[Theta,ind] = sort(Theta);
R = R(ind);
n = numel(x);
M = 6;
A = [ones(n,1),sin(Theta*(1:M)),cos(Theta*(1:M))];
coeffs = A\R;
Rhat = A*coeffs;
hold on
[xhat,yhat] = pol2cart(Theta,Rhat);
l2=xhat + mux;
k2=yhat + muy;
plot(x,y,'b+',l2,k2,'.r')
l2=circshift(l2,-228);
k2=circshift(k2,-157);
title("RTK GPS Data Plot for a structured path points' readings in a partially occluded area")
xlabel("UTM Easting (metres)")
ylabel("UTM Northing (metres)")
legend("UTM Data for a partially occluded area's structured path points","Best fit discrete curve for a partially occluded area's structured path points")
for i=1:397
    err_utm_easting_obs_mov(i)=abs(utm_easting_obs_mov(i)-min(utm_easting_obs_mov)-l2(i));
    err_utm_northing_obs_mov(i)=abs(utm_northing_obs_mov(i)-min(utm_northing_obs_mov)-k2(i));
    ++i;
end
err_utm_obs_mov=((err_utm_easting_obs_mov.^(2))+(err_utm_northing_obs_mov.^(2))).^(0.5);
figure
subplot(2,2,1)
plot(alt_open_stat,'r+')
hold on
p=polyfit([1:length(alt_open_stat)],alt_open_stat,1);
f=polyval(p,[1:length(alt_open_stat)]);
plot(f,'b')
legend("Altitude data for open area stationary point","Best fit line for Altitude data for open area stationary point")
hold off
title("Altitude Data Plot for a stationary point's readings in an open area")
ylabel("Altitude(metres)")
xlabel("Time (seconds)")
subplot(2,2,2)
plot(alt_obs_stat,'r+')
hold on
p=polyfit([1:length(alt_obs_stat)],alt_obs_stat,1);
f=polyval(p,[1:length(alt_obs_stat)]);
plot(f,'b')
legend("Altitude data for stationary point data for partially occluded area","Best fit line for Altitude data for partially occluded area stationary point")
hold off
title("Altitude Data Plot for a stationary point's readings in a partially occluded area")
ylabel("Altitude(metres)")
xlabel("Time (seconds)")
subplot(2,2,3)
plot(alt_open_mov,'r+')
hold on
p=polyfit([1:length(alt_open_mov)],alt_open_mov,1);
f=polyval(p,[1:length(alt_open_mov)]);
plot(f,'b')
legend("Altitude data for structured motion in open area","Best fit line for Altitude data for open area structured motion")
hold off
title("Altitude Data Plot for structured motion readings in an open area")
ylabel("Altitude(metres)")
xlabel("Time (seconds)")
subplot(2,2,4)
plot(alt_obs_mov,'r+')
hold on
p=polyfit([1:length(alt_obs_mov)],alt_obs_mov,1);
f=polyval(p,[1:length(alt_obs_mov)]);
plot(f,'b')
legend("Altitude data for structured motion in partially occluded area","Best fit line for Altitude data for partially occluded area structured motion")
hold off
title("Altitude Data Plot for structured motion readings in a partially occluded area ")
ylabel("Altitude(metres)")
xlabel("Time (seconds)")
figure
subplot(2,2,1)
plot(gps_quality_open_stat,'r+')
hold on
p=polyfit([1:length(gps_quality_open_stat)],gps_quality_open_stat,1);
f=polyval(p,[1:length(gps_quality_open_stat)]);
plot(f,'b')
legend("GPS Quality data for open area stationary point","Best fit line for GPS Quality data for open area stationary point")
hold off
title("GPS Quality Data Plot for a stationary point's readings in an open area")
ylabel("GPS Quality")
xlabel("Time (seconds)")
subplot(2,2,2)
plot(gps_quality_obs_stat,'r+')
hold on
p=polyfit([1:length(gps_quality_obs_stat)],gps_quality_obs_stat,1);
f=polyval(p,[1:length(gps_quality_obs_stat)]);
plot(f,'b')
legend("GPS Quality data for stationary point data for partially occluded area","Best fit line for GPS Quality data for partially occluded area stationary point")
hold off
title("GPS Quality Data Plot for a stationary point's readings in a partially occluded area")
ylabel("GPS Quality")
xlabel("Time (seconds)")
subplot(2,2,3)
plot(gps_quality_open_mov,'r+')
hold on
p=polyfit([1:length(gps_quality_open_mov)],gps_quality_open_mov,1);
f=polyval(p,[1:length(gps_quality_open_mov)]);
plot(f,'b')
legend("GPS Quality data for structured motion in open area","Best fit line for GPS Quality data for open area structured motion")
hold off
title("GPS Quality Data Plot for structured motion readings in an open area")
ylabel("GPS Quality")
xlabel("Time (seconds)")
subplot(2,2,4)
plot(gps_quality_obs_mov,'r+')
hold on
p=polyfit([1:length(gps_quality_obs_mov)],gps_quality_obs_mov,1);
f=polyval(p,[1:length(gps_quality_obs_mov)]);
plot(f,'b')
legend("GPS Quality data for structured motion in partially occluded area","Best fit line for GPS Quality data for partially occluded area structured motion")
hold off
title("GPS Quality Data Plot for structured motion readings in a partially occluded area ")
ylabel("GPS Quality")
xlabel("Time (seconds)")
figure
subplot(2,1,1)
plot(err_utm_open_mov)
title("Absolute values of open area structured motion UTM data error values from best fit line values")
xlabel("Time (seconds)")
ylabel("Absolute value of error(centimetres)")
subplot(2,1,2)
plot(err_utm_obs_mov)
title("Absolute values of partially occluded area structured motion UTM data error values from best fit line values")
xlabel("Time (seconds)")
ylabel("Absolute value of error(centimetres)")