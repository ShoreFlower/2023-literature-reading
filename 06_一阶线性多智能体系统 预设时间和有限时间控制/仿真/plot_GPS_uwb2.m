% 从rosbag中读取GPS的坐标数据，并加以作图显�?
clear, clc, close all
% Beautiful colors
blue      = [0, 0.4470, 0.7410];
orange    = [0.8500, 0.3250, 0.0980];
yellow    = [0.9290, 0.6940, 0.1250];
purple    = [0.4940, 0.1840, 0.5560];
green     = [0.4660, 0.6740, 0.1880];
lightblue = [0.3010, 0.7450, 0.9330];
red       = [0.6350, 0.0780, 0.1840];
black     = [0.0, 0.0, 0.0];
format long;

%bag=rosbag('gps_ATK_2021-10-11-17-29-19.bag');
[uwb_x_1,uwb_y_1,GPSz_UAV] = loadGPSbag('static_uwb_point_all_topic_2022-10-29-18-39-47.bag');
[uwb_x_2,uwb_y_2,GPSz_ATK] = loadGPSbag('square_uwb_point_all_topic_2022-10-29-18-41-20.bag');

figure(1);
scatter(uwb_x_1(:,1), uwb_y_1(:,1),'b*', 'linewidth', 1);
title('Robot-Lous');
legend('Robot-static');
ylabel('m', 'Interpreter', 'latex'); 
axis([0.5 2.5 0.5 2.5]);
% 以静态图�?有点的中心为圆心画圆�?
roundx = mean(uwb_x_1(:,1));
roundy = mean(uwb_y_1(:,1));
xmax = max(uwb_x_1(:,1));
xmin = min(uwb_x_1(:,1));
ymax = max(uwb_y_1(:,1));
ymin = min(uwb_y_1(:,1));
roundrx = xmax - xmin;
roundry = ymax - ymin;

% round = zeros(1021,1);
% for N=1:1:1021
% round(N,1) = sqrt((roundx-uwb_x_1(N,1))^2+(roundy-uwb_y_1(N,1))^2);
% roundmax = min(round(:,1));
% end
    
if roundrx > roundry
    roundr = roundrx/2;
else
    roundr = roundry/2;
end
% htextR = text(20,80, ['半径  R = ',num2str(roundmax)]);
% set( htextR, 'FontSize', 12,'Color','w');
para = [roundx - roundr, roundy - roundr, 2*roundr, 2*roundr];
rectangle('Position', para, 'Curvature', [1 1]);
axis equal
% legend('R=',roundmax);





% theta = 0:pi/20:2*pi; %角度[0,2*pi] 
% x = roundx+roundr*cos(theta);
% y = roundy+roundr*sin(theta);
% plot(x,y,'-')
% hold on
% plot(uwb_x_1(:,1),uwb_y_1(:,1),'bo')
% axis equal


hold on
figure(2);
scatter(uwb_x_2(:,1), uwb_y_2(:,1),'r*', 'linewidth', 1);
title('Robot-Lous');
legend('Robot-square');
ylabel('m', 'Interpreter', 'latex'); 
axis([0.5 2.5 0.5 2.5]);
% 
% figure(2);
% plot(1:length(satellitesused_UAV), satellitesused_UAV(:,1), 'linewidth', 2,'color', blue);
% hold on
% plot(1:length(satellitesused_ATK), satellitesused_ATK(:,1), 'linewidth', 2,'color', red)
% title('satellites used num');
% legend('GPS-UAV','GPS-ATK');
% axis([0 length(satellitesused_UAV) 0 40]);
% figure(2);
% plot(1:length(satellitesused_UAV), satellitesused_UAV(:,1), 'linewidth', 2,'color', blue);
% hold on
% plot(1:length(satellitesused_ATK), satellitesused_ATK(:,1), 'linewidth', 2,'color', red)
% title('satellites used num');
% legend('GPS-UAV','GPS-ATK');
% axis([0 length(satellitesused_UAV) 0 40]);
 

%% 从ROSbag中提取数�?  输出XYZ坐标和GPS卫星数量
function[x,y,z] = loadGPSbag(str)
bag=rosbag(str);
NavSatFix_message=select(bag,'MessageType','nav_msgs/Odometry   ');
NavSatFix_data=readMessages(NavSatFix_message);%,1:8000);
len = length(NavSatFix_data); 
x = zeros(len,1); 
y = zeros(len,1);
z = zeros(len,1);
for i=1:len
    x(i,1) = NavSatFix_data{i, 1}.Pose.Pose.Position.X ;
    y(i,1) = NavSatFix_data{i, 1}.Pose.Pose.Position.Y ;
    z(i,1) = NavSatFix_data{i, 1}.Pose.Pose.Position.Z ;
end

% GPSFix_message=select(bag,'MessageType','nav_msgs/Odometry'); 
% GPSFix_data=readMessages(GPSFix_message);%,1:8000);
% len = length(GPSFix_data); 
% 
% satellitesused = zeros(len,1); 
% for i=1:len
%     satellitesused(i,1) =GPSFix_data{i, 1}.Status.SatellitesUsed;
% end
% [GPSx,GPSy,GPSz] = latlngalt2xyz(latitude,longitude,altitude);

end

function [x,y,z] = latlngalt2xyz(lat, lng, alt)
    lat0 = deg2rad(42.293227);
    lng0 = deg2rad(-83.709657);
    alt0 = 270.;
    re = 6378135.;
    rp = 6356750.;
    d = (re * cos(lat0))^2 + (rp * sin(lat0))^2;
    rns = (re*rp)^2 / d^(3/2);
    rew = re^2 / sqrt(d);
    x = sin(lat - lat0) * rns;
    y = sin(lng - lng0) * rew * cos(lat0);
    z = alt0 - alt;
end
 