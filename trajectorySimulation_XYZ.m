function [pos_X,pos_Y,pos_Z,pos_lat,pos_lon,pos_alt] = trajectorySimulation_XYZ(settings,trajectoryTime,state,StartPotion_XYZ,s)

    %  lat0=40;      %初始纬度【latitude】
    %  long0=116;    %初始经度【longitude】
    %  alt0=100;     %初始高程【altitude】
    %  trajectoryTime    %时间 【s】
    %  state = 0;   %状态选择  0-静止状态  1-运动状态
    

    x0  = StartPotion_XYZ(1) + s;
    y0  = StartPotion_XYZ(2);
    z0  = StartPotion_XYZ(3);
 
    Rearth=6378137;        % 地球半径
    Rate=(2*pi*Rearth)/360;  % 计算地球表面上每度对应的距离

%     Distanc_asynchronous = settings.SpoofingDistance_asynchronous;
%     Distanc_synchronous =  settings.SpoofingDistance_synchronous;  %运动状态，最后总时间、总路程分别要对的上  s=Vmax*(Time_max+Time_transition);  TJTime= Time_max+2*Time_transition+Time_static;
    
    
    Frq=1000;      %模拟器读取的时间间隔0.001s?
    
    %% 根据速度设置场景 目前采用简单的经度拉偏方式
     if state == 1    %静止状态   
         V = zeros(1,trajectoryTime);
         V_x = V;    V_y = V;      V_z = V;   
         s_x_m  =   [V_x(1)/2,cumsum((V_x(1:end-1)+V_x(2:end))/2)];     %如果输入是向量，cumsum 返回一个向量，其中每个元素是原始向量中到该位置为止的所有元素的和
         s_y_m  =   [V_y(1)/2,cumsum((V_y(1:end-1)+V_y(2:end))/2)];  %这是一种近似计算位移的方法。
         s_z_m  =   [V_z(1)/2,cumsum((V_z(1:end-1)+V_z(2:end))/2)];     %在每个时间间隔内，假设速度是恒定的，那么位置的变化可以通过平均速度乘以时间间隔来计算。而时间间隔就是1s。
    
    elseif state == 2   %接收机运动  
         
         % 匀速直线运动
         Vmax = 3;  %最大速度，单位m/s  
         Time_max = trajectoryTime-1;    %满速度时间       
         Time_residue = trajectoryTime-1-Time_max; 
         V=[ones(1,Time_max),zeros(1,Time_residue),ones(1,1)]*Vmax; %
         V_x = V;    V_y = zeros(1,trajectoryTime);      V_z = zeros(1,trajectoryTime);           
         
         t = 1:length(V_x);    % 时间序号（t=1,2,...,n，对应第1到第n个时刻）
         s_x_m = V_x .* t;     % 各时刻累计位移（=速度×时间，Δt=1时）
         s_y_m = V_y .* t;  
         s_z_m = V_z .* t;  

%          匀速圆周运动1 (感觉这个好用)
%          Vmax = 4;              % 线速度大小，单位m/s
%          circle_num = 10;        % 40s内转动圈数
%          omega = 2 * pi * circle_num / trajectoryTime;  % 角速度 (rad/s)
%          R = Vmax / omega;                              % 圆周运动半径 (m)
% 
%          t_spoof = settings.SpoofingStart_Position / 1000;  % 转换为秒  %用于修正欺骗开启时距离欺骗源位置100m
%          theta_spoof = omega * t_spoof;  % 欺骗开启时的相位角
% 
%          t = 1:1:trajectoryTime;  %这一行很关键
%          theta_real = omega * t;     % 各时刻的角度 (rad)
%          theta = theta_real + theta_spoof; %为了在欺骗打开时回到其实设置点
% %          theta = theta_real;
% 
%          V_x = -Vmax * sin(theta);  % x方向速度分量
%          V_y = Vmax * cos(theta);   % y方向速度分量（顺时针圆周运动，若要逆时针可交换符号）
% 
%          s_x_m = zeros(1, trajectoryTime);  s_y_m = zeros(1, trajectoryTime);  s_z_m = zeros(1, trajectoryTime);
%          for i = 1:trajectoryTime
%              if i == 1
%                  s_x_m(i) = V_x(i) * 1;  % 第1秒位移
%                  s_y_m(i) = V_y(i) * 1;
%              else
%                  s_x_m(i) = s_x_m(i-1) + V_x(i) * 1;  % 累计位移（前i秒总和）
%                  s_y_m(i) = s_y_m(i-1) + V_y(i) * 1;
%              end
%          end
         

    elseif state == 3     %欺骗预设轨迹-运动
        
          y0  = StartPotion_XYZ(2)-4*settings.SpoofingStart_Position/1000; %y方向操作
          Vmax = 4;  %最大速度，单位m/s  
          Time_max = trajectoryTime-1;    %满速度时间       
          Time_residue = trajectoryTime-1-Time_max; 
          V=[ones(1,Time_max),zeros(1,Time_residue),ones(1,1)]*Vmax; %
          V_x = zeros(1,trajectoryTime);    V_y = V;      V_z = zeros(1,trajectoryTime);           

          t = 1:length(V_x);    % 时间序号（t=1,2,...,n，对应第1到第n个时刻）
          s_x_m = -V_x .* t;     % 各时刻累计位移（=速度×时间，Δt=1时）
          s_y_m = -V_y .* t;  
          s_z_m = -V_z .* t;  
     
    else     %欺骗源运动
        % 同步  600m
         Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
         Vmax = 40;  %最大速度，单位m/s  
         Time_max = 10;    %满速度时间
         Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;

         if (Time_static+Time_max+2*Time_transition+1)>trajectoryTime
             disp('The trajectory time setting is incorrect. Exiting!');
             return
         end
         Time_residue = trajectoryTime-1-Time_static-Time_max-2*Time_transition; 
         t =  1/Time_transition:1/Time_transition:1;
         V = [zeros(1,Time_static),t,ones(1,Time_max),(1-t),zeros(1,Time_residue),zeros(1,1)]*Vmax; %
         V_x = zeros(1,trajectoryTime);    V_y = V;      V_z = zeros(1,trajectoryTime);  
         s_x_m  =   [V_x(1)/2,cumsum((V_x(1:end-1)+V_x(2:end))/2)];     
         s_y_m  =   [V_y(1)/2,cumsum((V_y(1:end-1)+V_y(2:end))/2)];  
         s_z_m  =   [V_z(1)/2,cumsum((V_z(1:end-1)+V_z(2:end))/2)];          
    end

%% 位移计算
%根据速度-计算变化位移[原代码]
% s_x_m  =   [V_x(1)/2,cumsum((V_x(1:end-1)+V_x(2:end))/2)];     %如果输入是向量，cumsum 返回一个向量，其中每个元素是原始向量中到该位置为止的所有元素的和
% s_y_m  =   [V_y(1)/2,cumsum((V_y(1:end-1)+V_y(2:end))/2)];  %这是一种近似计算位移的方法。
% s_z_m  =   [V_z(1)/2,cumsum((V_z(1:end-1)+V_z(2:end))/2)];     %在每个时间间隔内，假设速度是恒定的，那么位置的变化可以通过平均速度乘以时间间隔来计算。而时间间隔就是1s。


%初始位置加上变化的位移得到实时位置
s_x = x0 - s_x_m;
s_y = y0 - s_y_m;
s_z = z0 - s_z_m;

% s_x = x0 + s_x_m;
% s_y = y0 + s_y_m;
% s_z = z0 +s_z_m;

%% 绘制加速度和位移曲线

% figure(1);  set(gcf,'color','w');
% subplot(2,1,1); plot(diff(V));title('加速度')
% subplot(2,1,2); plot(V);title('速度')

figure(2);  set(gcf,'color','w');
subplot(3,1,1); plot(s_x_m);  title('差值前X方向位移变化')
subplot(3,1,2); plot(s_y_m); title('差值前Y方向位移变化')
subplot(3,1,3); plot(s_z_m);  title('差值前Z方向位移变化')

figure(3);  set(gcf,'color','w');
subplot(3,1,1); plot(s_x);  title('差值前X方向实时位置')
subplot(3,1,2); plot(s_y); title('差值前Y方向实时位置')
subplot(3,1,3); plot(s_z);  title('差值前Z方向实时位置')


%% %对位置数据进行插值，以匹配模拟器的时间间隔

% 匀速直线插值方式
% len = length(s_x);
% pos_X=interp1(s_x,1/Frq:1/Frq:len,'spline')';    %spline
% pos_Y=interp1(s_y,1/Frq:1/Frq:len,'spline')';    %spline
% pos_Z=interp1(s_z,1/Frq:1/Frq:len,'spline')';    %spline

% 圆周运动插值方式
len = length(s_x); 
t_original = 0:1:(len-1);  % 0, 1, 2, ..., len-1 秒
t_target = 0:1/Frq:(len-1);  % 0, 0.001, 0.002, ..., len-1 秒
pos_X = interp1(t_original, s_x, t_target, 'spline')';
pos_Y = interp1(t_original, s_y, t_target, 'spline')';
pos_Z = interp1(t_original, s_z, t_target, 'spline')';



pos_X(abs(pos_X)<1e-5)=0;
pos_Y(abs(pos_Y)<1e-5)=0;
pos_Z(abs(pos_Z)<1e-5)=0;

figure(4);  set(gcf,'color','w');
subplot(3,1,1); plot(pos_X);  title('插值后X方向实时位置')
subplot(3,1,2); plot(pos_Y); title('插值后Y方向实时位置')
subplot(3,1,3); plot(pos_Z);  title('插值后Z方向实时位置')

figure(5);  set(gcf,'color','w');
plot(pos_X,pos_Y);  

P_lla =ecef2lla([pos_X,pos_Y,pos_Z]);  

pos_lat=P_lla(:,1);   
pos_lon=P_lla(:,2);     
pos_alt=P_lla(:,3);




% OutTime = (0:1/Frq:length(pos_X)/Frq)';

%% 地图显示轨迹
% figure(4);set(gcf,'color','w');
% geoplot(lat, lon, '-o', 'color',[0.1010 0.7450 0.93301],'LineWidth', 1.25);%连线图
% hold on
% geolimits([28.152777 28.159722], [112.930555 112.94444]);  % 设置纬度范围和经度范围
% geobasemap satellite %选择底图格式
% %标记起点和终点
% geoplot(lat(1), lon(1), '*','color',[0.835  0.078 0.184], 'MarkerSize', 20, 'LineWidth', 2);  
% text(lat(1), lon(1), ' Start', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right','color', 'r','FontSize', 15);
% geoplot(lat(end), lon(end), '*','color',[0.835  0.078 0.184], 'MarkerSize', 20, 'LineWidth', 2);  
% text(lat(end), lon(end), ' End', 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left','color', 'r','FontSize', 15);
% %标记位置
% sem_lon = 112.937; sem_lat = 28.15625;%学院精准位置  地图
% geoplot(sem_lat, sem_lon, 'h','color',[0.835  0.078 0.184], 'MarkerSize', 20, 'LineWidth', 2);
% text(sem_lat, sem_lon, 'the College of Semiconductors (College of Integrated Circuits), Hunan University, '...
% , 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left','color', 'r','FontSize', 15);
% grid on;
% set(gca,'LineWidth',1,'GridLineStyle','--','GridColor','k','Gridalpha',1);
% hold off
% set(gca, 'FontSize', 15);