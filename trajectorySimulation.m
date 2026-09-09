function [pos_X,pos_Y,pos_Z,lat,lon,alt] = trajectorySimulation(settings,trajectoryTime,state,StartPotion)

    %  lat0=40;      %初始纬度【latitude】
    %  long0=116;    %初始经度【longitude】
    %  alt0=100;     %初始高程【altitude】
    %  trajectoryTime    %时间 【s】
    %  state = 0;   %状态选择  0-静止状态  1-运动状态
    
    lat0  = StartPotion(1);
    long0 = StartPotion(2);
    alt0  = StartPotion(3);
 
    Rearth=6378137;        % 地球半径
    Rate=(2*pi*Rearth)/360;  % 计算地球表面上每度对应的距离

    Distanc_asynchronous = settings.SpoofingDistance_asynchronous;
    Distanc_synchronous =  settings.SpoofingDistance_synchronous;  %运动状态，最后总时间、总路程分别要对的上  s=Vmax*(Time_max+Time_transition);  TJTime= Time_max+2*Time_transition+Time_static;
    
    
    Frq=1000;      %模拟器读取的时间间隔0.001s?
    
    %% 根据速度设置场景 目前采用简单的经度拉偏方式
     if state == 1    %静止状态   
         V = zeros(1,trajectoryTime);
         V_lat = V;    V_long = V;      V_alt = V;   
     elseif state == 3         %运动状态，最后总时间、总路程分别要对的上  s=Vmax*(Time_max+Time_transition);  TJTime= Time_max+2*Time_transition+Time_static;
%% 同步
%          52m
%          Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
%          Vmax = 4;  %最大速度，单位m/s  
%          Time_max = 6;    %满速度时间
%          Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;
%          
%          96m
%          Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
%          Vmax = 6;  %最大速度，单位m/s  
%          Time_max = 12;    %满速度时间
%          Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;
%                   
%          150m
%          Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
%          Vmax = 10;  %最大速度，单位m/s  
%          Time_max = 10;    %满速度时间
%          Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;
% 
%          300m
%          Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
%          Vmax = 20;  %最大速度，单位m/s  
%          Time_max = 10;    %满速度时间
%          Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;
% 
%          450m
%          Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
%          Vmax = 30;  %最大速度，单位m/s  
%          Time_max = 10;    %满速度时间
%          Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;
% 
%          600m
         Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
         Vmax = 40;  %最大速度，单位m/s  
         Time_max = 10;    %满速度时间
         Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;
% 
%          900m
%          Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
%          Vmax = 60;  %最大速度，单位m/s  
%          Time_max = 10;    %满速度时间
%          Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;
%          
%          1200m
%          Time_static = 20;     %静止时长要大于无欺骗时长，保证欺骗加进来重叠几s。
%          Vmax = 80;  %最大速度，单位m/s  
%          Time_max = 10;    %满速度时间
%          Time_transition = (Distanc_synchronous - Time_max*Vmax)/Vmax;

         if (Time_static+Time_max+2*Time_transition+1)>trajectoryTime
             disp('The trajectory time setting is incorrect. Exiting!');
             return
         end
         Time_residue = trajectoryTime-1-Time_static-Time_max-2*Time_transition; 
         t =  1/Time_transition:1/Time_transition:1;
         V = [zeros(1,Time_static),t,ones(1,Time_max),(1-t),zeros(1,Time_residue),zeros(1,1)]*Vmax; %
         V_lat = zeros(1,trajectoryTime);    V_long = V;      V_alt = zeros(1,trajectoryTime);  
          
%%  异步
%          %120-600m  异步
%          Time_static = 10;     %静止时长等于无欺骗时长。
%          Vmax_as = 8;  %最大速度，单位m/s  
%          Time_max_as = 10;    %满速度时间
%          Time_transition_as = (Distanc_asynchronous - Time_max_as*Vmax_as)/Vmax_as;
% 
%          Vmax_s = 40;  %最大速度，单位m/s  
%          Time_max_s = 10;    %满速度时间
%          Time_transition_s = (Distanc_synchronous - Time_max_s*Vmax_s)/Vmax_s;
%          
%          if (Time_static+Time_max_as+2*Time_transition_as+Time_max_s+2*Time_transition_s+1)>trajectoryTime
%              disp('The trajectory time setting is incorrect. Exiting!');
%              return
%          end
%          
%          Time_overlap = trajectoryTime-1-Time_static-(Time_max_as+2*Time_transition_s+Time_max_as+2*Time_transition_s); 
%          t_as =  1/Time_transition_as:1/Time_transition_as:1;
%          t_s =  1/Time_transition_s:1/Time_transition_s:1;
% %          V=[zeros(1,Time_static),t_as*Vmax_as,ones(1,Time_max_as)*Vmax_as,(1-t_as)*Vmax_as,zeros(1,Time_overlap),t_s*Vmax_s,ones(1,Time_max_s)*Vmax_s,(1-t_s)*Vmax_s,zeros(1,1)*Vmax_s]; %
%          Vas = [zeros(1,Time_static),t_as,ones(1,Time_max_as),(1-t_as),zeros(1,Time_overlap)]*Vmax_as;
%          Vs = [t_s,ones(1,Time_max_s),(1-t_s),zeros(1,1)]*Vmax_s;
%          V = [Vas Vs];
%          V_lat = zeros(1,trajectoryTime);    V_long = V;      V_alt = zeros(1,trajectoryTime);  
         
    
     else   %为了让真实接收机轨迹也动起来设置的另一轨迹  
         
         %匀速直线运动
         Vmax = 2;  %最大速度，单位m/s  
         Time_max = 40;    %满速度时间       
         Time_residue = trajectoryTime-1-Time_max; 
         V=[ones(1,Time_max),zeros(1,Time_residue),zeros(1,1)]*Vmax; %
         V_lat = zeros(1,trajectoryTime);    V_long = V;      V_alt = zeros(1,trajectoryTime);           
         
        %将速度曲线重复指定次数，并调整形状以便于后续处理
%          V_times=1;    %场景重复次数
%          V=reshape((ones(V_times,1)*V)',1,V_times*length(V));    %场景重复后的速度
         %ones(V_times,1)*V是将行向量V复制V_times次，转置后reshape将其转为1行V_times*length(V)列，达到速到复制的效果
     end

%% 位移计算

%根据速度计算变化位移
s_lat_m  =   [V_lat(1)/2,cumsum((V_lat(1:end-1)+V_lat(2:end))/2)];     %如果输入是向量，cumsum 返回一个向量，其中每个元素是原始向量中到该位置为止的所有元素的和
s_long_m =   [V_long(1)/2,cumsum((V_long(1:end-1)+V_long(2:end))/2)];  %这是一种近似计算位移的方法。
s_alt_m  =   [V_alt(1)/2,cumsum((V_alt(1:end-1)+V_alt(2:end))/2)];     %在每个时间间隔内，假设速度是恒定的，那么位置的变化可以通过平均速度乘以时间间隔来计算。而时间间隔就是1s。

%初始位置加上变化的位移得到实时位置
s_lat=lat0+s_lat_m/Rate;
s_long=long0+s_long_m/Rate;
s_alt=alt0+s_alt_m/Rate;

%% 绘制加速度和位移曲线

figure(1);  set(gcf,'color','w');
subplot(2,1,1); plot(diff(V));title('加速度')
subplot(2,1,2); plot(V);title('速度')

figure(2);  set(gcf,'color','w');
subplot(3,1,1); plot(s_lat_m);  title('维度方向位移变化')
subplot(3,1,2); plot(s_long_m); title('经度方向位移变化')
subplot(3,1,3); plot(s_alt_m);  title('高程方向位移变化')

figure(3);  set(gcf,'color','w');
subplot(3,1,1); plot(s_lat);  title('维度方向实时位置')
subplot(3,1,2); plot(s_long); title('经度方向实时位置')
subplot(3,1,3); plot(s_alt);  title('高程方向实时位置')


%%
%将纬经高转换为地心地固坐标系（ECEF）XYZ
P=lla2ecef([s_lat',s_long',s_alt']);  

pos_X=P(:,1);   
pos_Y=P(:,2);     
pos_Z=P(:,3);
len=length(pos_X);

%对位置数据进行插值，以匹配模拟器的时间间隔
pos_X=interp1(pos_X,1/Frq:1/Frq:len,'spline')';
pos_Y=interp1(pos_Y,1/Frq:1/Frq:len,'spline')';
pos_Z=interp1(pos_Z,1/Frq:1/Frq:len,'spline')';

pos_X(abs(pos_X)<1e-5)=0;
pos_Y(abs(pos_Y)<1e-5)=0;
pos_Z(abs(pos_Z)<1e-5)=0;

wgs84 = wgs84Ellipsoid('kilometer');
[lat, lon, alt] = ecef2geodetic(wgs84,pos_X/1000, pos_Y/1000, pos_Z/1000);
alt = alt * 1000;

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