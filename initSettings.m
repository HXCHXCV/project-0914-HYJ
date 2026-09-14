function settings = initSettings()
% -------------------------------------------------------------------------
%                   SoftSim: GPS IF signal simulator 
% Author: 
%        Yafeng Li 
%    @ Beijing Information Science and Technology University(BISTU)
%    2022. 08. 18
% -------------------------------------------------------------------------
%
%% Simulator settings ============================================
% Signal length(milliseconds) to be simulated
settings.msToProcess        = 66000;        %[ms]
% Rinex version  
settings.rinexVersion        = 2;      % 2 or 3
% Name of the rinex file to be used in the simulation
settings.rinexfile           = 'brdc3140_251110.25n';  
% settings.rinexfile           = 'brdc1330new.25n';  %林雄给的
% settings.rinexfile           = 'brdc3540.14n';  %师兄给的

settings.IfFile             = 'Test.bin';

% RINEX 3.02 OBS export (GPS L1 C/A)
settings.obsEnable          = 1;
settings.obsInterval        = 1.0;       % 0.001 or 1.0 [s]
settings.directObsFile      = 'Direct.obs';
settings.virtualObsFile     = 'Virtual.obs';
settings.partialSpoofObsFile = 'PartialSpoof.obs';

settings.markerName         = 'SIM';
settings.markerNumber       = '0000';
settings.observerAgency     = 'SoftSim';
settings.receiverType       = 'SIM-RX';
settings.antennaType        = 'SIM-ANT';
settings.antDeltaHEN        = [0 0 0];


% Data type used to store one sample
settings.dataType           = 'int8';  
% settings.dataType           = 'schar';

% File Types
%1 - 8 bit real samples S0,S1,S2,...
%2 - 8 bit I/Q samples I0,Q0,I1,Q1,I2,Q2,...                      
settings.fileType           = 1;

% Intermediate, sampling and code frequencies
settings.IF                 = 1.548e6;              % [Hz]   1.364e6
% settings.samplingFreq       = 38.192e6;              % [Hz]
settings.samplingFreq       = 30.69e6;              % [Hz]
% settings.samplingFreq       = 51.15e6;              % [Hz]

%欺骗参数
settings.SpoofingEnable = 1;  %欺骗使能开关  1-on  o-off
settings.powerIncreaseFactor  =  10;  %[0~10]  %欺骗功率大于直达xdBm 


%欺骗类型
settings.SpoofingTypes = 1; %    1:地点欺骗   2:时间欺骗

% Position-spoofing channel mode
settings.SpoofingChannelMode = 1;          % 1-full channel, 2-partial channel
settings.PartialSpoofingChannelCount = 4; % First N visible channels in partial mode

%地点欺骗参数控制 轨迹  注意轨迹需要手动设置
settings.SpoofingStart_Position = 26000;  %无欺骗时长[ms]，需要保证时间小于欺骗场景中的静止时间，最好必须超过6s，因为FGI-GSRx会舍弃第一子帧定位[ms] 

settings.StartPotion_au = [28.15625, 112.937, 28];   %真实位置    %(28.15625,112.937,28) %学院地图位置 
settings.StartPotion_au_XYZ = lla2ecef(settings.StartPotion_au);
auECEF = lla2ecef(settings.StartPotion_au);

%转发欺骗源控制参数
settings.StartPotion_sp_flase_XYZ = [auECEF(1),  auECEF(2)+100 , auECEF(3)];
settings.StartPotion_sp_flase = ecef2lla(settings.StartPotion_sp_flase_XYZ);  %欺骗预设位置-虚拟轨迹   

settings.StartPotion_sp_XYZ = [auECEF(1)+100 , auECEF(2) , auECEF(3)];
settings.StartPotion_sp = ecef2lla(settings.StartPotion_sp_XYZ);  %欺骗源位置1  天线1   


%时间欺骗参数控制 TSA
settings.SpoofingStart_Time = 16000;  %无欺骗时长，需要保证时间小于欺骗场景中的静止时间，最好必须超过6s，因为FGI-GSRx会舍弃第一子帧定位？[ms] 
settings.SpoofingOverlap = 0;  %欺骗和直达的重合时间
settings.SpoofingPull = 20000;  %欺骗拉脱直达峰时时间
settings.SpoofingSeparation = settings.msToProcess- settings.SpoofingStart_Time - settings.SpoofingPull; 
settings.spoofingDelay = 3;  %码相位差控制 [0~2]/[-2~0]
settings.DelaySelection = 1; %码片延时方式选择，0-固定码片延时，1-诱导拉偏延时。 

%C/No
settings.CNoValues             = 45;   %直达载噪比

settings.codeFreqBasis      = 1.023e6;              % [Hz]
% Define number of chips in a code period
settings.codeLength         = 1023;
% Nominal carrier frequency
settings.carrFreqBasis      = 1575.42e6;            % [Hz]
% White noise amplitude (Sigma): should be set according to the ADC
% quantization level and max C/No values 
settings.gwnAmp             = 40;                    % 
% Front end bandwidth 
settings.FeBandwidth        = 1.3e6 *2;              % [Hz]
% Front end filter order 
settings.FeOrder            = 20;  
% Enable FE filter or not
settings.filterEn           = 1;                    % 1- on,  0 -off 
% Elevation mask to exclude signals from satellites at low elevation
settings.elevationMask      = 10;           %[degrees 0 - 90]

%% Constants ======================================================
settings.c                  = 299792458;    % The speed of light, [m/s]
settings.startOffset        = 68.802;       %[ms] Initial signal travel time


