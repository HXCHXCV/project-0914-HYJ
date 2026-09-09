% -------------------------------------------------------------------------
%                    SoftSim: GPS IF signal simulator
% Author:
%        Yafeng Li
%        @ Beijing Information Science and Technology University(BISTU)
% 2021. 08. 18
% -------------------------------------------------------------------------
%
%% Clean up the environment first =================================
clear; close all; clc;
format long g

%% Initialize constants, settings =================================
settings = initSettings();

%% Main script generating IF signals ==============================
disp(['   IF signal generating started at ', datestr(now)]);
% The main process
% signalGenerating(settings);  %无欺骗,只能设置固定位置
if settings.SpoofingTypes == 1
    signalGenerating_v1_SingPreset(settings);  %位置欺骗 单天线转发 预设位置高级转发
elseif settings.SpoofingTypes == 2
    signalGenerating_v2(settings);   %时间欺骗
end

% signalGenerating1(settings);  %有欺骗   添加了欺骗信号，但码相位差固定，且通过轨迹难以实现精准控制码相位差
% signalGenerating2(settings);  %有欺骗  通过控制传输时间差，完美实现了码相位差控制，并且多普勒频偏可调，但码相位差仍然固定
% signalGenerating3(settings);  %有欺骗  添加了多径信号
% signalGenerating4(settings);  %有欺骗  码相位差随时间变化，从而实现TSA式的诱导式欺骗  但是数据产生不能超过30s,检查问题
% signalGenerating5(settings);   %有欺骗  码相位差随时间变化，从而实现TSA式的诱导式欺骗 问题清除

%% Generate plot of generated IF data =============================
fprintf('Probing data (%s)...\n', settings.IfFile) 
probeData(settings);
disp(['   IF signal simulation is over at', datestr(now)])
