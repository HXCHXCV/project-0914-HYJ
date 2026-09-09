function signalGenerating_v2(settings)
% Main script generating IF signals
% Input:
%         settings    - IF simulator settings
%      
% -------------------------------------------------------------------------
%                   SoftSim: GPS IF signal simulator 
% Author: 
%        Yafeng Li 
%    @ Beijing Information Science and Technology University(BISTU)
%    2022. 08. 18
% -------------------------------------------------------------------------
%
%
%% Load trajectory ================================================
trajectoryTime = round(settings.msToProcess)/1000 + 1; %1s用来处理trajectory边界问题，不起任何作用
state = [1,2,3]; %状态选择，真实轨迹采用1和2，对应静止和运动，欺骗采用3-运动

%真实轨迹
StartPotion = settings.StartPotion_au;
[ECEFx,ECEFy,ECEFz,lat,lon,alt] = trajectorySimulation(settings, trajectoryTime,state(1),StartPotion); 
length_time = length(ECEFx);

trajectory = zeros(length_time,6);
for index = 1: length_time                                           
      trajectory(index,1:6) = [lat(index), lon(index), alt(index),ECEFx(index),ECEFy(index),ECEFz(index)]; 
end

%欺骗码片设置
% T0   = settings.SpoofingStart_Time;
% Tpull_async   = settings.SpoofingPull_as;
% Toverlap   = settings.SpoofingOverlap;
% Tpull_sync   = settings.SpoofingPull;
% % delay_async = settings.spoofingDelay_as;   % 异步最终要拉开的码片数
% delay_sync  = settings.spoofingDelay;      % 同步最终要拉开的码片数

%% Read ephemeris file ============================================
if settings.rinexVersion == 2
    [eph,ionoutc] = rinexeV2(settings.rinexfile);    
elseif settings.rinexVersion == 3
    [eph,ionoutc] = rinexeV3(settings.rinexfile);
end

%% Determine the visible satellite list ===========================
% Starting time of the simulation, corresponding to the first positions 
% in the trajectory 
startTime = eph(1).toc;                                         
% Initail position of the receiver trajectory
RxPosEcef = trajectory(1,4:6);  %接收机初始位置XYZ
% Visible satellite list (PRN#)
[satList,elevation,azimuth] = getVisibleSat(eph,startTime,RxPosEcef',settings); %计算卫星俯仰角

%% Sky plot and C/N0 seting========================================
figure(4);set(gcf,'color','w');
skyPlot(azimuth',elevation',satList');  %卫星天空图
% Set C/No: 1.5dB is added for compesating the cross-correlation interference
% between diffierent PRNs
CNoValuesdB = CNoSetting(satList,settings) + 1.5;  %CNoSetting(satList,settings)表示所有卫星的CN0设置  %+1.5后表示？
% Covert to unite of Hz
CNoValues = 10.^(CNoValuesdB/10);  %从dB·Hz转为Hz
% Calculate carrier amplitude 
carrAmp = 2 * settings.gwnAmp * sqrt(CNoValues/settings.samplingFreq);  %这一行是什么？载波幅度？
if  settings.fileType == 2
    carrAmp = carrAmp/sqrt(2);
end
%% Initialize data bits for each channle ==========================
for svIndex = 1:length(satList)
    PRN = satList(svIndex);
    % Genaerate Nav message according to the ephemeris
    navBits(svIndex).frameMsg = eph2sbf(eph(PRN),ionoutc); %#ok<*SAGROW>
    
    % Time to generate Nav Messages
    navBits(svIndex).ephTime.second = startTime;
    navBits(svIndex).ephTime.weekNrm = eph(1).weekNrm;
    
    % Modulate WN, TOW and CRC into Nav messages
    navBits(svIndex).dataWord = uint32(zeros(1,60));
    [navBits(svIndex).dataWord, navBits(svIndex).refTime] = ...
        generateNavMsg(navBits(svIndex).frameMsg,navBits(svIndex).ephTime,1,navBits(svIndex).dataWord);
    
    % Extract Nav data bits from nav message
    navBits(svIndex).dataBit = zeros(1,30*60);
    for bitindex = 1:60
        navBits(svIndex).dataBit((bitindex-1) * 30 + 1 : bitindex * 30) = ...
            double(bitget(navBits(svIndex).dataWord(bitindex),30:-1:1)) * 2 - 1;  
    end
end


%% Generate C/A code table
% Get a vector with the C/A code sampled 1x/chip
for svIndex = 1:length(satList)
    PRN = satList(svIndex);
    caCode = generateCAcode(PRN);
    caCodeTable(svIndex,:) = [caCode(end) caCode caCode(1)];
%     caCodeTable_sp(svIndex,:) = [caCode(end) caCode(1) caCode(end) caCode(1:(length(caCode)-1)) ];
end

%% Allocate variables and spaces ==================================
% Sample counts within each transmitting time calculation step (1 ms) 
blockSize = round(settings.samplingFreq * 0.001);
% Time interval for each transmitting time calculation step (the calculation
% step may not be exactly the same as 1 ms). The time interval of position
% samples in trajectory must equal blockTime.
blockTime = blockSize/settings.samplingFreq;
% Iteration count   锛堟瘡娆″惊鐜?ms銆傚垎鍓叉垚10涓彂灏勬椂闂磋绠楃偣锛?
iterCnt = round(settings.msToProcess/1000/blockTime);

% 欺骗加入时刻开关
spoofingEnable = zeros(1,iterCnt);
spoofingEnable(settings.SpoofingStart_Time+1:end) = 1;

% Initialize Rx Time and Rx postions 
RxTime(1) = startTime;
RxTime(2) = RxTime(1) + blockTime;
RxPosEcef(1:2,:)  = trajectory(1:2,4:6);
% Local oscilator frequency in rad
localOsFreq = (settings.carrFreqBasis - settings.IF) * 2 * pi;
% RF signal freq in rad
carrFreqRad = settings.carrFreqBasis * 2 * pi;
% Nav bit period
bitPeriod = 0.001 * 20;
% Front end filter coeffficient
coef = getFliterCoef(settings);
% Open the IF file to save senerated samples 
[fid, ~] = fopen(settings.IfFile, 'w');

% Start waitbar
hwb = waitbar(0,'IF signal generating ...');
barTimeMs  = round(iterCnt * blockTime * 1000); % [ms]

%% Generate IF siganl =============================================
disp('IF signal generating is undergoing, please wait ...')
for loopCnt =  1:iterCnt
    if settings.DelaySelection == 1
        if loopCnt <= (settings.SpoofingStart_Time + settings.SpoofingOverlap)  
            spoofingDelay = 0; 
        elseif loopCnt <= (settings.SpoofingStart_Time + settings.SpoofingOverlap + settings.SpoofingPull)    
            step = loopCnt - (settings.SpoofingStart_Time + settings.SpoofingOverlap);  % 整数步数    
            ratio_step = step / settings.SpoofingPull;  % 进度比例
            spoofingDelay = (settings.spoofingDelay / settings.codeFreqBasis) * ratio_step;  % 直接计算当前延迟,避免了Pullstep的重复累加
        else
            spoofingDelay = settings.spoofingDelay  / settings.codeFreqBasis;  %固定码片延迟
        end
    else
        spoofingDelay = settings.spoofingDelay  / settings.codeFreqBasis;  %固定码片延迟
    end
    
%     if settings.DelaySelection == 1
%         if loopCnt <= T0                    
%             spoofingDelay = 0;
%         elseif loopCnt <= T0 + Tpull_async       
%             step = (loopCnt - T0) / Tpull_async;
%             spoofingDelay = delay_async * (1-step);
%         elseif loopCnt <= T0 + Tpull_async + Toverlap          
%             spoofingDelay = 0;
%         elseif loopCnt <= T0 + Tpull_async + Toverlap + Tpull_sync    
%             step = (loopCnt - (T0 + Tpull_async + Toverlap)) / Tpull_sync;
%             spoofingDelay = delay_sync * step;
%         else                                   
%             spoofingDelay = delay_sync;
%         end    
%     else
%         spoofingDelay = delay_sync;
%     end
%     spoofingDelay = spoofingDelay / settings.codeFreqBasis;


    % wait bar ------------------------------------------------------------
    if (rem(loopCnt, 100) == 0)
%         Ln = newline;
        processStatus = ['Generating: ', int2str(loopCnt), ...
            ' ms ', ' of ', int2str(barTimeMs), ' msec'];
        try
            waitbar(loopCnt/barTimeMs,hwb,processStatus);
        catch
            % The progress bar was closed. It is used as a signal
            % to stop, "cancel" processing. Exit.
            disp('Progress bar closed, exiting...');
            return
        end
    end

    % Sum of loacal signals of all visible satellites
    if settings.fileType == 1
        localSigSum = zeros(1,blockSize); % samples of 1ms length
    elseif settings.fileType == 2
        localSigSum = complex(zeros(1,blockSize));
    end
      
    for svIndex = 1:length(satList)
        % Compute the transmitting time coresponding to the Rx time -------
        PRN = satList(svIndex);
        [TxTime,satClkErr] = GetTravelTime(RxTime,RxPosEcef,eph(PRN),settings);
        % include tgd, clock error and relativistic effect 
        TxTime = TxTime + satClkErr;

        % generate local code, carrier and Nav data -----------------------
        sapcing = (TxTime(2) - TxTime(1))/blockSize;
        TxTimeSample = linspace(TxTime(1),TxTime(2) - sapcing,blockSize);
        sapcing = (RxTime(2) - RxTime(1))/blockSize;
        RxTimeSample = linspace(RxTime(1),RxTime(2) - sapcing,blockSize);
        
        % local carrier samples
        if  settings.fileType == 1
            localCarr = cos(carrFreqRad * TxTimeSample - localOsFreq * RxTimeSample);
        elseif settings.fileType == 2
            localCarr = exp(-1i*(carrFreqRad * TxTimeSample - localOsFreq * RxTimeSample));
        end

        % Local code samples
        codePhase = TxTimeSample * settings.codeFreqBasis;
        codeIndex = ceil(rem(codePhase,settings.codeLength)) + 1;
        localCode = caCodeTable(svIndex,codeIndex);

        % Nav bit samples
        dataTime = TxTimeSample - (navBits(svIndex).refTime - 6);
        bitPhase = ceil(dataTime/bitPeriod);
        localDataBit = navBits(svIndex).dataBit(bitPhase);
        
        % Spoofing transmitting time
%         if loopCnt <= T0 + Tpull_async
%            TxTimeSample2 = TxTimeSample + spoofingDelay;
%         elseif loopCnt <= T0 + Tpull_async + Toverlap + Tpull_sync 
%            TxTimeSample2 = TxTimeSample - spoofingDelay;
%         end

           TxTimeSample2 = TxTimeSample - spoofingDelay;
    
        
        % Spoofing carrier samples
        if  settings.fileType == 1
            localCarr2 = cos(carrFreqRad * TxTimeSample2 - localOsFreq * RxTimeSample);
        elseif settings.fileType == 2
            localCarr2 = exp(-1i*(carrFreqRad * TxTimeSample2 - localOsFreq * RxTimeSample));
        end
        
        % Spoofing code samples
        codePhase2 = TxTimeSample2 * settings.codeFreqBasis;
        codeIndex2 = ceil(rem(codePhase2,settings.codeLength)) + 1;
        localCode2 = caCodeTable(svIndex,codeIndex2);
        
        % Spoofing Nav bit samples
        dataTime2 = TxTimeSample2 - (navBits(svIndex).refTime - 6);
        bitPhase2 = ceil(dataTime2/bitPeriod);
        localDataBit2 = navBits(svIndex).dataBit(bitPhase2);
        

        % Update Nav date bits --------------------------------------------
        if (TxTime(end) + 1) > (navBits(svIndex).refTime + 30)
            % Time to generate Nav Messages
            navBits(svIndex).ephTime.second = navBits(svIndex).refTime + 30.5;
                        
            % Modulate WN, TOW and CRC into Nav messages
            [navBits(svIndex).dataWord, navBits(svIndex).refTime] = ...
                generateNavMsg(navBits(svIndex).frameMsg,navBits(svIndex).ephTime, ...
                0,navBits(svIndex).dataWord);
            
            % Extract Nav data bits from nav message
            for bitindex = 1:60
                navBits(svIndex).dataBit((bitindex-1) * 30 + 1 : bitindex * 30) = ...
                double(bitget(navBits(svIndex).dataWord(bitindex),30:-1:1)) * 2 - 1;
            end
        end

        
        ampFactor = 10^(settings.powerIncreaseFactor/20); 
        
        localSig = (localCode .* localDataBit .* localCarr) * carrAmp(svIndex);
        localSig2 = (localCode2 .* localDataBit .* localCarr) * carrAmp(svIndex) * ampFactor * settings.SpoofingEnable * spoofingEnable(loopCnt);
        localSigSum = localSigSum + localSig +localSig2;


    end % svIndex = 1:length(satList)
    
    % Update Rx time and receiver position --------------------------------
    RxTime(1) = RxTime(2);
    RxTime(2) = RxTime(2) + blockTime;
    % Corresponding receiver positions 
    RxPosEcef(1,:)  = RxPosEcef(2,:);
    RxPosEcef(2,:)  = trajectory(loopCnt + 1,4:6);
    
    % Add nosie and filter ------------------------------------------------
    if settings.filterEn == 1
        % Generate WGN
        if  settings.fileType == 1
            gwnSamlple = wgn(1,blockSize,settings.gwnAmp^2,'linear');
        elseif settings.fileType == 2
            gwnSamlple = wgn(1,blockSize,settings.gwnAmp^2,'linear')...
                + 1i*wgn(1,blockSize,settings.gwnAmp^2,'linear');
        end
        % Add WGN
        localSigSum = localSigSum + gwnSamlple;
        % Do FE filtering 
        localSigSum = filter(coef,1,localSigSum);
    end
    
    % ADC quantization ----------------------------------------------------
    if strcmp(settings.dataType,'int8')
        localSigSum = localSigSum/max(real(localSigSum)) * 2^7;
        quantizedSig = int8(localSigSum);
        fwrite(fid,quantizedSig,settings.dataType);
    elseif strcmp(settings.dataType,'int16')
        localSigSum = localSigSum/max(real(localSigSum)) * 2^12;
        quantizedSig = int16(localSigSum); 
        fwrite(fid,quantizedSig,settings.dataType);
    end
    
    if settings.fileType == 2 
        quantizedSig1 = reshape([real(quantizedSig);imag(quantizedSig)],[],1);
        fwrite(fid,quantizedSig1,settings.dataType);
    end
    
end
%% clear environment
fclose(fid);
close(hwb) 

