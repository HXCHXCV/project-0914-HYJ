function signalGenerating_v1_SingPreset(settings)
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
state = [1,2,3,4]; %状态选择，真实轨迹采用1和2，对应静止和运动，   欺骗采用3-虚拟轨迹运动

%真实轨迹
s = 3*settings.SpoofingStart_Position/1000;
StartPotion_au_XYZ = settings.StartPotion_au_XYZ ;
[ECEFx,ECEFy,ECEFz,lat,lon,alt] = trajectorySimulation_XYZ(settings,trajectoryTime,state(2),StartPotion_au_XYZ,s);  %接收机匀速直线运动  补偿s
length_time = length(ECEFx);
trajectory = zeros(length_time,6);
for index = 1: length_time                                           
      trajectory(index,1:6) = [lat(index), lon(index), alt(index),ECEFx(index),ECEFy(index),ECEFz(index)]; 
end

%欺骗源轨迹
StartPotion_sp_XYZ = settings.StartPotion_sp_XYZ;
[ECEFx2,ECEFy2,ECEFz2,lat2,lon2,alt2] = trajectorySimulation_XYZ(settings, trajectoryTime,state(1),StartPotion_sp_XYZ,0); %欺骗源静止
trajectory2 = zeros(length_time,6);
for index = 1: length_time                                            
      trajectory2(index,1:6) = [lat2(index), lon2(index), alt2(index),ECEFx2(index),ECEFy2(index),ECEFz2(index)]; 
end

%欺骗虚拟轨迹
s3 = 4*settings.SpoofingStart_Position/1000;
StartPotion_sp_flase_XYZ = settings.StartPotion_sp_flase_XYZ;
[ECEFx3,ECEFy3,ECEFz3,lat3,lon3,alt3] = trajectorySimulation_XYZ(settings, trajectoryTime,state(3),StartPotion_sp_flase_XYZ,s3); %虚拟轨迹静止
trajectory3 = zeros(length_time,6);
for index = 1: length_time                                            
      trajectory3(index,1:6) = [lat3(index), lon3(index), alt3(index),ECEFx3(index),ECEFy3(index),ECEFz3(index)]; 
end

%% Read ephemeris file ============================================
if settings.rinexVersion == 2
    [eph,ionoutc] = rinexeV2(settings.rinexfile);    %生成星历这一步需要注意什么吗？比方说怎么生成的？格式是什么之类的
elseif settings.rinexVersion == 3
    [eph,ionoutc] = rinexeV3(settings.rinexfile);
end
ephRaw = eph;

%% Determine the visible satellite list ===========================
% Starting time of the simulation, corresponding to the first positions 
% in the trajectory 
% startTime = eph(1).toc;       
startTime = eph(1).toc + 14*60*60;    %14点的PRN  1、3、6、7、13、16、18、21、28、31。

eph = selectEphemerisByPrn_GPS(ephRaw,startTime);

% Initail position of the receiver trajectory
RxPosEcef = trajectory(1,4:6);  %接收机初始位置XYZ
RxPosEcef2 = trajectory2(1,4:6);  %欺骗源初始位置XYZ
RxPosEcef3 = trajectory3(1,4:6);  %虚拟轨迹初始位置XYZ

% Visible satellite list (PRN#)
[satList,elevation,azimuth] = getVisibleSat(eph,startTime,RxPosEcef',settings); %计算卫星俯仰角1
% [~,elevation2,azimuth2] = getVisibleSat(eph,startTime+1,RxPosEcef',settings); %计算卫星俯仰角2
%% Sky plot and C/N0 seting========================================
figure;set(gcf,'color','w');
skyPlot(azimuth',elevation',satList');  %卫星天空图
% skyPlot2(azimuth',elevation',satList');  %卫星天空图

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
            double(bitget(navBits(svIndex).dataWord(bitindex),30:-1:1)) * 2 - 1;  %???按理应该是1，0分别转化为-1，1
%         navBits(svIndex).dataBit((bitindex-1) * 30 + 1 : bitindex * 30) = ...
%             double(bitget(navBits(svIndex).dataWord(bitindex),30:-1:1)) * (-2) + 1;  
    end
end


for svIndex = 1:length(satList)
    PRN = satList(svIndex);
    % Genaerate Nav message according to the ephemeris
    navBits2(svIndex).frameMsg = eph2sbf(eph(PRN),ionoutc); %#ok<*SAGROW>
    
    % Time to generate Nav Messages
    navBits2(svIndex).ephTime.second = startTime;
    navBits2(svIndex).ephTime.weekNrm = eph(1).weekNrm;
    
    % Modulate WN, TOW and CRC into Nav messages
    navBits2(svIndex).dataWord = uint32(zeros(1,60));
    [navBits2(svIndex).dataWord, navBits2(svIndex).refTime] = ...
        generateNavMsg(navBits2(svIndex).frameMsg,navBits2(svIndex).ephTime,1,navBits2(svIndex).dataWord);
    
    % Extract Nav data bits from nav message
    navBits2(svIndex).dataBit = zeros(1,30*60);
    for bitindex = 1:60
        navBits2(svIndex).dataBit((bitindex-1) * 30 + 1 : bitindex * 30) = ...
            double(bitget(navBits2(svIndex).dataWord(bitindex),30:-1:1)) * 2 - 1;  %???按理应该是1，0分别转化为-1，1
%         navBits(svIndex).dataBit((bitindex-1) * 30 + 1 : bitindex * 30) = ...
%             double(bitget(navBits(svIndex).dataWord(bitindex),30:-1:1)) * (-2) + 1;  
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

% Initialize the two complete RINEX OBS streams. Their epoch selection is
% independent of spoofingEnable; both streams start at the first block.
obsEnable = isfield(settings,'obsEnable') && logical(settings.obsEnable);
if obsEnable
    if ~isscalar(settings.obsInterval) || ~isfinite(settings.obsInterval) || ...
            settings.obsInterval <= 0
        error('settings.obsInterval must be a positive finite scalar.');
    end

    obsStep = round(settings.obsInterval/blockTime);
    if obsStep < 1 || abs(obsStep*blockTime - settings.obsInterval) > 1e-12
        error('settings.obsInterval must be an integer multiple of blockTime (%.12g s).', blockTime);
    end

    obsEpochCount = floor((iterCnt - 1)/obsStep) + 1;
    emptyObsEpoch = struct('week',[],'sow',[],'RxPos',[],'sat',[], ...
        'C1C',[],'L1C',[],'D1C',[],'S1C',[]);
    directObsEpochs = repmat(emptyObsEpoch, 1, obsEpochCount);
    virtualObsEpochs = repmat(emptyObsEpoch, 1, obsEpochCount);
    obsCnt = 0;

    obsSat = cell(length(satList),1);
    for svIndex = 1:length(satList)
        obsSat{svIndex} = sprintf('G%02d', satList(svIndex));
    end

    % Fixed integer ambiguity per PRN, shared by Direct and Virtual. This
    % deterministic initialization does not alter the IF noise RNG state.
    obsAmbiguityCycles = -16500 + 1000*(1:32);
end

% 欺骗加入时刻开关
spoofingEnable = zeros(1,iterCnt);
spoofingEnable(settings.SpoofingStart_Position+1:end) = 1;

% Initialize Rx Time and Rx postions 
RxTime(1) = startTime;
RxTime(2) = RxTime(1) + blockTime;
RxPosEcef(1:2,:)  = trajectory(1:2,4:6);

RxTime2(1) = startTime;
RxTime2(2) = RxTime2(1) + blockTime;
RxPosEcef2(1:2,:)  = trajectory2(1:2,4:6);

% RxTime3(1) = startTime;
% RxTime3(2) = RxTime3(1) + blockTime;
RxPosEcef3(1:2,:)  = trajectory3(1:2,4:6);

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

    isObsEpoch = obsEnable && (mod(loopCnt - 1, obsStep) == 0);
    if isObsEpoch
        directC1C = zeros(length(satList),1);
        directL1C = zeros(length(satList),1);
        directD1C = zeros(length(satList),1);
        directS1C = zeros(length(satList),1);
        virtualC1C = zeros(length(satList),1);
        virtualL1C = zeros(length(satList),1);
        virtualD1C = zeros(length(satList),1);
        virtualS1C = zeros(length(satList),1);
    end

    if spoofingEnable(loopCnt) ==1
       Delay = zeros(1,length(satList)); 
       Delay = Delay + 0;       
    else
       Delay = zeros(1,length(satList));  
    end
    
    % wait bar ------------------------------------------------------------
    if (rem(loopCnt, 10) == 0)
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
        delay = Delay(svIndex);
        
        %迭代发射时
        [TxTime,satClkErr,travelTime] = GetTravelTime(RxTime,RxPosEcef,eph(PRN),settings);  %真实轨迹
        [TxTime2,satClkErr2,travelTime2] = GetTravelTime_deltaT2(RxTime2,RxPosEcef2,eph(PRN),settings,RxPosEcef,RxPosEcef3,delay); %欺骗虚拟---带转发

        % include tgd, clock error and relativistic effect 
        TxTime = TxTime + satClkErr;
        TxTime2 = TxTime2 + satClkErr2;

        if isObsEpoch
            [directC1C(svIndex),directL1C(svIndex),directD1C(svIndex),directS1C(svIndex)] = ...
                calcObsFromTxTime_GPSL1CA(RxTime,TxTime,travelTime,satClkErr, ...
                obsAmbiguityCycles(PRN),CNoValuesdB(svIndex),settings);
            [virtualC1C(svIndex),virtualL1C(svIndex),virtualD1C(svIndex),virtualS1C(svIndex)] = ...
                calcObsFromTxTime_GPSL1CA(RxTime2,TxTime2,travelTime2,satClkErr2, ...
                obsAmbiguityCycles(PRN),CNoValuesdB(svIndex),settings);
        end
        
        % generate local code, carrier and Nav data -----------------------
        sapcing = (TxTime(2) - TxTime(1))/blockSize;
        TxTimeSample = linspace(TxTime(1),TxTime(2) - sapcing,blockSize);
        sapcing = (RxTime(2) - RxTime(1))/blockSize;
        RxTimeSample = linspace(RxTime(1),RxTime(2) - sapcing,blockSize);
        
        spacing2 = (TxTime2(2) - TxTime2(1))/blockSize;
        TxTimeSample2 = linspace(TxTime2(1),TxTime2(2) - spacing2,blockSize);
        spacing2 = (RxTime2(2) - RxTime2(1))/blockSize;
        RxTimeSample2 = linspace(RxTime2(1),RxTime2(2) - spacing2,blockSize);
        
        % local carrier samples
        if  settings.fileType == 1
            localCarr = cos(carrFreqRad * TxTimeSample - localOsFreq * RxTimeSample);
            localCarr2 = cos(carrFreqRad * TxTimeSample2 - localOsFreq * RxTimeSample2);
        elseif settings.fileType == 2
            localCarr = exp(-1i*(carrFreqRad * TxTimeSample - localOsFreq * RxTimeSample));
            localCarr2 = exp(-1i*(carrFreqRad * TxTimeSample2 - localOsFreq * RxTimeSample2));
        end
        
        % Local code samples
        codePhase = TxTimeSample * settings.codeFreqBasis;
        codeIndex = ceil(rem(codePhase,settings.codeLength)) + 1;
        localCode = caCodeTable(svIndex,codeIndex);
        
        codePhase2 = TxTimeSample2 * settings.codeFreqBasis;
        codeIndex2 = ceil(rem(codePhase2,settings.codeLength)) + 1;
        localCode2 = caCodeTable(svIndex,codeIndex2);
        
        % Nav bit samples
        dataTime = TxTimeSample - (navBits(svIndex).refTime - 6);
        bitPhase = ceil(dataTime/bitPeriod);
        localDataBit = navBits(svIndex).dataBit(bitPhase);
        
        dataTime2 = TxTimeSample2 - (navBits2(svIndex).refTime - 6);
        bitPhase2 = ceil(dataTime2/bitPeriod);
        localDataBit2 = navBits2(svIndex).dataBit(bitPhase2);
        
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
        
        if (TxTime2(end) + 1) > (navBits2(svIndex).refTime + 30)
            % Time to generate Nav Messages
            navBits2(svIndex).ephTime.second = navBits2(svIndex).refTime + 30.5;
                        
            % Modulate WN, TOW and CRC into Nav messages
            [navBits2(svIndex).dataWord, navBits2(svIndex).refTime] = ...
                generateNavMsg(navBits2(svIndex).frameMsg,navBits2(svIndex).ephTime, ...
                0,navBits2(svIndex).dataWord);
            
            % Extract Nav data bits from nav message
            for bitindex = 1:60
                navBits2(svIndex).dataBit((bitindex-1) * 30 + 1 : bitindex * 30) = ...
                double(bitget(navBits2(svIndex).dataWord(bitindex),30:-1:1)) * 2 - 1;
            end
        end

        ampFactor = 10^(settings.powerIncreaseFactor/20); % 欺骗功率因子
         
     %%   半通道
%         Indicator_sp = [1,1,1,1,0,0,0,0,0,0]; %前四个分别是PRN1、PRN3、PRN6和PRN7
%         localSig = (localCode .* localDataBit .* localCarr) * carrAmp(svIndex);
%         localSig2 = (localCode2 .* localDataBit2 .* localCarr2) * carrAmp(svIndex) * ampFactor * settings.SpoofingEnable* spoofingEnable(loopCnt)*Indicator_sp(svIndex) ;
% 
%         localSigSum = localSigSum + localSig +localSig2;
%         
     %% 全通道
        localSig = (localCode .* localDataBit .* localCarr) * carrAmp(svIndex) * (1 - spoofingEnable(loopCnt));
        localSig2 = (localCode2 .* localDataBit2 .* localCarr2) * carrAmp(svIndex) * ampFactor * settings.SpoofingEnable * spoofingEnable(loopCnt);
        
        localSigSum = localSigSum + localSig +localSig2;
       
    end % svIndex = 1:length(satList)

    if isObsEpoch
        obsCnt = obsCnt + 1;
        obsSow = startTime + (loopCnt - 1)*blockTime;

        directObsEpochs(obsCnt).week = eph(1).weekNrm;
        directObsEpochs(obsCnt).sow = obsSow;
        directObsEpochs(obsCnt).RxPos = RxPosEcef(1,:);
        directObsEpochs(obsCnt).sat = obsSat;
        directObsEpochs(obsCnt).C1C = directC1C;
        directObsEpochs(obsCnt).L1C = directL1C;
        directObsEpochs(obsCnt).D1C = directD1C;
        directObsEpochs(obsCnt).S1C = directS1C;

        virtualObsEpochs(obsCnt).week = eph(1).weekNrm;
        virtualObsEpochs(obsCnt).sow = obsSow;
        virtualObsEpochs(obsCnt).RxPos = RxPosEcef(1,:);
        virtualObsEpochs(obsCnt).sat = obsSat;
        virtualObsEpochs(obsCnt).C1C = virtualC1C;
        virtualObsEpochs(obsCnt).L1C = virtualL1C;
        virtualObsEpochs(obsCnt).D1C = virtualD1C;
        virtualObsEpochs(obsCnt).S1C = virtualS1C;
    end
    
    % Update Rx time and receiver position --------------------------------
    RxTime(1) = RxTime(2);
    RxTime(2) = RxTime(2) + blockTime;
    % Corresponding receiver positions 
    RxPosEcef(1,:)  = RxPosEcef(2,:);
    RxPosEcef(2,:)  = trajectory(loopCnt + 1,4:6);
    
    RxTime2(1) = RxTime2(2);
    RxTime2(2) = RxTime2(2) + blockTime;
    RxPosEcef2(1,:)  = RxPosEcef2(2,:);
    RxPosEcef2(2,:)  = trajectory2(loopCnt + 1,4:6);
    
    RxPosEcef3(1,:)  = RxPosEcef3(2,:);
    RxPosEcef3(2,:)  = trajectory3(loopCnt + 1,4:6);
    
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

if obsEnable
    meta.markerName = settings.markerName;
    meta.markerNumber = settings.markerNumber;
    meta.observer = settings.observerAgency;
    meta.agency = '';
    meta.receiverNumber = '';
    meta.receiverType = settings.receiverType;
    meta.receiverVersion = '';
    meta.antennaNumber = '';
    meta.antennaType = settings.antennaType;
    meta.interval = settings.obsInterval;
    meta.leapSeconds = ionoutc.dtls;
    meta.timeSystem = 'GPS';
    meta.approxPosXYZ = trajectory(1,4:6);
    meta.antDeltaHEN = settings.antDeltaHEN;
    meta.rcvClockOffsAppl = 0;

    writeRinex302Obs_GPS_L1CA(settings.directObsFile,meta,directObsEpochs);
    writeRinex302Obs_GPS_L1CA(settings.virtualObsFile,meta,virtualObsEpochs);
    fprintf('RINEX OBS exported: %s and %s (epochs=%d)\n', ...
        settings.directObsFile,settings.virtualObsFile,obsCnt);
end
%% clear environment
fclose(fid);
close(hwb) 

