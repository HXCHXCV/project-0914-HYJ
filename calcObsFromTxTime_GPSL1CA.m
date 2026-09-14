function [C1C,L1C,D1C,S1C] = calcObsFromTxTime_GPSL1CA( ...
    RxTime,TxTime,travelTime,satClkErr,ambiguityCycles,CNo_dBHz,settings)
% Build one GPS L1 C/A observation from the existing IF time endpoints.
% TxTime must already include the satellite clock, TGD and relativistic
% correction used by the IF carrier/code generation path.
% travelTime is the geometric/relay propagation time calculated internally
% by the same GetTravelTime call. Using travelTime - satClkErr is
% algebraically equivalent to RxTime - TxTime, but avoids cancellation when
% differencing millisecond endpoints represented as large GPS SOW values.

if any(size(TxTime) ~= size(RxTime)) || ...
        any(size(travelTime) ~= size(RxTime)) || ...
        any(size(satClkErr) ~= size(RxTime))
    error('RxTime, TxTime, travelTime and satClkErr must have the same size.');
end

lambdaL1 = settings.c/settings.carrFreqBasis;
pseudorangeEndpoints = settings.c*(travelTime - satClkErr);
dt = RxTime(2) - RxTime(1);

if dt <= 0
    error('RxTime endpoints must be strictly increasing.');
end

C1C = pseudorangeEndpoints(1);
L1C = C1C/lambdaL1 + ambiguityCycles;
D1C = -(pseudorangeEndpoints(2) - pseudorangeEndpoints(1))/(lambdaL1*dt);
S1C = CNo_dBHz;
end
