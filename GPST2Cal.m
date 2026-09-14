function [y,mo,d,hh,mm,ss] = GPST2Cal(week, sow)
% Convert GPST week + seconds-of-week to calendar date (GPS timescale)
% MATLAB R2018b compatible (datenum/datevec).
gps0 = datenum(1980,1,6,0,0,0); % GPS epoch
dn = gps0 + (week*604800 + sow)/86400;
v = datevec(dn);
y=v(1); mo=v(2); d=v(3); hh=v(4); mm=v(5); ss=v(6);
end
