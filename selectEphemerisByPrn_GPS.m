function ephByPrn = selectEphemerisByPrn_GPS(ephRaw,targetTime)
%SELECTEPHEMERISBYPRN_GPS Build a PRN-indexed GPS ephemeris snapshot.
%   rinexeV2/rinexeV3 return records in NAV-file order. The simulator uses
%   eph(PRN), so select one record per GPS PRN for the simulation start time.

if isempty(ephRaw)
    error('Ephemeris input is empty.');
end

ephByPrn = repmat(makeBlankEphemeris(ephRaw(1)),1,32);
svprn = [ephRaw.svprn];

for prn = 1:32
    idx = find(svprn == prn);
    if isempty(idx)
        continue;
    end

    toe = [ephRaw(idx).toe];
    score = abs(arrayfun(@(x) check_t(targetTime - x), toe));

    if isfield(ephRaw,'svhealth')
        health = [ephRaw(idx).svhealth];
        if any(health == 0)
            score(health ~= 0) = score(health ~= 0) + 1e9;
        end
    end

    [~,best] = min(score);
    ephByPrn(prn) = ephRaw(idx(best));
end
end

function eph = makeBlankEphemeris(template)
eph = template;
fields = fieldnames(eph);
for k = 1:numel(fields)
    value = eph.(fields{k});
    if isnumeric(value)
        eph.(fields{k}) = NaN(size(value));
    end
end
eph.svprn = NaN;
end
