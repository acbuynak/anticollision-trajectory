% Timescaling
function s = LSPB(W,tf,N)
% given the number of samples generates timescaling vectors s sd and sdd
    [s,sd,sdd,T] = trapveltraj(W,N,'EndTime',tf);
    s = struct('s',s,'sd',sd,'sdd',sdd,'T',T);
end