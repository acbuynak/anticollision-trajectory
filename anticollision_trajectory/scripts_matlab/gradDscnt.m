
function xseq = gradDscnt(Start,End,Pfield,coordaxes,Gama)
% consider a startpoint, Start, and and ending point, End, in n-dimensional
% space with n coordinates to each. An n-dimensional field of weights to be
% minimized (n-dimensional double)

nax = size(coordaxes);
[a1,a2,a3] = size(Pfield);
% nPf = numel(Pfield);
j0 = Start;
je = End;
Pg = sym('Pg_',[1 n],'real');

[Pg(1),Pg(2), Pg(3)] = gradient(Pfield);

toll = Gama.toll;
gama = Gama.gama;

error = inf;

jn = j0;

ii = 1;
xseq(:,ii) = jn;
while error >= toll
    disp(error)
    S1n = find(coordaxes(1).dir == jn(1));
    S2n = find(coordaxes(2).dir == jn(2));
    S3n = find(coordaxes(3).dir == jn(3));
    In = sub2ind([a1,a2,a3], S1n, S2n, S3n);

    Pn = Pfield(In);
    Pn_1 = (Pn) - (gama* (Pg1g(In) + Pg2g(In) + Pg3g(In)));
    
    egrid1 = abs((Pfield - Pn_1));
    
    [~,indices] = min(egrid1(:));

    ni = numel(indices);
    
    if ni == 1
        Pn_1 = Pfield(indices);
        [S1n_1,S2n_1,S3n_1] = ind2sub([a1,a2,a3],indices);
        jn_1 = [coordaxes(1).dir(S1n_1),coordaxes(2).dir(S2n_1),coordaxes(3).dir(S3n_1),0,0,0];
        
    else
        disp('multy nodes')
        escape
    end
    
    error = double(sqrt(sum((je' - jn_1').^2)));
    
    ii = ii+1;
    xseq(:,ii) = jn_1;
    jn = jn_1;
    Pn = Pn_1;


end

end