Gama.toll = 1e-3;
Gama.gama = 0.1;

%function [] = gradDscnt(Start,End,Pfield,axes,Gama)
% consider a startpoint, Start, and and ending point, End, in n-dimensional
% space with n coordinates to each. An n-dimensional field of weights to be
% minimized (n-dimensional double)

nax = size(axes);
[a1,a2,a3] = size(Pfield);
nPf = numel(Pfield);
j0 = Start(1:n); je = End((1:n));
c0 = Start(end-n:end); ce = End(end-n:end);
Pg = sym('Pg_',[1 n],'real');

[Pg1g,Pg2g,Pg3g] = gradient(Pfield);

toll = Gama.toll;
gama = Gama.gama;

error = inf;

jn = j0;

ii = 1;

while error >=tolerance
    
    [S1, S2, S3] = ind2sub([a1,a2,a3] ,find(grid == jn));
    I = sub2ind(nPf, S1, S2, S3);
    Pn = Pfield(S1,S2,S3);
   
    Pn_1 = (Pn) - (gama* (Pg1g(I) + Pg2g(I) + Pg3g(I)));
    
    egrid(:) = norm(Pfield(:) - Pn_1);

    [~,indices] = min(egrid(:));

    ni = numel(indices);
    
    if ni == 1
      [S1, S2, S3] =  ind2sub([a1,a2,a3] ,indices);
    else
        S1a(1:ni) = 0; S2a(1:ni) = 0; S3a(1:ni) = 0;


    
    end
    
    error = norm(jn_1 - jn);



end

%end