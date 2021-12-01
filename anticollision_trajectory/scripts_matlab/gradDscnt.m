function xseq = gradDscnt(Startj,Endj,Startc,Endc,Pfield,coordaxes,Gama,Tf)
% consider a startpoint, Start, and and ending point, End, in n-dimensional
% space with n coordinates to each. An n-dimensional field of weights to be
% minimized (n-dimensional double)
 r = rateControl(120);
nax = size(coordaxes,2);
[a1,a2,a3] = size(Pfield);
% nPf = numel(Pfield);
j0 = Startj;
je = Endj;
c0 = Startc;
ce = Endc;
Tfa = c0;
Tfe = ce;
% Pg = sym('Pg_',[1 nax],'real')

[Pg1g,Pg2g, Pg3g] = gradient(Pfield);

toll = Gama.toll;
gama = Gama.gama;

dTf = (Tfe - Tfa)./1000;
error = double(norm(dTf));

jn = j0;

ii = 1;
jj = ii;
xseq(:,ii) = jn;
figure(98)
h = plot(ii,error,'b-');grid on;xlabel('iter');ylabel('err');
    while error >= toll
        disp(['err = ', num2str(error)])
        
        S1n = find(coordaxes(1).dir == jn(1));
        S2n = find(coordaxes(2).dir == jn(2));
        S3n = find(coordaxes(3).dir == jn(3));
        In = sub2ind([a1,a2,a3], S1n, S2n, S3n);
    
        Pn = Pfield(In);
        Pn_1 = (Pn) - (gama* (Pg1g(In) + Pg2g(In) + Pg3g(In)));
        disp([Pn,Pn_1])
        % pause(0.5)
        egrid1 = abs((Pfield - Pn_1));
        
        [~,indices] = min(egrid1(:));
        
        ni = numel(indices);
        
        if ni == 1
            [S1n_1,S2n_1,S3n_1] = ind2sub([a1,a2,a3],indices);
            jn_1 = [coordaxes(1).dir(S1n_1),coordaxes(2).dir(S2n_1),coordaxes(3).dir(S3n_1),0,0,0];
        else
            disp('multy nodes')
            break
        end
        
        Tfa = Tf(jn_1(1),jn_1(2),jn_1(3),jn_1(4),jn_1(5),jn_1(6));
        dTf = (Tfe - Tfa)./1000;
        error = double(norm(dTf));
        
        ii = ii+1;
        jj = jj+1;
        
        xseq(:,ii) = jn_1;
        jn = jn_1;
    
        if jj == 10
            jj = 0;
        elseif ii > 1000
            break
        end
        h.XData(ii)= ii;h.YData(ii) = error;title(num2str(ii));
        waitfor(r)
        clc
    end

end