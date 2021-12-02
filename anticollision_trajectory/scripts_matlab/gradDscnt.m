function xseq = gradDscnt(Startj,Endj,Startc,Endc,Potential,Gradient,coordaxes,Gama,Tf)
% consider a startpoint, Start, and and ending point, End, in n-dimensional
% space with n coordinates to each. An n-dimensional field of weights to be
% minimized (n-dimensional double)
r = rateControl(120);
nax = size(coordaxes,2);
a1 = length(coordaxes(1));
a2 = length(coordaxes(2));
a3 = length(coordaxes(3));
% nPf = numel(Pfield);
j0 = Startj;
c0 = double(Startc)
ce = double(Endc)
Tfa = c0;
Tfe = ce;

toll = Gama.toll;
gama = Gama.gama;
(Tfe - Tfa)
dTf = (Tfe - Tfa)./1000;
error = double(norm(dTf));
pause
jn(:,1) = j0(1:3);

ii = 1;
jj = ii;
xseq(:,ii) = j0;
figure(98)
h = plot(ii,error,'b-');grid on;xlabel('iter');ylabel('err');
pause
    while error >= toll
        disp(['err = ', num2str(error)])
        
        S1n = find(coordaxes(1).dir == jn(1));
        S2n = find(coordaxes(2).dir == jn(2));
        S3n = find(coordaxes(3).dir == jn(3));
        In = sub2ind([a1,a2,a3], S1n, S2n, S3n);
        Pn = Pfield(In);

        jn_1 = (jn) - (gama* Gradient(jn(1),jn(2),jn(3)));
        disp([jn,jn_1,Pn])
        
        egrid1 = [coordaxes(1).dir - jn_1(1);
                  coordaxes(2).dir - jn_1(2);
                  coordaxes(3).dir - jn_1(3);];
        
        [~,indices] = min(egrid1);
        
        ni = numel(indices);
        
        if ni == 1
            [S1n_1,S2n_1,S3n_1] = ind2sub([a1,a2,a3],indices);
            jn_1 = [coordaxes(1).dir(S1n_1);
                    coordaxes(2).dir(S2n_1);
                    coordaxes(3).dir(S3n_1)];
        else
            disp('multy nodes')
            break
        end
        
        Tfa = Tf(jn_1(1),jn_1(2),jn_1(3),0,0,0);
        dTf = (Tfe - Tfa)./1000;
        error = double(norm(dTf));
        
        ii = ii+1;
        jj = jj+1;
        
        xseq(:,ii) = [jn_1;0;0;0];
        jn = jn_1;
    
        if jj == 10
            jj = 0;
        elseif ii > 1000
            disp('no min found')
            break
        end
        h.XData(ii)= ii;h.YData(ii) = error;title(num2str(ii));
        waitfor(r)
        clc
    end

end