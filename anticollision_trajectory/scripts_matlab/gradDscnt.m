function xseq = gradDscnt(Startj,Endj,Startc,Endc,Potential,Gradient,coordaxes,Gama,Tf)
% consider a startpoint, Start, and and ending point, End, in n-dimensional
% space with n coordinates to each. An n-dimensional field of weights to be
% minimized (n-dimensional double)
r = rateControl(80);
nax = size(coordaxes,2);
a1 = length(coordaxes(1).dir);
a2 = length(coordaxes(2).dir);
a3 = length(coordaxes(3).dir);
% nPf = numel(Pfield);
j0 = Startj;
c0 = double(Startc);
ce = double(Endc);
Tfa = c0;
Tfe = ce;

toll = Gama.toll;
gama = Gama.gama;

dTf = (Tfe - Tfa);
error = double(sqrt(sum(dTf.^2)));

jn(:,1) = j0(1:3);

ii = 1;
jj = ii;
xseq(:,ii) = j0;
figure(98)
subplot(1,3,1)
h = plot(ii,error,'b-');grid on;xlabel('iter');ylabel('err');
subplot(1,3,2)
h2 = plot3(c0(1),c0(2),c0(3)); grid on
subplot(1,3,3)
h3 = plot3(jn(1),jn(2),jn(3)); grid on
    while error >= toll
        disp(['err = ', num2str(error)])
        
        S1n = find(coordaxes(1).dir == jn(1));
        S2n = find(coordaxes(2).dir == jn(2));
        S3n = find(coordaxes(3).dir == jn(3));
         
        Pn = Potential(coordaxes(1).dir(S1n),coordaxes(1).dir(S2n),coordaxes(1).dir(S3n));

        jn_1 = double((jn) - (((gama*1) * Gradient(jn(1),jn(2),jn(3)))));
        disp([jn,jn_1])
        
        egrid1 = coordaxes(1).dir - jn_1(1);
        egrid2 = coordaxes(2).dir - jn_1(2);
        egrid3 = coordaxes(3).dir - jn_1(3);

        Tfa = Tf(jn_1(1),jn_1(2),jn_1(3),0,0,0)./1000;
        dTf = (Tfe - Tfa);
        
        error = double(sqrt(sum(dTf.^2)));
        
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
        h2.XData(ii)= Tfa(1)/1000;h2.YData(ii) = Tfa(2)/1000;h2.ZData(ii) = Tfa(3)/1000;title(num2str(ii));
        h3.XData(ii)= jn(1);h3.YData(ii) = jn(2)/1000;h3.ZData(ii) = jn(3)/1000;title(num2str(ii));
        waitfor(r)
        clc
    end

end