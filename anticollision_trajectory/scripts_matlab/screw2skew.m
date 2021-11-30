function skew = screw2skew(screw)
% 6x1 screw vector to skre symetric representation
    skew = [vect2skew(screw(1:3)),screw(4:6);0,0,0,0];
end
