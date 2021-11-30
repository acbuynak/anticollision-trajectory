function skew = vect2skew(vect)
% takes a 3 dimensional vector and converts it to its skew symetric
% representation
    skew = [    0   ,-vect(3), vect(2);
             vect(3),   0    ,-vect(1);
            -vect(2), vect(1),   0];
end