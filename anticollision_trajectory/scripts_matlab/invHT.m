% Inverse Homogeneous Transform
function iHT = invHT(HT)
% takes a homogeneous transformation T and calculates its inverse using
% properties of T and not inv(T)
    
    R = HT(1:3,1:3);
    P = HT(1:3,4);
    
    iHT = [transpose(R),-transpose(R)*P;0,0,0,1];

end
