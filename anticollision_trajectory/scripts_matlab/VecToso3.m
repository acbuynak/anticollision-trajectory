function so3mat = VecToso3(omg)
% from lynch
so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
end
