function Jc=ConstarintJacobian(J)
    Jc1 = [[1,1]; J{1}];
    Jc2 = [[1,1]; J{2}];
    O = zeros(size(Jc1));
    Jc = [Jc1 O; O Jc2];
end