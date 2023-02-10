function J=Jacobian(l,q)
    for i = 1:1:length(l)
         li = l{i};
         qi = q{i};
         si1 = sin(qi(1));
         ci1 = cos(qi(1));
         si12 = sin(qi(1)+qi(2));
         ci12 = cos(qi(1)+qi(2));
         Ji = [-li(1)*si1-li(2)*si12 -li(2)*si12;
               li(1)*ci1+li(2)*ci12 li(2)*ci12];
         J{i} = Ji;
    end
end