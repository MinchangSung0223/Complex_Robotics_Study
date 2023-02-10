function q=get_q(q_a,q_p,S_a,S_p)
    q_= pinv(S_a)*q_a + pinv(S_p)*q_p;
    q={};
    n = length(q_a);
    m = length(q_p)/length(q_a)+1;
    for i =1:1:n
        q{i} = q_((i-1)*m+1:(i-1)*m+m)
    end
end