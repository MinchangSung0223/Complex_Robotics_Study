function [S_p,q_p]=get_qp(q)
    n = length(q);
    m = length(q{1});
    q = reshape(cell2mat(q),n*m,1);
    S_p = eye(n*m); % Selection Matrix
    S_p(4:m:end,:)=[];
    q_p = S_p*q;
end