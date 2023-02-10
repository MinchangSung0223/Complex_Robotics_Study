function [S_a,q_a]=get_qa(q)
    n = length(q);
    m = length(q{1});
    q = reshape(cell2mat(q),n*m,1);
    S_a_ = eye(n*m); % Selection Matrix
    S_a = S_a_(4:m:end,:);
    q_a = S_a*q;
end