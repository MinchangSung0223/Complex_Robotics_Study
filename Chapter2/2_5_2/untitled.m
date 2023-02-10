N=1; %counter
Point=1; %secondary counter
Q=25;  %amount of data points the sphere has
r=20; %radius of the sphere 
[X,Y,Z] = sphere(Q); %generating coordinates
%count=Q+1;
count=length(X);
M=zeros(((Q+1)^2),3); %assigning a matrix M to speed up code.
for N=1:(count)
    M(((N-1)*count)+1:N*count,1)=r*X(N,:); %%rewriting to one matrix
    M(((N-1)*count)+1:N*count,2)=r*Y(N,:);
    M(((N-1)*count)+1:N*count,3)=r*Z(N,:);
    N=N+1;
end
%%end of code of sphere
count=sqrt(length(M));
count2=length(M);
seen=nan(count2,3); %filling a "seen" matrix in which the points are located that have been "seen".
dt=delaunayTriangulation(M(:,1),M(:,2));
    UU=1;
  for TT=1:count^2
      Point=M(TT,:)+[0.001 0.001 0.001];
      p=rand
      if p<=0.5  %selecting random points
          seen=Point;
          vertexId = nearestNeighbor(dt, seen(1),seen(2));
         tri11 = dt(vertexId, [1:end 1]);
         hold on
          patch(M(tri11,1), M(tri11,2), 'r', 'LineWidth',1, 'FaceColor','g')
          tri1(UU,:)=dt(vertexId,:);
          UU=UU+1;
      end
  end
triplot(dt);