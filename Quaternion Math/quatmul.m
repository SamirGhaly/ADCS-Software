function q=quatmul(q1,q2)
%% Rearrange Input
aw=q1(1);
ax=q1(2);
ay=q1(3);
az=q1(4);

bw=q2(1);
bx=q2(2);
by=q2(3);
bz=q2(4);
q1=[aw ax ay az];
q2=[bw bx by bz];
q=zeros(1,4);
q(1)=aw*bw-ax*bx-ay*by-az*bz;
q(2)=aw*bx+ax*bw+ay*bz-az*by; %i
q(3)=aw*by+ay*bw+az*bx-ax*bz; %j
q(4)=aw*bz+az*bw+ax*by-ay*bx; %k

q=[q(1) q(2) q(3) q(4)]';
end