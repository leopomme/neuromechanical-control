function	q= invKin(l,x);

l1= l(1);
l2= l(2);
l12= l1^2;
l22= l2^2;
x1= x(1);
x2= x(2);
xx= x1^2 + x2^2;

c2= (xx-l12-l22)/(2*l1*l2);
s2= sqrt(1-c2^2);
q2= atan2(s2,c2);

beta= atan2(x2,x1);
cPsi= (xx+l12-l22)/(2*l1*sqrt(xx));
psi= acos(cPsi);
if q2 < 0
	q1= beta + psi;
else
	q1= beta - psi;
end;

q= zeros(2,1);
q(1)= q1;q(2)= q2;