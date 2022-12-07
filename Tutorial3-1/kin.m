function	x= kin(l,q)

l1= l(1);	l2= l(2);
q1= q(1);	q12= q1 + q(2);
c1= cos(q1);	s1= sin(q1);
c12= cos(q12);	s12= sin(q12);

x=zeros(2,1);
x(1)= l1*c1 + l2*c12;
x(2)= l1*s1 + l2*s12;