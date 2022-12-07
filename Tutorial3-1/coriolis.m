function	C= coriolis(I,m,l,cl,Q,Qv)

s2= sin(Q(2));
C= zeros(2,1);

p1= -m(2)*l(1)*cl(2)*s2;
p2= m(2)*l(1)*cl(2)*s2;

C(1)= p1*((Qv(2)^2)+ 2*Qv(1)*Qv(2));
C(2)= p2*Qv(1)^2;



