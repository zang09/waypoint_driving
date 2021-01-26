function Xout = PrjP2L(X1,X2,Xc)

x1 = X1(1);
x2 = X2(1);
y1 = X1(2);
y2 = X2(2);

q = atan2( y2-y1, x2-x1);
Xcc = Xc - X1;
ycc = -Xcc(1)*sin(q) + Xcc(2)*cos(q);


Xout = Xc + ycc* [sin(q), -cos(q)];
end

