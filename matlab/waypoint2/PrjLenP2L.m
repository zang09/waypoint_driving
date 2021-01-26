function out = PrjLenP2L(X1, X2, Xc)

x1 = X1(1);
y1 = X1(2);
x2 = X2(1);
y2 = X2(2);

xc = Xc(1);
yc = Xc(2);

delx = x2 - x1;
dely = y2 - y1;
delL = sqrt( dely^2 + delx^2);
sinq = dely / delL;
cosq = delx / delL;

out = -(xc - x1)*sinq + (yc - y1)*cosq;

end

