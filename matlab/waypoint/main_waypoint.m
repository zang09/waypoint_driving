clc
clear all
close all

%%Define way points
P(1).X = [10 1];
P(2).X = [8 5];
P(3).X = [12 15];
P(4).X = [-2, 10];
P(5).X = [-4, -2];
P(6).X = [0, -3];
P(7).X = [-4,-8];

Rc = 1.1; % arc radius
%%calculate Line segments
for i = 1:length(P)-1
    L(i).S = P(i).X;
    L(i).F = P(i+1).X;
    L(i).angle = atan2( P(i+1).X(2)-P(i).X(2), P(i+1).X(1)-P(i).X(1) );
    L(i).angle_deg = L(i).angle/pi*180;
    
end

%calculate TurnPoints
for i = 1: length(P)-2
    TP(i+1).X = P(i+1).X;
    TP(i+1).diffAngle =  (L(i+1).angle - L(i).angle);
    
    if TP(i+1).diffAngle < -pi
        TP(i+1).diffAngle = TP(i+1).diffAngle + 2*pi;
    elseif TP(i+1).diffAngle > pi
        TP(i+1).diffAngle = TP(i+1).diffAngle - 2*pi;
    end
    
    TP(i+1).diffAngle_deg = TP(i+1).diffAngle /pi*180;
    
    if TP(i+1).diffAngle <0 %Right turn
        TP(i+1).turn = 'R';
        TP(i+1).turnAngle = pi + TP(i+1).diffAngle;
        arcAngle = L(i+1).angle - TP(i+1).turnAngle/2;
    end
    if TP(i+1).diffAngle >0 %Left turn
        TP(i+1).turn = 'L';
        TP(i+1).turnAngle = pi - TP(i+1).diffAngle;
        arcAngle = L(i+1).angle + TP(i+1).turnAngle/2;
    end
    TP(i+1).arcDist = Rc/sin( TP(i+1).turnAngle/2 );
    TP(i+1).tanDist = Rc/tan( TP(i+1).turnAngle/2 );
    TP(i+1).arcPoint = P(i+1).X + TP(i+1).arcDist * [cos( arcAngle) , sin( arcAngle )];
        
    L(i).F = L(i).F - TP(i+1).tanDist * [cos( L(i).angle ) , sin( L(i).angle  )];
    L(i+1).S = L(i+1).S + TP(i+1).tanDist * [cos( L(i+1).angle ) , sin( L(i+1).angle  )];
    
    
    
     
end


figureA = figure('color', 'w', 'Position', [700 100 800 800]);
plot( P(1).X(1), P(1).X(2), 'k*');
hold on
for i = 2:length(P)
    plot( P(i).X(1), P(i).X(2), 'ro');
end

for i = 1: length(P)-1
    line( [P(i).X(1), P(i+1).X(1)], [P(i).X(2), P(i+1).X(2)]);
    plot(L(i).S(1), L(i).S(2), 'bo');
    plot(L(i).F(1), L(i).F(2), 'bo');
    
end

for i = 2: length(P)-1
    plot( TP(i).arcPoint(1), TP(i).arcPoint(2), 'go');
end
hold off
grid on
axis equal
