clc
clear all
close all

%%Define way points
P(1).X = [15 1];
P(2).X = [8 5];
P(3).X = [12 15];
P(4).X = [-2, 10];
P(5).X = [-4, -2];
P(6).X = [0, -3];
P(7).X = [-4,-8];

R_t = 1.1; % turn radius
dist_b = 1; % boundary distance
q_b = 5 * (pi/180) ; %boundary angle (deg -> rad)

%%calculate Line segments
for k = 1:length(P)-1
    L(k).S = P(k).X;
    L(k).F = P(k+1).X;
    L(k).angle = atan2( P(k+1).X(2)-P(k).X(2), P(k+1).X(1)-P(k).X(1) );
    L(k).angle_deg = L(k).angle/pi*180;
    
end

%calculate TurnPoints
for k = 1: length(P)-2
    TP(k+1).X = P(k+1).X;
    TP(k+1).diffAngle =  (L(k+1).angle - L(k).angle);
    
    if TP(k+1).diffAngle < -pi
        TP(k+1).diffAngle = TP(k+1).diffAngle + 2*pi;
    elseif TP(k+1).diffAngle > pi
        TP(k+1).diffAngle = TP(k+1).diffAngle - 2*pi;
    end
    
    TP(k+1).diffAngle_deg = TP(k+1).diffAngle /pi*180;
    
    
    TP(k+1).turnAngle = abs(TP(k+1).diffAngle);
    TP(k+1).q1 = pi - TP(k+1).turnAngle;
    TP(k+1).arcDist = R_t/sin( TP(k+1).q1/2 );
    TP(k+1).tanDist = R_t/tan( TP(k+1).q1/2 );
    
    L(k).F = L(k).F - TP(k+1).tanDist * Rot2( L(k).angle );
    
    if TP(k+1).diffAngle <0 %Right turn
        TP(k+1).turn = 'R';
        TP(k+1).q2 = L(k+1).angle - TP(k+1).q1/2; %turn center to turn point
        L(k+1).S = L(k).F + R_t* Rot2(L(k).angle -pi/2) +  R_t*  Rot2( L(k).angle + pi/2 + TP(k+1).diffAngle) ;
        
    end
    if TP(k+1).diffAngle >0 %Left turn
        TP(k+1).turn = 'L';
        TP(k+1).q2 = L(k+1).angle + TP(k+1).q1/2;%turn center to turn point
        L(k+1).S = L(k).F + R_t*( Rot2(L(k).angle +pi/2) ) + R_t* Rot2(L(k).angle -pi/2 + TP(k+1).diffAngle) ;
    end
    
    
    TP(k+1).arcPoint = P(k+1).X + TP(k+1).arcDist * Rot2(TP(k+1).q2);
    
end






t0 = 0;                      % Initial value
ts = 0.1;                    % Sampling rate
tf = 70;                     % Final value

t    = t0:ts:tf;
iter = fix((tf-t0)/ts);     % Number of iteration

%%
clc
close all

X(1,:) = [P(1).X(1), P(1).X(2)+0.2,  L(1).angle+0.1]; %initial pose of mobile robot


V_mean = 0.4;
k_p = 1; %point number
turnMode = 0 % 0: straght, 1: turn

for k = 1:iter
    
    dist_P = LenPP(P(k_p+1).X,X(k,:)); %distance between mr and target point
    D_prj(k) = PrjLenP2L(P(k_p).X, P(k_p+1).X, X(k,:) ); %projection distance from line segment, when line on X-aix, +val means +y
    q_del = X(k,3) - L(k_p).angle;
    
    if dist_P < R_t && turnMode ==0
        turnMode = 1
    end
    
    if turnMode ==1 && q_del < q_b
        k_p = k_p +1
        turnMode = 0
    end
    
    
    
    
    
    %right left velocity input
    if turnMode == 0
        u(k,:) = V_mean*[1 1] + V_mean/4*[ -( D_prj(k)/dist_b), +(D_prj(k)/dist_b)];
    else
        u(k,:) = V_mean/2*[1 1] + V_mean/2*[ -1 +1];
    end
    
    
    
    X(k+1,:) = rk4('mr_kinematics',X(k,:),u(k,:),ts,k);  % function y_n=rk4(fn,y,u,ts,j)
    if X(k+1,3) < -pi
        X(k+1,3) = X(k+1,3) + 2*pi;
    elseif X(k+1,3) > pi
        X(k+1,3) = X(k+1,3) - 2*pi;
    end
    
end


figure(1)
plot(D_prj)






figureA = figure('color', 'w', 'Position', [1500 100 800 800]);
plot( P(1).X(1), P(1).X(2), 'k*');
hold on
for k = 2:length(P)
    plot( P(k).X(1), P(k).X(2), 'ro');
end

for k = 1: length(P)-1
    line( [P(k).X(1), P(k+1).X(1)], [P(k).X(2), P(k+1).X(2)]);
    plot(L(k).S(1), L(k).S(2), 'b*');
    plot(L(k).F(1), L(k).F(2), 'ko');
    
end

for k = 2: length(P)-1
    plot( TP(k).arcPoint(1), TP(k).arcPoint(2), 'go');
end

% plot(X(:,1), X(:,2), 'b', 'linewidth',2,'LineStyle',':');
plot(X(:,1), X(:,2), 'bo');

hold off
grid on
axis equal

























