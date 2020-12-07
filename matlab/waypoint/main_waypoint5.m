clc
clear all
close all

%%Define way points
P(1).X = [15 1]/2;
P(2).X = [8 5]/2;
P(3).X = [12 15]/2;
P(4).X = [-2, 10]/2;
P(5).X = [-4, -2]/2;
P(6).X = [0, -3]/2;
P(7).X = [-4,-8]/2;

% R_t = 0.2; % turn radius [m]
% dist_b = 0.1; % boundary distance
% q_b = 5 * (pi/180) ; %boundary angle (deg -> rad)
% V_mean = 15;% straight speed
% dist_v0 = 0.02*V_mean; %virtual targetv[m]
% alpha = 0.5; % rotation gain


R_t = 0.5; % turn radius [m]

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


ts = 0.1;                    % Sampling rate
dist_b = R_t*2; % boundary distance
q_b = 30 * (pi/180) ; %boundary angle (deg -> rad)
V_mean = 1;% straight speed m/s
dist_v0 = 1*V_mean; %virtual targetv [m]
alpha = 0.3; % rotation gain
t_filter = 0.1; %lowpass filter time[sec]

filterSize = fix(t_filter/ts);


t0 = 0;                      % Initial value
tf = 500;                     % Final value

t    = t0:ts:tf;
iter = fix((tf-t0)/ts);     % Number of iteration
k_fin = iter;


%%
clc
close all

X(1,:) = [P(1).X(1) - 1, P(1).X(2) + 1  L(1).angle+3.14]; %initial pose of mobile robot
q_low(1) = 0;



k_p = 1; %point number
turnMode = 0 ;% 0: straght, 1: turn
q_low(1) = 0;
for k = 1:iter
    k_num(k) = k_p;
    turnMode_num(k) = turnMode;
    
    %dist_P = LenPP(P(k_p+1).X,X(k,:)); %distance between mr and target point
    D_prj(k) = PrjLenP2L(P(k_p).X, P(k_p+1).X, X(k,:) ); %projection distance from line segment, when line on X-aix, +val means +y
    X_prj(k,:) = PrjP2L( P(k_p).X, P(k_p+1).X, X(k,1:2) );
    
        
    if abs(D_prj(k)) > dist_b % out-bounded, dis_v short. in-bounded, dis_v long.
        dist_v =  dist_v0/4;
    else
        dist_v =  dist_v0;
    end
    
    %virtual angle between Xc, Xprj, Xvir
    q_v(k) = atan2( D_prj(k), dist_v); % plus: when line on X-aix, +val means +y
    
    
    if k<filterSize
    q_v_low(k) = q_v(k);
    else
    q_v_low(k) = sum(q_v(k-filterSize+1:k));
    end
    
    X_vir(k,:) = X_prj(k,:) + dist_v*Rot2(L(k_p).angle);
    
    %reference angle
    q_ref(k) = L(k_p).angle - q_v_low(k);
    
    
    %angle difference
    del_q(k) = X(k,3) - q_ref(k);
    
    if del_q(k) < -pi
        del_q(k) = del_q(k) + 2*pi;
    elseif del_q(k) > pi
        del_q(k) = del_q(k) - 2*pi;
    end
    
    if norm( X(k,1:2) - P(k_p+1).X ) < R_t && turnMode == 0 
        k_p = k_p+1
        turnMode = 1;
%         [k k_p turnMode]
    end
    if k_p>1 && norm( X(k,1:2) - P(k_p).X ) > R_t && turnMode ==1
        turnMode = 0;
%         alpha = 0.3;
    end
    
    alpha = 0.1+0.4*abs(tanh(del_q(k)/q_b));
    
       
       
    if k_p == length(P)
        k_fin = k;
        t_fin = k*ts;
        break
    end
    
    
    
    
    
    
    
    
    

    
    %control input
    u(k,:) = (1-alpha)*V_mean*[1 1] + alpha*V_mean*[ -tanh(del_q(k)/q_b), +tanh(del_q(k)/q_b)]; %right left
    
    
    
    
    
    X(k+1,:) = rk4('mr_kinematics',X(k,:),u(k,:),ts,k);  % function y_n=rk4(fn,y,u,ts,j)
    if X(k+1,3) < -pi
        X(k+1,3) = X(k+1,3) + 2*pi;
    elseif X(k+1,3) > pi
        X(k+1,3) = X(k+1,3) - 2*pi;
    end
    
end


T=t0:ts:t_fin-ts;

figure(1)
subplot(211)
plot(T, q_v_low)
xlabel('time')
ylabel('virtual target angle, rad')

subplot(212)
plot(T(1:length(u(:,1))), u(:,1))
hold on
plot(T(1:length(u(:,2))), u(:,2))
hold off
xlabel('time')
ylabel('wheel velocity profile')






figureA = figure('color', 'w', 'Position', [200 100 800 800]);
plot(X(:,1), X(:,2), 'go');
hold on
for k = 1:length(P)
    plot( P(k).X(1), P(k).X(2), 'ro');
end

for k = 1: length(P)-1
    line( [P(k).X(1), P(k+1).X(1)], [P(k).X(2), P(k+1).X(2)], 'linewidth',2);
    %     plot(L(k).S(1), L(k).S(2), 'b*');
    %     plot(L(k).F(1), L(k).F(2), 'ko');
    %
end

% for k = 2: length(P)-1
%     plot( TP(k).arcPoint(1), TP(k).arcPoint(2), 'ro');
% end

% plot(X(:,1), X(:,2), 'b', 'linewidth',2,'LineStyle',':');

hold off
grid on
axis equal


frame = 2;
interval = fix(k_fin/frame);

figure_set = figure('color', 'w', 'Position', [1000 100 800 800]);

for j = 1:interval
    
    plot( P(1).X(1), P(1).X(2), 'k*');
    hold on
    for k = 2:length(P)
        plot( P(k).X(1), P(k).X(2), 'ro');
    end
    
    for k = 1: length(P)-1
        line( [P(k).X(1), P(k+1).X(1)], [P(k).X(2), P(k+1).X(2)], 'linewidth',2);
        %     plot(L(k).S(1), L(k).S(2), 'b*');
        %     plot(L(k).F(1), L(k).F(2), 'ko');
        %
    end
    
    plot(X_vir(frame*j,1), X_vir(frame*j,2), 'r*');
    plot(X_prj(frame*j,1), X_prj(frame*j,2), 'y*');
    plot(X(1:frame*j,1), X(1:frame*j,2), 'go');
    
    hold off
    grid on
    axis equal
    axis([ X(frame*j,1)-3 X(frame*j,1)+3 X(frame*j,2)-3 X(frame*j,2)+3 ])
    k_str =  num2str(k_num(frame*j));
    turnMode_str = num2str(turnMode_num(frame*j));
    del_q_str = num2str(del_q(frame*j)*180/pi);
    xlabel(['k_p = ' k_str ', turnMode = ' turnMode_str ', del_q = ' del_q_str ])
    movie_set(j) = getframe(figure_set);
    
    
end

% movie2avi(movie_set,'test_movie','fps', frame/5, 'quality',20);
