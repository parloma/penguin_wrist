%% TEST IK POSE
%IK Pinguino Real values

clc
clear all
close all

%% Fixed Point in Space

P4 = [0;0;36.57];

%% Inertial frame
RF0fixed = eye(3,3);

%% Desired pose (in rad)
roll = 0.0; 
pitch = 0.0;
yaw = 0.0;

%% Locus of desired Pose (CIRCLE)

locus_of_desired_pose = zeros(3,1);

vectorCircle_P4 = zeros(3,1);

Rtest = 6;
centerTest = P4 + [0;0;10];

i=1;
for t=0:pi/15:2*pi

%TEST Circle
locus_of_desired_pose(:,i) = centerTest + Rtest*cos(t)*RF0fixed(1:3,1) + Rtest*sin(t)*RF0fixed(1:3,2);
%normal to the plane
vectorCircle_P4(:,i) = locus_of_desired_pose(:,i) - P4;
vectorCircle_P4(:,i) = vectorCircle_P4(:,i)/norm(vectorCircle_P4(:,i));
i=i+1;
end

%% Compute R given only Zvector, gives infinitely solutions: Gram-Schimdt solution
%gives a stable Perpendicular vector to Zvector; hence the remaining one is
%computed as the cross product
for vectorNumber=1:length(vectorCircle_P4)
%vectorNumber = 20;

imin = 1;
for i=1:3
    if vectorCircle_P4(i,vectorNumber) < vectorCircle_P4(imin,vectorNumber)
        imin = i;
    end
end
imin = 2; %%bias of y axis to be as close as possible to the Y axis of the inertial frame: 
%needed to keep the yaw angle as small as possible. High yaw values yields
%high joints variations from the rest pose!!!!

Yvector = zeros(3,1);
dt    = vectorCircle_P4(imin,vectorNumber);

Yvector(imin) = 1;
%Yvector(2) = 1;
for i=1:3
    Yvector(i) = Yvector(i) -  dt*vectorCircle_P4(i,vectorNumber);
end

Yvector = Yvector./norm(Yvector);

R0fixedtoP4 = [cross(Yvector,vectorCircle_P4(:,vectorNumber)) ,Yvector,vectorCircle_P4(:,vectorNumber)];

%con R=Rz*Ry*Rx (quella convenzionale) su assi fissi quindi inizio da roll
%poi pitch ed infine yaw di RF0 fixed
%R0fixedtoP4 = rotz(rad2deg(yaw))*roty(rad2deg(pitch))*rotx(rad2deg(roll));

RPY = rot2rpy(R0fixedtoP4);

roll = deg2rad(RPY(1))
pitch = deg2rad(RPY(2))
yaw = deg2rad(RPY(3))


%Homogeneous transformation
T0fixedtoP4 = rd2tom(R0fixedtoP4, P4);

%% Triangle Vertices

%ALTO

% w.r.t. to RFP4

p4alto_rfp4 = [0;-14.87;0;1];

% w.r.t to RF0fixed

p4alto_rf0 =  T0fixedtoP4 * p4alto_rfp4;

%

%MEDIO

% w.r.t. to RFP4

p4medio_rfp4 = [-14.87*cosd(30);14.87*sind(30);0;1];

% w.r.t to RF0fixed

p4medio_rf0 =  T0fixedtoP4 * p4medio_rfp4;

%

%BASSO

% w.r.t. to RFP4

p4basso_rfp4 = [14.87*cosd(30);14.87*sind(30);0;1];

% w.r.t to RF0fixed

p4basso_rf0 =  T0fixedtoP4 * p4basso_rfp4;


%% Plane normal given by the desired pose

plane_n = R0fixedtoP4(1:3,3);


%% Equations Parameters definition
    %L 120-Angled: Alto Basso Medio creano una circonferenza con centro in Z=19.06 e raggio R nel piano XY w.r.t. RF0fixed

    %L ALTO (Medio e Basso formano lo stesso cerchio )
    l1a = 19.33;
    l2a = 22.01;
    psi = deg2rad(120);
    %Raggio cerchio generato da L-120angled nel piano XY
    Rxy = l1a-l2a*cos(psi);
    %centro del medesimo cerchio
    centroXY = [0;0;19.06];

%% punti sul triangolo centri terzo giunto per L 90_angled
    %ogni punto sopra e' centro di un cercio che giace su un piano perpendicolare al piano del triangolo
    
    % dato che la struttura e' ad L angolata di 90 gradi il centro del raggio
    % NON e':
    %centro_L2alto = p4alto_rf0(1:3);
    %ma e' proprio P4

    % il raggio e' uguale per tutte e tre le Ldue
    Rl2 = 35.03;
    
    %ALTO
    %piano ZX di RFP4 : Rpiano_alto = R0fixedtoP4
    
    %MEDIO
    %piano ZX di RFP4 ruotato di 60 deg

    Rpiano_medio = R0fixedtoP4*rotz(60);
    
    %BASSO
    %piano ZX di RFP4 ruotato di 120 deg

    Rpiano_basso = R0fixedtoP4*rotz(120);
    
%% PLOTs

% eq circonferenza parametrizzata di L 120angled alto-medio-basso (amb)

locus_of_pointLamb = zeros(3,1); %L alto_medio_basso



i=1;
for t=0:pi/15:2*pi

locus_of_pointLamb(:,i) = centroXY + Rxy*cos(t)*RF0fixed(1:3,1) + Rxy*sin(t)*RF0fixed(1:3,2);
i=i+1;
end

figure;
fig_handler = axes;

plot3(locus_of_pointLamb(1,:),locus_of_pointLamb(2,:),locus_of_pointLamb(3,:));

hold on
% PLOT CIRCLE TEST
plot3(locus_of_desired_pose(1,:),locus_of_desired_pose(2,:),locus_of_desired_pose(3,:), 'k');
plot_ref_frame(RF0fixed,[0 0 0]',[255/255 0 0 ;0 255/255 0; 0 0 255/255],1,0.4,10);
plot_Plane(P4,double(plane_n),-5:1:5,-5:1:5,fig_handler);
plot_ref_frame(double(R0fixedtoP4),P4,[255/255 0 0 ;0 255/255 0; 0 0 255/255],1,0.4,10);
plot3(locus_of_pointLamb(1,:),locus_of_pointLamb(2,:),locus_of_pointLamb(3,:));

%ALTO
%piano ZX di RFP4

i=1;
for t=0:pi/15:2*pi

locus_of_pointLduealto(:,i) = P4 + Rl2*cos(t)*R0fixedtoP4(1:3,3) + Rl2*sin(t)*R0fixedtoP4(1:3,1);
i=i+1;
end
plot3(locus_of_pointLduealto(1,:),locus_of_pointLduealto(2,:),locus_of_pointLduealto(3,:),'b');

%MEDIO
%piano ZX di RFP4 ruotato di 60 deg

Rpiano_medio = R0fixedtoP4*rotz(60);

i=1;
for t=0:pi/15:2*pi

locus_of_pointLduemedio(:,i) = P4 + Rl2*cos(t)*Rpiano_medio(1:3,3) + Rl2*sin(t)*Rpiano_medio(1:3,1);
i=i+1;
end
plot3(locus_of_pointLduemedio(1,:),locus_of_pointLduemedio(2,:),locus_of_pointLduemedio(3,:),'r');

%BASSO
%piano ZX di RFP4 ruotato di 120 deg

Rpiano_basso = R0fixedtoP4*rotz(120);

i=1;
for t=0:pi/15:2*pi

locus_of_pointLduebasso(:,i) = P4 + Rl2*cos(t)*Rpiano_basso(1:3,3) + Rl2*sin(t)*Rpiano_basso(1:3,1);
i=i+1;
end
plot3(locus_of_pointLduebasso(1,:),locus_of_pointLduebasso(2,:),locus_of_pointLduebasso(3,:),'g');

% Triangle vertices
plot3(p4alto_rf0(1,:),p4alto_rf0(2,:),p4alto_rf0(3,:),'b*');
plot3(p4medio_rf0(1,:),p4medio_rf0(2,:),p4medio_rf0(3,:),'ro');
plot3(p4basso_rf0(1,:),p4basso_rf0(2,:),p4basso_rf0(3,:),'g+');
%line connecting them
t_vertices = [p4alto_rf0(1:3)';p4medio_rf0(1:3)';p4basso_rf0(1:3)';p4alto_rf0(1:3)']; %to close the triangle
plot3(t_vertices(:,1),t_vertices(:,2),t_vertices(:,3));

    
%% IK parameters

A = centroXY(3) - P4(3);
B = Rl2*sin(pitch);
C = Rl2*cos(pitch)*cos(roll);

D = l1a-l2a*cos(psi);
E = Rl2*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw));
F = Rl2*cos(pitch)*sin(yaw);

H = Rl2*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch));
I = Rl2*cos(pitch)*cos(yaw);
L = Rl2*(sin(pitch)/2 - (3^(1/2)*cos(pitch)*sin(roll))/2);

M = Rl2*((cos(pitch)*sin(yaw))/2 + (3^(1/2)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/2);
N = Rl2*((cos(pitch)*cos(yaw))/2 - (3^(1/2)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/2);
O = Rl2*(sin(pitch)/2 + (3^(1/2)*cos(pitch)*sin(roll))/2);
P = Rl2*((cos(pitch)*sin(yaw))/2 - (3^(1/2)*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)))/2);
R = Rl2*((cos(pitch)*cos(yaw))/2 + (3^(1/2)*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)))/2);

%% ALTO solve t3 -> 2 solutions:
 % eq3alto = A - C*cos(t3) + B*sin(t3)
 % sols: 1) -2*atan2(B + (- A^2 + B^2 + C^2)^(1/2), A + C)
 %       2) -2*atan2(B - (- A^2 + B^2 + C^2)^(1/2), A + C)
 
%eps_zero = 1e-4;

temp = - A^2 + B^2 + C^2;
%testa temp se negativo:
if temp<0.0
    disp('Non real solutions');
    return;
end

i=1;

yplus = (B + (- A^2 + B^2 + C^2)^(1/2));
yminus = (B - (- A^2 + B^2 + C^2)^(1/2));
x = A + C;

    if (  ( yplus ~= 0) && (x ~= 0) )

        St3alto(i) = -2*atan2(yplus,x);
        i=i+1;
    end
    if (  (yminus ~= 0) && (x ~= 0) )

        St3alto(i) = -2*atan2(yminus,x);
        i=i+1;
    end 
 
% Solve for t1 alto: eq 1 and 2:
%eq2alto = E*cos(t3) + D*sin(t1) - F*sin(t3) 
%eq1alto = D*cos(t1) - H*cos(t3) - I*sin(t3)
%Combination to avoid asin and acos but using atan2

%           -E*cos(t3) + F*sin(t3)
% tan(t1) = ----------------------
%           H*cos(t3) + I*sin(t3)

y1 = -E*cos(St3alto(1)) + F*sin(St3alto(1));
x1 = H*cos(St3alto(1)) + I*sin(St3alto(1));

y2 = -E*cos(St3alto(2)) + F*sin(St3alto(2));
x2 = H*cos(St3alto(2)) + I*sin(St3alto(2));

if(i==3)
    disp('found 2 solutions for t3 alto')
    disp('solve for t1 alto...')
    disp('trying first solution:')
    found_solutions = 0;
    
    
    
    if ( ( y1 ~=0) || ( x1 ~= 0) )
        %solve for t1
        %sol 1
        St1alto_1 = atan2( y1, x1 )
        
        found_solutions = 1;
        
    else
        disp('atan2(0,0) Undefined...NO IK Available for sol 1 !!! ');
        %return;
    end
    
    disp('trying second solution:')
    if ( ( y2 ~=0) || ( x2 ~= 0) )
        %solve for t1
        %sol 2
        St1alto_2 = atan2( y2, x2 ) 
        found_solutions = 1;
    else
        disp('atan2(0,0) Undefined...NO IK Available for sol 2 !!! ');
        %return;
    end
    
    if(found_solutions == 0)
        disp('t1 alto cannot be found...No IK Available for given poses...quit()')
        return;
    end
    
    
    
elseif(i==2)
    disp('found only 1 solution for t3 alto')
    disp('solve for t1 alto...')
    if ( y1 ~=0 ) || ( x1 ~= 0) 
        St1alto_1 = atan2( y1, x1 )
    else
        disp('atan2(0,0) Undefined...NO IK Available...Quit() !!! ');
        return;
    end
elseif(i==1)
    disp('found No solutions for t3 alto...quit!!')
    return;
else
    disp('Error t3 alto');
    return;
end

%% MEDIO solve t3 -> 2 solutions:
 %eq3medio = A - C*cos(t3) + L*sin(t3)
    %sols: 1) -2*atan2(L + (- A^2 + C^2 + L^2)^(1/2), A + C)
    %      2) -2*atan2(L - (- A^2 + C^2 + L^2)^(1/2), A + C)

temp = - A^2 + C^2 + L^2;
%testa temp se negativo:
if temp<0.0
    disp('Non real solutions');
    return;
end

i=1;

yplus = (L + (- A^2 + L^2 + C^2)^(1/2));
yminus = (L - (- A^2 + L^2 + C^2)^(1/2));
x = A + C;

    if (  ( yplus ~= 0) && (x ~= 0) )

        St3medio(i) = -2*atan2(yplus,x);
        i=i+1;
    end
    if (  (yminus ~= 0) && (x ~= 0) )

        St3medio(i) = -2*atan2(yminus,x);
        i=i+1;
    end


% Solve for t1 medio: eq 1 and 2:
%eq2medio = E*cos(t3) + D*sin(t1) - M*sin(t3)
%eq1medio = D*cos(t1) - H*cos(t3) - N*sin(t3)
%Combination to avoid asin and acos but using atan2

%           -E*cos(t3) + M*sin(t3)
% tan(t1) = ----------------------
%           H*cos(t3) + N*sin(t3)


y1 = -E*cos(St3medio(1)) + M*sin(St3medio(1));
x1 = H*cos(St3medio(1)) + N*sin(St3medio(1));

y2 = -E*cos(St3medio(2)) + M*sin(St3medio(2));
x2 = H*cos(St3medio(2)) + N*sin(St3medio(2));

if(i==3)
    disp('found 2 solutions for t3 medio')
    disp('solve for t1 medio...')
    disp('trying first solution:')
    found_solutions = 0;
    
    
    
    if ( ( y1 ~=0) || ( x1 ~= 0) )
        %solve for t1
        %sol 1
        St1medio_1 = atan2( y1, x1 )
        
        found_solutions = 1;
        
    else
        disp('atan2(0,0) Undefined...NO IK Available for sol 1 !!! ');
        %return;
    end
    
    disp('trying second solution:')
    if ( ( y2 ~=0) || ( x2 ~= 0) )
        %solve for t1
        %sol 2
        St1medio_2 = atan2( y2, x2 ) 
        found_solutions = 1;
    else
        disp('atan2(0,0) Undefined...NO IK Available for sol 2 !!! ');
        %return;
    end
    
    if(found_solutions == 0)
        disp('t1 medio cannot be found...No IK Available for given poses...quit()')
        return;
    end
    
    
    
elseif(i==2)
    disp('found only 1 solution for t3 medio')
    disp('solve for t1 medio...')
    if ( y1 ~=0 ) || ( x1 ~= 0) 
        St1medio_1 = atan2( y1, x1 )
    else
        disp('atan2(0,0) Undefined...NO IK Available...Quit() !!! ');
        return;
    end
elseif(i==1)
    disp('found No solutions for t3 medio...quit!!')
    return;
else
    disp('Error t3 medio');
    return;
end

%% BASSO solve t3 -> 2 solutions:
 %eq3basso = A - C*cos(t3) - O*sin(t3)
    %sols: 1) 2*atan2(O + (- A^2 + C^2 + O^2)^(1/2), A + C)
    %      2) 2*atan2(O - (- A^2 + C^2 + O^2)^(1/2), A + C)

temp = - A^2 + C^2 + O^2;
%testa temp se negativo:
if temp<0.0
    disp('Non real solutions');
    return;
end

i=1;

yplus = (O + (- A^2 + C^2 + O^2)^(1/2));
yminus = (O - (- A^2 + C^2 + O^2)^(1/2));
x = A + C;

    if (  ( yplus ~= 0) && (x ~= 0) )

        St3basso(i) = 2*atan2(yplus,x);
        i=i+1;
    end
    if (  (yminus ~= 0) && (x ~= 0) )

        St3basso(i) = 2*atan2(yminus,x);
        i=i+1;
    end

% Solve for t1 basso: eq 1 and 2:
%eq2basso = E*cos(t3) + D*sin(t1) + P*sin(t3)
%eq1basso = D*cos(t1) - H*cos(t3) + R*sin(t3)
%Combination to avoid asin and acos but using atan2

%           -E*cos(t3) - P*sin(t3)
% tan(t1) = ----------------------
%           H*cos(t3) - R*sin(t3)


y1 = -E*cos(St3basso(1)) - P*sin(St3basso(1));
x1 = H*cos(St3basso(1)) - R*sin(St3basso(1));

y2 = -E*cos(St3basso(2)) - P*sin(St3basso(2));
x2 = H*cos(St3basso(2)) - R*sin(St3basso(2));

if(i==3)
    disp('found 2 solutions for t3 basso')
    disp('solve for t1 basso...')
    disp('trying first solution:')
    found_solutions = 0;
    
    
    
    if ( ( y1 ~=0) || ( x1 ~= 0) )
        %solve for t1
        %sol 1
        St1basso_1 = atan2( y1, x1 )
        
        found_solutions = 1;
        
    else
        disp('atan2(0,0) Undefined...NO IK Available for sol 1 !!! ');
        %return;
    end
    
    disp('trying second solution:')
    if ( ( y2 ~=0) || ( x2 ~= 0) )
        %solve for t1
        %sol 2
        St1basso_2 = atan2( y2, x2 ) 
        found_solutions = 1;
    else
        disp('atan2(0,0) Undefined...NO IK Available for sol 2 !!! ');
        %return;
    end
    
    if(found_solutions == 0)
        disp('t1 basso cannot be found...No IK Available for given poses...quit()')
        return;
    end
    
    
    
elseif(i==2)
    disp('found only 1 solution for t3 basso')
    disp('solve for t1 basso...')
    if ( y1 ~=0 ) || ( x1 ~= 0) 
        St1basso_1 = atan2( y1, x1 )
    else
        disp('atan2(0,0) Undefined...NO IK Available...Quit() !!! ');
        return;
    end
elseif(i==1)
    disp('found No solutions for t3 basso...quit!!')
    return;
else
    disp('Error t3 basso');
    return;
end


%% PLOT find intersection point between L120angled and L90angled

    %find clooser solution to home pose
    disp('Solutions: q1 ALTO:' )
    if( abs(St1alto_1-0) < abs(St1alto_2-0) )
        solAlto = St1alto_1
    else
        solAlto = St1alto_2
    end
    
    disp('Solutions: q1 MEDIO:' )
    if( abs(St1medio_1-(-(2/3)*pi)) < abs(St1medio_2-(-(2/3)*pi)) )
        solMedio = St1medio_1
    else
        solMedio = St1medio_2
    end
    
    disp('Solutions: q1 BASSO:' )
    if(abs(St1basso_1-((2/3)*pi)) < abs(St1basso_2-((2/3)*pi)))
        solBasso = St1basso_1
    else
        solBasso = St1basso_2
    end


Palto = centroXY + Rxy*cos(solAlto)*RF0fixed(1:3,1) + Rxy*sin(solAlto)*RF0fixed(1:3,2);
plot3(Palto(1),Palto(2),Palto(3),'k*');

Pmedio = centroXY + Rxy*cos(solMedio)*RF0fixed(1:3,1) + Rxy*sin(solMedio)*RF0fixed(1:3,2);
plot3(Pmedio(1),Pmedio(2),Pmedio(3),'k*');

Pbasso = centroXY + Rxy*cos(solBasso)*RF0fixed(1:3,1) + Rxy*sin(solBasso)*RF0fixed(1:3,2);
plot3(Pbasso(1),Pbasso(2),Pbasso(3),'k*');


solAltoDeg = rad2deg(solAlto)
solMedioDeg = rad2deg(solMedio)
solBassoDeg = rad2deg(solBasso)

drawnow;
Frame(vectorNumber) = getframe;

end


