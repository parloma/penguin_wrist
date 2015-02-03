%% IK Pinguino Geometric Approach

clc
clear all
close all


% IK

% definizione orientamento desiderato

% posa desiderata w.r.t the RF0 fixed = rotx(30deg)

roll = sym('roll','real');
pitch = sym('pitch','real');
yaw = sym('yaw','real');

%con R=Rz*Ry*Rx (quella convenzionale) su assi fissi quindi inizio da roll
%poi pitch ed infine yaw di RF0 fixed
R0fixedtoP4 = rotzSYM(yaw)*rotySYM(pitch)*rotxSYM(roll);

R0fixedtoP4 = subs(R0fixedtoP4,[roll,pitch,yaw],[pi/2,0,0]);

%normal to plane == z component of R0fixedtoP4

%punto P4 (in solidworks) costante per ogni q w.r.t R0fixed

P4 = [0;0;36.57]; % w.r.t R0fixed

%Homogeneous transformation

T0fixedtoP4 = rd2tom(double(R0fixedtoP4), P4);

plane_n = R0fixedtoP4(1:3,3);


% inertial frame
RF0fixed = eye(3,3);


%L ALTO
 l1a = 19.33;
 l2a = 22.01;
 psi = deg2rad(120);
 
%% L Alto Basso Medio creano una circonferenza con centro in Z=19.06 e raggio R nel piano XY w.r.t. RF0fixed

Rxy = l1a-l2a*cos(psi);

centro = [0;0;19.06];

% eq circonferenza parametrizzata

locus_of_pointLamb = zeros(3,1); %L alto_medio_basso

i=1;
for t=0:pi/15:2*pi

locus_of_pointLamb(:,i) = centro + Rxy*cos(t)*RF0fixed(1:3,1) + Rxy*sin(t)*RF0fixed(1:3,2);
i=i+1;
end

figure;
fig_handler = axes;

plot3(locus_of_pointLamb(1,:),locus_of_pointLamb(2,:),locus_of_pointLamb(3,:));

hold on
plot_ref_frame(RF0fixed,[0 0 0]',[255/255 0 0 ;0 255/255 0; 0 0 255/255],1,0.4,10);
plot_Plane(P4,double(plane_n),-5:1:5,-5:1:5,fig_handler);
plot_ref_frame(double(R0fixedtoP4),P4,[255/255 0 0 ;0 255/255 0; 0 0 255/255],1,0.4,10);
plot3(locus_of_pointLamb(1,:),locus_of_pointLamb(2,:),locus_of_pointLamb(3,:));

%% punti sul triangolo

%ALTO

% w.r.t. to RFP4

p4alto_rfp4 = [0;-14.87;0;1];

% w.r.t to RF0fixed

p4alto_rf0 =  T0fixedtoP4 * p4alto_rfp4;

plot3(p4alto_rf0(1,:),p4alto_rf0(2,:),p4alto_rf0(3,:),'b*');

%MEDIO

% w.r.t. to RFP4

p4medio_rfp4 = [-14.87*cosd(30);14.87*sind(30);0;1];

% w.r.t to RF0fixed

p4medio_rf0 =  T0fixedtoP4 * p4medio_rfp4;

plot3(p4medio_rf0(1,:),p4medio_rf0(2,:),p4medio_rf0(3,:),'ro');

%BASSO

% w.r.t. to RFP4

p4basso_rfp4 = [14.87*cosd(30);14.87*sind(30);0;1];

% w.r.t to RF0fixed

p4basso_rf0 =  T0fixedtoP4 * p4basso_rfp4;

plot3(p4basso_rf0(1,:),p4basso_rf0(2,:),p4basso_rf0(3,:),'g+');


%% TEST if point belong to plane

dot(double(plane_n),(p4basso_rf0(1:3)-P4))

dot(double(plane_n),(p4medio_rf0(1:3)-P4))

dot(double(plane_n),(p4alto_rf0(1:3)-P4))


%% ogni punto sopra e' centro di un cercio che giace su un piano perpendicolare al piano del triangolo
 %dato
 
 %definizione piano passante per p4alto_rf0 e perpendicolare a piano
 %triangolo
 %ALTO -> Piano ZY di RFP4
%  syms a b c real
%  
%  
%  Sa = solve(dot([a,b,c],double(plane_n))==0,c,'Real',true);
%  n_plane_p4alto(1)=1;
%  n_plane_p4alto(2)=-1;
%  n_plane_p4alto(3) = double(subs(Sa,[a,b],[n_plane_p4alto(1),n_plane_p4alto(2)]) );  
%  %normalizzo
%  n_plane_p4alto = n_plane_p4alto./norm(n_plane_p4alto);
%  
%  plot_Plane(p4alto_rf0(1:3),double(n_plane_p4alto),-5:1:5,-5:1:5,fig_handler);

locus_of_pointLduealto = zeros(3,1); %L2 alto
locus_of_pointLduemedio = zeros(3,1); %L2 medio
locus_of_pointLduebasso = zeros(3,1); %L2 basso

% dato che la struttura e' ad L angolata di 90 gradi il centro del raggio
% NON e':
%centro_L2alto = p4alto_rf0(1:3);
%ma e' proprio P4

% il raggio e' uguale per tutte e tre le Ldue
Rl2 = 35.03;

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


syms t1 t3 real

St1t3alto = solve( centro(1) + Rxy*cos(t1) == P4(1) + Rl2*cos(t3)*R0fixedtoP4(1,3) + Rl2*sin(t3)*R0fixedtoP4(1,1), centro(2) + Rxy*sin(t1) == P4(2) + Rl2*cos(t3)*R0fixedtoP4(2,3) + Rl2*sin(t3)*R0fixedtoP4(2,1),t1,t3, 'Real',true );


syms t1 t3 real

St1t3medio = solve( centro(1) + Rxy*cos(t1) == P4(1) + Rl2*cos(t3)*Rpiano_medio(1,3) + Rl2*sin(t3)*Rpiano_medio(1,1), centro(2) + Rxy*sin(t1) == P4(2) + Rl2*cos(t3)*Rpiano_medio(2,3) + Rl2*sin(t3)*Rpiano_medio(2,1),t1,t3, 'Real',true );


syms t1 t3 real

St1t3basso = solve( centro(1) + Rxy*cos(t1) == P4(1) + Rl2*cos(t3)*Rpiano_basso(1,3) + Rl2*sin(t3)*Rpiano_basso(1,1), centro(2) + Rxy*sin(t1) == P4(2) + Rl2*cos(t3)*Rpiano_basso(2,3) + Rl2*sin(t3)*Rpiano_basso(2,1),t1,t3, 'Real',true );



