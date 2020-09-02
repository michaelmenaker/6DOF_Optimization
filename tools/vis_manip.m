clearvars;
close all;
%optimize link lengths and distance from print to base
L = 8*ones(4,1);

%dh parameters
a = [0;0;0;L(2);L(3);L(4)];
alpha = [0;90;0;0;0;0];
d = [L(1);0;0;0;0;0];
theta = zeros(6,1);

%inverse kinematics
xd = 6; yd = 6; zd = 12;
zd = zd + L(4);
theta(1) = atan2d(yd,xd);
c3 = (xd^2 + yd^2 + (zd - L(1))^2 - L(2)^2 - L(3)^2)/(2*L(2)*L(3));
s3 = sqrt(1-c3^2);
theta(4) = atan2d(s3,c3);
theta(3) = atan2d(zd-L(1),sqrt(xd^2+yd^2))-atan2d(L(3)*s3,L(2)+L(3)*c3);


%normal constraint
theta(5) = -(90 + theta(3) + theta(4));

%transformation matrices
T = dhTransform([a alpha d theta]);
B = [0;0;0;1];

%forward kinematics
J1 = T(:,:,1)*B;
J2 = T(:,:,1)*T(:,:,2)*T(:,:,3)*B;
J3 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*B;
J4 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*B;
J5 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*B;

%compute links
t = 0:.01:1;
L1 = (J1-B) * t + repmat(B,[1,length(t)]);
L2 = (J2-J1) * t + repmat(J1,[1,length(t)]);
L3 = (J3-J2) * t + repmat(J2,[1,length(t)]);
L4 = (J4-J3) * t + repmat(J3,[1,length(t)]);
L5 = (J5-J4) * t + repmat(J4,[1,length(t)]);

%plot joints and links
figure(1)
plot3(B(1),B(2),B(3),'o','Color','k');
hold on;
plot3(L1(1,:),L1(2,:),L1(3,:),'Color','b');
plot3(J1(1),J1(2),J1(3),'o','Color','k');
plot3(L2(1,:),L2(2,:),L2(3,:),'Color','b');
plot3(J2(1),J2(2),J2(3),'o','Color','k');
plot3(L3(1,:),L3(2,:),L3(3,:),'Color','b');
plot3(J3(1),J3(2),J3(3),'o','Color','k');
plot3(L4(1,:),L4(2,:),L4(3,:),'Color','b');
plot3(J4(1),J4(2),J4(3),'o','Color','k');
plot3(L5(1,:),L5(2,:),L5(3,:),'Color','b');
plot3(J5(1),J5(2),J5(3),'o','Color','k');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('4R 3D Robotic Manipulator');



%optimization of link length and relative position of build plate
L_lim = 3:.01:12;
%create cube object
cube_origin = [6;-6;0];%bottom left corner
side_length = 12;%1x1x1ft cube
v1 = cube_origin;
v2 = cube_origin + [side_length;0;0];
v3 = cube_origin + [side_length;side_length;0];
v4 = cube_origin + [0;side_length;0];
v5 = v1 + [0;0;side_length];
v6 = v2 + [0;0;side_length];
v7 = v3 + [0;0;side_length];
v8 = v4 + [0;0;side_length];

%compute cube links
S1 = (v2-v1) * t + repmat(v1,[1,length(t)]);
S2 = (v3-v2) * t + repmat(v2,[1,length(t)]);
S3 = (v4-v3) * t + repmat(v3,[1,length(t)]);
S4 = (v1-v4) * t + repmat(v4,[1,length(t)]);
S5 = (v6-v5) * t + repmat(v5,[1,length(t)]);
S6 = (v7-v6) * t + repmat(v6,[1,length(t)]);
S7 = (v8-v7) * t + repmat(v7,[1,length(t)]);
S8 = (v5-v8) * t + repmat(v8,[1,length(t)]);
S9 = (v5-v1) * t + repmat(v1,[1,length(t)]);
S10 = (v6-v2) * t + repmat(v2,[1,length(t)]);
S11 = (v7-v3) * t + repmat(v3,[1,length(t)]);
S12 = (v8-v4) * t + repmat(v4,[1,length(t)]);

%plot cube 
plot3(v1(1),v1(2),v1(3),'o','Color','k');
plot3(v2(1),v2(2),v2(3),'o','Color','k');
plot3(v3(1),v3(2),v3(3),'o','Color','k');
plot3(v4(1),v4(2),v4(3),'o','Color','k');
plot3(v5(1),v5(2),v5(3),'o','Color','k');
plot3(v6(1),v6(2),v6(3),'o','Color','k');
plot3(v7(1),v7(2),v7(3),'o','Color','k');
plot3(v8(1),v8(2),v8(3),'o','Color','k');
plot3(S1(1,:),S1(2,:),S1(3,:),'Color','g');
plot3(S2(1,:),S2(2,:),S2(3,:),'Color','g');
plot3(S3(1,:),S3(2,:),S3(3,:),'Color','g');
plot3(S4(1,:),S4(2,:),S4(3,:),'Color','g');
plot3(S5(1,:),S5(2,:),S5(3,:),'Color','g');
plot3(S6(1,:),S6(2,:),S6(3,:),'Color','g');
plot3(S7(1,:),S7(2,:),S7(3,:),'Color','g');
plot3(S8(1,:),S8(2,:),S8(3,:),'Color','g');
plot3(S9(1,:),S9(2,:),S9(3,:),'Color','g');
plot3(S10(1,:),S10(2,:),S10(3,:),'Color','g');
plot3(S11(1,:),S11(2,:),S11(3,:),'Color','g');
plot3(S12(1,:),S12(2,:),S12(3,:),'Color','g');