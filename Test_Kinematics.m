% UR5 DH parameters (compliant with Simscape/URDF model)
a2=0.425; a3 = 0.39225; a5=0; a6=0;
d1=0.089159; d5=0.09465; d6=0.0823;
d4=0.10915; % Negative d4 correctly shifts DH frame 4 in global +Y direction
syms s1_ s2_ s3_ s4_ s5_ s6_ 'real'
DHpram=[0,pi/2,s1_,d1,1;
    a2,0,s2_,0,1;
    a3,0,s3_,0,1;
    0,pi/2,s4_,d4,1;
    0,-pi/2,s5_,d5,1;
    0,0,s6_,d6,1];

A1=Ai(DHpram(1,:));
A2=Ai(DHpram(2,:));
A3=Ai(DHpram(3,:));
A4=Ai(DHpram(4,:));
A5=Ai(DHpram(5,:));
A6=Ai(DHpram(6,:));

T=A1*A2*A3*A4*A5*A6;
pos=simplify(T(1:3,4))';

n=1000;
t=linspace(0,2,n);

% Compute base 5th-order polynomial trajectory profile vectors
s6=(3*t.^5)/16 - (15*t.^4)/16 + (5*t.^3)/4 + (3*t)/1125899906842624;
ds6=(15*t.^4)/16 - (15*t.^3)/4 + (15*t.^2)/4 + 3/1125899906842624;
dds6=(15*t.^3)/4 - (45*t.^2)/4 + (15*t)/2;

s1=s6;ds1=ds6;dds1=dds6;
s2=s6;ds2=ds6;dds2=dds6;
s3=s6;ds3=ds6;dds3=dds6;
s4=s6;ds4=ds6; dds4=dds6;
s5=s6;ds5=ds6; dds5=dds6;

Posi=zeros(n,3);
for i=1:n
Posi(i,:)=subs(pos,[s1_ s2_ s3_ s4_ s5_ s6_],[s1(i) s2(i) s3(i) s4(i) s5(i) s6(i)]);
end

plot(out.out1(:,4),out.out1(:,1),t,Posi(:,1));
figure (2) 
plot(out.out1(:,4),out.out1(:,2),t,Posi(:,2));
figure (3) 
plot(out.out1(:,4),out.out1(:,3),t,Posi(:,3));

function A = Ai(Dh)
ai=Dh(1); alphai=Dh(2);
si=Dh(3); di=Dh(4);
A=[cos(si) -sin(si)*cos(alphai) sin(si)*sin(alphai) ai*cos(si)
    sin(si) cos(si)*cos(alphai) -cos(si)*sin(alphai) ai*sin(si)
    0            sin(alphai)          cos(alphai)           di
    0                  0                   0                  1];
end