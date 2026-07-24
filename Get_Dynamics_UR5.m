%%%%%% Example UR16e robot
clc,clear;
syms s1 s2 s3 s4 s5 s6  ds1 ds2 ds3 ds4 ds5 ds6 dds1 dds2 dds3 dds4 dds5 dds6 'real'
syms g m1 m2 m3 m4 m5 m6 'real'
syms a2 a3 d1 d4 d5 d6 'real'
syms I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz 'real'
syms I4xx I4yy I4zz I5xx I5yy I5zz I6xx I6yy I6zz 'real'

syms pc1x  pc1y pc1z pc2x  pc2y pc2z pc3x  pc3y pc3z 'real'
syms pc4x  pc4y pc4z pc5x  pc5y pc5z pc6x  pc6y pc6z 'real'

gc=[0;0;g];
m=[m1;m2;m3;m4;m5;m6];
q=[s1;s2;s3;s4;s5;s6]; dq=[ds1; ds2; ds3;ds4; ds5; ds6];
ddq=[dds1; dds2; dds3; dds4; dds5; dds6];

I1=[I1xx 0 0;0 I1yy 0;0 0 I1zz];
I2=[I2xx 0 0;0 I2yy 0;0 0 I2zz];
I3=[I3xx 0 0;0 I3yy 0;0 0 I3zz];
I4=[I4xx 0 0;0 I4yy 0;0 0 I4zz];
I5=[I5xx 0 0;0 I5yy 0;0 0 I5zz];
I6=[I6xx 0 0;0 I6yy 0;0 0 I6zz];

DHpram=[0,pi/2,s1,d1,1;
    a2,0,s2,0,1;
    a3,0,s3,0,1;
    0,pi/2,s4,d4,1;
    0,-pi/2,s5,d5,1;
    0,0,s6,d6,1];

% ICi relative to frame i-1 expressed in frame i

P0=[pc1x  pc1y pc1z;
    pc2x  pc2y pc2z;
    pc3x  pc3y pc3z;
    pc4x  pc4y pc4z;
    pc5x  pc5y pc5z;
    pc6x  pc6y pc6z];


I(:,:,1)=I1;
I(:,:,2)=I2;
I(:,:,3)=I3;
I(:,:,4)=I4;
I(:,:,5)=I5;
I(:,:,6)=I6;

[M,C,G]=DH_dyn(DHpram,P0,q,dq,I,m,gc);
tau = M * ddq + C * dq + G;
% convert symbolic to string (human-readable)
S = char(tau);                     % creates a char representation
fid = fopen('tau_sym.txt','w');
fprintf(fid,'%s\n',S);
fclose(fid);


% simplify(tau);
% save('UR5.mat');
