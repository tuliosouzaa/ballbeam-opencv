T=0.2;

%%inicia parametros
m = 2/1000;
g = 9.812;
L = 48.5/100;
d = 11.8/100;
b = 0.002;

%%SS
[x,u,y,dx] = trim('NAOLINEAR');
[A,B,C,D]=linmod('NAOLINEAR');
[num,den]=ss2tf(A,B,C,D);
sys=ss(A,B,C,D);
syz=c2d(sys,T,'zoh');

%%TF CONTINUO
G=tf(num,den);
G1=tf(num,den,'iodelay',T);
%%%

%%GZ COM PADE
[num1,den1]=pade(0.3,1);
padeS=tf(num1,den1);
G2=G*padeS;
Gz=c2d(G1,T,'zoh');
%%%

%%SS CONTROL
A=syz.A;
B=syz.B;
C=syz.C;
D=syz.D;

p1 = -1.25+1j;
p2 = -1.25-1j;
p1 = exp(T*p1);
p2 = exp(T*p2);

K = place(A,B,[p1 p2]);

po1 = -30;
po2 = -80;
po1 = exp(T*po1);
po2 = exp(T*po2);

L = place(A',C',[po1 po2])';

Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B*(1/0.606);
       zeros(size(B))];
Cce = [C zeros(size(C))];

sys_f = ss(Ace,Bce,Cce,0,0.1);
step(.75*sys_f);