%%State Space%%
%%%%%%%%%%%%%%%

%%Ts = 5s
%%MUP = 5%

syz=c2d(sys,T);
A = syz.A;
B = syz.B;
C = syz.C;
D = syz.D;

p1 = -8.3;
p2 = -83;

ob1 = -100;
ob2 = -120;

p1 = exp(T*p1);
p2 = exp(T*p2);

ob1 = exp(T*ob1);
ob2 = exp(T*ob2);

K = place(A,B,[p1 p2]);
L = place(A',C',[ob1 ob2])';