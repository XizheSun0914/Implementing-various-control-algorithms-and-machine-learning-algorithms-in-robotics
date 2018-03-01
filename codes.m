av=10;
ac=5;
af=10;
as=0.01;
xi=0.1;
[Am,Bm,Cm,Dm]=tf2ss([1],[1,0])
[As,Bs,Cs,Ds]=tf2ss([1],[10,0])
[Ah,Bh,Ch,Dh]=tf2ss([288],[1,12,60,144,144])
[Ae,Be,Ce,De]=tf2ss([0.16,0,240],[0.008,0.48,12,120])
A=[zeros(9,2),[zeros(1,3);[20,0,30000];[-60,-1500,-15000;1,0,0;0,1,0];zeros(4,3)],[[0,0,0,288];zeros(4,4);[-12,-60,-144,-144;1,0,0,0;0,1,0,0;0,0,1,0]]]
B=[zeros(9,3),[[0,0,-1,0];[0,0,0,-1];[zeros(3,1),[1;0;0],zeros(3,2)];[[1;0;0;0],zeros(4,3)]]]
C=[av*Cm,-av*Cs,zeros(1,3),zeros(1,4); -ac*Cm,0,zeros(1,3),ac*Ch; 0,0,-af*Ce,zeros(1,4);0,0,zeros(1,3),zeros(1,4);0,0,Ce,zeros(1,4);0,Cs,zeros(1,3),zeros(1,4);Cm,0,zeros(1,3),zeros(1,4)]
D=[zeros(4,3),[zeros(4,2),[0,0;0,0;af,0;0,as]];xi*eye(3),zeros(3,2),zeros(3,2)]
G=ss(A,B,C,D)
[K,CL,GAM]=h2syn(G,3,2);
K
Controller=tf(K)
