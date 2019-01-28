clear all;
close all;
clc;

K = [25 3.75];

Phi = [ 1 0.1; 0 1];
gamma = [0.005 ; 0.1];
H=[1 0];
Lp = [1.35; 6.0003];
Lc=[0.75; 6.0003];
J=[0];

P_Est_Q = [Phi -gamma*K; 
           Lp*H Phi-gamma*K-Lp*H];
Peig=eig(P_Est_Q)

T=0.1;
SYS=ss(P_Est_Q,[gamma; gamma],[H H],J,T);
Tfinal=5;


[Y,T,X1]=initial(SYS,[10 10 5 5]',Tfinal);

figure(1)
subplot(2,1,1)
plot(X1(:,1));
hold on
plot(X1(:,2));
hold on
plot(X1(:,3));
hold on
plot(X1(:,4));
hold on
grid on
title('P-Estimator Regulator');
xlabel('Time');
ylabel('Magnitude');
legend('X1-True Position','X2-True Velocity','X1 P-Esti-position','X2 P-Esti-Velocity','location','best');

subplot(2,1,2)
plot(X1(:,1)-X1(:,3));
hold on
plot(X1(:,2)-X1(:,4));
grid on
title('State Tracking Error');
xlabel('Time');
ylabel('Error');
legend('X1 error','X2 error');


C_Est_Q = [Phi -gamma*K; 
           Lc*H*Phi Phi-gamma*K-Lc*H*Phi];
Ceig=eig(C_Est_Q)



T=0.1;
SYS=ss(C_Est_Q,[gamma; gamma],[H H],J,T);
Tfinal=5;


[Y,T,X2]=initial(SYS,[10 10 5 5]',Tfinal);

figure(2)
subplot(2,1,1)
plot(X2(:,1));
hold on
plot(X2(:,2));
hold on
plot(X2(:,3));
hold on
plot(X2(:,4));
hold on
grid on
title('C-Estimator Regulator');
xlabel('Time');
ylabel('Magnitude');
legend('X1-True Position','X2-True Velocity','X1 C-Esti-position','X2 C-Esti-Velocity','location','best');

subplot(2,1,2)
plot(X2(:,1)-X2(:,3));
hold on
plot(X2(:,2)-X2(:,4));
grid on
title('State Tracking Error');
xlabel('Time');
ylabel('Error');
legend('X1 error','X2 error');



PnC_Est_Q =[Phi -gamma*K -gamma*K; 
            Lp*H Phi-gamma*K-Lp*H [0 0; 0 0]; 
            Lc*H*Phi [0 0; 0 0] Phi-gamma*K-Lc*H*Phi ]; 



T=0.1;
SYS=ss(PnC_Est_Q,[gamma; gamma; gamma],[H H H],J,T);
Tfinal=10;


[Y,T,X3]=initial(SYS,[10 10 5 5 5 5]',Tfinal);


figure(3)
subplot(2,1,1)
plot(X3(:,1));
hold on
plot(X3(:,2));
hold on
plot(X3(:,3));
hold on
plot(X3(:,4));
hold on
plot(X3(:,5));
hold on
plot(X3(:,6))
grid on
title('P+C-Estimator Regulator Without weight(1) (tends to infinity)');
xlabel('Time');
ylabel('Magnitude');
legend('X1-True Position','X2-True Velocity','X1 P-Esti-position','X2 P-Esti-Velocity','X1 C-Esti-position','X2 C-Esti-Velocity','location','best');


subplot(2,1,2)
plot(X3(:,1)-X3(:,3));
hold on
plot(X3(:,2)-X3(:,4));
hold on
plot(X3(:,1)-X3(:,5));
hold on
plot(X3(:,2)-X3(:,6));
grid on
title('State Tracking Error');
xlabel('Time');
ylabel('Error');
legend('X1 p-error','X2 p-error','X1 c-error','X2 c-error');

% 50-50

PnC_Est_Q =[Phi -gamma*K/2 -gamma*K/2; 
            Lp*H Phi-gamma*K-Lp*H [0 0; 0 0]; 
            Lc*H*Phi [0 0; 0 0] Phi-gamma*K-Lc*H*Phi ]; 



T=0.1;
SYS=ss(PnC_Est_Q,[gamma; gamma; gamma],[H H H],J,T);
Tfinal=10;


[Y,T,X4]=initial(SYS,[10 10 5 5 5 5]',Tfinal);


figure(4)
subplot(2,1,1)
plot(X4(:,1));
hold on
plot(X4(:,2));
hold on
plot(X4(:,3));
hold on
plot(X4(:,4));
hold on
plot(X4(:,5));
hold on
plot(X4(:,6))
grid on
title('P+C-Estimator Regulator 50%P-50%C(1)');
xlabel('Time');
ylabel('Magnitude');
legend('X1-True Position','X2-True Velocity','X1 P-Esti-position','X2 P-Esti-Velocity','X1 C-Esti-position','X2 C-Esti-Velocity','location','best');


subplot(2,1,2)
plot(X4(:,1)-X4(:,3));
hold on
plot(X4(:,2)-X4(:,4));
hold on
plot(X4(:,1)-X4(:,5));
hold on
plot(X4(:,2)-X4(:,6));
grid on
title('State Tracking Error');
xlabel('Time');
ylabel('Error');
legend('X1 p-error','X2 p-error','X1 c-error','X2 c-error');


%30-70

PnC_Est_Q2 =[Phi -gamma*K*0.3 -gamma*K*0.7; 
            Lp*H Phi-gamma*K-Lp*H [0 0; 0 0]; 
            Lc*H*Phi [0 0; 0 0] Phi-gamma*K-Lc*H*Phi ]; 



T=0.1;
SYS=ss(PnC_Est_Q2,[gamma; gamma; gamma],[H H H],J,T);
Tfinal=10;


[Y,T,X5]=initial(SYS,[10 10 5 5 5 5]',Tfinal);


figure(5)
subplot(2,1,1)
plot(X5(:,1));
hold on
plot(X5(:,2));
hold on
plot(X5(:,3));
hold on
plot(X5(:,4));
hold on
plot(X5(:,5));
hold on
plot(X5(:,6))
grid on
title('P+C-Estimator Regulator 30%P-70%C(3)');
xlabel('Time');
ylabel('Magnitude');
legend('X1-True Position','X2-True Velocity','X1 P-Esti-position','X2 P-Esti-Velocity','X1 C-Esti-position','X2 C-Esti-Velocity','location','best');

subplot(2,1,2)
plot(X5(:,1)-X5(:,3));
hold on
plot(X5(:,2)-X5(:,4));
hold on
plot(X5(:,1)-X5(:,5));
hold on
plot(X5(:,2)-X5(:,6));
grid on
title('State Tracking Error');
xlabel('Time');
ylabel('Error');
legend('X1 p-error','X2 p-error','X1 c-error','X2 c-error');

