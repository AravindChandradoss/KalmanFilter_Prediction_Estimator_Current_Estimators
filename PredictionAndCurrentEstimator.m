clear all;
close all;

Phi = [ 1 0.1; 
      0 1];
gamma = [0.005; 
     0.1];
H= [1 0];
J=[0];
T=0.1;
Ds_p = [0.325+0.38i  0.325-0.38i]'; %desire pole

%P_Estimator
Lp_T = place(Phi',H',Ds_p)
eig((Phi-Lp_T'*H));
SYS=ss((Phi-Lp_T'*H),gamma,H,J,T);
Tfinal=5;
[Y,T,X]=initial(SYS,[10 10]',Tfinal);

figure(1)
subplot(2,1,1)
plot(X(:,1));
grid on
hold on
plot(X(:,2));
title('P-Estimator Closed loop');
xlabel('Time');
ylabel('Magnitude (Tends to zero coz of feedbacks)');
legend('X1','X2');

%C_Estimator
T=0.1;
Lc_T = place(Phi',Phi'*H',Ds_p)
eig((Phi-Lc_T'*H));
SYS=ss((Phi-Phi*Lc_T'*H),gamma,H,J,T);
Tfinal=5;
[Y,T,X1]=initial(SYS,[10 10]',Tfinal);

figure(1)
subplot(2,1,2)
plot(X1(:,1));
grid on
hold on
plot(X1(:,2));
title('C-Estimator Closed loop');
xlabel('Time');
ylabel('Magnitude (Tends to zero coz of feedbacks)');
legend('X1','X2');

%%%%%%%%%%%%%%%%%
%Taking feedback as zero
k=[0 0];

%P_Estimate 
P_Est_Q = [Phi -gamma*k; Lp_T'*H Phi-gamma*k-Lp_T'*H];
eig(P_Est_Q)

T=0.1;
SYS=ss(P_Est_Q,[gamma; gamma],[H H],J,T);
Tfinal=3;


[Y,T,X1]=initial(SYS,[5 5 0 0]',Tfinal);

figure(3)

subplot(2,2,1)
plot(X1(:,1));
hold on
plot(X1(:,2));
hold on
plot(X1(:,3));
hold on
plot(X1(:,4));
hold on
grid on
title('P-Estimator Tracking');
xlabel('Time');
ylabel('Magnitude');
legend('X1-True Position','X2-True Velocity','X1 P-Esti-position','X2 P-Esti-Velocity');
subplot(2,2,3)
plot(X1(:,1)-X1(:,3));
hold on
plot(X1(:,2)-X1(:,4));
grid on
title('Tracking Error');
xlabel('Time');
ylabel('Error');
legend('X1 error','X2 error');


%C_Estimate 

C_Est_Q = [Phi -gamma*k; Lc_T'*H*Phi Phi-gamma*k-Lc_T'*H*Phi];
eig(C_Est_Q)
T=0.1;
SYS=ss(C_Est_Q,[gamma; gamma],[H H],J,T);
Tfinal=3;


[Y,T,X2]=initial(SYS,[5 5 0 0]',Tfinal);

%figure(4)
subplot(2,2,2)
plot(X2(:,1));
hold on
plot(X2(:,2));
hold on
plot(X2(:,3));
hold on
plot(X2(:,4));
hold on
grid on
title('C-Estimator Tracking');
xlabel('Time');
ylabel('Magnitude');
legend('X1-True Position','X2-True Velocity','X1 C-Esti-position','X2 C-Esti-Velocity');
subplot(2,2,4)
plot(X2(:,1)-X2(:,3));
hold on
plot(X2(:,2)-X2(:,4));
grid on
title('Tracking Error');
xlabel('Time');
ylabel('Error');
legend('X1 error','X2 error');

