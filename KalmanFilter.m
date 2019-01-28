clear all

Tsim=600;

% Process (considered to be the actual process)
T=0.01;

Phi=[1 T; 
     0 1];
 
Gamma=[(T^2)/2; 
        T];
Gamma1=[0.00; 
        0.01];
H=[1 0];

[n,n]=size(Phi);
[n,m]=size(Gamma);
[n,m]=size(Gamma1);
[p,n]=size(H);

u=randn(m,Tsim);

Rw=0.001*eye(m,m);
w=sqrt(Rw)*randn(m,Tsim);

Rv=0.1*eye(p,p);
v=sqrt(Rv)*randn(p,Tsim);

% Initialize, allocate memory

x=0*ones(n,Tsim+1);
y=0*ones(p,Tsim);

P=0*ones(n,n,Tsim);
M=0*ones(n,n,Tsim+1); 
M(:,:,1)=eye(n,n); % (a guess)

xbar=0*ones(n,Tsim+1); % (a guess)
xhat=0*ones(n,Tsim); 


% Kalman filter, for state estimate

for k=1:Tsim
    
    x(:,k+1)=Phi*x(:,k)+Gamma*u(:,k)+Gamma1*w(:,k); % Case when KF knows model exactly
    y(:,k)=H*x(:,k)+v(:,k);
        
    P(:,:,k)=M(:,:,k)-M(:,:,k)*H'*inv(H*M(:,:,k)*H'+Rv)*H*M(:,:,k);
    xhat(:,k)=xbar(:,k)+P(:,:,k)*H'*inv(Rv)*(y(:,k)-H*xbar(:,k));
    
    xbar(:,k+1)=Phi*xhat(:,k)+Gamma*u(:,k);
    M(:,:,k+1)=Phi*P(:,:,k)*Phi'+Gamma1*Rw*Gamma1';
    
end
    
% Plot results

time=0:Tsim;
timeless=0:Tsim-1;

for i=1:601
    M1(i) = M(1,1,i);
    M2(i) = M(1,2,i);
    M3(i) = M(2,2,i);
end 
for i=1:600
    P1(i) = P(1,1,i);
    P2(i) = P(1,2,i);
    P3(i) = P(2,2,i);
end 

figure(1)
subplot(2,1,1)
plot(time,x,'b',timeless,xhat,'r')
title('x (blue) and xhat (red)')
xlabel('Time step')
ylabel('Magnitude')
legend('X1','X2','X1-hat','X2-hat','location','best')
subplot(212)
plot(time,x,'b',time,xbar,'r')
title('x (blue) and xbar (red)')
xlabel('Time step')
ylabel('Magnitude')
legend('X1','X2','X1-bar','X2-bar','location','best')

figure(2)
subplot(2,1,1)
plot(time,M1,'linewidth',1.5)
hold on
plot(time,M2,'linewidth',1.5)
hold on
plot(time,M3,'linewidth',1.5)
title('M Matrix')
xlabel('Time step')
ylabel('Magnitude')
legend('Covariance 1 and 1','Covariance 1 and 2','Covariance 2 and 2','location','best')


subplot(2,1,2)
plot(timeless,P1,'linewidth',1.5)
hold on
plot(timeless,P2,'linewidth',1.5)
hold on
plot(timeless,P3,'linewidth',1.5)
title('P Matrix')
xlabel('Time step')
ylabel('Magnitude')
legend('Covariance 1 and 1','Covariance 1 and 2','Covariance 2 and 2','location','best')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
