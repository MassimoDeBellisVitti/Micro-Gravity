sampleTime=0.01;
ts=SampleTime;

g0= -9.80665;
load("attitude_platform_DataFile.mat")

var_tau = 0;

var_W_n =1e-9;    
var_Q_n =1e-8;  

Theta = diag(var_W_n*ones(1, 3))*ts;
Psi = diag(var_Q_n*ones(1, 4))*ts;

R=blkdiag(Theta,Psi);
lambda=7.74264;
Q = lambda * R ;

H= eye(7);

J0 = diag([0.15847048678352, 0.14744391045352, 0.22612353624895]);

P0=eye(7)*1e-0;

simOut = sim('SimplifiedModel');
    
    inn = simOut.yout{7}.Values.Data;  
    rmse = sqrt(mean(inn.^2, 'all'))

    var = simOut.yout{5}.Values.Data;  
    var_values = mean(var(end,:))