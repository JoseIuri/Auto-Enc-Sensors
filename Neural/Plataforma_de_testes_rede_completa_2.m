%% Clearing workspace
clear all
close all
clc
%% Running V-REP and loading test scene
%winopen('Plataforma_de_testes.ttt');
%% Loading V-REP remote interface - client side
vrep = remApi('remoteApi');
%% Closing any previously opened connections
vrep.simxFinish(-1);
%% Connecting to remote V-REP API server
retCod = 0;
connectionAddress = '127.0.0.1';
connectionPort = 19997;
waitUntilConnected = true;
doNotReconnectOnceDisconnected = true;
timeOutInMs = 5000;
commThreadCycleInMs = 5;
while(retCod == 0)
    [clientID]=vrep.simxStart(connectionAddress,connectionPort,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs);
    if(clientID > -1),
        fprintf('Starting\n');
        retCod = 1;
    else
        fprintf ('Waiting\n');
    end
end
%% Getting robot handles
[retCod,plataforma] = vrep.simxGetObjectHandle(clientID,'UAI',vrep.simx_opmode_blocking);
[retCod,disco] = vrep.simxGetObjectHandle(clientID,'Disco',vrep.simx_opmode_blocking);

[retCod,S13] = vrep.simxGetObjectHandle(clientID,'S_1',vrep.simx_opmode_blocking);
[retCod,S12] = vrep.simxGetObjectHandle(clientID,'S_2',vrep.simx_opmode_blocking);
[retCod,S11] = vrep.simxGetObjectHandle(clientID,'S_3',vrep.simx_opmode_blocking);
[retCod,S10] = vrep.simxGetObjectHandle(clientID,'S_4',vrep.simx_opmode_blocking);
[retCod,S9] = vrep.simxGetObjectHandle(clientID,'S_5',vrep.simx_opmode_blocking);
[retCod,S8] = vrep.simxGetObjectHandle(clientID,'S_6',vrep.simx_opmode_blocking);
[retCod,S7] = vrep.simxGetObjectHandle(clientID,'S_7',vrep.simx_opmode_blocking);
[retCod,S6] = vrep.simxGetObjectHandle(clientID,'S_8',vrep.simx_opmode_blocking);
[retCod,S5] = vrep.simxGetObjectHandle(clientID,'S_9',vrep.simx_opmode_blocking);
[retCod,S4] = vrep.simxGetObjectHandle(clientID,'S_10',vrep.simx_opmode_blocking);
[retCod,S3] = vrep.simxGetObjectHandle(clientID,'S_11',vrep.simx_opmode_blocking);
[retCod,S2] = vrep.simxGetObjectHandle(clientID,'S_12',vrep.simx_opmode_blocking);
[retCod,S1] = vrep.simxGetObjectHandle(clientID,'S_13',vrep.simx_opmode_blocking);
[retCod,S26] = vrep.simxGetObjectHandle(clientID,'S_14',vrep.simx_opmode_blocking);
[retCod,S25] = vrep.simxGetObjectHandle(clientID,'S_15',vrep.simx_opmode_blocking);
[retCod,S24] = vrep.simxGetObjectHandle(clientID,'S_16',vrep.simx_opmode_blocking);
[retCod,S23] = vrep.simxGetObjectHandle(clientID,'S_17',vrep.simx_opmode_blocking);
[retCod,S22] = vrep.simxGetObjectHandle(clientID,'S_18',vrep.simx_opmode_blocking);
[retCod,S21] = vrep.simxGetObjectHandle(clientID,'S_19',vrep.simx_opmode_blocking);
[retCod,S20] = vrep.simxGetObjectHandle(clientID,'S_20',vrep.simx_opmode_blocking);
[retCod,S19] = vrep.simxGetObjectHandle(clientID,'S_21',vrep.simx_opmode_blocking);
[retCod,S18] = vrep.simxGetObjectHandle(clientID,'S_22',vrep.simx_opmode_blocking);
[retCod,S17] = vrep.simxGetObjectHandle(clientID,'S_23',vrep.simx_opmode_blocking);
[retCod,S16] = vrep.simxGetObjectHandle(clientID,'S_24',vrep.simx_opmode_blocking);
[retCod,S15] = vrep.simxGetObjectHandle(clientID,'S_25',vrep.simx_opmode_blocking);
[retCod,S14] = vrep.simxGetObjectHandle(clientID,'S_26',vrep.simx_opmode_blocking);
%% Defining robot initial position
% Position
xp0 = 0;
yp0 = 0;
zp0 = 0;
% Orientation
ap0 = 0;
bp0 = pi;
cp0 = 0;
[retCod] = vrep.simxSetObjectPosition(clientID,plataforma,-1,[xp0,yp0,zp0],vrep.simx_opmode_oneshot);
[retCod] = vrep.simxSetObjectOrientation(clientID,plataforma,-1,[ap0,bp0,cp0],vrep.simx_opmode_oneshot);
%% Parâmetros da Plataforma de testes

x = -47:1:47;
L = 0.064;
D = 0.0478;
p = 0.001;
yp = x*p;


alphaMax = (180/pi)*atan(((D/p)-abs(x))/(L/(2*p)));
alphavMax = 0.5*fix(alphaMax/0.5);

N =  2*alphavMax/0.5 + 1;

Ntotal = sum(N);


%% Defining V-REP client side controller parameters
np = Ntotal;
hd = 50e-3;
tc = 0;
td = 0;

%
t = zeros(np,1);
%

%% Test Parameters


Gama = [-47:1:47];
AlphaMax = (180/pi)*atan(((D/p)-abs(Gama))/(L/(2*p)));
AlphavMax = 0.5*fix(alphaMax/0.5);

Thetam = [];
Gamam = [];

for c = 1:length(Gama)
    m = -AlphavMax(c):0.5:AlphavMax(c);
    Gamam = [Gamam (zeros(1,length(m)) + Gama(c))];
    Theta = m;
    Thetam = [Thetam Theta];
end

Gamam = 0.001*Gamam;
Thetam = (pi/180)*Thetam;

Pv = 0:1:(length(Gamam)-1);

tf = length(Pv)*hd;

id = 1;
iy = 1;
in = 1;

Z1 = zeros((length(Pv)), 7);
Z2 = zeros((length(Pv)), 7);
%% Starting V-REP simulation
[retCod] = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);



%% Main control loop - V-REP client side
t0 = cputime;
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S1, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S2, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S3, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S4, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S5, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S6, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S7, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S8, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S9, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S10, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S11, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S12, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S13, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S14, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S15, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S16, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S17, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S18, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S19, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S20, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S21, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S22, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S23, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S24, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S25, vrep.simx_opmode_streaming);
[retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S26, vrep.simx_opmode_streaming);

while tc < tf,
    tc = cputime - t0;
    %% Current sampling instant
    if tc > td,     
        t(id) = tc;
        if iy <= length(Pv)
        [retCod] = vrep.simxSetObjectPosition(clientID,plataforma,-1,[xp0,Gamam(iy),zp0],vrep.simx_opmode_oneshot);
        [retCod] = vrep.simxSetObjectOrientation(clientID,disco,-1,[0,0,Thetam(iy)],vrep.simx_opmode_oneshot);
        
        %% reading IR Sensors
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S1, vrep.simx_opmode_buffer);
        Z1(iy,1) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S2, vrep.simx_opmode_buffer);
        Z1(iy,2) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S3, vrep.simx_opmode_buffer);
        Z1(iy,3) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S4, vrep.simx_opmode_buffer);
        Z1(iy,4) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S5, vrep.simx_opmode_buffer);
        Z1(iy,5) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S6, vrep.simx_opmode_buffer);
        Z1(iy,6) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S7, vrep.simx_opmode_buffer);
        Z1(iy,7) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S8, vrep.simx_opmode_buffer);
        Z1(iy,8) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S9, vrep.simx_opmode_buffer);
        Z1(iy,9) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S10, vrep.simx_opmode_buffer);
        Z1(iy,10) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S11, vrep.simx_opmode_buffer);
        Z1(iy,11) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S12, vrep.simx_opmode_buffer);
        Z1(iy,12) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S13, vrep.simx_opmode_buffer);
        Z1(iy,13) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S14, vrep.simx_opmode_buffer);
        Z2(iy,1) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S15, vrep.simx_opmode_streaming);
        Z2(iy,2) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S16, vrep.simx_opmode_streaming);
        Z2(iy,3) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S17, vrep.simx_opmode_streaming);
        Z2(iy,4) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S18, vrep.simx_opmode_streaming);
        Z2(iy,5) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S19, vrep.simx_opmode_streaming);
        Z2(iy,6) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S20, vrep.simx_opmode_streaming);
        Z2(iy,7) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S21, vrep.simx_opmode_streaming);
        Z2(iy,8) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S22, vrep.simx_opmode_streaming);
        Z2(iy,9) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S23, vrep.simx_opmode_streaming);
        Z2(iy,10) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S24, vrep.simx_opmode_streaming);
        Z2(iy,11) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S25, vrep.simx_opmode_streaming);
        Z2(iy,12) = auxData(1,11);
        [retCod,detectionState,auxData,auxPacketInfo]=vrep.simxReadVisionSensor(clientID, S26, vrep.simx_opmode_streaming);
        Z2(iy,13) = auxData(1,11);
        
        end
       
        %% Next sampling instant
        iy = iy + 1;
        td = td + hd;
        id = id + 1;
    end
end
%% Stoping V-REP simulation
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
fprintf('Ending\n');
%
vrep.simxFinish(clientID);
vrep.delete();