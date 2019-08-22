load dados_rede.mat

X = X*2 - 1;

hiddenSize = 10;
hiddenSize2 = 2;
autoenc1 = trainAutoencoder(X,hiddenSize,...
    'L2WeightRegularization',0.001,...
    'SparsityRegularization',8,...
    'SparsityProportion',0.005,...
    'DecoderTransferFunction','logsig');

features1 = encode(autoenc1,X);

autoenc2 = trainAutoencoder(features1,hiddenSize2,...
    'L2WeightRegularization',0.001,...
    'SparsityRegularization',8,...
    'SparsityProportion',0.005,...
    'DecoderTransferFunction','logsig');

features2 = encode(autoenc2,features1);

inputs = features2;
targets = [10*Gamam; Thetam];
 
% Create a Fitting Network

hiddenLayerSize = 4;
net = fitnet(hiddenLayerSize);

% Set up Division of Data for Training, Validation, Testing
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 15/100;
net.divideParam.testRatio = 15/100;
 
% Train the Network
[net,tr] = train(net,inputs,targets);

stackednet = stack(autoenc1, autoenc2, net);

deepnet = train(stackednet,X,targets);



%Test
outputs = deepnet(X);
errors = gsubtract(outputs,targets);
performance = perform(net,targets,outputs)