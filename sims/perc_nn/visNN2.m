function [Y,Xf,Af] = myNeuralNetworkFunction(X,~,~)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Auto-generated by MATLAB, 09-Jul-2020 15:37:12.
%
% [Y] = myNeuralNetworkFunction(X,~,~) takes these arguments:
%
%   X = 1xTS cell, 1 inputs over TS timesteps
%   Each X{1,ts} = Qx2 matrix, input #1 at timestep ts.
%
% and returns:
%   Y = 1xTS cell of 1 outputs over TS timesteps.
%   Each Y{1,ts} = Qx1 matrix, output #1 at timestep ts.
%
% where Q is number of samples (or series) and TS is the number of timesteps.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [-30;-30];
x1_step1.gain = [0.0666666666666667;0.0666666666666667];
x1_step1.ymin = -1;

% Layer 1
b1 = [-30.027141530035788008;0.8815238484627497817;5.1289861003278822693;1.3791298056233134428;-1.4979437425935291373;1.4200482355200279017;1.5149499984350154325;1.0604759918388775297;1.0794546395074957612;0.69120218609111527819;-4.9742952096754704527;4.4745604514596575285;-4.9478256783920357975;-2.0208096270415252249;-0.64348061616646878846;1.4439386823184223729;6.8148567026308572991;-2.3154950170806358756;0.84005106838869625374;30.403505634108014988];
IW1_1 = [15.081918607573809865 13.805690569305848925;-0.068374646429792573787 -0.47158144449147271215;0.43392781620752207683 -5.2465810165856145986;-3.1939470927442599191 2.0784068529698793704;3.3012851506541291968 -2.3919243819951363506;-3.7252409947764477671 1.5683998917882970847;-4.1821831067350370148 1.4987346671920975805;0.19783244315068343466 -0.8511291107598332184;-3.0120363782550998444 1.6379141507606151951;-3.3093651271897503996 1.6535420254157584541;-0.15408252592799126512 5.3789073456936051798;2.8341596027779449862 -5.9531561906937717765;-0.28322837396762634565 5.2018607893772372819;-0.62404908626680410944 -2.1871074860645403071;-1.4464481672301421344 0.15600872686868169636;-3.4760785585480991067 1.7755572188484385343;1.8519579263657075519 -7.9629510520444206634;-3.2792579094805773821 -2.096892681647950063;1.7109205795035511422 -0.29131522190874303835;-9.1951472810730798813 -20.113551580533748364];

% Layer 2
b2 = 5.5408434539127382479;
LW2_1 = [27.577378157347677501 -6.2979847977899154898 -8.1189304181203745259 17.849735849830071999 6.7734857889888573368 14.55713865853774891 -4.2533147946490537095 3.0044933672766322985 -10.091253353063073916 1.320199971638555736 5.9890802738757518142 -0.058845666320819840778 -14.321809482776981071 0.10289219476461283764 1.6889952049985836879 -12.993623566676753001 -0.13333509761653031256 0.031346680299160926964 1.1554497799551672887 23.704480043209940732];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = 0.00343551856653235;
y1_step1.xoffset = 0;

% ===== SIMULATION ========

% Format Input Arguments
isCellX = iscell(X);
if ~isCellX
    X = {X};
end

% Dimensions
TS = size(X,2); % timesteps
if ~isempty(X)
    Q = size(X{1},1); % samples/series
else
    Q = 0;
end

% Allocate Outputs
Y = cell(1,TS);

% Time loop
for ts=1:TS
    
    % Input 1
    X{1,ts} = X{1,ts}';
    Xp1 = mapminmax_apply(X{1,ts},x1_step1);
    
    % Layer 1
    a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*Xp1);
    
    % Layer 2
    a2 = repmat(b2,1,Q) + LW2_1*a1;
    
    % Output 1
    Y{1,ts} = mapminmax_reverse(a2,y1_step1);
    Y{1,ts} = Y{1,ts}';
end

% Final Delay States
Xf = cell(1,0);
Af = cell(2,0);

% Format Output Arguments
if ~isCellX
    Y = cell2mat(Y);
end
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
y = bsxfun(@minus,x,settings.xoffset);
y = bsxfun(@times,y,settings.gain);
y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
x = bsxfun(@minus,y,settings.ymin);
x = bsxfun(@rdivide,x,settings.gain);
x = bsxfun(@plus,x,settings.xoffset);
end