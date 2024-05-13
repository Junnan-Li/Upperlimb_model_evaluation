% Path Initial 
% 
%
%

path_repo = pwd;

% Path of the Repo should be change 
filepath = fileparts(mfilename('fullpath'));

% Path of the OpenSim toolbox

Path_OpenSimToolbox = strcat(filepath, '\..\OpenSimToolbox');
% Path_functions = strcat(filepath, '\functions');

cd(Path_OpenSimToolbox)
run .\init.m;
cd(path_repo)
% Path_example= strcat(filepath, '\examples');
% 
% addpath(genpath(Path_functions));
% addpath(Path_classes);
% % addpath(genpath(Path_Code_tool));
% addpath(Path_Model);
% addpath(Path_Geometry);
% 
% addpath(Path_example);