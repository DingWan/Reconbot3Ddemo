%% Cuboid Path Geneation Test
% clear;clc;

% input: length, width, height and step of the Study Points

% x_bound = [-6, 16];y_bound = [-6, 16];z_bound = [-1, 3];%432steps
% x_bound = [-6, 16];y_bound = [-4, -1];z_bound = [-1, -1];%92steps
% x_bound = [-6, 16];y_bound = [-7, 0];z_bound = [-1, -1];%161steps
% x_bound = [-6, 16];y_bound = [-6, 0];z_bound = [1, 3];%483steps
x_bound = [-6, 6];y_bound = [-6, 6];z_bound = [-6, 6];%11638steps
% x_bound = [-6, 16];y_bound = [-6, -4];z_bound = [-2, 1];%276steps
% x_bound = [-6, 16];y_bound = [-6, 16];z_bound = [-2, -2];%529steps

step = 1;

cuboidPath = pathGeneration(x_bound,y_bound,z_bound,step);

save('pathCuboid.mat','cuboidPath');

%Systemdata.Path.Path0 = importdata('pathCuboid.mat');
Systemdata.Path.Path0 = cuboidPath;
Data.Path.Path0 = cuboidPath;
Systemdata.Workspace_xyzbound = [x_bound;y_bound;z_bound];
Data.Workspace_xyzbound = [x_bound;y_bound;z_bound];

%----------------- plot Path output  --------------
pathOP = Data.Path.Path0; % pathOP: path output
sizepathOP = size(pathOP);
 i = 1:sizepathOP; 
 plot3(pathOP(i,3), pathOP(i,4), pathOP(i,5),'k-','MarkerSize',5); 
 hold on;   
 axis equal;
%----------------------------------------------