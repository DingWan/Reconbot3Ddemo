function [cuboidPath, xyzStepLength] = pathGeneration(x_bound,y_bound,z_bound,step)


% initialization 
xStep = x_bound(1):step:x_bound(2);
yStep = y_bound(1):step:y_bound(2);
zStep = z_bound(1):step:z_bound(2);

xStepLength = length(xStep);
yStepLength = length(yStep);
zStepLength = length(zStep);

% mutiple of matrix
temp = zeros(1,yStepLength*xStepLength);
for i = 1:yStepLength
    temp(1,(i-1)*xStepLength+1:(i-1)*xStepLength+xStepLength) = yStep(i);
end
yStep = temp;
clear temp;


xIStep = fliplr(xStep);
yIStep = fliplr(yStep);
% zIStep = fliplr(zStep);

pathStepLength = xStepLength * yStepLength * zStepLength;
xLoop = pathStepLength / xStepLength;
yLoop = pathStepLength / yStepLength / xStepLength;
zLoop = pathStepLength / zStepLength;


% start calculation
tic;
cuboidPath = zeros(pathStepLength,3);
for i=1:xLoop
    if(rem(i,2)>0)
        cuboidPath((i-1)*xStepLength+1:(i-1)*xStepLength+xStepLength,1) = transpose(xStep);
    else
        cuboidPath((i-1)*xStepLength+1:(i-1)*xStepLength+xStepLength,1) = transpose(xIStep);
    end
end

for i=1:yLoop
    if(rem(i,2)>0)
        cuboidPath((i-1)*yStepLength*xStepLength+1:(i-1)*yStepLength*xStepLength+yStepLength*xStepLength,2) = transpose(yStep);
    else
        cuboidPath((i-1)*yStepLength*xStepLength+1:(i-1)*yStepLength*xStepLength+yStepLength*xStepLength,2) = transpose(yIStep);
    end
end

for i=1:zStepLength
    cuboidPath(((i-1)*zLoop+1):(i*zLoop),3) = zStep(i);
end

column1 = 1:pathStepLength;
column2 = linspace(0,(pathStepLength-1)/10,pathStepLength);
column6 = linspace(0,0,pathStepLength);

cuboidPath = [column1',column2',cuboidPath,column6',column6',column6']; % enlarge the studyPoints matrix

xyzStepLength = [xStepLength, yStepLength, zStepLength];

toc

end


