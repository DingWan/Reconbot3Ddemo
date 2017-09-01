function cad2matCollisionCheck(filename)
% CAD2MATDEMO, a demonstration of importing 3D CAD data into Matlab.
% To get CAD data into Matlab, the process is:
%
% 1) Export the 3D CAD data as an ASCII STL (or Pro/E render SLP) file.
% 2) This Matlab routine reads the CAD data
% 3) Once read, the CAD data is rotated around a bit.
%
% Program has been tested with: AutoCAD, Cadkey, and Pro/Engineer.
% Should work with most any CAD programs that can export STL.
% 
% Format Details:  STL is supported, and the color version of STL
% that Pro/E exports, called 'render.'  The render (SLP) is just 
% like STL but with color added.
% 
% Note: This routine has both the import function and some basic
% manipulation for testing.  The actual reading mechanism is located
% at the end of this file.
  
if nargin == 0    
   filename = {   'RCB_BaseLow.stl';
                  'RCB_BaseUP.stl'; 
                  'RCB_BaseJointA1C1.stl';
                  'RCB_LowLinkA1C1.stl'; 
                  'RCB_UpLinkA1C1.stl'; 
                  'RCB_UPjointA1C1.stl';
                  'RCB_BaseJointA2C2.stl';
                  'RCB_LowLinkA2C2.stl'; 
                  'RCB_UpLinkA2C2.stl'; 
                  'RCB_UPjointA2C2.stl';
                  'RCB_MP.stl'};
end

%--------------- Read the CAD data file: ---------------
    % Move it around.
    % To use homogenous transforms, the n by 3 Vertices will be turned to 
    % n by 4 vertices, then back to 3 for the set command.
    % Note: n by 4 needed for translations, not used here, but could, using tl(x,y,z)
% [F1, V1, C1] = rndread(filename{1});
% V1 = [V1(:,1:3), ones(length(V1),1)];
% BaseLow = struct('F1',F1, 'V1',V1, 'C1',C1);
% 
% [F2, V2, C2] = rndread(filename{2});
% V2 = [V2(:,1:3), ones(length(V2),1)];
% BaseUP = struct('F2',F2, 'V2',V2, 'C2',C2);
% 
% [F3, V3, C3] = rndread(filename{3});
% V3 = [V3(:,1:3), ones(length(V3),1)];
% BaseJoint_A1C1 = struct('F3',F3, 'V3',V3, 'C3',C3);

[F4, V4, C4] = rndread(filename{4});
V4 = [V4(:,1:3), ones(length(V4),1)];
LowLink_A1C1 = struct('F4',F4, 'V4',V4, 'C4',C4);

% [F5, V5, C5] = rndread(filename{5});
% V5 = [V5(:,1:3), ones(length(V5),1)];
% UpLink_A1C1 = struct('F5',F5, 'V5',V5, 'C5',C5);
% 
% [F6, V6, C6] = rndread(filename{6});
% V6 = [V6(:,1:3), ones(length(V6),1)];
% UPjoint_A1C1 = struct('F6',F6, 'V6',V6, 'C6',C6);
% 
% [F7, V7, C7] = rndread(filename{7});
% V7 = [V7(:,1:3), ones(length(V7),1)];
% BaseJoint_A2C2 = struct('F7',F7, 'V7',V7, 'C7',C7);
% 
% [F8, V8, C8] = rndread(filename{8});
% V8 = [V8(:,1:3), ones(length(V8),1)];
% LowLink_A2C2 = struct('F8',F8, 'V8',V8, 'C8',C8);
% 
% [F9, V9, C9] = rndread(filename{9});
% V9 = [V9(:,1:3), ones(length(V9),1)];
% UpLink_A2C2 = struct('F9',F9, 'V9',V9, 'C9',C9);
% 
% [F10, V10, C10] = rndread(filename{10});
% V10 = [V10(:,1:3), ones(length(V10),1)];
% UPjoint_A2C2 = struct('F10',F10, 'V10',V10, 'C10',C10);
% 
[F11, V11, C11] = rndread(filename{11});
V11 = [V11(:,1:3), ones(length(V11),1)];
MP = struct('F11',F11, 'V11',V11, 'C11',C11);
%-------------------------------------------- 
 
% Set File '??' as a example: 
F = F11(:,1:3);
V = V11(:,1:3);
C = C11;

clf;
  p = patch('faces', F, 'vertices' ,V);
    %set(p, 'facec', 'b');              % Set the face color (force it)
    set(p, 'facec', 'flat');            % Set the face color flat
    set(p, 'FaceVertexCData', C);       % Set the color (from file)
    %set(p, 'facealpha',.4)             % Use for transparency
    set(p, 'EdgeColor','none');         % Set the edge color
    %set(p, 'EdgeColor',[1 0 0 ]);      % Use to see triangles, if needed.
    light                               % add a default light
    daspect([1 1 1])                    % Setting the aspect ratio
    view(3)                             % Isometric view
    xlabel('X'),ylabel('Y'),zlabel('Z')
    title(['Imported CAD data from ' filename{3}])
    drawnow                             %, axis manual
    %
    disp(['CAD file ' filename{3} ' data is read, will now show object rotating'])
    
    hold on
    
    CollisionLines;

  
%% Move it around
function [fout, vout, cout] = rndread(filename)
% Reads CAD STL ASCII files, which most CAD programs can export.
% Used to create Matlab patches of CAD 3D data.
% Returns a vertex list and face list, for Matlab patch command.
% 
% filename = 'hook.stl';  % Example file.
%
fid=fopen(filename, 'r'); %Open the file, assumes STL ASCII format.
if fid == -1 
    error('File could not be opened, check name or path.')
end
%
% Render files take the form:
%   
%solid BLOCK
%  color 1.000 1.000 1.000
%  facet
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%    outer loop
%      vertex 5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 5.000000e-01 -5.000000e-01
%    endloop
% endfacet
%
% The first line is object name, then comes multiple facet and vertex lines.
% A color specifier is next, followed by those faces of that color, until
% next color line.
%
CAD_object_name = sscanf(fgetl(fid), '%*s %s');  %CAD object name, if needed.
%                                                %Some STLs have it, some don't.   
vnum=0;       %Vertex number counter.
report_num=0; %Report the status as we go.
VColor = 0;
%
while feof(fid) == 0                    % test for end of file, if not then do stuff
    tline = fgetl(fid);                 % reads a line of data from file.
    fword = sscanf(tline, '%s ');       % make the line a character string
% Check for color
    if strncmpi(fword, 'c',1) == 1;    % Checking if a "C"olor line, as "C" is 1st char.
       VColor = sscanf(tline, '%*s %f %f %f'); % & if a C, get the RGB color data of the face.
    end                                % Keep this color, until the next color is used.
    if strncmpi(fword, 'v',1) == 1;    % Checking if a "V"ertex line, as "V" is 1st char.
       vnum = vnum + 1;                % If a V we count the # of V's
       report_num = report_num + 1;    % Report a counter, so long files show status
       if report_num > 249;
           disp(sprintf('Reading vertix num: %d.',vnum));
           report_num = 0;
       end
       v(:,vnum) = sscanf(tline, '%*s %f %f %f'); % & if a V, get the XYZ data of it.
       c(:,vnum) = VColor;              % A color for each vertex, which will color the faces.
    end                                 % we "*s" skip the name "color" and get the data.                                          
end
%   Build face list; The vertices are in order, so just number them.
%
fnum = vnum/3;      %Number of faces, vnum is number of vertices.  STL is triangles.
flist = 1:vnum;     %Face list of vertices, all in order.
F = reshape(flist, 3,fnum); %Make a "3 by fnum" matrix of face list data.
%
%   Return the faces and vertexs.
%
fout = F';  %Orients the array for direct use in patch.
vout = v';  % "
cout = c';
%
fclose(fid);