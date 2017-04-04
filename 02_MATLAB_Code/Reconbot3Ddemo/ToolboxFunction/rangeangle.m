function [rng, ang] = rangeangle(pos, refpos, refaxes)
%rangeangle Range and angle calculation
%   [RNG,ANG] = rangeangle(POS) returns the range RNG (in meters) and angle
%   ANG (in degrees) of the input position POS with respect to the origin.
%   POS must be a 3xN matrix with each column specifying a position in the
%   form of [x; y; z] (in meters) coordinates. RNG is a 1xN vector whose
%   entries are the ranges for the corresponding positions specified in
%   POS. ANG is an 2xN matrix whose columns are the angles, in the form of
%   [azimuth; elevation], for the corresponding positions specified in POS.
%
%   [RNG,ANG] = rangeangle(POS,REFPOS) returns the ranges and angles of POS
%   with respect to the reference position specified in REFPOS. REFPOS is a
%   3x1 vector specifying the reference position in the form of [x; y; z]
%   (in meters) coordinates.
%
%   [RNG,ANG] = rangeangle(POS,REFPOS,REFAXES) returns the ranges and
%   angles of POS in the local coordinate system whose origin is REFPOS and
%   whose axes are defined in REFAXES. REFAXES must be a 3x3 matrix with
%   each column specifying the direction of an axis for the local
%   coordinate system in the form of [x; y; z] coordinates.
%
%   % Example:
%   %   A target is located at [500; 0; 0] meters and a receiver is located
%   %   at [100; 100; 100] meters. The receiver's local coordinates are 
%   %   given by [0 1 0;0 0 1;1 0 0]. Determine the range and angle of the 
%   %   target with respect to the receiver expressed in the receiver's
%   %   local coordinate system
%
%   [tgt_rng,tgt_ang] = rangeangle([500; 0; 0],[100; 100; 100],...
%                           [0 1 0;0 0 1;1 0 0])
%
%   See also phased, global2localcoord, local2globalcoord.

%   Copyright 2010-2011 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>
    
phased.internal.narginchk(1,3,nargin);

if nargin < 3
    refaxes = eye(3);
end

if nargin < 2
    refpos = [0; 0; 0];
end

eml_assert_no_varsize(2:3, pos, refpos, refaxes);
sigdatatypes.validate3DCartCoord(pos,'rangeangle','Pos');
sigdatatypes.validate3DCartCoord(refpos,'rangeangle','RefPos',{'column'});
sigdatatypes.validate3DCartCoord(refaxes,'rangeangle','RefAxes',{'size',[3 3]});

lclcoord = phased.internal.global2localcoord(pos,'rs',refpos,refaxes);
rng = lclcoord(3,:);
ang = lclcoord(1:2,:);


% [EOF]
