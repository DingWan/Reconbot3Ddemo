function L = fspl(R,lambda)
%fspl     Free space path loss
%   L = fspl(R,LAMBDA) returns the free space path loss L (in dB) suffered
%   by a signal with wavelength LAMBDA (in meters) when it is propagated in
%   free space for a distance of R (in meters). R can be a vector but
%   LAMBDA must be a scalar. L has the same dimensionality as R. Each
%   element in L is the free space path loss for the corresponding
%   propagation distance specified in R.
%
%   Note that the best case is lossless so the loss is always greater than
%   or equal to 0 dB.
%
%   % Example:
%   %   Calculate the free space loss for a signal whose wavelength is 30
%   %   cm. The signal is propagated for 1 km.
%
%   L = fspl(1000,0.3)
%
%   See also phased, phased.FreeSpace.

%   Copyright 2010 The MathWorks, Inc.

%   Reference
%   [1] John Proakis, Digital Communications, 4th Ed., McGraw-Hill, 2001

%#codegen 
%#ok<*EMCA

eml_assert_no_varsize(2, R,lambda);
sigdatatypes.validateDistance(R,'fspl','R',{'vector'});
sigdatatypes.validateDistance(lambda,'fspl','LAMBDA',...
    {'scalar','positive'});

L = 4*pi*R./lambda;
% L(L<1) = 1;
L = validateLoss(L);
L = mag2db(L);

end

function L = validateLoss(L)
    for m = 1:numel(L)
        if L(m)<1
            L(m)=1;  % Loss cannot be less than 1
        end
    end
end



% [EOF]
