function dataOut = resampleData(gOld, dataIn, gNew)
% Resamples the data to make it compatible with another grid (new grid must
% have the same domain as the old grid
%
% function dataOut = resampleData(dataIn)
%
% Inputs:
%   gOld        - old grid
%   dataIn      - data to resample
%   gNew        - new grid
%
% Outputs:
%   dataOut     - resampled data corresponding to gNew
% 
% Mo Chen, 2013-10-24
%

switch gOld.dim
    case 2
        dataOut = interpn(gOld.xs{1}, gOld.xs{2}, dataIn, ...
            gNew.xs{1}, gNew.xs{2});        
    case 4
        dataOut = interpn(gOld.xs{1}, gOld.xs{2}, gOld.xs{3}, gOld.xs{4}, dataIn, ...
            gNew.xs{1}, gNew.xs{2}, gNew.xs{3}, gNew.xs{4});
    otherwise
        error('resampleData has only been implemented for grids of 2 or 4 dimensions!')
end