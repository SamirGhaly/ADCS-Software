%==========================================================================
%
% background  Sets the plot background for drawing celestial bodies in 3D.
%
%   background(spec)
%
% See also planet3D, ground_track.
%
% Copyright © 2021 Tamas Kis
% Last Update: 2021-12-12
% Website: https://tamaskis.github.io
% Contact: tamas.a.kis@outlook.com
%
% TECHNICAL DOCUMENTATION:
% https://tamaskis.github.io/documentation/Visualizing_Celestial_Bodies_in_3D.pdf
%
% REFERENCES:
%   [1] https://www.solarsystemscope.com/textures/
%
%--------------------------------------------------------------------------
%
% ------
% INPUT:
% ------
%   spec    - (char) 'Black', 'Stars', or 'Milky Way'
%
% -----
% NOTE:
% -----
%   --> The function call on "background" MUST occur before the function
%       call on "planet3D".
%
%==========================================================================
function background(spec)
    
    % picture background
    if (~strcmpi(spec,'Black'))
        
        % determines which image background to use
        if strcmpi(spec,'Earth')
            cdata_background = imread('earth.png');
        elseif strcmpi(spec,'Stars')
            cdata_background = imread('stars.png');
        end
        
        % axis for background
        axis;
        
        % renders background
        imshow(cdata_background);
        
        % starts new axes object on which planet can be rendered
        axes;
        
        % turns coordinate axes off
        axis off;
        
    % black background
    elseif strcmpi(spec,'Black')
        axis;
        set(gca,'color','k');
    end
    
end