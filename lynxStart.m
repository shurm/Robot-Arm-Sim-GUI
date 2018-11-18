function lynxStart()
% lynxStart  Loads kinematic data for a Lynx AL5D manipulator and sets
%   variables.
%

global lynx robotName q pennkeys

%%SET THIS
pennkeys = 'sol';

lynx.firstFrame = true;
lynx.hardware_on = false;
lynx.showFrame = true;
lynx.showJoints = true;
lynx.showShadow = true;
lynx.showGripper = false;

% Home pose
q = [0,0,0,0,0,0];


plotLynx(q);


% Set global variables in the base workspace
evalin('base', 'global lynx q')


