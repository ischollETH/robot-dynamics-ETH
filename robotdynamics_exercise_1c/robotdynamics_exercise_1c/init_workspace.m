% Add folders to path
thisfile = which(mfilename);
exercisefolder = fileparts(thisfile);
cd(exercisefolder);
addpath(genpath(exercisefolder));
cd('..\');
cd('..\');
addpath(genpath('Matlab_Functions'));
clear thisfile exercisefolder;
