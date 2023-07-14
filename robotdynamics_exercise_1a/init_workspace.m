% Add folders to path
thisfile = which(mfilename);
exercisefolder = fileparts(thisfile);
cd(exercisefolder);
addpath(genpath(exercisefolder));
cd('..\');
addpath(genpath('Matlab_Functions'));
cd('robotdynamics_exercise_1a');
clear thisfile exercisefolder;
