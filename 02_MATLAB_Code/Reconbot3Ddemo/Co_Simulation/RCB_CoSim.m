% Adams / MATLAB Interface - Release 2014.0.0
system('taskkill /IM scontrols.exe /F >NUL');clc;
global ADAMS_sysdir; % used by setup_rtw_for_adams.m
global ADAMS_host; % used by start_adams_daemon.m
machine=computer;
datestr(now)
if strcmp(machine, 'SOL2')
   arch = 'solaris32';
elseif strcmp(machine, 'SOL64')
   arch = 'solaris32';
elseif strcmp(machine, 'GLNX86')
   arch = 'linux32';
elseif strcmp(machine, 'GLNXA64')
   arch = 'linux64';
elseif strcmp(machine, 'PCWIN')
   arch = 'win32';
elseif strcmp(machine, 'PCWIN64')
   arch = 'win64';
else
   disp( '%%% Error : Platform unknown or unsupported by Adams/Controls.' ) ;
   arch = 'unknown_or_unsupported';
   return
end
if strcmp(arch,'win64')
   [flag, topdir]=system('adams2014_x64 -top');
else
   [flag, topdir]=system('adams2014 -top');
end
if flag == 0
  temp_str=strcat(topdir, '/controls/', arch);
  addpath(temp_str)
  temp_str=strcat(topdir, '/controls/', 'matlab');
  addpath(temp_str)
  temp_str=strcat(topdir, '/controls/', 'utils');
  addpath(temp_str)
  ADAMS_sysdir = strcat(topdir, '');
else
  addpath( 'C:\MSC~1.SOF\ADAMS_~1\2014\controls/win64' ) ;
  addpath( 'C:\MSC~1.SOF\ADAMS_~1\2014\controls/win32' ) ;
  addpath( 'C:\MSC~1.SOF\ADAMS_~1\2014\controls/matlab' ) ;
  addpath( 'C:\MSC~1.SOF\ADAMS_~1\2014\controls/utils' ) ;
  ADAMS_sysdir = 'C:\MSC~1.SOF\ADAMS_~1\2014\' ;
end
ADAMS_exec = '' ;
ADAMS_host = 'giselle.igm.rwth-aachen.de' ;
ADAMS_cwd ='C:\DING\ADAMS ideas\Reconbot\Reconbot_DualArm_CoSim\ADAMS_Files'  ;
ADAMS_prefix = 'RCB_CoSim' ;
ADAMS_static = 'no' ;
ADAMS_solver_type = 'C++' ;
if exist([ADAMS_prefix,'.adm']) == 0
   disp( ' ' ) ;
   disp( '%%% Warning : missing ADAMS plant model file(.adm) for Co-simulation or Function Evaluation.' ) ;
   disp( '%%% If necessary, please re-export model files or copy the exported plant model files into the' ) ;
   disp( '%%% working directory.  You may disregard this warning if the Co-simulation/Function Evaluation' ) ;
   disp( '%%% is TCP/IP-based (running Adams on another machine), or if setting up MATLAB/Real-Time Workshop' ) ;
   disp( '%%% for generation of an External System Library.' ) ;
   disp( ' ' ) ;
end
ADAMS_init = '' ;
ADAMS_inputs  = 'q0!q11!q12!q14!q21!q22!q23!LeftArmAngle!RightArmAngle!Slide' ;
ADAMS_outputs = 'TCP_x!TCP_Y!TCP_Z!Torque0!Torque11!Torque12!Torque14!Torque21!Torque22!Torque23' ;
ADAMS_pinput = 'RCB_CoSim.ctrl_pinput' ;
ADAMS_poutput = 'RCB_CoSim.ctrl_poutput' ;
ADAMS_uy_ids  = [
                   1
                   2
                   3
                   4
                   5
                   6
                   7
                   9
                   10
                   8
                   11
                   12
                   13
                   24
                   25
                   26
                   27
                   28
                   29
                   30
                ] ;
ADAMS_mode   = 'non-linear' ;
tmp_in  = decode( ADAMS_inputs  ) ;
tmp_out = decode( ADAMS_outputs ) ;
disp( ' ' ) ;
disp( '%%% INFO : ADAMS plant actuators names :' ) ;
disp( [int2str([1:size(tmp_in,1)]'),blanks(size(tmp_in,1))',tmp_in] ) ;
disp( '%%% INFO : ADAMS plant sensors   names :' ) ;
disp( [int2str([1:size(tmp_out,1)]'),blanks(size(tmp_out,1))',tmp_out] ) ;
disp( ' ' ) ;
clear tmp_in tmp_out ;
% Adams / MATLAB Interface - Release 2014.0.0
