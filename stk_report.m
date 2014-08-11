%
% @file        stk_report.m
% @brief       Get report from STK.
%
% @author      Marsil de Athayde Costa e Silva
% 
% @date        2013
% @version     1.0
% @pre         
% @bug         
% @copyright   Copyright (c) 2014
%
% @details     Please checkout this paper before:
% DE ATHAYDE COSTA E SILVA, MARSIL; DE FIGUEIREDO, HELOSMAN VALENTE; BOGLIETTI, BENJAMIN GILLES NICOLAS; SAOTOME, OSAMU; VILLANI, EMÍLIA; KIENITZ, KARL HEINZ. A Framework for Development of Satellite Attitude Control Algorithms. Journal of Control, Automation and Electrical Systems, v. 1, p. 1, 2014. 
% @article{
%   year={2014},
%   issn={2195-3880},
%   journal={Journal of Control, Automation and Electrical Systems},
%   doi={10.1007/s40313-014-0141-7},
%   title={A Framework for Development of Satellite Attitude Control Algorithms},
%   url={http://dx.doi.org/10.1007/s40313-014-0141-7},
%   publisher={Springer US},
%   keywords={Model-based development; Software-in-the-loop; Hardware-in-the-loop; Attitude control simulator},
%   author={de Athayde Costa e Silva, Marsil and de Figueiredo, HelosmanValente and Boglietti, BenjaminGillesNicolas and Saotome, Osamu and Villani, Emília and Kienitz, KarlHeinz},
%   pages={1-11},
%   language={English}
%   }
% 
%

function [ mag_BODY, acc_BODY, torque_grav ] = stk_report( t0, time, timestep, satpath )

[ secData, secNames ] = stkReport( satpath, 'report_ima', t0, time, timestep );

% campo em Tesla BODY
[ mag_BODY_x, type ] = stkFindData( secData{1}, 'x' );
[ mag_BODY_y, type ] = stkFindData( secData{1}, 'y' );
[ mag_BODY_z, type ] = stkFindData( secData{1}, 'z' );
mag_BODY = [ mag_BODY_x, mag_BODY_y, mag_BODY_z ];

% aceleracao do acelerometro
[ acc_BODY_x, type ] = stkFindData( secData{2}, 'ax' );
[ acc_BODY_y, type ] = stkFindData( secData{2}, 'ay' );
[ acc_BODY_z, type ] = stkFindData( secData{2}, 'az' );
acc_BODY = [ acc_BODY_x, acc_BODY_y, acc_BODY_z ];

% torque gravitacional
[ torque_grav_x ,type] = stkFindData( secData{3}, 'x' );
[ torque_grav_y ,type] = stkFindData( secData{3}, 'y' );
[ torque_grav_z ,type] = stkFindData( secData{3}, 'z' );
torque_grav = [ torque_grav_x, torque_grav_y, torque_grav_z ];

return