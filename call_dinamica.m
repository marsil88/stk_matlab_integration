%
% @file        call_dinamica.m
% @brief       Calls the dynamic model of the satellite.
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

try
    fclose( u1 );
    fclose( u2 );
    stkClose( conid );
    stkClose();
catch ME
    disp( 'Nao fechou' );
end

u1 = udp( '192.168.0.52', 35204, 'LocalPort', 35308 );
u2 = udp( '127.0.0.1', 40001, 'LocalPort', 40000 ); % camera

% u1 = udp( '127.0.0.1', 35204, 'LocalPort', 35308 );

u1.DatagramTerminateMode = 'off';
u1.InputBufferSize = 2048;
u1.Timeout = 10;
fopen( u1 );
fopen( u2 );

%% Variaveis usadas no STK
% Satellite path
satpath = '*/Satellite/musat1';
% Cenario path
cenario = 'Scenario/cenario_tese';
% Central body
cb = 'Earth';

t0 = 0;
ts = 0.5;
tf = 10 * 60;

stkInit;
remMachine = stkDefaultHost;
conid = stkOpen( remMachine );

stkExec( conid, [ 'Animate ' cenario ' Reset' ] );
stkExec( conid, [ 'SetAnimation ' cenario ' TimeStep ' num2str( ts ) ] );

%% Chama a dinamica somente com o integrador
i = 1;

INST_FALHA = 20 * 60;
falha = 0; % roda X - 1 roda Y - 2 roda Z - 3 nenhum - 0

acc_BODY    = zeros( round( ( tf - t0 ) / ts ) + 1, 3 );
mag_BODY    = zeros( round( ( tf - t0 ) / ts ) + 1, 3 );
torque_grav = zeros( round( ( tf - t0 ) / ts ) + 1, 3 );

w0          = [ 0 0 0 ];
q0          = [ 1 0 0 0 ];
W0          = [ 0 0 0 ];
U0          = [ 0 0 0 ];

X0 = [ w0 q0 W0 U0 ];

X = zeros( round( tf - t0 ) / ts, 13 );
X( i, : ) = X0;

quat = X( i, [ 5 6 7 4 ] )';
stkSetAttitudeCBI( satpath, cb, 0, quat );
tic;
sendUDP( u1, [ X0 mag_BODY( i, : ) acc_BODY( i, : ) ], 0 );
for t = t0 : ts : tf
    
    [ U u1 ] = receiveUDP( u1 );
    
    X0( 11:13 ) = U;
    [ tout, yout ] = ode23tb( @dinamica_esfera, [ t t+ts ], X0, odeset(), torque_grav( i, : ) );
    X0 = yout( end, : );
    i = i+1;
    X( i, : ) = X0;
    
    quat = X0( :, [ 5 6 7 4 ] )';
    stkSetAttitudeCBI( satpath, cb, t+ts, quat );
    
    stkExec( conid, [ 'Animate ' cenario ' Step Forward' ] );
    [ mag_BODY( i, : ), acc_BODY( i, : ), torque_grav( i, : ) ] = stk_report( t, t, ts, satpath );
    
    if t > INST_FALHA
        sendUDP( u1, [ X0 mag_BODY( i, : ) acc_BODY( i, : ) ], 1 );
    else
        sendUDP( u1, [ X0 mag_BODY( i, : ) acc_BODY( i, : ) ], 0 );
    end
    
    if( u2.BytesAvailable > 0 )
        str = fread( u2, u2.BytesAvailable );
        stkExec( conid, 'VO * SnapFrame ToFile "C:\Users\Marsil\Desktop\ima_stk\stk.bmp"' );
    end
    
    toc;
end

a = [];
[ a( :, 1 ) a( :, 2 ) a( :, 3 ) ] = quat2angle( X( :, 4:7 ), 'xyz' );

w = X( :, 1:3 );
q = X( :, 4:7 );
W = X( :, 8:10 );
U = X( :, 11:13 );


fclose( u1 );
fclose( u2 );
stkClose( conid );
stkClose();