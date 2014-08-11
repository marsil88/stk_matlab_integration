%
% @file        dinamica_esfera.m
% @brief       Dynamics model of the satellite.
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

function [ dx ] = dinamica_esfera( t, X, Tex )

w = X( 1:3 );
q = X( 4:7 );
W = X( 8:10 );
U = X( 11:13 );

I = [ 0.135845202 -0.000022908  -0.000174394;
     -0.000022908  0.135910014   0.000229520;
     -0.000174394  0.000229520   0.122729410; ];

Ix = I( 1, 1 );
Iy = I( 2, 2 );
Iz = I( 3, 3 );

J  = 0.000624567 + 3e-06;

Aw = [ -1  0  0;
        0 -1  0;
        0  0 -1; ];
Bw = [ 1 0 0;
       0 1 0;
       0 0 1; ];
% Kp = [ 0.0149    0.0150    0.0135 ];
% Kr = [ 0.2174    0.2175    0.1964 ];
% 
% qe = quatconj( quatmultiply( quatinv( q' ), qt' ) );
% 
% dU = [ Kp( 1 ) / J       0       0 Kr( 1 ) / J       0       0;
%              0 Kp( 2 ) / J       0       0 Kr( 2 ) / J       0;
%              0       0 Kp( 3 ) / J       0       0 Kr( 3 ) / J; ] * [ qe(2:4)'; w ];

dU = [ 0; 0; 0 ];

dW = Aw * W + Bw * U;

dw( 1 ) = ( ( Iy - Iz ) * w( 2 ) * w( 3 ) ...
          - J * dW( 1 ) ...
          + ( w( 3 ) * W( 2 ) - w( 2 ) * W( 3 ) ) * J ...
          + Tex( 1 ) ) / Ix;

dw( 2 ) = ( ( Iz - Ix ) * w( 3 ) * w( 1 ) ...
          - J * dW( 2 ) ...
          + ( w( 1 ) * W( 3 ) - w( 3 ) * W( 1 ) ) * J ...
          + Tex( 2 ) ) / Iy;

dw( 3 ) = ( ( Ix - Iy ) * w( 1 ) * w( 2 ) ...
          - J * dW( 3 ) ...
          + ( w( 2 ) * W( 1 ) - w( 1 ) * W( 2 ) ) * J ...
          + Tex( 3 ) ) / Iz;

dq = ( 1/2 ) * [      0 -w( 1 ) -w( 2 ) -w( 3 );
                 w( 1 )       0  w( 3 ) -w( 2 );
                 w( 2 ) -w( 3 )      0   w( 1 );
                 w( 3 )  w( 2 ) -w( 1 )      0; ] * q;

dx = [ dw'; dq; dW; dU; ];

return