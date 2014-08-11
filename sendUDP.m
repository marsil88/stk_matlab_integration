%
% @file        sendUDP.m
% @brief       Send data via UDP.
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

function sendUDP( u1, X0, falha )

Mid     = uint8( 25 );

Acc     = single( X0( 17:19 ) );
Ast     = uint32( [ 1 1 1 ] );

Mag     = single( 1e4*X0( 14:16 ) );
Mst     = uint32( [ 1 1 1 ] );

Gyr     = single( X0( 1:3 ) );
Gst     = uint32( [ 1 1 1 ] );

Quat    = single( X0( 4:7 ) );
[ a, b, c] = quat2angle( Quat, 'xyz' );
Euler   = single( real( [ a b c ] ) );

Motor   = int16( X0( 8:10 ) * 60 / (2*pi) );

switch falha
    case 0
        Most    = uint32( [ 1 1 1 ] );
    case 1
        Most    = uint32( [ 0 1 1 ] );
    case 2
        Most    = uint32( [ 1 0 1 ] );
    case 3
        Most    = uint32( [ 1 1 0 ] );
end

Opt     = uint8( 26 );

data = [];
data = [ data ( typecast( Mid, 'uint8' ) ) ];

data = [ data typecast( swapbytes( Acc( 1 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Ast( 1 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Acc( 2 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Ast( 2 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Acc( 3 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Ast( 3 ) ), 'uint8' ) ];

data = [ data typecast( swapbytes( Mag( 1 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Mst( 1 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Mag( 2 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Mst( 2 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Mag( 3 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Mst( 3 ) ), 'uint8' ) ];

data = [ data typecast( swapbytes( Gyr( 1 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Gst( 1 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Gyr( 2 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Gst( 2 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Gyr( 3 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Gst( 3 ) ), 'uint8' ) ];

data = [ data typecast( swapbytes( Euler ), 'uint8' ) ];

data = [ data typecast( swapbytes( Quat ), 'uint8' ) ];

data = [ data typecast( swapbytes( Motor( 1 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Most( 1 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Motor( 2 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Most( 2 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Motor( 3 ) ), 'uint8' ) ];
data = [ data typecast( swapbytes( Most( 3 ) ), 'uint8' ) ];

data = [ data typecast( swapbytes( Opt ), 'uint8' ) ];

fwrite( u1, data );

return