%
% @file        receiveUDP.m
% @brief       Receive data from UDP.
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

function [ U u1 ] = receiveUDP( u1 )

a = uint8( fread( u1, 12 ) );

if isempty( a )
    U = [ 0 0 0 ];
else
    % {'uint8','uint8','int16','uint8','int16','uint8','int16','uint8', 'uint8'}
    U( 1 ) = double( typecast( a( 4:-1:3 ), 'int16' ) );
    U( 2 ) = double( typecast( a( 7:-1:6 ), 'int16' ) );
    U( 3 ) = double( typecast( a( 10:-1:9 ), 'int16' ) );
end

U = U * 2 * pi / 60;

return