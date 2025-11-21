%% Copyright 2014 Keith Clark, Peter Robinson
%%
%% Licensed under the Apache License, Version 2.0 (the "License");
%% you may not use this file except in compliance with the License.
%% You may obtain a copy of the License at
%%
%%     http://www.apache.org/licenses/LICENSE-2.0
%%
%% Unless required by applicable law or agreed to in writing, software
%% distributed under the License is distributed on an "AS IS" BASIS,
%% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%% See the License for the specific language governing permissions and
%% limitations under the License.

?-op(990,xfx,'<=').
?-op(200,xf,'<=').
?-op(200,xf,'?->').
?-op(200,xf,'~>').
?-op(200,xf,'~~>').
?-op(200,fx,'^').
?-op(200,fx,'?').
?-op(920,xfx,'=>').
?-op(920,xfx,'~>').
?-op(920,xfx,'~>>').
?-op(920,xfx,':>>').
?- op(600,xfx,'..').

?-op(1010,xfx,'::').

?-op(906,fx,'@').
?-op(906,fx,'forall').

?- op(910, xfy,  '&' ). 
?-op(700,xfx,'=?').   
?-op(700,xfx,'=@').   
?-op(700,xfx,'$=').   
?-op(500,xfy,'<>').
?-op(500,xfy,'++').   
?-op(1120,xfy,'<>?').
?-op(1120,xfy,'++?').   
?-op(600,xfx,'in').
?-op(600,xfx,'subs').
?- op(400, xfx, 'default').
?- op(200, fy, 'not').
?- op(907, fy, 'type').
?- op(190,fx, '#').
?- op(1101, xfx, '::=').
?- op(50, xfx, '?').

?- op(1028, xfx,  '^^' ).
?- op(700, xfx,  '@=..' ).
