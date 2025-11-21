


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

main(_Args) :-
    assert('$running_teleo'),
    version(V),
    catch('$qulog_process_initial_goal', Ball, '$qulog_catch_init_goal'(Ball)),
    write_term_list([wa('Teleor Version '), w(V), nl]),
    global_state_set('$indent_level', 0),
    global_state_set('$break_level', 0),
    global_state_set('$prompt', '| ?~> '),
    '$qulog_interpreter'.
