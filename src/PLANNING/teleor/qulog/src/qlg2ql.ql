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


?- dynamic('$runtime'/0).

main([File|Args]) :-
    errornl(Args),
    op(0, xfx, in),
   (
      member('-Q', Args)
    ->
      true
    ;
      assert('$running_teleo')
    ),
    (
      catch(qconsult_(File), Err, process_error(Err, File))
    ->
      (
        current_predicate('qmain'/1)
      ->
        !
      ;
        writeL_(stderr, [nl_, "Error: No definition of qmain/1", nl_]),
        halt(1)
      ),
      op(0, fy, not),
      op(0, xfx, '..'),
      atom_concat(File, '.ql', QlFile),
      open(QlFile, write, S),
      set_std_stream(1, S),
      dump2file,
      %add_apply_clauses, 
      reset_std_stream(1), close(S)
    ;
      writeL_(stderr, [Err, nl_, File, " not translated", nl_, nl_]), 
      halt(1)
    ).

    
process_error(Err, File) :-
     writeL_(stderr, [Err, nl_, File, " not translated", nl_, nl_]), 
     halt(1).

dump2file :-
    findall(F/N, ('$asserted_file_info'(_, Info), functor(Info, F, N)),
            AllInfo),
    sort(AllInfo, SortedInfo),
    (
      '$running_teleo'
    ->
      write_canonical('?-'(assert('$running_teleo'))), write('.\n')
    ;
      true
    ),
    dump_sys_op_decls,
    dump2file_aux(SortedInfo),
    pconsult_listing.


dump_sys_op_decls :-
    write_canonical('?-'( op(190,fx, '#'))),write('.\n').
   
    
dump2file_aux([]).
dump2file_aux([(F/N)|Rest]) :-
    dump2file_aux1(F, N),
    dump2file_aux(Rest).


dump2file_aux1('$goal', 1) :- 
    '$goal'(G),
    errornl(xxxxxx(G)),
    G \= pconsult(_),
     write_canonical('?-'(G)),write('.\n'),
     fail.
dump2file_aux1('$goal', 1) :- !.
dump2file_aux1('$runtime_goal', 1) :- !.
dump2file_aux1('$user_type_descr', 4) :- !.
dump2file_aux1('$user_type_info', 5) :-
    '$user_type_info'(A,B,C,D,E),
    write_canonical('?-'(assert('$user_type_info'(A,B,C,D,E)))),write('.\n'),
    fail.
dump2file_aux1('$user_type_info', 5) :- !.
dump2file_aux1('$tel_atomic', 2) :-
    '$tel_atomic'(A,B),
    write_canonical('?-'(assert('$tel_atomic'(A,B)))),write('.\n'),
    fail.
dump2file_aux1('$tel_atomic', 2) :- !.
dump2file_aux1('$tel_start', 1) :-
    '$tel_start'(A),
    write_canonical('?-'(assert('$tel_start'(A)))),write('.\n'),
    fail.
dump2file_aux1('$tel_start', 1) :- !.
dump2file_aux1('$percept', 1) :-
    '$percept'(A),
    functor(A, F, N),
    write_canonical('?-'(dynamic(F/N))),write('.\n'),
    write_canonical('?-'(assert('$percept'(A)))),write('.\n'),
    fail.
dump2file_aux1('$percept', 1) :- !.
dump2file_aux1('$robotic_action', 1) :-
    '$robotic_action'(A),
    write_canonical('?-'(assert('$robotic_action'(A)))),write('.\n'),
    fail.
dump2file_aux1('$robotic_action', 1) :- !.

dump2file_aux1('$belief', 1) :-
    '$belief'(A),
    functor(A, F, N),
    write_canonical('?-'(dynamic(F/N))),write('.\n'),
    write_canonical('?-'(assert('$belief'(A)))),write('.\n'),
    call(A),
    write('?-'(assert(A))),write('.\n'),
    fail.
dump2file_aux1('$belief', 1) :- !.
dump2file_aux1('$resource', 1) :-
    '$resource'(A),
    write_canonical('?-'(assert('$resource'(A)))),write('.\n'),
    fail.
dump2file_aux1('$resource', 1) :- !.
dump2file_aux1('$resource_info', 3) :- !.
dump2file_aux1('$resource_info', 3) :-
    '$resource_info'(A,B,C),
    write_canonical('?-'(assert('$resource_info'(A,B,C)))),write('.\n'),
    fail.
dump2file_aux1('$resource_info', 3) :- !.
dump2file_aux1('$primitive_resource_info', 3) :- !.
dump2file_aux1('$state_belief', 1) :-
    '$asserted_file_info'(_, '$state_belief'(B)),
    write_canonical('?-'(assert('$state_belief'(B)))),write('.\n'),
    write_canonical('?-'(dynamic(B/1))),write('.\n'),
    call(B(X)),
    write('?-'(assert((B(X))))),write('.\n'),
    fail.
dump2file_aux1('$state_belief', 1) :- !.
dump2file_aux1(overlapping_resources_, 2) :-
    !,
    clause(overlapping_resources_(R1,R2), Body),
    write_canonical('?-'(assert((overlapping_resources_(R1,R2) :-  Body)))),
    write('.\n').
dump2file_aux1('$belief', 1) :-
    !, dumpbeliefs.
dump2file_aux1(qmain, 1) :-
    clause(qmain(Args), Body),
    portray_clause((main(Args1) :- map('$atom2string', Args1, Args),Body)),
    fail.
dump2file_aux1(qmain, 1) :- !.
dump2file_aux1(F, _N) :-
    '$state_belief'(F), !.
dump2file_aux1(F, N) :-
    functor(T, F, N),
    '$belief'(T), !.
dump2file_aux1(F, N) :-
    listing(F/N).

dumpbeliefs :-
    '$belief'(B),
    B \= message_(_,_,_),
    write_canonical('?-'(assert(('$belief'(B))))),write('.\n'),
    fail.
dumpbeliefs.

pconsult_listing :-
    '$pconsulted_preds'(F, N),
    listing(F/N),
    fail.
pconsult_listing.

% add_apply_clauses :-
%     errornl(apply_clauses),
%     '$user_type_info'(F, fun, '->'('$tuple_type'(ArgTypes), _), _, _),
%     length(ArgTypes, N),
%     length(Args, N),
%     '$generate_apply_name'(F, ApplyName),
%     '$list2tuple'(Args, ArgsTuple),
%     append(Args, [Result], RelArgs),
%     Body =.. [F|RelArgs],
%     portray_clause((ApplyName(F, ArgsTuple, Result) :-  Body)),
%     fail.
% add_apply_clauses.