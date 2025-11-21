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

%% Used to check and then translate the builtin declarations in
%% builtin_decls.qlg to builtin_decls.ql for inclusion in the qulog/teleor
%% library

main([File]) :-
    op_table(comp),
    dynamic('$builtin_type_info'/5),
    (
      qconsult_(File)
    ->
      atom_concat(File, '.ql', QlFile),
      open(QlFile, write, S),
      set_std_stream(1, S),
      dump2file,
      add_apply_clauses,
      reset_std_stream(1), close(S)
    ;
      writeL_(stderr, [nl_, File, " not translated", nl_, nl_]), 
      halt(1)
    ).

dump2file :-
    %dump_state_beliefs,
    findall(F/N, ('$asserted_file_info'(_, Info), functor(Info, F, N)),
            AllInfo),
    sort(AllInfo, SortedInfo),
    dump2file_aux(SortedInfo).

% dump_state_beliefs :-
%     '$state_belief'(Bel),
%     dump_a_state_belief(Bel),
%     fail.
% dump_state_beliefs.

% dump_a_state_belief(remote_query_timeout) :-
%     writeq('$builtin_state_belief'(remote_query_timeout)),
%     write('.\n'),
%     writeq('$builtin_belief'(remote_query_timeout)),
%     write('.\n').


dump2file_aux([]).
dump2file_aux([(F/N)|Rest]) :-
    errornl(dump__(F, N)),
    dump2file_aux1(F, N),
    dump2file_aux(Rest).

%%dump2file_aux1(A,B) :- errornl(dump2file_aux1(A,B)),fail.


dump2file_aux1('$user_type_descr', 4) :-
    '$user_type_descr'(A,B,C,D),
    not_ignored(A),
    transform_simple_terms(transform_c, '$builtin_type_descr'(A,B,C,D), Trans),
    %transform_subterms(transform_v, Trans, VarTrans),
    writeq(Trans), write('.\n'),
    fail.

dump2file_aux1('$user_type_info', 5) :-
    '$user_type_info'(A,B,C,D,E),
    not_ignored(A),
    transform_simple_terms(transform_c, '$user_type_info'(A,B,C,D,E), Trans),
    write_type_info(Trans),
    fail.
dump2file_aux1('$user_type_info', 5) :-
    writeq(('$builtin_type_info'(user_exception,defined_type,user_exception,'$constr_enum_type'([default_exception_('$none_')]),[]) :- \+'$user_type_info'(user_exception, _, _, _, _))),
    write('.\n'),
    fail.

dump2file_aux1('$user_default_args', 2) :-
    '$user_default_args'(A,B),
    transform_simple_terms(transform_c, '$builtin_default_args'(A,B), Trans),
    writeq(Trans), write('.\n'),
    fail.


dump2file_aux1(_, _).


write_type_info('$user_type_info'(A,B,C,D,E)) :-
    add_running_teleor(A), !,
    writeq(('$builtin_type_info'(A,B,C,D,E) :- '$running_teleo')),
    write('.\n').
write_type_info('$user_type_info'(A,B,C,D,E)) :-
    writeq('$builtin_type_info'(A,B,C,D,E)),
    write('.\n').

add_running_teleor(resources_hook).
add_running_teleor(handle_message).
add_running_teleor(handle_invalid_message).
add_running_teleor(handle_template_message).
add_running_teleor(init_message_handler).
add_running_teleor(init_percept_handler).
add_running_teleor(handle_percepts).
add_running_teleor(start_priority_task).
add_running_teleor(start_agent).
add_running_teleor(start_embedded_agent).
add_running_teleor(start_task).
add_running_teleor(kill_task).
add_running_teleor(kill_agent).
add_running_teleor(start_percept_handler).
add_running_teleor(actions).
add_running_teleor(refresh_bs).
add_running_teleor(task).
add_running_teleor(running).
add_running_teleor(waiting).
add_running_teleor(resources).
add_running_teleor(get_active_resources).
add_running_teleor(get_waiting_resources).
add_running_teleor(received_message).

not_ignored(A) :-
    \+ignored(A).


ignored(tel_action_term).
ignored(tel_percept_term).
ignored('$belief').
ignored('$percept').
ignored(message).
ignored(user_exception).
ignored(resource).

transform_c(X, '-') :- X == unary_minus, !.
transform_c(X, '@') :- X == 'compiler@', !.
transform_c(X, ':') :- X == 'compiler:', !.
transform_c(X, '=?') :- X == 'compiler=?', !.
transform_c(X, '@..') :- X == 'compiler@..', !.
transform_c(X, 'raise') :- X == 'compilerraise', !.
transform_c(X, 'to') :- X == 'compilerto', !.
transform_c(X, 'to_thread') :- X == 'compilerto_thread', !.
transform_c(X, 'peek_messages') :- X == 'compilerpeek_messages', !.
transform_c(X, X).



transform_v(V, V) :- var(V), !.
transform_v('$$var$$'(VName), V) :-
    ip_lookup(varnames, VName, V),
    get_var_name(V, VName), !.
transform_v('$$var$$'(VName), V) :- !,
    ip_set(varnames, VName, V),
    name2var(VName, V).
transform_v(T, T).

name2var(Name, Var) :-
    open_string(write, WStream),
    write_term_list(WStream, [Name, pc(0'.)]), %'
    stream_to_string(WStream, String),
    open_string(read(String), RStream),
    readR(RStream, Var),
    close(RStream).


add_apply_clauses :-
    findall(F, '$user_type_info'(F, fun, _, _, _), RFuns),
    sort(RFuns, Funs),
    member(F, Funs),
    F \= '-',
    F \= 'unary_minus',
    once('$user_type_info'(F, fun, '->'('$tuple_type'(ArgTypes), _), _, _)),
    length(ArgTypes, N),
    length(Args, N),
    '$generate_apply_name'(F, ApplyName),
    append(Args, [Result], RelArgs),
    generate_body(F, RelArgs, Body),
    %Body =.. [F|RelArgs],
    Clause = (ApplyName(F, Args, Result) :-  Body),
    transform_simple_terms(transform_c, Clause, TClause),
    portray_clause(TClause),
    fail.
add_apply_clauses :-
    portray_clause(('-$apply'('-', (A,B), C) :- !, -(A,B,C))),
    portray_clause(('-$apply'('-', A, C) :- -(A,C))).

generate_body('<>', [A,B,C], append(A,B,C)) :- !.
generate_body('++', [A,B,C], string_concat(A,B,C)) :- !.
generate_body(F, Args, Body ) :- Body =.. [F|Args].