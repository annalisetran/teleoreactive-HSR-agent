


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

?-op(300,fx,'watch__').

%%?-dynamic('$watch_clauses'/1).
?-dynamic('$watch_saved'/3).
?-dynamic('$query_var_types'/1).
?-assert('$num_solutions'(5)).
?-dynamic('$top_level_consulted_file'/1).

version('1.0').

%% Process an initial goal (as in QP)
'$qulog_process_initial_goal' :-
    '$initial_goal'(GS),
    open_string(read(GS), S),
    read(S, Goal),
    call(Goal),    
    !.
'$qulog_process_initial_goal'.

%% bomb out if initial goal causes an exception
'$qulog_catch_init_goal'(Ball) :-
    '$display_exception_info'(Ball),
    thread_exit.

'$display_exception_info'(exception(Data)) :-
    !,
    write_list(stderr, ["Qu-Prolog exception: ", exception(Data), nl_]).
'$display_exception_info'(Ball) :-
    writeL_(stderr, ["Qulog exception - exception term: ", Ball, nl_]).

%% Deal with exceptions at the top-level
'$qulog_interpreter' :-
    ip_lookup('$locally_handled_exceptions', Handlers),
    (   var(Handlers)
        ->
            Handlers = []
        ;
            true
    ),
    ip_set('$locally_handled_exceptions', [abort(_, _, _)|Handlers]),
    catch('$qulog_fetch_execute', Ball, '$qulog_interpreter_catch'(Ball)),
    ip_set('$locally_handled_exceptions', Handlers),
    !.
'$qulog_interpreter' :-
    '$qulog_interpreter'.

%% The main interpreter repeat-fail loop
'$qulog_fetch_execute' :-
    repeat,
    global_state_lookup('$prompt',Prompt),
    global_state_set(watch__,0),
    write_term_list([nl, wa(Prompt)]),
    flush_output,
    %ip_set('$in_interpreter_query', true),
    interpreter_read_query__(FullQuery, Vars),
    %% initialize var info for the type checker
    '$get_underscores'(FullQuery, Vars, Unders),
    ip_set('$global_vars', []),
    ip_set('$local_vars', []),
    ip_set('$underscore_vars', Unders),
    freeze_term(FullQuery),
    '$num_solutions'(Sols),
    (
     FullQuery = '$num_query'(N, Query)
    ->
     global_state_set('$total_solns2print', N),
      FVars = Vars
    ;
     FullQuery = '$vars_query'(QVars, Query)
    ->
     filter('$keep'(QVars), Vars, FVars),
     global_state_set('$total_solns2print', Sols)
    ; 
     FullQuery = '$num_vars_query'(N, QVars, Query)
    ->
     filter('$keep'(QVars), Vars, FVars),
     global_state_set('$total_solns2print', N)
    ; 
     Query = FullQuery,
     FVars = Vars,
     global_state_set('$total_solns2print', Sols)
    ),
    %ip_set('$in_interpreter_query', fail),
    (
     '$qulog_exit_interpreter'(Query)  % Control-D entered
    ;
      nl,
      %% solve query, display answers and fail (back to repeat for next query) 
     '$qulog_solve_query'(Query, FVars)
    ),
    !.

'$keep'(FreeVars, (V = _)) :- member_eq(V, FreeVars).
'$ignore'(FreeVars, U) :- \+'$keep'(FreeVars, U).

%% Handle exceptions produced by the query
'$qulog_interpreter_catch'(Ball) :-
    '$qulog_interpreter_catch_msg'(Ball),
    fail.
    
'$qulog_interpreter_catch_msg'(Ball) :-
    nonvar(Ball),
    Ball = '$exception'(abort(_, _, [Msg])),
    !,
    write_atom(stderr, Msg),
    write_atom(stderr, '\n').
'$qulog_interpreter_catch_msg'(Ball) :-
    nonvar(Ball),
    Ball = exception(undefined_predicate(_, Goal, _)),
    '@functor'(Goal, Pred, Arity),
    !,
    write_atom(stderr, 'no definition for '),
    write(stderr, Pred),
    write_atom(stderr, '/'),
    '$write_integer'(stderr, Arity),
    write_atom(stderr, '\n').
'$qulog_interpreter_catch_msg'(Ball) :-
    nonvar(Ball),
    Ball = exception(permission_error(unrecoverable, assert(Call, _), default)),
    !,
    (
      Call = (Head :- _Body)
    ->
      true
    ;
      Call = Head
    ),
    '@functor'(Head, F, N),
    writeL_(stderr, ["QuProlog exception: ", F/N, " is already defined", nl_]).
'$qulog_interpreter_catch_msg'(Ball) :-
    nonvar(Ball),
    Ball = exception(Data),
    !,
    errornl(Ball),
    '$valid_exception_term'(Data, Type, Sev, Goal, Msg0),
    '$get_exception_message'(Data, Type, Sev, Goal, Msg0, Msg),
    write_term_list(stderr, Msg),
    '$exception_severity'(Sev, DefaultAction, _, _),
    call_predicate(DefaultAction).
'$qulog_interpreter_catch_msg'(Ball) :-
    nonvar(Ball),
    Ball = ctrlC_reset,
    !.
'$qulog_interpreter_catch_msg'(Ball) :-
    writeL_(stderr, ["Qulog exception - exception term: ", Ball, nl_]).


'$qulog_exit_interpreter'(G) :- G == end_of_file.

/*
 * Solve the query and print the result.
 *
 * mode '$solve_query'(@term, @list(compound)).
 */
'$qulog_solve_query'(Query, Vars) :-
    global_state_set('$has_solution', no),
    global_state_lookup('$total_solns2print', M),
    global_state_set('$num_solns2print', M),
    %% type check - Kind is instantiated to either act(query) or rel(query)
    %% RealQuery is instantiated to the "compiled" version of Query
    once('$get_call_kind'(Query, RealQuery, Vars, Kind)),
    ip_lookup('$local_vars', Locals),
    filter('$ignore'(Locals), Vars, QVars),
    ip_set('$local_vars', []),
    ip_set('$global_vars', []),
    '$qulog_solve_query_aux'(Kind, RealQuery, QVars).

%%'$qulog_solve_query_aux'(A,B,C) :- errornl('$qulog_solve_query_aux'(A,B,C)),fail.
%% rel queries can have multiple solutions, act have exactly one solution
%% (or throw an exception)
'$qulog_solve_query_aux'(rel(_), Query, Vars) :-
    '$qulog_interpreter_call'(Query),
    global_state_set('$has_solution', yes),
    '$qulog_print_result'(Vars),
    !,
    fail.
'$qulog_solve_query_aux'(rel(_), _Query, _Vars) :-
    global_state_lookup('$has_solution', no),
    !,
    write('no\n'),
    fail.
'$qulog_solve_query_aux'(rel(_), _Query, _Vars) :-
    !,
    global_state_lookup('$total_solns2print', M),
    global_state_lookup('$num_solns2print', N),
    (
     N = M
    ->
     write('no more solutions\n')
    ;
     true
    ),
    fail.


'$qulog_solve_query_aux'(act(_), Query, Vars) :-
    '$qulog_interpreter_call'(Query),
    (
      Vars = []
    ->
      true
    ;
      global_state_set('$num_vars_printed', 0),
     '$qulog_print_bindings_section'(Vars, _)
    ),    
    write('success\n'),
    !,
    fail.
'$qulog_solve_query_aux'(act(_), _Query, _Vars) :-
    write('fail\n'),
    !,
    fail.



%%%%%%%%%%%%%%%%%%%%%%%%%%%% Put Query interface code here %%%%%%%%%%%%%%

% Debgging code



?-op(1150,fx,prolog).
%?-op(1150,xfx,<~).


%%'$qulog_interpreter_call'(X) :- errornl('$qulog_interpreter_call'(X)), fail.

'$qulog_interpreter_call'(pconsult(File)) :-
    !,
    consult(File).

'$qulog_interpreter_call'(set_num_answers(N)) :-
    !,
    N > 0,
    retract('$num_solutions'(_)),
    assert('$num_solutions'(N)).

'$qulog_interpreter_call'(unwatch(RelList)) :-
    !,
    forall(member(Rel, RelList),'$qulog_interpreter_call'('$unwatch'(Rel))).
% '$qulog_interpreter_call'(unwatchIO(RelList)) :-
%     !,
%     forall(member(Rel, RelList), '$qulog_interpreter_call'('$unwatchIO'(Rel))).
'$qulog_interpreter_call'(unwatch) :-
     !,
     forall('$watched'(Rel), '$qulog_interpreter_call'('$unwatch'(Rel))).
% '$qulog_interpreter_call'(unwatchIO) :-
%     !,
%     forall('$watched'(Rel), '$qulog_interpreter_call'('$unwatchIO'(Rel))).
'$qulog_interpreter_call'(watch(RelList)) :-
    !,
    forall(member(R, RelList), '$qulog_interpreter_call'('$watch'(R))).
% '$qulog_interpreter_call'(watchIO(RelList)) :-
%     !,
%     forall(member(R, RelList), '$qulog_interpreter_call'('$watchIO'(R))).
'$qulog_interpreter_call'(watched) :-
    !,
    forall('$watched'(C), write_term_list([C, '\n'])).


'$qulog_interpreter_call'('$watch'(Rel)) :-
    '$type_info'(Rel, rel, _, _, _),
      '$watched'(Rel),
      !,
      write_term_list([Rel,' already being watched\n']).
     
%% Watching functions
'$qulog_interpreter_call'('$watch'(Fun)) :-
      '$type_info'(Fun, fun, (Type -> _), _, _),
      !,
      (
        Type = '$tuple_type'(TL)
      ->
        length(TL, N)
      ;
        N = 1
      ),
      watch_trans_function_clauses_(Fun, N).
%% Watching relations
'$qulog_interpreter_call'('$watch'(Rel)) :-
      once((
       '$type_info'(Rel, rel, _, RelCall, _)
	;
	'$type_info'(Rel, act, _, RelCall, _)
      )),
      !,
      '@functor'( RelCall, Rel, N),
      '@=..'(RelCall, [Rel|_]),
      watch_trans_clauses_(Rel, N).

'$qulog_interpreter_call'('$watch'(T)) :-
    !,
    writeL_(stderr, ["Error: ", T, " is not defined",nl_]),
    fail.

'$qulog_interpreter_call'('$unwatch'(Fun)) :-
    '$generate_apply_name'(Fun, ApplyName),
    '$ho_function_watched_clause'(ApplyName), !,
    retract('$watched'(Fun)),
    Head = ApplyName(_, _, _),
    forall((clause(Head, Body), Body = ('$ho_watched_neck', _)),
           retract((Head :-  Body))),
    forall(retract('$watch_saved'(Fun, HeadPattern, Body)),
           assert((HeadPattern :- Body))).
'$qulog_interpreter_call'('$unwatch'(Fun)) :-
    '$user_type_info'(Fun, fun, (_ -> _), Kind,  _),
    Kind \= '$function',
    !,
    retract('$watched'(Fun)),
   '$generate_apply_name'(Fun, ApplyName),
   atom_concat2(ApplyName,'_watched',NFun),
   WBody = watch__ (NFun, _),
   Head = ApplyName(_, _, _),
   (
     clause(Head, WBody)
   ->
     retract((Head :- WBody))
   ;
     true
   ),
   retractall(NFun(_, _, _, _)),
   forall(retract('$watch_saved'(Fun, HeadPattern, Body)),
          assert((HeadPattern :- Body))).

'$qulog_interpreter_call'('$unwatch'(Fun)) :-
    '$type_info'(Fun, fun, (Type -> _), _, _),
    retract('$watched'(Fun)),
    !,
    %%retractall('$watch_clauses'(Fun)),
    (
      Type = '$tuple_type'(TL)
    ->
      length(TL, N)
    ;
      N = 1
    ),
    N1 is N+1,
    atom_concat2(Fun,'_watched',NFun),
    '@functor'( FunCall, Fun, N1),
    retract( (FunCall:- watch__ _)),
    '@functor'(NRelCall, NFun, N1),
    retractall(NRelCall),
    forall(retract('$watch_saved'(Fun, HeadPattern, Body)),
           assert((HeadPattern :- Body))).

'$qulog_interpreter_call'('$unwatch'(Rel)) :-
      retract('$watched'(Rel)),
      !,
      % (
      %   retractall('$watch_clauses'(Rel))
      % ;
      %   true
      % ),
      (
       '$type_info'(Rel, rel, _, RelCall, _)
      ;
       '$type_info'(Rel, act, _, RelCall, _)
      ),
      '@functor'( RelCall, Rel, N),
      atom_concat2(Rel,'_watched',NRel),
      N1 is N+1,
      '@functor'( NRelCall, NRel, N1),
      retract( (RelCall:- watch__ _)),
      retractall(NRelCall),
      forall(retract('$watch_saved'(Rel, HeadPattern, Body)),
               assert((HeadPattern :- Body))).

'$qulog_interpreter_call'('$unwatch'(Rel)) :-
      !,
      write_term_list([Rel,' was not being watched\n']).


% '$qulog_interpreter_call'('$watchIO'(Rel)) :-
%       '$watch_clauses'(Rel),
%       !,
%       write_term_list([Rel,' rules already being watched\n']).
% '$qulog_interpreter_call'('$watchIO'(Rel)) :-
%        '$watched'(Rel),
%        !,
%       assert('$watch_clauses'(Rel)).
% '$qulog_interpreter_call'('$watchIO'(Rel)) :-
%     (
%      '$type_info'(Rel, rel, _, _, _)
%     ;
%      '$type_info'(Rel, act, _, _, _)
%     ),
%     !,
%     assert('$watch_clauses'(Rel)),
%     '$qulog_interpreter_call'('$watch'(Rel)).
% '$qulog_interpreter_call'('$watchIO'(Fun)) :-
%       '$type_info'(Fun, fun, _, _, _),
%       !,
%       assert('$watch_clauses'(Fun)),
%       '$qulog_interpreter_call'('$watch'(Fun)).
% '$qulog_interpreter_call'('$watchIO'(T)) :-
%     !,
%     writeL_(stderr, ["Error: ", T, " is not defined",nl_]),
%     fail.


% '$qulog_interpreter_call'('$unwatchIO'(Rel)) :-
%     '$qulog_interpreter_call'('$unwatch'(Rel)),
%     !.

% '$qulog_interpreter_call'('$unwatchIO'(Rel)) :-
%       !,
%       write_term_list([Rel,' clauses were not being watched\n']).


'$qulog_interpreter_call'(prolog) :-
    !,
    global_state_lookup('$prompt', Prompt),
    global_state_set('$prompt', '| ?- '),
    interpreter,
    global_state_set('$prompt', Prompt).

'$qulog_interpreter_call'(prolog(Query)) :-
    !,
    thaw_term(Query),
    call_prolog_(Query).

    
'$qulog_interpreter_call'(types) :-
    !,
    types.

'$qulog_interpreter_call'(types(Rel)) :-
    !,
   types(Rel).
'$qulog_interpreter_call'(stypes(Rel)) :-
    !,
   stypes(Rel).
'$qulog_interpreter_call'(stypes) :-
    !,
    stypes.
'$qulog_interpreter_call'(show) :-
    !,
    show.
'$qulog_interpreter_call'(show(Rel)) :-
    !,
   show(Rel).
'$qulog_interpreter_call'(ground(T)) :-
    !,
    ground(T).
'$qulog_interpreter_call'(isa(Term,T)) :-
    !,
    thaw_term((Term, T)),
    isa(Term,T). 

'$qulog_interpreter_call'(logging(Addr)) :-
   logger_(Addr).
'$qulog_interpreter_call'(default_agent(MyName, Addr, Interface)) :-
   default_agent(MyName, Addr, Interface).
'$qulog_interpreter_call'(bs) :-
   '$show_bs'.
   
 

'$qulog_interpreter_call'(Query) :-
    thaw_term(Query),
    call(Query).



'$get_call_kind'(consult(File), interpreter_consult_(File), _, act(query)) :-
    !,
    (
      atom(File)
    ->
      true
    ;
      writeL_(stderr, ["Error: Illegal query ", consult(File), nl_,
                      File, " is not an atom",nl_]),
      fail
    ).
'$get_call_kind'(pconsult(File),pconsult(File), _, act(query)) :-
    !,
    (
      atom(File)
    ->
      true
    ;
      writeL_(stderr, ["Error: Illegal query ", pconsult(File), nl_,
                      File, " is not an atom",nl_]),
      fail
    ).

'$get_call_kind'(unwatch, unwatch,_, act(query)) :-
    !.
'$get_call_kind'(set_num_answers(N), set_num_answers(N),_, act(query)) :-
    !.

'$get_call_kind'(watch(Rel), watch(Rel), _,  act(query)) :-
    !.
'$get_call_kind'(unwatch(Rel),unwatch(Rel), _, act(query)) :-
      !.
% '$get_call_kind'(watchIO(Rel), watchIO(Rel), _, act(query)) :-
%       !.
% '$get_call_kind'(unwatchIO(Rel),unwatchIO(Rel), _, act(query)) :-
%     !.
'$get_call_kind'(watched, watched,_, act(query)) :-
    !.
'$get_call_kind'(prolog, prolog,_, act(query)) :-
    !.
'$get_call_kind'(prolog(Query),prolog(Query), _, rel(query)) :-
    !.
'$get_call_kind'(types,types,_, act(query)) :-
    !.
'$get_call_kind'(types(Rel),types(Rel),_, act(query)) :-
    !.
'$get_call_kind'(stypes,stypes,_, act(query)) :-
    !.
'$get_call_kind'(stypes(Rel),stypes(Rel),_, act(query)) :-
    !.
'$get_call_kind'(show, show,_, act(query)) :-
    !.
'$get_call_kind'(show(Rel), show(Rel), _, act(query)) :-
    !.
'$get_call_kind'(logging(Addr), logging(Addr), _, act(query)) :-
    !.
'$get_call_kind'(unlog, unlog, _, act(query)) :-
    !.
'$get_call_kind'(bs, bs, _, act(query)) :-
    !.
'$get_call_kind'(default_agent(N,A,B), default_agent(N, A, B), _, act(query)) :-
    !.
'$get_call_kind'(type(Term,T),type(Term,T),_Vars, rel(query)) :-
    !.
'$get_call_kind'(ground(Term),ground(Term),_Vars, rel(query)) :-
    !.
'$get_call_kind'(isa(Term,T),isa(Term,T),Vars, rel(query)) :-
    !,
    '$check_query'(isa(Term,T), Vars).
'$get_call_kind'(Query, TQuery, Vars, GKind) :-
    '$default_args'(Query, FullQuery), !,
    '$get_call_kind'(FullQuery, TQuery, Vars, GKind).
'$get_call_kind'(Query, TQuery, Vars, GKind) :-
    freeze_term(Query),
    '$check_query'(Query, Vars),
    %%thaw_term(Query),
    ip_set(colons_used, false),
    (
      check_query_mode_type__(Query, Vars, Kind)
    ->
      freeze_term(Query),
      '$translate_query'(Query, TQuery),
      thaw_term(TQuery),
      (
        Query = (_ :: _)
      ->
        ip_set(colons_used, true)
      ;
        true
      ),
      Kind = GKind
    ;
     '$try_name_vars'(Vars),
      writeL_(stderr, [nl_, "Error: Illegal query ", show_(Query), nl_]),
      !,
      fail
    ).
'$get_call_kind'(Query, _TQuery, Vars, _GKind) :-
    '$vars2names'(Query, Vars),
    writeL_(stderr, [nl_, "Error: Invalid query ", show_(Query), nl_]),
    fail.


call_prolog_(Query) :- var(Query), !, Query = 42.
call_prolog_('&'(Q1, Q2)) :-
    !,
    call_prolog_(Q1),
    call_prolog_(Q2).
call_prolog_(Q) :-
    call(Q).
    
%%'$qulog_print_result'(Vars) :-errornl('$qulog_print_result'(Vars)),fail.
    
'$qulog_print_result'([]) :-
    !,
    write('yes\n'),
    flush_output.
'$qulog_print_result'(Vars) :-
    global_state_decrement('$num_solns2print', N),
    global_state_lookup('$total_solns2print', M),
    (
     M is N+1
    ->
     true
    ;
     write_term_list(['...',nl])
    ),
    global_state_set('$num_vars_printed', 0),
    '$qulog_print_bindings_section'(Vars, _QVT),
    (
     global_state_lookup('$num_vars_printed', 0)
    ->
     write_term_list([yes,nl])
    ;
     true
    ),
    flush_output,
    N = 0,
    global_state_set('$num_solns2print', M),
    '$qulog_interpreter_cont_section'(Vars).

'$has_bindings'(Vars) :-
    '$vars2names'(Vars, Vars),
    member((A=B), Vars), A \= B.

strip_tuple__('$tuple'(L), L) :- !.
strip_tuple__(X, [X]).


'$qulog_print_bindings_section'([], _).
'$qulog_print_bindings_section'(ValVarList, _) :-
    map('$strip_out_vals', ValVarList, Values),
    collect_vars(Values, ValuesVars),
    map('$add_mode_type', ValuesVars, VMT),
    map('$bindings_get_type'(VMT), Values, ValTypes),
    transform_simple_terms('$strip_empty_type', ValTypes, SValTypes),
    !,
    collect_vars(SValTypes, TypeVars),
    reverse(TypeVars, RevTypeVars),
    '$name_type_vars'([_|RevTypeVars]),
    '$qulog_print_bindings_section_entries'(ValVarList, SValTypes).

%%'$qulog_print_bindings_section_entries'(A,B) :- errornl('$qulog_print_bindings_section_entries'(A,B)),fail.
'$qulog_print_bindings_section_entries'([], []).
'$qulog_print_bindings_section_entries'([=(Value,Var)|Vs], [T1|Ts]) :-
    (
      var(Value), get_var_name(Value, Var)
    ->
      true
    ;
      global_state_increment('$num_vars_printed', _),
      writeL_([uq_(Var), " = ", wr_(Value:T1), nl_])
    ),
    '$qulog_print_bindings_section_entries'(Vs, Ts).

'$strip_out_vals'(=(Value, _), Value).

'$split_on_type'((A:B), A, B).

'$strip_out_types'((_:B), B).

'$add_mode_type'(V, (V : '!'(_))).

%%'$bindings_get_type'(VMT, Term, Type) :- errornl('$bindings_get_type'(VMT, Term, Type)),fail.
'$bindings_get_type'(VMT, Term, Type) :-
    has_type2__(Term, Type, VMT).
    % (
    %   compound(Type1), ( Type1 = atom_naming(_) ; Type1 = term_naming(_) )
    % ->
    %   ip_set('$non_atom_type', true),
    %   findall(T, (has_type2__(Term, T1, VMT),
    %                  '$strip_naming'(T1,T)), AltTypes),
    %   ip_set('$non_atom_type', false),
    %   merge_types__(AltTypes, Type)
    % ;
    %   Type = Type1
    % ).
    
'$name_type_vars'([]).
'$name_type_vars'([V|Vs]) :-
    (
      set_var_name(V, 'Ty')
    ;
      true
    ), !,
    '$name_type_vars'(Vs).


    

'$get_values'([], []).
'$get_values'([=(Value, _)|Tail], [Value|VTail]) :-
    '$get_values'(Tail, VTail).

'$qulog_interpreter_cont_section'(_Vars) :-
    get_line(Line),
    (
     Line = -1
    ->
     Tokens = []
    ;
     string2tokens__(Line, Tokens, _, _)
    ),
    (
     Tokens = []
    ->
     true
    ;
     Tokens = [syntax('..')]
    ->
     global_state_lookup('$total_solns2print', M),
     global_state_set('$num_solns2print', M),
     fail
    ;
     Tokens = [syntax('..'), int(N)]
    ->
      global_state_set('$num_solns2print', N),
      global_state_set('$total_solns2print', N),
      fail
     ;
      true
    ).


interpreter_consult_(File) :-
    absolute_file_name(File, FilePath),
    '$top_level_consulted_file'(File1),
    FilePath \= File1,
    !,
    writeL_(stderr, ["Error: Only one file can be consulted at the top level of the interpreter. ", nl_, File1, " has already been consulted.", nl_]),
    fail.

interpreter_consult_(File) :-
    absolute_file_name(File, FilePath),
    '$consulted_timestamp'(G, _, OldTime),
    FilePath \= G,
    concat_atom([G, '.qlg'], Gqlg),
    stat(Gqlg, stat(Time, _)),
    Time > OldTime, !,
    %% at least one file that File consults directly or indirectly
    %% has changed so remove add info from all files.
    forall(retract('$consulted_timestamp'(F, _, _)), '$delete_all_code'(F)),
    top_level_qconsult_(FilePath).
interpreter_consult_(File) :-
    absolute_file_name(File, FilePath),
    concat_atom([FilePath, '.qlg'], Fileqlg),
    (
     stat(Fileqlg, stat(Time, _))
    ->
     true
    ;
     concat_atom([FilePath, '.qlg'], Fileqlg),
     writeL_(["Cannot read ", Fileqlg, nl_]),
     writeL_(["Error consulting ", Fileqlg, nl_]),
     fail
    ),
    (
      '$consulted_timestamp'(FilePath, FilePath, Time)
    ->
      writeL_(["Neither ", FilePath, " nor any files consulted from this file",
              nl_, "has changed since the last consult - file not (re)consulted", nl_])
    ;
      forall(retract('$consulted'(File1)),
             assert('$saved_consulted'(File1))),
      retractall('$consulted_timestamp'(FilePath, FilePath, _)),
      assert('$consulted_timestamp'(FilePath, FilePath, Time)),
      top_level_qconsult_(FilePath)
    ).
      

