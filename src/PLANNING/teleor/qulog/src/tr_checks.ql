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


?- compile_time_only(consult(op_decls)).
?- dynamic('$ho_resource_info'/3).

%% Clause is a TR procedure
'$check_tel'(Clause, VarNames) :-
    Clause = '$tel'(Head, Rules), !,
    '@..rev'(Head, TR, Args),
    '$type_info'(TR, _, Types, TRTerm, _),
    (
      Head = TRTerm
    ->
     true
    ;
      %% the functor of the head is declared but the term
      %% does not match the declaration
     '$display_error'(already_declared(Head, Clause, VarNames))
    ),
    (
      length(Args, N), length(Types, N)
    ->
      true
    ;
      '$display_error'(tr_head_arity_error(Clause, VarNames))
    ),
    %'$add_annotations'(Types, ATypes0, '!'),
    %push_modes_in__(ATypes0, ATypes),
    %% Add ! modes to types in preparation for type checking
    '$check_tr_head_args'(Args, Types, VTB, Head, VarNames),
    %% VTB is the varible type binding for the head
    '$get_term_singleton_vars'(Clause, VarNames, Singletons),
    %% Singletons is used in check_tr_rule below to warn about
    %% singletons within a single TR rule within the TR procedure
    forall(member('tr~>'(RuleLHS, RuleRHS), Rules),
	   once((
            '$check_tr_rule'(RuleLHS, RuleRHS,
                             VTB, Head, 'tr~>'(RuleLHS, RuleRHS), 
                             VarNames, Singletons)
           ;
             true
           ))
          ).

%% For TR procedures the args of the head have to be distinct variables
%% We check for distinct variables by freezing a variable when it's first
%% seen - if we see that variable again it will be frozen.
%% We generate the variable-type-binding for the head
%%'$check_tr_head_args'(A,B,C,D,F) :- errornl('$check_tr_head_args'(A,B,C,D,F)),fail.
'$check_tr_head_args'([], [], [], _Head, _VarNames).
'$check_tr_head_args'([A|Args], [T|Types], [(A:T)|VTB], Head, VarNames) :-
    (
     ( nonvar(A) ; frozen_var(A) )
    ->
     thaw_term(A),
     '$display_error'(tr_head_nonvar(A, Head, VarNames)),fail
    ;
     freeze_term(A)
    ),
    '$check_tr_head_args'(Args, Types, VTB, Head, VarNames).

check_tr_error__(_Err, _TR, _Call, Error) :-
    nonvar(Error), !.
check_tr_error__(Err, _TR, _Call, Error) :-
    nonvar(Err), Err = body_call_error(_, _), !,
    Error = body(Err).
check_tr_error__(Err, _TR, Call, Error) :-
    nonvar(Err), !,
    Error = body(body_call_error(Call, Err)).
check_tr_error__(_, _, _, _).

%%'$check_tr_rule'(LHS, RHS, VMT, Head, Rule, VarNames,Singletons) :- errornl('$check_tr_rule'(LHS, RHS, VMT, Head, Rule, VarNames,Singletons)), fail.

'$check_tr_rule'(LHS, RHS, VTB, Head, Rule, VarNames, Singletons) :-
    Clause = '$tr'(Head, Rule),
    %% Warn of singleton variables within a given rule of a TR procedure
    '$check_for_tr_rule_singleton_vars'(Head, Rule, VarNames, Singletons),
    freeze_term((Head, Rule), _FVars),
    '$check_tr_rule_LHS'(LHS, Clause, VarNames),
    '$check_tr_rule_RHS'(RHS, Clause, VarNames),
    thaw_term((LHS, RHS, Clause)),
    '$get_underscores'(Clause, VarNames, Unders),
    RuleInfo = rule_info(tel, tel(Head, Rule), VarNames, Unders),
    '$check_tr_rule_LHS_types'(LHS, VTB, OutVTB, OutLE, Unif, RuleInfo),
    '$check_tr_rule_RHS_types'(RHS, OutVTB, OutLE, Unif, RuleInfo).
    
'$check_for_tr_rule_singleton_vars'(Head, Rule, VarNames, Singletons) :-
    %% We repeat Head below because a variable in the head not used in Rule
    %% doesn not count as a singleton
    '$get_term_singleton_vars'(Head+Head+Rule, VarNames, Singles),
    %% We ignore overall singletons as we will get such a warning earlier
    %% in the processing
    diff_list(Singles, Singletons, LocalSingles),
    LocalSingles \= [],
    '$warn_of_single_vars_aux'(LocalSingles, SVars),
    thaw_term((Head, Rule)),
    '$vars2names'((Head, Rule), VarNames),
    writeL_(["Singleton variables warning: ",uq_(SVars), " in", nl_,
            Rule, nl_, "of the TR procedure ", Head, nl_,nl_]),
    fail.
'$check_for_tr_rule_singleton_vars'(_, _,_,_).

'$get_term_singleton_vars'(Term, VarNames, Singles) :-
    open_string(write, WS),
    (
      thaw_term(Term),
      '$vars2names'(Term, VarNames),
      writeL_(WS, [Term, "."]),fail
    ;
      true
    ),
    stream_to_string(WS, TermString),
    string2tokens__(TermString, _, VarList, _),
    '$collect_var_tables'(VarList, _Variables, _VarNames, Singles, []).


'$check_tr_rule_LHS'('$while'(_, G, '$time'(_, T)), Clause,  VarNames) :-
    !,
    '$check_goal_expressions'(G, Clause, VarNames, rel),
    '$check_expressions'(T, Clause, VarNames).
'$check_tr_rule_LHS'('$while'(_, G, '$time'(_, W, T)), Clause, VarNames) :-
    !,
    '$check_goal_expressions'(G, Clause, VarNames, rel),
    '$check_goal_expressions'(W, Clause, VarNames, rel),
    '$check_expressions'(T, Clause, VarNames).
'$check_tr_rule_LHS'('$while'(_, G, W), Clause, VarNames) :-
    !,
    '$check_goal_expressions'(G, Clause, VarNames, rel),
    '$check_goal_expressions'(W, Clause, VarNames, rel).
'$check_tr_rule_LHS'('$inhibit'(_, G, '$time'(_, T)), Clause, VarNames) :-
    !,
    '$check_goal_expressions'(G, Clause, VarNames, rel),
    '$check_expressions'(T, Clause, VarNames).
'$check_tr_rule_LHS'('$inhibit'(_, G, '$time'(_, U, T)), Clause, VarNames) :-
    !,
    '$check_goal_expressions'(G, Clause, VarNames, rel),
    '$check_goal_expressions'(U, Clause, VarNames, rel),
    '$check_expressions'(T, Clause, VarNames).
'$check_tr_rule_LHS'('$inhibit'(_, G, U), Clause, VarNames) :-
    !,
    '$check_goal_expressions'(G, Clause, VarNames, rel),
    '$check_goal_expressions'(U, Clause, VarNames, rel).
'$check_tr_rule_LHS'(G, Clause, VarNames) :-
    '$check_goal_expressions'(G, Clause, VarNames, rel).


'$check_tr_rule_LHS_types'('$while'(_, G, '$time'(_, T)),
                           InVTB, OutVTB, OutLE, Unif, RuleInfo) :-
    !,
    '$check_types_tr_rule'(G, InVTB, OutVTB, [], MidLE, Unif, RuleInfo),
    '$check_time_TR_expression'(T, OutVTB, MidLE, OutLE, Unif, RuleInfo).
'$check_tr_rule_LHS_types'('$while'(_, G, '$time'(_, W, T)),
                           InVTB, OutVTB, OutLE, Unif, RuleInfo) :-
    !,
    '$check_types_tr_rule'(G, InVTB, OutVTB, [], Mid1LE, Unif, RuleInfo),
    '$check_if_ground_TR_call'(W, OutVTB, Unif, RuleInfo),
    '$check_types_tr_rule'(W, OutVTB, _, Mid1LE, Mid2LE, Unif, RuleInfo),
    '$check_time_TR_expression'(T, OutVTB,  Mid2LE, OutLE, Unif, RuleInfo).
'$check_tr_rule_LHS_types'('$while'(_, G, W),
                           InVTB, OutVTB, OutLE, Unif, RuleInfo) :-
    !,
    '$check_types_tr_rule'(G, InVTB, OutVTB, [], MidLE, Unif, RuleInfo),
    '$check_if_ground_TR_call'(W, OutVTB, Unif, RuleInfo),
    '$check_types_tr_rule'(W, OutVTB, _, MidLE, OutLE, Unif, RuleInfo).
'$check_tr_rule_LHS_types'('$inhibit'(_, G, '$time'(_, T)),
                           InVTB, OutVTB, OutLE, Unif, RuleInfo) :-
    !,
    '$check_types_tr_rule'(G, InVTB, OutVTB, [], MidLE, Unif, RuleInfo),
    '$check_time_TR_expression'(T, OutVTB, MidLE, OutLE, Unif, RuleInfo).
'$check_tr_rule_LHS_types'('$inhibit'(_, G, '$time'(_, U, T)),
                           InVTB, OutVTB, OutLE, Unif, RuleInfo) :-
    !,
    '$check_types_tr_rule'(G, InVTB, OutVTB, [], MidLE, Unif, RuleInfo),
    '$check_if_ground_TR_call'(U, OutVTB, Unif, RuleInfo),
    '$check_types_tr_rule'(U, OutVTB, _, MidLE, Mid1LE, Unif, RuleInfo),
    '$check_time_TR_expression'(T, OutVTB, Mid1LE, OutLE, Unif, RuleInfo).
'$check_tr_rule_LHS_types'('$inhibit'(_, G, U),
                           InVTB, OutVTB, OutLE, Unif, RuleInfo) :-
    !,
    '$check_types_tr_rule'(G, InVTB, OutVTB, [], MidLE, Unif, RuleInfo),
    '$check_if_ground_TR_call'(U, OutVTB, Unif, RuleInfo),
    '$check_types_tr_rule'(U, OutVTB, _, MidLE, OutLE, Unif, RuleInfo).
'$check_tr_rule_LHS_types'(G, InVTB, OutVTB, OutLE, Unif, RuleInfo) :-
    '$check_types_tr_rule'(G, InVTB, OutVTB, [], OutLE, Unif, RuleInfo).




'$check_if_ground_TR_call'(G, VTB, Unif, RuleInfo) :-
    '$goals2list'(G, GList),
    RuleInfo = rule_info(_, Clause, _VarNames, Unders),
    check_globals_locals__(GList, [], Globals, [], _L,
                           Unders, part, _E),
    diff_list(Globals, Unders, RealGlobals),
    non_ground__(RealGlobals, VTB, Unif, NonGround),
    NonGround \= [], !,
    Err = non_ground_call(NonGround, G),
    Clause = tel(TR, Rule),
    check_tr_error__(Err, TR, Rule, Error),
    process_errors__([err__(Error, tel, tel, G)], RuleInfo).
'$check_if_ground_TR_call'(_G, _VTB, _Unif, _RuleInfo).

'$check_time_TR_expression'(T, VTB, InLE, OutLE, Unif, RuleInfo) :-
    RuleInfo = rule_info(_, Clause, _VarNames, Unders),
    has_type__(T, '!'(num), VTB, VTB, _, InLE, OutLE, Unif,
               Unders, 1, min_time(T), Err),
    Clause = tel(TR, Rule),
    check_tr_error__(Err, TR, Rule, Error),
    process_errors__([err__(Error, tel, tel, T)], RuleInfo).



%%'$check_types_tr_rule'(G, InVTB, OutVTB, InLE, OutLE, Unif, RuleInfo) :- errornl('$check_types_tr_rule'(G, InVTB, OutVTB, InLE, OutLE, Unif, RuleInfo)),fail.
'$check_types_tr_rule'(G, InVTB, OutVTB, InLE, OutLE, Unif, RuleInfo) :-
    RuleInfo = rule_info(_, Clause, _, Unders),
    '$goals2list'(G, GL),
    type_check_body__(GL, rel, InVTB, OutVTB, Unders, InLE, OutLE,
                      [], Unif, Err, []),
    Clause = tel(TR, Rule),
    check_tr_error__(Err, TR, Rule, Error),
    process_errors__([err__(Error, tel, tel, G)], RuleInfo).

'$check_types_tr_rule_TR_action'(TR, VTB, InLE, Unif, RuleInfo) :-
    RuleInfo = rule_info(_, Clause, _, Unders),
    '$annotated_type_info'(Types, TR),
    '@..rev'(TR, _, Args),
    (
      length(Types, N), length(Args, N)
    ->
      type_check_call_aux__([Types], TR, Args, VTB, _NextVTB, Unders,
                            InLE, _, Unif, Err)
    ;
      Err = arity_error(TR, Types)
    ),
    Clause = tel(TR1, Rule),
    check_tr_error__(Err, TR1, Rule, Error),
    process_errors__([err__(Error, tel, tel, TR)], RuleInfo).
    

'$check_tr_rule_RHS'('++'(A, G), Clause, VarNames) :-
    !,
    once((
          '$check_goal_expressions'(G, Clause, VarNames, act)
         ;
          '$display_error'(non_action_in_tr_goal(G, Clause, VarNames))
         )),
    %%'$goals2list'(G, GList),
    %%'$check_pp_actions'(GList, [], Clause, VarNames),
    '$check_tr_rule_RHS'(A, Clause, VarNames).

 '$check_tr_rule_RHS'(timed_seq(Seq), Clause, VarNames) :-
    !,
    '$check_tr_rule_RHS_ts'(Seq, Clause, VarNames).
'$check_tr_rule_RHS'([A], _Clause, _VarNames) :-
    '$type_info'(_, act, _, A, _), !.
'$check_tr_rule_RHS'(Actions, Clause, VarNames) :-
    '$check_tr_rule_RHS_tr_or_actions'(Actions, Clause, VarNames).

%%'$check_pp_actions'(Gs, Seen, _, _) :- errornl('$check_pp_actions'(Gs, Seen)),fail.
'$check_pp_actions'([], _, _Clause, _VarNames).
'$check_pp_actions'([G|GList], Seen, Clause, VarNames) :-
    G = (G1 , G2), !,
    '$check_pp_actions'([G1, G2|GList], Seen, Clause, VarNames).
'$check_pp_actions'([G|GList], Seen, Clause, VarNames) :-
    functor(G, GF, _), member(GF, Seen), !,    
    '$check_pp_actions'(GList, Seen, Clause, VarNames).
'$check_pp_actions'([G|GList], Seen, Clause, VarNames) :-
    G = '$forall_actions'(_, _, Acts),
    '$check_pp_actions'([Acts|GList], Seen, Clause, VarNames).
'$check_pp_actions'([G|GList], Seen, Clause, VarNames) :-
    '$user_type_info'(_, act, _, G, _),
    !,
    findall(Body, clause(G, Body),  Clauses),
    '$list2actions'(Clauses, CList),
    '$goals2list'(CList, BodyList),
    append(BodyList, GList, AllList),
    '$check_pp_actions'(AllList, Seen, Clause, VarNames).
'$check_pp_actions'([G|GList], Seen, Clause, VarNames) :-
    '$is_not_allowed_pp_action'(G), !,
    '$display_warning'(non_allowed_action_in_tr_goal(G, Clause, VarNames)),
    '$check_pp_actions'(GList, Seen, Clause, VarNames).
'$check_pp_actions'([_G|GList], Seen, Clause, VarNames) :-
    '$check_pp_actions'(GList, Seen, Clause, VarNames).

'$is_not_allowed_pp_action'(G) :-
    '$builtin_type_info'(_, act, _, G, _),!,
    \+'$is_allowed_pp_action'(G).
'$is_not_allowed_pp_action'(from(_, _, _)).
'$is_not_allowed_pp_action'(from_thread(_, _, _)).
'$is_not_allowed_pp_action'(receive(_)).
'$is_not_allowed_pp_action'(case(_)).
'$is_not_allowed_pp_action'(wait(_)).
'$is_not_allowed_pp_action'(wait_case(_)).
'$is_not_allowed_pp_action'(try_except(_, _)).
'$is_not_allowed_pp_action'(repeat_until(_, _)).
'$is_not_allowed_pp_action'(atomic_action(_)).
'$is_not_allowed_pp_action'(wait(_, _, _)).
'$is_not_allowed_pp_action'('$remote_query'(_, _)).
'$is_not_allowed_pp_action'('$remote_query'(_, _, _)).

'$is_allowed_pp_action'(send_robotic_message(_)).
'$is_allowed_pp_action'(remember_for(_, _)).
'$is_allowed_pp_action'(to(_, _)).
'$is_allowed_pp_action'(to_thread(_, _)).
'$is_allowed_pp_action'('+:='(_, _)).
'$is_allowed_pp_action'('+:='(_, _)).
'$is_allowed_pp_action'('-:='(_, _)).
'$is_allowed_pp_action'(':='(_, _)).
'$is_allowed_pp_action'(log_list(_)).

%%'$check_tr_rule_RHS_tr_or_actions'(A,B,C) :- errornl('$check_tr_rule_RHS_tr_or_actions'(A,B,C)),fail.
'$check_tr_rule_RHS_tr_or_actions'([TR], _Clause, _VarNames) :-
    compound(TR),
    '@functor'(TR, TRF, _),
    ( var(TRF) ; compound(TRF) ), !.

'$check_tr_rule_RHS_tr_or_actions'([TR], Clause, VarNames) :-
    functor(TR, TRF, _),
    '$type_info'(TRF, tel, _, TR1, _), !,
    (
     TR = TR1
    ->
     true
    ;
     '$display_error'(wrong_num_args(TR, Clause, VarNames))
    ).
'$check_tr_rule_RHS_tr_or_actions'(Actions, Clause, VarNames) :-
    '$check_tr_rule_RHS_actions'(Actions, Clause, VarNames).

'$check_tr_rule_RHS_actions'([], _, _).
'$check_tr_rule_RHS_actions'(['$nil'], _, _) :- !.
'$check_tr_rule_RHS_actions'([A|Actions], Clause, VarNames) :-
    (
     '$tel_action'(A)
    ->
      true
    ;
      '$display_error'(non_pa_in_tr_actions(A, Clause,
                                                 VarNames))
    ),
    '$check_tr_rule_RHS_actions'(Actions, Clause, VarNames).
    

%%'$check_tr_rule_RHS_ts'(A, _, _) :- errornl('$check_tr_rule_RHS_ts'(A)),fail.
'$check_tr_rule_RHS_ts'(['$time'(_, A, T)], Clause, VarNames) :-
    !,
    '$check_expressions'(T, Clause, VarNames),
    '$check_tr_rule_RHS_tr_or_actions'(A, Clause, VarNames).
'$check_tr_rule_RHS_ts'([A], Clause, VarNames) :-
    !,
    '$check_tr_rule_RHS_tr_or_actions'(A, Clause, VarNames).
'$check_tr_rule_RHS_ts'(['$time'(_, A, T)|Seq], Clause, VarNames) :-
    !,
    '$check_expressions'(T, Clause, VarNames),
    '$check_tr_rule_RHS_tr_or_actions'(A, Clause, VarNames),
    '$check_tr_rule_RHS_ts'(Seq, Clause, VarNames).

%%'$check_tr_rule_RHS_types'(A, VTB, InLE, Unif, RuleInfo) :- errornl('$check_tr_rule_RHS_types'(A, VTB, InLE, Unif, RuleInfo)),fail.
'$check_tr_rule_RHS_types'('++'(A, G),  VTB, InLE, Unif, RuleInfo) :-
    !,
    '$check_if_ground_TR_call'(G, VTB, Unif, RuleInfo),
    '$check_types_tr_rule'(G, VTB, _, InLE, MidLE, Unif, RuleInfo),
    '$check_tr_rule_RHS_types'(A, VTB, MidLE, Unif, RuleInfo).

'$check_tr_rule_RHS_types'(timed_seq(Seq),  VTB, InLE, Unif, RuleInfo) :-
    !,
    '$check_tr_rule_RHS_ts_types'(Seq, VTB, InLE, Unif, RuleInfo).
'$check_tr_rule_RHS_types'(Actions,  VTB, InLE, Unif, RuleInfo) :-
    '$check_tr_rule_RHS_tr_or_actions_types'(Actions, VTB, InLE, Unif,
                                             RuleInfo).

'$check_tr_rule_RHS_tr_or_actions_types'([TR], VTB, InLE, Unif, RuleInfo) :-
    compound(TR),
    '@..rev'(TR, F, Args),
    var(F), !,
    (
      type_member_of__(F:M(tel('$tuple_type'(Types))), VTB), M = '!'
    ->
      ( length(Types, N), length(Args, N)
      ->
          strip_modes__(Types, StrippedTypes),
          has_type_term__('$tuple_type'(StrippedTypes), '!', '$tuple'(Args),
                          VTB, [], _OutVTB, InLE, _OutLE, Unif, [],
                          0, TR, Err)
      ;
          Err = arity_error(TR, Types)
      )
    ;
      Err = mode_error(0, TR, TR, '!', non_ground_var_functor(F))
    ),
    check_tr_error__(Err, TR, TR, Error),
    process_errors__([err__(Error, tel, tel, TR)], RuleInfo),
    '$save_ho_resource_info'(TR, Args, Types, RuleInfo).
'$check_tr_rule_RHS_tr_or_actions_types'([TR], VTB, InLE, Unif, RuleInfo) :-
    '$type_info'(_, tel, _, TR, _), !,
    '$check_if_ground_TR_call'(TR, VTB, Unif, RuleInfo),
    '$check_types_tr_rule_TR_action'(TR, VTB, InLE, Unif, RuleInfo).
'$check_tr_rule_RHS_tr_or_actions_types'(Actions, VTB, InLE, Unif, RuleInfo) :-
    '$check_tr_rule_RHS_actions_types'(Actions, VTB, InLE, Unif, RuleInfo).

'$check_tr_rule_RHS_actions_types'([], _, _, _, _).
'$check_tr_rule_RHS_actions_types'(['$nil'], _, _, _, _) :- !.
'$check_tr_rule_RHS_actions_types'([A|Actions], VTB, InLE, Unif, RuleInfo) :-
    '$check_if_ground_TR_call'(A, VTB, Unif, RuleInfo),
    RuleInfo = rule_info(_, Clause, _VarNames, Unders),
    has_type__(A, '!'(tel_action_term), VTB, VTB, _, InLE, OutLE, Unif,
               Unders, 0, A, Err),
    Clause = tel(TR, Rule),
    check_tr_error__(Err, TR, Rule, Error),
    process_errors__([err__(Error, tel, tel, A)], RuleInfo),
    '$check_tr_rule_RHS_actions_types'(Actions, VTB, OutLE, Unif, RuleInfo).
    
'$check_tr_rule_RHS_ts_types'(['$time'(_, A, T)], VTB, InLE, Unif, RuleInfo) :-
    !,
    '$check_time_TR_expression'(T, VTB, InLE, OutLE, Unif, RuleInfo),
    '$check_tr_rule_RHS_tr_or_actions_types'(A, VTB, OutLE, Unif, RuleInfo).
'$check_tr_rule_RHS_ts_types'([A],  VTB, InLE, Unif, RuleInfo) :-
    !,
    '$check_tr_rule_RHS_tr_or_actions_types'(A,  VTB, InLE, Unif, RuleInfo).
'$check_tr_rule_RHS_ts_types'(['$time'(_, A, T)|Seq], VTB, InLE,
                              Unif, RuleInfo) :-
    !,
    '$check_time_TR_expression'(T, VTB, InLE, OutLE, Unif, RuleInfo),
    '$check_tr_rule_RHS_tr_or_actions_types'(A, VTB, OutLE, Unif, RuleInfo),
    '$check_tr_rule_RHS_ts_types'(Seq, VTB, OutLE, Unif, RuleInfo).

'$run_tr_checks' :-
    once('$current_consulted_file'(File)),
    '$saved_input_with_vars'(Clause, Vars, File),
    Clause = '$tel'(Head, Rules),
    member(Rule, Rules),
    Rule = 'tr~>'(_, '++'(_, G)),
    '$goals2list'(G, GList),
    '$check_pp_actions'(GList, [], '$tr'(Head, Rule), Vars),
    fail.   
'$run_tr_checks' :- 
    '$check_tel_start'.
'$run_tr_checks'.


%% Check that no times or actions are used between the tel_start program
%% and the first tel_atomic found in the call chain

%% resources are used so check is done
'$check_tel_start' :-
    \+ (( ('$user_type_info'(resource, _, resource, T,_),
              T \= '$enum_type'([all__]));
          '$tel_atomic'(_) )), !.
'$check_tel_start' :-
    \+'$tel_start'(_), !,
    '$display_error'(no_tel_start).

    
'$check_tel_start' :-
    '$tel_start'(TRF),
    '$user_type_info'(TRF, _, _, TRHead, _),
    '$check_call_trees'(TRHead, no_times, [], _),
    fail.

%% check the possible call stack for time/robotic action errors
%%'$check_call_trees'(TR,  _Times, Done) :- errornl('$check_call_trees'(TR, _Times, Done)),fail.
%% Already checked
'$check_call_trees'(TR, _Times, Done, _) :-
    functor(TR, P, N),
    \+ \+ member(P/N, Done), !.

%% At a tel_atomic call
'$check_call_trees'(TR, Times, Done, Atomic) :-
    '$type_info'(TRF, _, _, TR, _),
    '$tel_atomic'(TRF),
    !,
    '$resource_info'(TR, Res,  _),
    (
      var(Atomic), Times = has_times
    ->
      %% resources are used but a time has been used earlier
     '$display_error'(tel_atomic_timeout(TR))
    ;
      '$tel'(TR, Body),
      '$check_first_nil'(TR, Body),
      functor(TR, P, N),
        Atomic = atomic(TR, Res),
        '$check_call_trees_body'(Body, TR, Times,
                                 [(P/N)|Done], Atomic)
    ).
%% Action is not task atomic and so is a set of robotic actions
'$check_call_trees'(TR, Times, Done, Atomic) :-
    '$tel'(TR, Body),
    functor(TR, P, N),
    '$check_call_trees_body'(Body, TR, Times, [(P/N)|Done], Atomic).

%%'$check_call_trees_body'(Body, TR, Times, Done, Atomic) :- errornl('$check_call_trees_body'(Body, TR, Times, Done, Atomic)),fail.
'$check_call_trees_body'(Body, TR, Times, Done, Atomic) :-
    '$index_member'('tr~>'(Head, RHS), Index, Body),
    '$times_of'(Head, RHS, Times, OTimes),
    '$check_call_trees_RHS'(RHS, Index, TR, OTimes, Done, Atomic),
    fail.

%%'$check_call_trees_RHS'(RHS, Index, TR, OTimes, Done, Atomic) :- errornl('$check_call_trees_RHS'(RHS, Index, TR, OTimes, Done, Atomic)),fail.
'$check_call_trees_RHS'([], _, _, _, _, _) :-
    !.

'$check_call_trees_RHS'(_, _, TR, has_times, _, Atomic) :-
    var(Atomic),
    !,
    '$display_error'(tel_atomic_timeout(TR)).
'$check_call_trees_RHS'(timed_seq(_), _, TR, _, _, Atomic) :-
    var(Atomic),
    !,
    '$display_error'(tel_atomic_timeout(TR)).
'$check_call_trees_RHS'((['$nil'] ++ _), _, _TR, _, _, _Atomic) :-
    !.
'$check_call_trees_RHS'((_ ++ _), _, TR, _, _, Atomic) :-
    var(Atomic),
    !,
    '$display_error'(tel_atomic_goal(TR)).
'$check_call_trees_RHS'((RHS ++ _), Index, TR, Times, Done, Atomic) :-
    !,
    '$check_call_trees_RHS'(RHS, Index, TR, Times, Done, Atomic).
'$check_call_trees_RHS'(['$nil'], _TR, _Index, _Times, _Done, _) :-
    !.
'$check_call_trees_RHS'([R|RHS], Index, TR, Times, Done, Atomic) :-
    !,
    '$check_call_trees_RHS_aux'(R, Index, TR, Times, Done, Atomic),
    '$check_call_trees_RHS'(RHS, Index, TR, Times, Done, Atomic).
'$check_call_trees_RHS'(timed_seq(List), Index, TR, Times, Done, Atomic) :-
    '$check_call_trees_RHS'(List, Index, TR, Times, Done, Atomic).


%%'$check_call_trees_RHS'(RHS, _Index, _TR, Times, Done) :-
%%    '$check_call_trees'(RHS, Times, Done).

%%'$check_call_trees_RHS_aux'(RHS, Index, TR, OTimes, Done, Atomic) :- errornl('$check_call_trees_RHS_aux'(RHS, Index, TR, OTimes, Done, Atomic)),fail.
'$check_call_trees_RHS_aux'(T, _Index, _TR, _Times, _Done, _Atomic) :-
    compound(T),
    functor(T, F, _),
    var(F), !.
'$check_call_trees_RHS_aux'('$time'(_, As, _), Index, TR, Times,
                            Done, Atomic) :-
    !,
    '$check_call_trees_RHS'(As, Index, TR, Times, Done, Atomic).
'$check_call_trees_RHS_aux'(As, Index, TR, Times, Done, Atomic) :-
    list(As),
    !,
    '$check_call_trees_RHS'(As, Index, TR, Times, Done, Atomic).
'$check_call_trees_RHS_aux'(R, _Index, TR, _Times, _Done, Atomic) :-
    var(Atomic),
    '$tel_action'(R), !,
    '$display_error'(non_tel_atomic_robotic(TR)).
'$check_call_trees_RHS_aux'(R, Index, _TR, _Times, _Done, Atomic) :-
    '$tel_action'(R), !,
    '$robotic_resource_info'(R, Res,  _),
    Atomic = atomic(_, Resources),
    ( (member(R1:T, Res), \+member_eq(R1:T, Resources))
    ->
      '$display_error'(tel_atomic_resource(R, Index, Atomic, R1))
    ;
      true
    ).
'$check_call_trees_RHS_aux'(TR, Index, _TR, Times, Done, Atomic) :-
    nonvar(Atomic),
    %'$type_info'(TRF, _, _, TR, _),
    %'$tel_atomic'(TRF,_),
    !,
    '$resource_info'(TR, Res,  _),
    ( nonvar(Atomic),  Atomic = atomic(_, Resources),
        (member(R:T, Res), \+member_eq(R:T, Resources))
    ->
      '$display_error'(tel_atomic_resource(TR, Index, Atomic, R))
    ;
      '$check_call_trees'(TR, Times, Done, atomic(TR, Res))
    ).
'$check_call_trees_RHS_aux'(TR, _Index, _TR, Times, Done, Atomic) :-
    !,
    '$check_call_trees'(TR, Times, Done, Atomic).

'$check_first_nil'(_, ['tr~>'(_, ['$nil'])|_]) :-
    !.
'$check_first_nil'(TR, _Rules) :-
    '$display_error'(tel_atomic_non_nil_first(TR)).

%% determine if a given rule mentions times
%%'$times_of'(A,B,C,D) :- errornl('$times_of'(A,B,C,D)),fail.
'$times_of'(_, _, has_times, has_times) :- !.
'$times_of'(TT, _, _Times, has_times) :-
    '$is_time_term'(TT), !.
'$times_of'('$while'(_, _,TT), _, _Times, has_times) :-
    '$is_time_term'(TT), !.
'$times_of'('$inhibit'(_, _,TT), _, _Times, has_times) :-
    '$is_time_term'(TT), !.
'$times_of'(_, _, T, T).

'$is_time_term'('$time'(_, _)).
'$is_time_term'('$time'(_, _, _)).


'$save_ho_resource_info'(TR, Args, Types, RuleInfo) :-
    '$user_type_info'(resource, _, resource,T,_),
    T \= '$enum_type'([all__]), !,
    '$zip_type'(Args, Types, AT),
    '$add_file_info'('$ho_resource_info'(_, _, _)),
    assert('$ho_resource_info'(TR, AT, RuleInfo)).
    
'$save_ho_resource_info'(_TR, _Args, _Types, _).