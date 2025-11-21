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


/*
  compiler for TR code

*/
?- op(500, fy, '++').
?-op(500,xfy,'++').
?- op(600,xfx,'..').
?-op(920,xfx,'~>').
?- op(1028, xfx,  'inhibit' ). 


%% Add information about what resources TR procedures and robotic actions
%% use
'$add_tr_resource_info' :-
    '$tel'(Term, _),
    functor(Term, P, _),
    '$assert_resource_info'(P),
    fail.
'$add_tr_resource_info':-
    '$assert_robotic_action_resources'.

%% If at least one type has been declared as a resource then extract
%% the resource types from each TR declaration and associate those resources
%% with that TR procedure
'$assert_resource_info'(P) :-
    '$user_type_info'(resource, _, resource, T,_),
    T \= '$enum_type'([all__]),
    !,
    '$type_info'(P, _, Types, Term, _),
    '@..rev'(Term, _, Args),
    '$zip_type'(Args, Types, A3),
    '$add_file_info'('$resource_info'(_, _, P)),
    assert('$resource_info'(Term, A3, P)).
%% If no resources are declared set the resources for each TR procedure to be
%% all__
'$assert_resource_info'(P) :-
    !,
    '$type_info'(P, _, _Types, Term, _),
    '$add_file_info'('$resource_info'(_, _, P)),
    assert('$resource_info'(Term, [all__], P)).

'$zip_type'([], [], []).
'$zip_type'([V|Vs], [T|Ts], [(V:T1)|VTs]) :-
    ( T = _(T1) ; T = T1),
    type_le__(T1, resource), !,
    %'$resource'(T1), !,
    '$zip_type'(Vs, Ts, VTs).
'$zip_type'([_V|Vs], [_T|Ts],VTs) :-
    '$zip_type'(Vs, Ts, VTs).

%% collect resources for robotic actions
'$assert_robotic_action_resources' :-
    '$user_type_info'(tel_action_term, defined_type, tel_action_term, Defn, _),
    Defn = '$constr_enum_type'(Alts),
    member(D, Alts),
    '@..rev'(D, A, Types),
    length(Types, N),
    length(Vars, N),
    '@..'(A, Vars, Patt),
    '$zip_type'(Vars, Types, A3),
    '$add_file_info'('$robotic_resource_info'(_, _, A)),
    assert('$robotic_resource_info'(Patt, A3, A)),
    fail.
'$assert_robotic_action_resources'.

%% Checking the resource uses


%% If there are no resource or tel_atomic
%% declarations then there is nothing to do
'$check_resource_uses' :-
    \+((
       '$user_type_info'(resource, _, resource,T,_),
       T \= '$enum_type'([all__]),
       '$tel_atomic'(_)
      )),
    !.
'$check_resource_uses' :-
    '$check_resource_uses1',
    '$check_resource_uses2', !.
'$check_resource_uses'.

%% if resources are declared then there must be at least one tel_atomic
%% declared
'$check_resource_uses1' :-
    '$tel_atomic'(_), !.
'$check_resource_uses1' :-
    '$display_error'(no_tel_atomic).

%% Check that the actions (recursively) of each tel_atomic TR program
%% only uses resources that are listed in the call
'$check_resource_uses2' :-
    findall(TRF, '$tel_atomic'(TRF), AllTA),
    '$check_resource_uses_list'(AllTA, []).

%%'$check_resource_uses_list'(A,B) :- errornl('$check_resource_uses_list'(A,B)),fail.
%% '$check_resource_uses_list'(ToDo, Seen)
%% ToDo is the list of TR procedures to be checked, Seen is the list of
%% TR procedures that have been checked
'$check_resource_uses_list'([], _).
'$check_resource_uses_list'([TRF|Rest], Seen) :-
    '$type_info'(TRF, _, _, TR, _),
    '$resource_info'(TR, Resources,  _),
    '$tel'(TR, TRBody),
    '$check_resource_uses_aux'(TRBody, TR, Resources, 1, [], CalledTRs),
    sort(CalledTRs, SortedCalledTRs),
    NewSeen = [TRF|Seen],
    diff_list(SortedCalledTRs, NewSeen, ToCheck),
    union_list(Rest, ToCheck, StillToCheck),
    '$check_resource_uses_list'(StillToCheck, NewSeen).

%%'$check_resource_uses_aux'(A,B,C,D,E,F) :- errornl('$check_resource_uses_aux'(A,B,C,D,E,F)),fail.
'$check_resource_uses_aux'([], _TR, _Resources, _I, Out, Out).
'$check_resource_uses_aux'(['tr~>'(_LHS,RHS)|TRBList], TR, Resources,
                           Index, In, Out) :-
    '$check_resource_uses_aux2'(RHS, TR, Resources, Index, In, Mid),
    Index1 is Index+1,
    '$check_resource_uses_aux'(TRBList, TR, Resources, Index1, Mid, Out).


%%'$check_resource_uses_aux2'(Actions, TR, Resources, Index, In, Out) :- errornl('$check_resource_uses_aux2'(Actions, TR, Resources, Index, In, Out)),fail.

%% This is like the above predicate but for the action of the given rule
'$check_resource_uses_aux2'([TR1], TR, Resources, Index, In, Out) :-
    '$ho_resource_info'(TR1, Res,
                        rule_info(tel, tel(TR, 'tr~>'(_, [TR1])),_,_)), !,
    %% TR1 has a variable functor
    diff_list(Res, Resources, NRes),
    Out = In,
    (
      NRes = []
    ->
      true
    ;
      %% TR1 uses resources other than those in TR
      '$display_error'(invalid_resources(TR, Index, NRes))
    ).
'$check_resource_uses_aux2'((Actions ++ _), TR, Resources, Index, In, Out) :-
    !,
   '$check_resource_uses_aux2'(Actions, TR, Resources, Index, In, Out). 
'$check_resource_uses_aux2'(['$nil'], _TR, _Resources, _Index, In, Out) :-
    !, In = Out.
'$check_resource_uses_aux2'([timed_seq(TS)], TR, Resources, Index, In, Out) :-
    !,
    %% a timed sequence
    %% for a timed sequence we collect all the extra resources from each
    %% entry in the sequence
    '$extract_ts_actions'(TS, As),
    '$check_resource_uses_aux3'(As, TR, Resources, Index, In, Out).
'$check_resource_uses_aux2'([TR1], TR, Resources, Index, In, Out) :-
    '$type_info'(TRF, tel, _, TR1, _),
    %% a TR program that need checking
    '$resource_info'(TR1, Res,  _), !,
    diff_list(Res, Resources, NRes),
    Out = [TRF|In],
    (
      NRes = []
    ->
      true
    ;
      %% TR1 uses resources other than those in TR
      '$display_error'(invalid_resources(TR, Index, NRes))
    ).
'$check_resource_uses_aux2'(ActionList, TR, Resources, Index, In, Out) :-
    !,
    '$collect_resources'(ActionList, ActionResources),
    diff_list(ActionResources, Resources, NRes),
    Out = In,
    (
      NRes = []
    ->
      true
    ;
      %% the robotic actions use resources other than those in TR
      '$display_error'(invalid_resources(TR, Index, NRes))
    ).
    

%% For checking timed sequences - check each element in the list
%% and return the overall collection of extra resources
'$check_resource_uses_aux3'(A,B,C,D,E,F) :- errornl('$check_resource_uses_aux3'(A,B,C,D,E,F)),fail.
'$check_resource_uses_aux3'([], _TR, _Resources, _Index, In, Out) :-
    In = Out.
'$check_resource_uses_aux3'([A|As], TR, Resources, Index, In, Out) :-
    '$check_resource_uses_aux2'(A, TR, Resources, Index, In, Mid),
    '$check_resource_uses_aux3'(As, TR, Resources, Index, Mid, Out).


'$extract_ts_actions'([], []).
'$extract_ts_actions'([TSA|Rest], Actions) :-
    '$extract_ts_actions'(Rest, RestActions),
    (
     TSA = '$time'(_, A, _)
    ->
     Actions = [A|RestActions]
    ;
     Actions = [TSA|RestActions]
    ).
    



%% Collect all the resources of a list of robotic actions
'$collect_resources'([], []).
'$collect_resources'([G|Goals], LHSRes) :-
    '$collect_resources'(Goals, LHSres1),
    (
      '$robotic_resource_info'(G, Res,  _)
    ->
      union_list(Res, LHSres1, LHSRes)
    ;
      LHSRes = LHSres1
    ).


'$strip_types'([], []).
'$strip_types'([(X:_)|Xs], [X|Ys]) :-
    '$strip_types'(Xs, Ys).

'$set_tel_vars'(Head) :-
    '$type_info'(_, tel, Types, Head, _),
    '@..rev'(Head, _, Args),
    '$set_tel_vars_aux'(Args, Types, Vars),
    ip_set('$tel_vars', Vars).

%%'$set_tel_vars_aux'(Args, Types, Vars) :- errornl('$set_tel_vars_aux'(Args, Types, Vars)),fail.
'$set_tel_vars_aux'([], [], []).
'$set_tel_vars_aux'([A|Args], [T|Types], [A|Vars]) :-
    var(A), T = 'tel'(_), !,
    '$set_tel_vars_aux'(Args, Types, Vars).
'$set_tel_vars_aux'([_A|Args], [_T|Types], Vars) :-
    '$set_tel_vars_aux'(Args, Types, Vars).

%% expand a TR procedure

'$expand_a_TR'(Head, Body) :-
    %%TR_prog = '$tel'(Head, Body),
    '$saved_input_with_vars'('$tel'(Head,Body), Vars, _),
    %%'$get_underscores'(TR_prog, Vars, Unders),
    %% save the underscore vars -
    %%  when the RID is constructed these vars are ignored
    ip_set('$non_underscore_vars', Vars),
    freeze_term(Body),
    %% TR calls with var functors are treated specially - save the list
    %% of such functors
    '$set_tel_vars'(Head),
    %% expand functions and translate goals and turn each rule into
    %% a uniform structure
    '$translate_TR_body'(Body, TransBody),
    %% convert the body into clauses of tr_rule
    '$expand_one_TR'(Head, TransBody, 1, ClausesTR, InhibitClauses),
    (
      functor(Head, HeadF, _), '$tel_atomic'(HeadF)
    ->
      %% This TR procedure is a task atomic - deal with resouces
      (
       '$resource_info'(Head, TypesResources, _)
      ->
       true
      ;
       TypesResources = []
      ),
     (
      TypesResources = [all__]
     ->
      Resources = [all__]
     ;
      '$strip_types'(TypesResources, Resources)
     ),
      %% Resources is the list of resources this task atomic procedure uses
      %% (or all__ if no resources declared)
     TR = Head,
     AtomicClause =
    (
      (
        tr_rule(TR, AT, Now, TermSeq, wait_choice(TR), InTime, OutDelay,
                Actions,  ResourcesUsed) :-
        ResourcesUsed = no,
        %% not called from within an outer task atomic
        this_task_name(Task),
        \+'__can_run__'(Task, Resources), !,
        %% not runnable
        AT \= wait,
        %% If we are already waiting then no change to the call stack
        Actions = [],
        %% We now enter the wait queue and so need to stop all actions
        %% we were executing
        TermSeq = [wait_choice(TR)],
        min_timeout_(Now, InTime, OutDelay)
      )
    ),
      ClausesTR = [NilClause|RestClausesTR],
      %% We insist that the first rule of a task atomic procedure has
      %% a nil action - if the guard of the first rule is true
      %% we "escape" out of the procedure - i.e. we no longer consider the
      %% procedure to be running (or waiting).
      %% Below we inject the above rule between the first rule and the rest
      %% to manage waiting and running
      %% Note NilClause is a list of clauses - usually with a single
      %% entry but can have 2 in case of a while - turn AtomicClause
      %% into a singleton so we have a list of list of clauses
      FullClausesTR = [NilClause, [AtomicClause]|RestClausesTR]
    ;
      %% Not task atomic so no extra rule required
      FullClausesTR = ClausesTR
    ),
    functor(Head, HeadF, HeadN),
    functor(Template, HeadF, HeadN),
    '$add_file_info'(tr_rule(Template, _, _, _, _, _, _, _, _)),
    %% The inhibit clauses go before any other clauses for the procedure
    forall(member(C, InhibitClauses), assert(C)),
    %% FullClausesTR is a list of lists so two levels of member required
    forall((member(Clauses, FullClausesTR), member(C, Clauses)), assert(C)).

'$translate_TR_body'(Body, Trans) :-
    '$update_whilstinhibits'(Body, TransBody),
    '$translate_TR_rules'(TransBody, Trans).

'$translate_TR_rules'([], []).
'$translate_TR_rules'([R|Rs], [RT|RsT]) :-
    '$translate_a_tr_rule'(R, RT),
    '$translate_TR_rules'(Rs, RsT).

%%'$translate_a_tr_rule'(A,B) :- errornl('$translate_a_tr_rule'(A)),fail.
'$translate_a_tr_rule'('tr~>'(LHS, rb(RHS, Acts)),
                       'tr~>'(TLHS, rb(TRHS, TActs))) :-
    LHS = whilstinhibit(G, W, U),
    '$expand_funs'(W, TW, WExtra),
    '$expand_funs'(U, TU, UExtra),
    '$expand_funs'(RHS, TRHS, RExtra),
    '$translate_goal'(G, TransG),
    '$translate_goal'(Acts, TActs),
    '$translate_timed_goal'(TW, TransW0),
    '$translate_timed_goal'(TU, TransU0),
    (
      WExtra = []
    ->
      TransW = TransW0
    ;
      '$list2tuple'(WExtra, WEC),
     TransW = (WEC, TransW)
    ),
    (
      UExtra = []
    ->
      TransU = TransU0
    ;
      '$list2tuple'(UExtra, UEC),
       TransU = (UEC, TransU0)
    ),
    (
      RExtra = []
    ->
      TLHS = whilstinhibit(TransG,  TransW, TransU)
    ;
      '$list2tuple'(RExtra, RGoals),
      TLHS = whilstinhibit((TransG, RGoals), TransW, TransU)
    ).

'$translate_timed_goal'('$time'(_, G, T), '$time'(TransG, T)) :-
    !,
    '$translate_goal'(G, TransG).
'$translate_timed_goal'('$time'(_, T), '$time'(T)) :- !.
'$translate_timed_goal'(G, TransG) :-
    '$translate_goal'(G, TransG).



'$update_whilstinhibits'(['tr~>'(true, RBody)], WBody) :-
    !,
    '$update_whilstinhibits_aux'(true, WHead),
    '$add_action_wrapper'(RBody, WRBody),
    WBody = [(WHead ~> WRBody)].
'$update_whilstinhibits'(['tr~>'(RHead, RBody)], WBody) :-
    !,
    '$update_whilstinhibits_aux'(RHead, WHead),
    '$update_whilstinhibits_aux'(true, TrueHead),
    '$add_action_wrapper'(RBody, WRBody),
    WBody = ['tr~>'(WHead,WRBody), 'tr~>'(TrueHead, rb(pa([error]), true))].
'$update_whilstinhibits'(['tr~>'(RHead, RBody) | BRest], WBody) :-
    !,
    '$update_whilstinhibits_aux'(RHead, WHead),
    '$add_action_wrapper'(RBody, WRBody),
    '$update_whilstinhibits'(BRest, WBody1),
    WBody = ['tr~>'(WHead, WRBody)|WBody1].

'$update_whilstinhibits_aux'('$inhibit'(_, G, U),
                           whilstinhibit(G, '$none', U)) :- !.
'$update_whilstinhibits_aux'('$while'(_, G, W),
                           whilstinhibit(G, W, '$none')) :- !.

'$update_whilstinhibits_aux'(G, whilstinhibit(G, '$none', '$none')) :- !.



'$add_action_wrapper'(Body, WBody) :-
    Body = '++'(G,A),!,
    '$add_action_wrapper'(G, rb(WG, _)),
    WBody = rb(WG,A).

'$add_action_wrapper'(Body, WBody) :-
    Body = ['$nil'], !,
    WBody = rb(pa([]), true).
'$add_action_wrapper'(Body, WBody) :-
    Body = [TR], '$type_info'(_, tel, _, TR, _), !,
    WBody = rb(TR, true).
'$add_action_wrapper'(Body, WBody) :-
    %% a variable functor TR call
    Body = [TR], functor(TR, F, _), var(F),  ip_lookup('$tel_vars', PV),
    member_eq(F, PV), !,
    WBody = rb(TR, true).

'$add_action_wrapper'(Body, WBody) :-
    Body = timed_seq(Seq), !,
    '$add_action_wrapper_ts'(Seq, WSeq),
    WBody = rb(timed_seq(WSeq), true).


'$add_action_wrapper'(Body, rb(pa(Body), true)).

'$add_action_wrapper_ts'([], []).
'$add_action_wrapper_ts'([S|Seq], [WS|WSeq]) :-
    (
      S = '$time'(_, B,T)
    ->
      '$add_action_wrapper'(B, rb(BW, _)),
      WS = (BW : T)
    ;
      '$add_action_wrapper'(S, rb(BW, _)),
      WS = (BW : -1)
    ),
    '$add_action_wrapper_ts'(Seq, WSeq).

 
'$process_vars_of_rule'(TR,  RHead, Whilst, Inhibit, RBody, RIDVars) :-
    collect_vars(TR, TRVars),
    collect_vars((TR, RHead), TRHVars),
    collect_vars((Whilst, Inhibit), WIVars),
    collect_vars(RBody, BodyVars),
    diff_list(TRHVars, TRVars, GuardVars),
    %% GuardVars are the variables that are in the guard but not the head
    ip_lookup('$non_underscore_vars', NamedVars),
    '$get_underscores'(RHead, NamedVars, Locals),
    '$get_underscores'(WIVars, NamedVars, WILocals),
    diff_list(WIVars, WILocals, WINonLVars),    
    %% Locals are the underscore vars in the guard so remove them
    %% to get the RID vars
    diff_list(GuardVars, Locals, GuardVars1),
    intersect_list(GuardVars1, WINonLVars, KeepVars),
    intersect_list(GuardVars1, BodyVars, RIDVars1),
    union_list(RIDVars1, KeepVars,  RIDVars).

    
generate_indexed_TR(TR, Index, TR_I) :-
    functor(TR, F, N),
    concat_atom([F, N, Index], '_', TR_I).

%% expand a single TR program
%% '$expand_one_TR'(Head, Rules, Index, Clauses, InhibitClauses)
%% Head is the TR procedure head
%% Rules is the list of rules
%% Index is the index of the rule (starts with 1 and increments as the
%%     rules are processed
%% Clauses is the list of tr_rule clauses generated by processing the rules
%% InhibitClauses are the extra tr_rule clauses generated by rules with
%%     an inhibit
%%'$expand_one_TR'(A,B,C,D,E) :- errornl('$expand_one_TR'(A,B,C,D,E)),fail.
'$expand_one_TR'(_, [], _, [], []) :- !.
'$expand_one_TR'(Head, ['tr~>'(whilstinhibit(RHead, Whilst, Inhibit),RBody)|BRest],
                 Index, AllClausesTR, InhibitClauses) :-
    '$build_rule'(Head, Index, RHead, Whilst, Inhibit, RBody,
                  RuleList, InhibitList),
    !,
    Index1 is Index+1,
    %% Rules list will ususally have one element unless its a while rule
    %% In any case AllClauses is a list of lists of clauses
    AllClausesTR = [RuleList|ClausesTR],
    %%append(RuleList, ClausesTR, AllClausesTR),
    %% InhibitList will either be empty if not an inhibit rule
    append(InhibitList, RestInhibitList, InhibitClauses),
    '$expand_one_TR'(Head, BRest, Index1, ClausesTR, RestInhibitList).

'$repeat'(0, _RPA, []) :- !.
'$repeat'(N, RPA, [RPA|Rest]) :- N1 is N-1, '$repeat'(N1, RPA, Rest).

%% '$build_rule'(TR, Index, RHead, Whilst, Inhibit, RBody,
%%               RuleList, InhibitList)
%% TR is the head of the TR procedure
%% Index is the index of this rule
%% RHead is the guard of the rule
%% Whilst is the while part of the rule ($none if no while)
%% Inhibit is the inhibit part of the rule ($none if no while)
%% RBody is the RHS of the rule
%% RuleList and Inhibit lists are lists of generated tr_rule/9 clauses
%% the head of each tr_rule clause is of the form
%% tr_rule(TR, InRID, Now, TermSeq, LastTerm, 
%%         InTimes, OutDelay, Actions, 
%%         ResourcesUsed)
%% where
%% TR is the head of the TR procedure
%% InRID is incomming rule ID ($none if this is new call on this clause
%%     used to check if the instantiation of guard variables have changed
%%     if so then its a refiring, if not its a continuation
%% Now is the current time (used to calculate delays)
%% TermSeq is the complete call stack
%% InTimes are all the times from earlier in the call stack
%% OutDelay is the time delay to the closest time in the call stack
%% Actions are the new actions from this call stack evaluation
%% ResourcesUsed is the set of resources used by this call stack
%% NOTE: we need InTimes to be the list of all times earlier in the call stack
%% rather than the minimum time because, when this time expires, this min
%% time is now in the past and needs to be ignored. By keeping all the times
%% we can compute the new minimum (future) time.
%%'$build_rule'(A,B,C,D,E,F,G,H) :- errornl('$build_rule'(A,B,C,D,E,F,G,H)),fail.
'$build_rule'(TR, _Index, _RHead, _Whilst, _Inhibit, RBody, RuleList, []) :-
    RBody = rb(A, _G),
    A = pa([error]), !,
    %% We add this as a new last rule if the guard of the last rule
    %% is not true
    %% This generated clause will only be called if no guard of
    %% earlier rules are true
    RuleHead = tr_rule(TR, _, _,  _TermsSeq, _, _, _, _, _),
    RuleBody = throw(no_matching_tr_rule(TR)),
    RuleList = [(RuleHead :- RuleBody)].
    
'$build_rule'(TR, Index, RHead, Whilst, Inhibit, RBody, 
              RuleList, InhibitList) :-
    RBody = rb(A, G),
    '$tel_atomic'(_),
    A = pa([]), !,
    A1 = [],
    '$process_vars_of_rule'(TR, RHead, Whilst, Inhibit, RBody, RIDVars),
    RuleHead = tr_rule(TR, InRID, Now, TermSeq, Term, InTimes, OutDelay,
                       Actions, ResourcesUsed),
    (
      G == true
    ->
      Goal_and_Action = true
    ;
      Goal_and_Action = '$delay_tr_call'(G)
    ),
    (
      functor(TR, TRF, _), '$tel_atomic'(TRF), Index \= 1
    ->
      Cleanup = true
    ;
      Cleanup = check_if_should_clean_up_(ResourcesUsed, InRID)
    ),
    RuleBody = (
                 RHead, !,
                 TimeoutGoal,
                 RID = rid(Index, RIDVars, T),
                 InRID \=RID,
                 T = Timeout,
                 Cleanup,
                 Term = rule_choice(TR, RID, InTimes, ResourcesUsed, _), 
                 Actions = A1,
                 TermSeq = [Term],
 		 Goal_and_Action,
                 min_timeout_(Now, OutTimes, OutDelay)
                 ),
    Rule = (RuleHead :- RuleBody),
    '$build_whilst_inhibit_rules'(Whilst, Inhibit, TR, Index, RIDVars,
                                  Now, InTimes, OutTimes,
                                  Rule, RuleList, InhibitList,
                                  TimeoutGoal, Timeout).

'$build_rule'(TR, Index, RHead, Whilst, Inhibit, RBody, 
              RuleList, InhibitList) :-
    RBody = rb(A, G),
    A = pa(A1), !,
    '$process_vars_of_rule'(TR, RHead, Whilst, Inhibit, RBody, RIDVars),
    RuleHead = tr_rule(TR, InRID, Now, TermSeq,  Term, InTimes, OutDelay,
                       Actions,  ResourcesUsed),
    (
      G == true
    ->
      Goal_and_Action = true
    ;
      Goal_and_Action = '$delay_tr_call'(G)
    ),

    RuleBody = (
                 RHead, !,
                 TimeoutGoal,
                 RID = rid(Index, RIDVars, T),
                 InRID \= RID,
                 T = Timeout,
                 Term = rule_choice(TR,  RID, InTimes, ResourcesUsed, _), 
                 Actions = A1,
                 TermSeq = [Term],
		 Goal_and_Action,
                 min_timeout_(Now, OutTimes, OutDelay)
                  ),
    Rule = (RuleHead :- RuleBody),
    '$build_whilst_inhibit_rules'(Whilst, Inhibit,  TR,  Index, RIDVars,
                                  Now, InTimes, OutTimes,
                                  Rule, RuleList, InhibitList,
                                  TimeoutGoal, Timeout).


'$build_rule'(TR, Index, RHead, Whilst, Inhibit, RBody, 
              RuleList, InhibitList) :-
    RBody = rb(A, G),
    A = timed_seq([pa(A1):T1|_]), !,
    '$process_vars_of_rule'(TR, RHead, Whilst, Inhibit, RBody, RIDVars),
    (
      functor(TR, TRF, _), '$tel_atomic'(TRF)
    ->
      OutResourcesUsed = yes
    ;
      OutResourcesUsed = ResourcesUsed
    ),
    (
      G == true
    ->
      Goal_and_Action = true
    ;
      Goal_and_Action = '$delay_tr_call'(G)
    ),

    RuleHead = tr_rule(TR, InRID, Now, TermSeq, TSTerm, InTimes, OutDelay,
                       Actions, ResourcesUsed),
 
    TimeGoal = (NowTS = Now+T1, OutTimes = [NowTS|MidTimes]),
    RuleBody = (
                 RHead, !,
                 TimeoutGoal,
                 A = timed_seq(AV),
                 RID = rid(Index, RIDVars, T),
                 InRID \= RID,
                 T = Timeout,
                 Term = rule_choice(TR, RID, InTimes, ResourcesUsed, _),
                 TimeGoal,
                 TSTerm = timed_seq(NowTS, AV, AV,
                                    MidTimes, OutResourcesUsed),
                 Actions = A1,
                 TermSeq = [Term, TSTerm],
		 Goal_and_Action,
                 min_timeout_(Now, OutTimes, OutDelay)
               ),
    
    Rule = (RuleHead :- RuleBody),
    '$build_whilst_inhibit_rules'(Whilst, Inhibit,  TR,  Index, RIDVars,
                                  Now, InTimes, MidTimes, 
                                  Rule, RuleList, InhibitList,
                                  TimeoutGoal, Timeout).

'$build_rule'(TR, Index, RHead, Whilst, Inhibit, RBody, 
              RuleList, InhibitList) :-
    RBody = rb(A, G),
    A = timed_seq([TR1:T1|_]), !,
    '$process_vars_of_rule'(TR, RHead, Whilst, Inhibit, RBody, RIDVars),
    (
      functor(TR, TRF, _), '$tel_atomic'(TRF)
    ->
      OutResourcesUsed = yes
    ;
      OutResourcesUsed = ResourcesUsed
    ),
    (
      G == true
    ->
      Goal = true
    ;
      Goal = '$delay_tr_call'(G)
    ),
    RuleHead = tr_rule(TR, InRID, Now, TermSeq, LastTerm, InTimes, OutDelay,
                       Actions, ResourcesUsed),

    TimeGoal = (NowTS = Now+T1,  OutTimes = [NowTS|MidTimes]),
    RuleBody = (
                 RHead, !,
                 TimeoutGoal,
                 A = timed_seq(AV),
                 RID = rid(Index, RIDVars, T),
                  InRID \= RID,
                T = Timeout,
                 TimeGoal,
                 Term = rule_choice(TR, RID, InTimes, ResourcesUsed, _), 
		TSTerm = timed_seq(NowTS, AV, AV,
				   MidTimes, ResourcesUsed),
                 TermSeq = [Term, TSTerm|MidTerms],
                 Goal,
                 tr_rule(TR1, none, Now, MidTerms, LastTerm, OutTimes, OutDelay,
                         Actions, OutResourcesUsed)
	      ),
    Rule = (RuleHead :- RuleBody),
    '$build_whilst_inhibit_rules'(Whilst, Inhibit,  TR,  Index, RIDVars,
                                  Now, InTimes, MidTimes,
                                  Rule, RuleList, InhibitList,
                                  TimeoutGoal, Timeout).
    

'$build_rule'(TR, Index, RHead, Whilst, Inhibit, RBody, 
              RuleList, InhibitList) :-
    RBody = rb(A, G),
    A = TR1,
    '$process_vars_of_rule'(TR, RHead, Whilst, Inhibit, RBody, RIDVars),
    (
      G == true
    ->
      Goal = true
    ;
      Goal = '$delay_tr_call'(G)
    ),
    (
      functor(TR, TRF, _), '$tel_atomic'(TRF)
    ->
      OutResourcesUsed = yes,
      Cleanup = true
    ;
      '$tel_atomic'(_)
    ->
      Cleanup = check_if_should_clean_up_(ResourcesUsed, InRID),
      OutResourcesUsed = ResourcesUsed
    ;
      Cleanup = true,
      OutResourcesUsed = ResourcesUsed
    ),
    
    RuleHead = tr_rule(TR, InRID, Now, TermSeq, LastTerm, InTimes, OutDelay,
                       Actions, ResourcesUsed),
    RuleBody = (
                 RHead,!,
                 TimeoutGoal,
                 RID = rid(Index, RIDVars, T),
                 InRID \= RID,
                 T = Timeout,
                 Cleanup,
                 Term = rule_choice(TR, RID, InTimes, OutResourcesUsed, _), 
                 TermSeq = [Term|MidTerms],
                 Goal,
                 tr_rule(TR1, none, Now, MidTerms, LastTerm, OutTimes, OutDelay,
                         Actions, OutResourcesUsed)
               ),
    Rule = (RuleHead :- RuleBody),
    '$build_whilst_inhibit_rules'(Whilst, Inhibit,  TR,  Index, RIDVars,
                                  Now, InTimes, OutTimes,
                                  Rule, RuleList, InhibitList,
                                  TimeoutGoal, Timeout).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% inhibit + time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'$build_whilst_inhibit_rules'('$none', '$time'(Inhibit, Time), TR,
                               Index, RIDVars, Now, InTimes, OutTimes,
                              Rule, RuleList, InhibitList,
                              TimeoutGoal, Timeout) :-
    !,
    TimeoutGoal = (Timeout is Now + Time),
    OutTimes = [Timeout|InTimes],
    RuleList = [Rule],
    InhibitList = [
                   (tr_rule(TR, CurrRID, Now, TermSeq, _, _InTimes, _OutDelay,
                           NewActions, ResourcesUsed) :-
                   CurrRID = rid(Index, RIDVars, TO),
                       Now =< TO,
                       !, fail),
                   (tr_rule(TR, CurrRID, Now, TermSeq, _, _InTimes, _MinTime,
                            NewActions, ResourcesUsed) :-
                   CurrRID = rid(Index, RIDVars, _),
                       Inhibit,
                       !, fail)
                  ].
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% inhibit time only
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
'$build_whilst_inhibit_rules'('$none', '$time'(Time), TR,
                               Index, RIDVars, Now, InTimes, OutTimes,
                              Rule, RuleList, InhibitList,
                              TimeoutGoal, Timeout) :-
    !,
    TimeoutGoal = (Timeout is Now + Time),
    OutTimes = [Timeout|InTimes],
    RuleList = [Rule],
    InhibitList = [
                   (tr_rule(TR, CurrRID, Now, _TermSeq, _, _InTimes, _MinTime,
                            _NewActions, _ResourcesUsed) :-
                  CurrRID = rid(Index, RIDVars, TO),
                        Now =< TO,
                       !, fail)
                  ].
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% whilst + time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

'$build_whilst_inhibit_rules'('$time'(Whilst, Time), '$none',  TR,
                              Index, RIDVars, Now, InTimes, OutTimes,
                              Rule, RuleList, [], TimeoutGoal, Timeout) :-
    !,
    TimeoutGoal = (Timeout is Now + Time),
    OutTimes = [Timeout|InTimes],
    RuleList = [(tr_rule(TR, CurrRID, Now, TermSeq, _, _InTimes, MinTime,
                            NewActions, ResourcesUsed) :-
                   CurrRID = rid(Index, RIDVars, TO),
                       Now =< TO,
                       !, fail),
                   (tr_rule(TR, CurrRID, Now, TermSeq, _, _InTimes, MinTime,
                             NewActions, ResourcesUsed) :-
                   CurrRID = rid(Index, RIDVars, _),
                       Whilst,
                       !, fail),
                Rule].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% whilst time only
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 '$build_whilst_inhibit_rules'('$time'(Time), '$none',  TR,
                              Index, RIDVars, Now, InTimes, OutTimes,
                              Rule, RuleList, [], TimeoutGoal, Timeout) :-
    !,
    TimeoutGoal = (Timeout is Now + Time),
    OutTimes = [Timeout|InTimes],
    RuleList = [(tr_rule(TR, CurrRID, Now, _TermSeq, _, _InTimes, _MinTime,
                            _NewActions, _ResourcesUsed) :-
                   CurrRID = rid(Index, RIDVars, TO),
                       Now =< TO,
                    !, fail),
                Rule].   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% no while or inhibit
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 '$build_whilst_inhibit_rules'('$none', '$none',  _TR,
                              _Index, _RIDVars, _Now, InTimes, OutTimes,
                               Rule, RuleList, [], TimeoutGoal, _Timeout) :-
     !,
     OutTimes = InTimes,
     RuleList = [Rule],
     TimeoutGoal = true.
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% inhibit no time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'$build_whilst_inhibit_rules'('$none', Inhibit, TR,
                               Index, RIDVars, Now, InTimes, OutTimes,
                              Rule, RuleList, InhibitList,
                              TimeoutGoal, _Timeout) :-
    !,
    TimeoutGoal = true,
    OutTimes = InTimes,
    RuleList = [Rule],
    InhibitList = [
                   (tr_rule(TR, CurrRID, Now, _TermSeq, _, _InTimes, _MinTime,
                            _NewActions, _ResourcesUsed) :-
                    CurrRID = rid(Index, RIDVars, _),
                       Inhibit,
                       !, fail)
                  ].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% whilst no time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

'$build_whilst_inhibit_rules'(Whilst, '$none',  TR,
                              Index, RIDVars, Now, InTimes, OutTimes,
                              Rule, RuleList, [], TimeoutGoal, _Timeout) :-
    !,
    TimeoutGoal = true,
    OutTimes = InTimes,
    RuleList = [(tr_rule(TR, CurrRID, Now, _TermSeq, _, _InTimes, _MinTime,
                            _NewActions, _ResourcesUsed) :-
                   CurrRID = rid(Index, RIDVars, _),
                       Whilst,
                      !, fail),
                Rule].
     



'$filter_retry_list'([], []).
'$filter_retry_list'([Clause|Rest], RetryList) :-
    Clause = (_ :- overlap_at(_, _, 1, _), _), !,
    '$filter_retry_list'(Rest, RetryList).
'$filter_retry_list'([Clause|Rest], RetryList) :-
    Clause = (_ :- overlap(_, []), _), !,
    '$filter_retry_list'(Rest, RetryList).
'$filter_retry_list'([Clause|Rest], [Clause|RetryList]) :-
    '$filter_retry_list'(Rest, RetryList).


'$tr_compute_rule_body0'(A, A) :- errornl('FIX'), !.

'$tr_compute_rule_body0'((RBody++G), rb(RBody, G)) :-
    !,
    '$tr_compute_rule_body'(RBody, RBody, true).


'$tr_compute_rule_body0'(RBody, rb(RBody, _)) :-
    !,
    '$tr_compute_rule_body'(RBody, RBody, true).


%% A conjuction of robotic actions
%% is converted to a list of actions and wrapped in a pa(...)
'$tr_compute_rule_body'({}, pa([]), _) :- !.
'$tr_compute_rule_body'({}(RBody), pa(CBody), _) :-
    !,
    goals2list(RBody, CBody).

%'$tr_compute_rule_body'(wait_repeat(A, T, N), wait_repeat(A, T, N), _) :- !.
'$tr_compute_rule_body'(RBody, RBody, _NoSeq) :-
    functor(RBody, P, N),
    functor(Copy, P, N),
    '$tel'(Copy, _), !.
'$tr_compute_rule_body'(RBody, RBody, NoSeq) :-
    '$tr_compute_cyclic_rule_body'(RBody, RBody, NoSeq),!.
% a TR as an atom
'$tr_compute_rule_body'(RBody, RBody, _).

'$tr_compute_cyclic_rule_body'(_, _, fail) :-
    write_term_list(stderr, ['Error - timed sequence inside a timed sequence is not allowed', nl]),
    thread_exit.

'$tr_compute_cyclic_rule_body'([], [], _NoSeq).
 
'$tr_compute_cyclic_rule_body'([A:T|Rest], [CA:T|CRest], NoSeq) :-
    !,
    '$tr_compute_rule_body'(A, CA, fail),
    '$tr_compute_cyclic_rule_body'(Rest, CRest, NoSeq).
'$tr_compute_cyclic_rule_body'([A], R, _NoSeq) :-
    '$tr_compute_rule_body'(A, CA, fail),
    R = [(CA:(-1))].    




xxx_ :-
    listing(tr_rule/9).
