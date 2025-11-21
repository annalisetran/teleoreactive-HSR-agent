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

%% Implementation of watch

?- compile_time_only(consult(op_decls)).
?- op(300,fx,'watch__').


?-dynamic('$watched'/1).
?-dynamic('$ho_function_watched_clause'/1).

%% How watch works
%%
%% Relations:
%% Take the example of user defined member relation:
%% memb(X, [X|_])
%% memb(X, [_|T]) <= memb(X, T)
%%
%% which asserts the QP code
%% memb(X, [X|_])
%% memb(X, [_|T]) :- memb(X, T)
%% When we call (in the interpreter)
%% watch memb.
%% we delete the old code (but save it away so that it can easily be resored
%% if we call unwatch memb.)
%% The code is replace by
%% memb(A, B) :- watch__ (memb_watched, memb(A, B)).
%% and
%% memb_watched(A, B, C) :-
%%     name_the_vars_([D = X_]),
%%     rebuild_name_map_([D = X_], F),
%%     D = B, [D |G] = C,
%%     watch_debug_write_(memb(D, [D |G]), F, E, memb, 1)
%% memb_watched(A, B, C) :-
%%     name_the_vars_([D = X_, E = T_]),
%%     rebuild_name_map_([D = X_, E = T_], G),
%%     D = B, [H |E] = C,
%%     watch_debug_write_(memb(D, [H |E]) <= memb(D, E), G, F, memb, 2),
%%     memb(D, E)
%%
%% When memb(A,B) is called - watch__ (memb_watched, memb(A, B))
%% is called which generates information about variables and call level
%% and makes a call to memb_watched which has the effect of calling the
%% original memb but with information output about the call stack and
%% variable bindings.
%%
%% For actions and simple functions a similar approach is taken but because
%% there is no backtracking then the process is simpler as  there
%% are no choicepoints that we need to keep track of.
%%
%% For higher order functions we need to update the apply definition
%% for that function.  For example consider
%% curry(F)(X)(Y) -> F(X,Y)
%% This definition generates the apply clauses
%% 'curry$apply'(curry, [A], curry(A)) :- 
%%     !.
%% 'curry$apply'(curry(A), [B], curry(A)(B)) :- 
%%     !.
%% 'curry$apply'(curry(A)(B), [C], D) :- 
%%     !,
%%     apply_(A, [B, C], E),
%%     unify(E, D).
%%
%% After watching curry this definition changes to
%%
%% 'curry$apply'(curry, [A], curry(A)) :- 
%%     !.
%% 'curry$apply'(curry(A), [B], curry(A)(B)) :- 
%%     !.
%% 'curry$apply'(curry(A)(B), [C], D) :- 
%%     '$ho_watched_neck',
%%     name_the_vars_([A = 'F_', B = 'X_', C = 'Y_']),
%%     global_state_increment(watch__, E),
%%     F = E,
%%     watch_debug_function_write_(curry(A)(B)(C), 1, F, (curry(A)(B)(C) -> A(B, C)), [A = 'F', B = 'X', C = 'Y']),
%%     ( true
%%       ->
%%         !,
%%         watch_debug_function_write_succeed_(curry(A)(B)(C), F, A(B, C)),
%%         apply_(A, [B, C], G),
%%         unify(G, D),
%%         watch_debug_function_write_result_(curry(A)(B)(C), 1, D, F) 
%%     ;
%%         watch_debug_function_write_fail_(F),
%%         global_state_decrement(watch__, H),
%%         fail 
%%     ).
%% This definition displays evaluation information in the same way as for
%% normal functions




%% The global state variable watch__ stores the call depth (level)
%% that is incremented when a call is made and decremeted when a call exits


%% watched functions are handled directly by the modified call
%% this is simpler because all variables in the head are ground at the
%% time of call
watch__ (fun(_NF), Call) :-
    !,
    global_state_increment(watch__,_WL),
    Call.
%% NR is, for example, memb_watched and Call is the call being made -
%% eg memb(A, B)
watch__ (NR,Call) :-
    %% a new call so increment the call depth
    global_state_increment(watch__,WL),
    %% name any unnamed variables so variables remain linked below
    name_vars(Call),
    '$watch_call2string'(Call, String),
    %% String is a pretty printed version of the call
    %% (using the variable names)
    %% We now create a choicepoint so that backtracking will produce
    %% output before continuing to backtrack
    (
      write(WL), write(':'),
      write_string(String),
      nl
    ;
      write(WL), write(':'),
      write_string(String),
      write(' no (more) proofs\n'), 
      fail
    ),
    '@=..'(Call, [_|Args]),
    %% Args is the list of arguments of the call
    collect_vars(Call,CallVars),
    %% CallVars are all the variables in the call
    construct_(CallVars,NamePairs),
    %% above constructs a list of Name=Var pairs of all vars in Call
    %% that will have been given names by above name_vars(Call)
    %% This list is passed into the watched call so that output bindings
    %% can be displayed
    '@=..'(NewCall, [NR,NamePairs|Args]),
    %% EG NewCall = memb_watched(NamePairs, X, [X|_])
    NewCall,
    %% After calling NewCall we pretty print the original call with
    %% new variable bindings
    '$watch_call2string'(Call, NewString),
    %% again we set up a choicepoint for displaying info on backtracking
    (
      writeL_([WL,":",NewString, " succeeded\n" ])
    ;
      display_call_num_(WL),
      write_string(String),
      writeL_([" seeking another proof\n"]), 
      fail
    ).

%% pretty print the call as a string taking care to translate apply calls
%% back to original call
'$watch_call2string'(Apply(F, CArgs, _), String) :-
    atom_search(Apply, 1, '$apply', _), !,
    open_string(write, Stream),
    '@=..'(HO_Call, [F|CArgs]),
    writeTerm_(Stream, HO_Call),
    stream_to_string(Stream, String).
'$watch_call2string'(Call, String) :-
    open_string(write, Stream),
    writeTerm_(Stream, Call),
    stream_to_string(Stream, String).

    

%% watch_debug_write_(Clause, Vars, HeadPairs, Rel, I)
%% Clause : the rule to display info for
%% Vars: a mapping between varibles and variable names
%% HeadPairs : like Vars but for variables in the head
%% Rel : the name of the relation
%% I :the rule index (1 means the first rule for Rel)
%%watch_debug_write_(Clause, Vars, HeadPairs, Rel, I) :- errornl(watch_debug_write_(Clause, Vars, HeadPairs, Rel, I)),fail.
watch_debug_write_(Clause, Vars, HeadPairs, _Rel, I) :-
    global_state_lookup(watch__,CL),
    (
      prune_(HeadPairs,PrunedHeadPairs),
      %% PrunedHeadPairs is HeadPairs but with variables that have not been
      %% instantiated (to another variable or term) removed
      diff_list(PrunedHeadPairs, Vars, DiffPrunedHeadPairs),
      writeL_(["  Call ",CL," unifies rule ",I]),
      writeL_(["\n     "]),displayBV_(Vars),
      % (
      %   '$watch_clauses'(Rel)
      % ->
      %   %% watchIO was called and so extra information about
      %   %% bindings for input variables is given
      %   writeL_(["\n     "]),displayBV_(Vars)
      % ;
      %   true
      % ),
      %% display the output variables - i.e. vars instantiated by the call
      display_(DiffPrunedHeadPairs),
      do_watch_debug_write_body_(Clause)
    ;
      %% on backtracking this info is displayed before backtracking further
      writeL_(["  no (more) proofs using rule ",I,
               " trying next rule for call ",CL,nl_]),
      fail
    ).

%% Instantiate unnamed variables to underscore and pretty print body
do_watch_debug_write_body_(Clause) :-
    collect_vars(Clause, Vars),
    filter(var_name_filter_, Vars, UVars),
    '$unify_underscore'(UVars),
    %% all unnamed vars are instantiated to '$$var$$'('_') which
    %% pretty prints as _
    watch_debug_write_body_(Clause),
    %% fail to remove instantiation of unnamed vars
    fail.
do_watch_debug_write_body_(_).

do_watch_debug_write_function_body_(Clause) :-
    collect_vars(Clause, Vars),
    filter(var_name_filter_, Vars, UVars),
    '$unify_underscore'(UVars),
    %% all unnamed vars are instantiated to '$$var$$'('_') which
    %% pretty prints as _
    watch_debug_write_function_body_(Clause),
    fail.
do_watch_debug_write_function_body_(_).

var_name_filter_(X) :- \+ get_var_name(X, _).

%% Pretty print rule body
watch_debug_write_body_((_ :: Test <= Body)) :-
    !,
    writeL_(["  Rule body is:\n    :: ", Test, " <=\n"]),
    show_body_( Body,6,stdout), nl.
watch_debug_write_body_((_ <= Body)) :-
    !,
    writeL_(["  Rule body is:\n"]),
   show_body_(Body,6,stdout),nl.
watch_debug_write_body_((_ :: Test ~> Body)) :-
    !,
    writeL_(["  Rule body is:\n    :: ", Test, " ~>\n"]),
    show_body_( Body,6,stdout), nl.
watch_debug_write_body_((_ ~> Body)) :-
    !,
    writeL_(["  Rule body is:\n"]),
   show_body_(Body,6,stdout),nl.
watch_debug_write_body_((_ :: Test)) :-
    !,
    writeL_(["  Rule body is:  :: ", Test, nl_]).
watch_debug_write_body_(_):-
    writeL_(["  No rule body\n"]).  

%%Pretty print function rule body
watch_debug_write_function_body_((_ :: Test -> Body)) :-
    !,
    writeL_(["\n  Function body is:\n    :: "]),
    show_cond_(Test, 0, stdout),
    writeL_([" ->\n"]),
    show_body_( Body,6,stdout), nl.
watch_debug_write_function_body_((_ -> Body)) :-
    !,
    writeL_(["\n  Function body is:\n"]),
   show_body_(Body,6,stdout),nl.

%% As in the example at the beginning of the file we have
%% name_the_vars_([B = 'X_'])
%% This will, if B does not already have a name give it a name like X_0, X_1 ..
%% with the suffix being the first number n foe which X_n is not already a
%% used name. THis means that for recursive calls like memb then we will
%% see X_0 for the first call, X_1 for the next level of recursion etc.
name_the_vars_([]).
name_the_vars_([(V=A)|Rest]) :-
    (
      var(V), \+ get_var_name(V, _)
    ->
      with_local_exception_handler(set_var_name(V, A), _, true)
    ;
      true
    ),
    name_the_vars_(Rest).


%% For displaying the output bindings of variables in a call
display_([]) :- !,     writeL_([nl_, "      output none\n"]).
display_(Pairs) :-
   writeL_([nl_, "      output "]),
   forall( member((V=Nm),Pairs), writeL_([uq_(Nm=V),"  "])),
   nl. 

%% For displaying bindings for input variables
displayBV_([]) :- !.
displayBV_(Pairs) :-
   prune_(Pairs, Vars),
   (
     Vars = []
   ->
     writeL_([" input none"])
   ;
     writeL_([" input"]),
     forall(member((V=Nm),Vars),  writeL_(["  ",uq_(Nm=V)]))
   ). 


% prune removes binding pairs where the var is still unbound
prune_([],[]):-!.
prune_([V=Nm|Pairs],[V=Nm|PrunedPairs]) :-
   nonvar(V), !,
   prune_(Pairs, PrunedPairs).
prune_([V=Nm|Pairs],[V=Nm|PrunedPairs]) :-
   get_var_name(V, Vn), Vn \= Nm, !,
   prune_(Pairs, PrunedPairs).
prune_([_|Pairs],PrunedPairs) :- 
   prune_(Pairs, PrunedPairs).



%% Generate the watch version of each function clause
%% Fun is the name of the function and N is the function arity
watch_trans_function_clauses_(Fun,_N) :-
    ip_set('$tel_vars', []),
    '$user_type_info'(Fun, fun, (_ -> Type), '$function',  _),
    '$higher_order_type'(Type),
    !,
    '$generate_apply_name'(Fun, ApplyName),
    findall(c(Clause), ( get_clause_(ApplyName(_, _, _), Clause),
                           Clause \= (_ :- !)),
            New),
    '$get_function_head_for_name'(Fun, FHead),
    findall(o(Clause, Vars), get_original_clause_(FHead, Clause, Vars), Orig),
    %% A higher order function - need to modify apply clauses
    atom_concat2(Fun,'_watched',NFun),
    watch_trans_function_clauses_aux_HO_(Orig, New, 1,Fun, NFun),
    assert('$watched'(Fun)).
watch_trans_function_clauses_(Fun,_N) :-
    ip_set('$tel_vars', []),
    '$user_type_info'(Fun, fun, (_ -> _), Kind,  _),
    Kind \= '$function',
    !,
    '$generate_apply_name'(Fun, ApplyName),
    findall(c(Clause), ( get_clause_(ApplyName(_, _, _), Clause),
                           Clause \= (_ :- !)),
            New),
    '$get_function_head_relact_for_name'(Fun, FHead),
    findall(o(ClauseOut, Vars),
            (get_original_clause_(FHead, Clause, Vars),
                '$replace_clause_head_functor'(ApplyName, Clause, ClauseOut)),
            Orig),
    %% A higher order function - need to modify apply clauses
    atom_concat2(ApplyName,'_watched',NFun),
    watch_trans_clauses_aux_(Orig, New, 1,Fun, NFun),
    %functor( RelCall, ApplyName, 3),
    '@functor'(HeadPattern, ApplyName, 3),
    forall(watch_retract_HO_(HeadPattern, Body),
           assert('$watch_saved'(Fun, HeadPattern, Body))),
    once('$watch_saved'(Fun, RelCall, _)),
    assert((RelCall :- watch__ (NFun,RelCall))),
    assert('$watched'(Fun)).     
watch_trans_function_clauses_(Fun,N) :-
    ip_set('$tel_vars', []),
    '@functor'(FHead, Fun, N),
    N1 is N+1,
    %% The function is compiled to a relation so get the relation clauses
    %% and function clauses
    '@functor'(RelHead, Fun, N1),
    findall(c(Clause), get_clause_(RelHead, Clause), New),
    findall(o(Clause, Vars), get_original_clause_(FHead, Clause, Vars), Orig),
    atom_concat2(Fun,'_watched',NFun),
    watch_trans_function_clauses_aux_(Orig, New, 1,Fun,NFun),
    '@functor'(FunCall, Fun, N1),
    '@functor'(HeadPattern, Fun, N1),
    '@=..'(FunCall, [_|Args]),
    '@=..'(WFunCall, [NFun|Args]),
    thaw_term(Vars),

    forall(watch_retract_(HeadPattern, Body),
           assert('$watch_saved'(Fun, HeadPattern, Body))),
    assert((FunCall :- watch__ (fun(NFun),WFunCall))),
    assert('$watched'(Fun)).

%%watch_trans_function_clauses_aux_(A,B,C,D,E) :- errornl(watch_trans_function_clauses_aux_(A,B,C,D,E)),fail.
watch_trans_function_clauses_aux_([], [], _,_,_).
watch_trans_function_clauses_aux_([], [c(H1)], _,_,NRel) :-
    watch_translate_clause_(H1, H1T),
    H1T = (Head :- FBody),
    '@..rev'(Head, _, Args),
    '@..'(NRel, Args, NRelCall),
    assert((
            NRelCall :-
	   writeL_(["  no matching rule - exception raised", nl_]),
            %throw(no_matching_function_rule(Head))
	   FBody
           )).

watch_trans_function_clauses_aux_([o(H1,Vars)|T1], [c(H2)|T2], N,Rel,NRel) :-
    N1 is N+1,
    H2 = (Head :- Body),
    (
      H1 = (FHead :: TEST -> Expr)
    ->
      true
    ;
      H1 = (FHead -> Expr), TEST = true
    ),
    '$translate_body'(TEST, TransTest),
    '$tuple2list'(Body, BodyL),
    append(G1, ['!'|Rest], BodyL),
    (
      G1 = []
    ->
      Test = true
    ;
      '$list2tuple'(G1, Test)
    ),
    '$tuple2list'(TransTest, TransTuple),
    '$list2tuple'(TransTuple, TransTest1),
    unify_tests_(Test, TransTest1),
    '$list2tuple'(Rest, BodyCall),
    '@=..'(Head, [F|Args]),
    '@=..'(NRelCall, [NRel|Args]),
    append(InArgs, [Result], Args),
    '@=..'(FHead, [F|InArgs]),
    add_underscore_to_vars_(Vars, UnderVars),
    assert((NRelCall :-
	   name_the_vars_(UnderVars),
            global_state_lookup(watch__,CL),
            watch_debug_function_write_(FHead, N, CL, H1, Vars),
            ( 
              Test
            ->
              !,
              watch_debug_function_write_succeed_(FHead, CL, Expr),
              BodyCall,
              watch_debug_function_write_result_(FHead, N, Result, CL)
            ;
              watch_debug_function_write_fail_(CL), fail
            )
           )),
    watch_trans_function_clauses_aux_(T1, T2, N1,Rel,NRel).

%%unify_tests_(A,B) :- errornl(unify_tests_(A,B)),fail.
unify_tests_((A, As), (A, Bs)) :-
    !,
    unify_tests_(As, Bs).
unify_tests_(A, A) :- !.
unify_tests_(As, ((X=Y), Bs)) :-
    !,
    X=Y,
    unify_tests_(As, Bs).
unify_tests_(((X=Y), As),  Bs) :-
    !,
    X=Y,
    unify_tests_(As, Bs).

    
/* sentinal neck call to determine if watched */
'$ho_watched_neck'.

%%watch_trans_function_clauses_aux_HO_(A,B,C,D,E) :- errornl(watch_trans_function_clauses_aux_HO_(A,B,C,D,E)),fail.
watch_trans_function_clauses_aux_HO_([], [], _,_,_).
watch_trans_function_clauses_aux_HO_([], [c(H1)], _,Rel,_NRel) :-
    watch_translate_clause_(H1, H1T),
    H1T = (Head :- FBody),
    '$change_to_watch'(Head, FBody, Rel),
    WatchedBody = ('$ho_watched_neck',
                      writeL_(["  no matching rule - exception raised", nl_]),
                      FBody
                  ),
    assert((
            Head :- WatchedBody
           )),
    '@functor'(Head, Apply, _),
    assert('$ho_function_watched_clause'(Apply)).

watch_trans_function_clauses_aux_HO_([o(H1,Vars)|T1], [c(H2)|T2],
                                    N,Rel,NRel) :-
    N1 is N+1,
    H2 = (Head :- Body),
    once((
      H1 = (FHead :: TEST -> Expr)
    ;
      H1 = (FHead -> Expr), TEST = true
    )),
    '$translate_body'(TEST, TransTest),
    '$tuple2list'(Body, BodyL),
    append(G1, ['!'|Rest], BodyL),
    (
      G1 = []
    ->
      Test = true
    ;
      '$list2tuple'(G1, Test)
    ),
    '$tuple2list'(TransTest, TransTuple),
    '$list2tuple'(TransTuple, TransTest1),
    unify_tests_(Test, TransTest1),
    '$list2tuple'(Rest, BodyCall),
    Head = ApplyName(F1, F1Args, Result),
    '@=..'(FHead, [F1|F1Args]),
    add_underscore_to_vars_(Vars, UnderVars),
    '$change_to_watch'(Head, Body, Rel),
    WatchedBody =
    ( '$ho_watched_neck',
        name_the_vars_(UnderVars),
        global_state_increment(watch__,CL),
        watch_debug_function_write_(FHead, N, CL, H1, Vars),
        ( 
          Test
        ->
          !,
          watch_debug_function_write_succeed_(FHead, CL, Expr),
          BodyCall,
          watch_debug_function_write_result_(FHead, N, Result, CL)
        ;
          watch_debug_function_write_fail_(CL),
          global_state_decrement(watch__, _),
          fail
        )),
    assert((Head :- WatchedBody)),
    assert('$ho_function_watched_clause'(ApplyName)),
    watch_trans_function_clauses_aux_HO_(T1, T2, N1,Rel,NRel).

'$change_to_watch'(Head, Body, Fun) :-
    clause(Head, Body),
    retract((Head :- Body)),
    assert('$watch_saved'(Fun, Head, Body)),
    fail.
'$change_to_watch'(_Head, _Body, _Fun).


%% display information about matching against the head of a function rule
watch_debug_function_write_(FHead, N, CL, Clause, _Vars) :-
    writeL_([CL, " : ", FHead, " (matches rule ", N, ") "]),
    %%'$get_ultimate_functor'(FHead, F),
    do_watch_debug_write_function_body_(Clause), !.
    % (
    %  '$watch_clauses'(F)
    % ->
    %  do_watch_debug_write_function_body_(Clause)
    % ;
    %  writeL_([nl_])
    % ), !.

%% display infomation about commiting to a function rule
watch_debug_function_write_succeed_(Head, CL, Expr) :-
    writeL_([sp_(4), CL, " : ", Head, "  -> ", Expr, nl_]).

%% display information about failing the guard of a function rule.
watch_debug_function_write_fail_(CL) :-
    writeL_([CL, " : Guard failed - trying next rule", nl_]).

%% display info about the return value for the function call
watch_debug_function_write_result_(FHead, _N, Result, CL) :-
    %%watch, !,
    writeL_([sp_(4), CL, " : ", FHead, " <- ", Result, nl_]).


%% generate code for watching relations and actions - see example
%% at beginning of file
watch_trans_clauses_(Rel,N) :-
    ip_set('$tel_vars', []),
    '@functor'(Head, Rel, N),
    findall(c(Clause), get_clause_(Head, Clause), New),
    findall(o(Clause, Vars), get_original_clause_(Head, Clause, Vars), Orig),
    atom_concat2(Rel,'_watched',NRel),
    watch_trans_clauses_aux_(Orig, New, 1,Rel,NRel),
    '@functor'( RelCall, Rel, N),
    '@functor'(HeadPattern, Rel, N),
    forall(watch_retract_(HeadPattern, Body),
           assert('$watch_saved'(Rel, HeadPattern, Body))),
    assert((RelCall :- watch__ (NRel,RelCall))),
    assert('$watched'(Rel)). 


watch_trans_clauses_aux_([], [], _,_,_).
%% Actions have a trailing catchall rule that throws an exception
%% if no rule matches - this adds extra output in this case
watch_trans_clauses_aux_([], [c(H1)], _N,_Rel,NRel) :-
    watch_translate_clause_(H1, H1T),
    H1T = (Head :- FBody),
    '@..rev'(Head, _, Args),
    '@..'(NRel, [_|Args], NRelCall),
    assert((
            NRelCall :-
	   writeL_(["  no matching rule - exception raised", nl_]),
	   FBody
           )).
watch_trans_clauses_aux_([o(H1,Vars)|T1], [c(H2)|T2], N,Rel,NRel) :-
    watch_translate_clause_(H1, H1T),
    %% H1T is a Prolog translation of the Qulog rule H1
    (
      H1T = (Head :- Body)
    ->
      '$tuple2list'(Body, BodyL),
      delete_all(true, BodyL, StrippedBodyL),
      '$list2tuple'(StrippedBodyL, FBody),
      (
        FBody = true
      ->
        H1T = Head,
        TH = Head,
        FBody = true
      ;      
        TH = (Head :- FBody)
      )
    ;
      H1T = Head,
      TH = Head,
      FBody = true
    ),
    (
      H2 = (Head :- H2Body)
    ->
      true
    ;
      H2 = Head, H2Body = true
    ),
    (
      unify_tests_(FBody, H2Body)
    ->
      true
    ;
      errornl(something_wrong_with_watch_trans(TH, H2, H1, H1T, H2))  %% Just in case
    ),
    '@..rev'(Head, _, Args),
    length(Args, Len),
    length(VArgs, Len),
    '@..'(NRel, [HeadPairs|VArgs], NRelCall),
    %% NRelCall is the same NRel (e.g. mem_watched) with new var args
    %% and an extra new first arg
    make_unifications_(Args, VArgs, Unifs),
    %% HeadPairs arg will be a list of VarName=Var for all the vars in the call
    %% found by watch__ just before NRel is called.
    add_underscore_to_vars_(Vars, UnderVars),
    %% If Vars is [D = 'X', E = 'T'] then UnderVars = [D = 'X_', E = 'T_']
    %% THis is so name_the_vars will produce X_0, T_0, X_1, T_1 etc
    %% Because watch_debug_write_ needs to output the new names for the
    %% variables in UnderVars after name_the_vars_ has been called we
    %% remap the variables to their new names to get NewUnderVars
    assert(
           (NRelCall :-
               name_the_vars_(UnderVars),
               rebuild_name_map_(UnderVars, NewUnderVars),
               Unifs,
               watch_debug_write_(H1, NewUnderVars, HeadPairs, Rel, N),
               FBody
           )
          ),
    N1 is N+1,
    watch_trans_clauses_aux_(T1, T2, N1,Rel,NRel).
             

display_call_num_(1) :- !,  nl, writeL_([1, ":"]).
display_call_num_(WL) :- writeL_([WL, ":"]).

%%watch_translate_clause_(A,B) :- errornl(watch_translate_clause_(A,B)),fail.
watch_translate_clause_((H :: Test <= Body), (H :- TransBody)) :- !,
    '$translate_body'(((Test & !) & Body), TransBody).
watch_translate_clause_((H  <= Body), (H :- TransBody)) :- !,
    '$translate_body'(Body, TransBody).
watch_translate_clause_((H :: Test ~> Body), (H :- TransBody)) :- !,
    '$translate_body'(((Test & !) & Body), TransBody).
watch_translate_clause_((H  ~> Body), (H :- TransBody)) :- !,
    '$translate_body'((! & Body), TransBody).
watch_translate_clause_((H :: Test), (H :- TransBody)) :- !,
    '$translate_body'((Test & !), TransBody).
watch_translate_clause_(H, H).

watch_retract_(Head, Body) :-
    clause(Head, Body),
    (
      Body = true
    ->
      retract(Head)
    ;
      retract((Head :- Body))
    ).
watch_retract_HO_(Head, Body) :-
    clause(Head, Body),
    (
      Body = '!'
    ->
      fail
    ;
      Body = true
    ->
      retract(Head)
    ;
      retract((Head :- Body))
    ).

%% EG add_underscore_to_vars_([A = 'X', B = 'Y'], U)
%% U = [A = 'X_', B = 'Y_']
add_underscore_to_vars_([], []).
add_underscore_to_vars_([(V=N)|Vars], [(V= NU)|UnderVars]) :-
    atom_concat(N, '_', NU),
    add_underscore_to_vars_(Vars, UnderVars).
    
get_clause_(Head, Clause) :-
    clause(Head, Body),
    (
      Body = true
    ->
      Clause = Head
    ;
      Clause = (Head :- Body)
    ).

get_original_clause_(Head, Clause, Vars) :-
    '$saved_input_with_vars'(Clause, Vars, _),
    clause_head_(Clause, Head).




'$replace_clause_head_functor'(ApplyName, (Head :: Test <=  Body),
                               (RHead :: Test <= Body)) :-
    !,
    '@..rev'(Head, F, Args),
    %'$list2tuple'(Args, ArgTuple),
    RHead = ApplyName(F, Args, _).
'$replace_clause_head_functor'(ApplyName, (Head <=  Body),
                               (RHead  <= Body)) :-
    !,
    '@..rev'(Head, F, Args),
    %'$list2tuple'(Args, ArgTuple),
    RHead = ApplyName(F, Args, _).
'$replace_clause_head_functor'(ApplyName, (Head),
                               (RHead)) :-
    !,
    '@..rev'(Head, F, Args),
    %'$list2tuple'(Args, ArgTuple),
    RHead = ApplyName(F, Args, _).
    


clause_head_((H :: _ -> _), H) :- !.
clause_head_((H -> _), H) :- !.
clause_head_((H :: _ <= _), H) :- !.
clause_head_((H <= _), H) :- !.
clause_head_((H :: _ ~> _), H) :- !.
clause_head_((H ~> _), H) :- !.
clause_head_((H :: _), H) :- !.
clause_head_(H, H) :- !.


%% Build a Var=Name list for a list of variables
construct_([],[]):-!.
construct_([V|Vs],[V=NameV|NamePairs]) :-
    get_var_name(V,NameV), !,
    construct_(Vs,NamePairs).
construct_([V|Vs],[(V='_')|NamePairs]) :-
    construct_(Vs,NamePairs).

make_unifications_([], [], true).
make_unifications_([A|Args], [V|VArgs], ((A=V),Unifs)) :-   
    make_unifications_(Args, VArgs, Unifs).


rebuild_name_map_([], []).
rebuild_name_map_([(V=_)|HeadPairs], [(V=New)|NewHeadPairs]) :-
    get_var_name(V, New), !,
    rebuild_name_map_(HeadPairs, NewHeadPairs).
rebuild_name_map_([(V=_)|HeadPairs], [(V='_')|NewHeadPairs]) :-
    rebuild_name_map_(HeadPairs, NewHeadPairs).


