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

%% for storing the seed for the random number generator
?- dynamic('$random_seed'/1).
%% for storing time at which belief store updates occur (a timestamp)
?- dynamic('__time__'/1, 0).
?- dynamic('$user_type_descr'/2).

%% The time at which this process starts
?- gettimeofday(T), assert('$start_time'(T)).
%% In case the user wants to store a DB of messages received with a timestamp
?- assert('$belief'(received_message(_, _, _))).
?- dynamic(received_message/3).

%% For storing the name of a task and the top-level TR procedure it is running
?- assert('$belief'(task(_, _))).
?- dynamic(task/2).
%% For the user to get resources currently used by a running task
?- dynamic(resources/2).
%% Users can declare default args for rel/act - this is used to add defaults
%% if required
?- dynamic('$user_default_args'/2).

        
%% qmain has a builtin declaration but is user defined
'$dynamic_builtin'(qmain).

%% Used by show so that code with bultin declarations but user definitions
%% can be listed
'$user_defined'(handle_message).
'$user_defined'(handle_template_message).
'$user_defined'(init_message_handler).
'$user_defined'(init_percept_handler).
'$user_defined'(post_process_percepts).

%% for parsing subscribe strings for use in pedro_subscribe
?- op_table_inherit(subscribe_reader, user).
?- op(subscribe_reader, 1150, xfx, '::').
%% for default args
?- op(400, xfx, 'default').


'$supress_show_type'(resources_hook, 0).
'$supress_show_type'(init_message_handler, 0).
'$supress_show_type'(init_percept_handler, 0).
'$supress_show_type'(received_message, 3).
'$supress_show_type'('$dynamic', 0).
'$supress_show_type'('$null_action', 0).
'$supress_show_type'('$true', 0).

'$supress_builtin_type'('$dynamic').
'$supress_builtin_type'('$list_constr').
'$supress_builtin_type'('$set').
'$supress_builtin_type'('$time_').
'$supress_builtin_type'('$set_enum').
'$supress_builtin_type'('$true').
'$supress_builtin_type'(type_info).
'$supress_defined_type'(type_kind_).
'$supress_defined_type'('$dynamic').

'$should_suppress_show_type'(G) :-
    '$supress_show_type'(G, N),
    \+current_predicate(G/N).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Interface between Qulog and QuProlog
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'$try_name_vars'([]).
'$try_name_vars'([var(V, N)|Rest]) :-
    V = '$$var$$'(N), !,
    '$try_name_vars'(Rest).
'$try_name_vars'([_|Rest]) :-
    '$try_name_vars'(Rest).

'$qp_set_tuple_to_qlg'(V, V) :- var(V), !.
'$qp_set_tuple_to_qlg'((A,B), Trans) :-
    !,
    '$tuple2list'((A,B), Lst),
    map('$qp_set_tuple_to_qlg', Lst, TLst),
    Trans = '$tuple'(TLst).
'$qp_set_tuple_to_qlg'({}(Tuple), Trans) :-
    !,
    '$tuple2list'(Tuple, Lst),
    map('$qp_set_tuple_to_qlg', Lst, TLst),
    Trans = '$set_enum'(TLst).
'$qp_set_tuple_to_qlg'(Comp, Trans) :-
    compound(Comp), !,
    Comp =.. Lst,
    map('$qp_set_tuple_to_qlg', Lst, TLst),
    Trans =.. TLst.
'$qp_set_tuple_to_qlg'(X, X).

'$qlg_set_tuple_to_qp'(V, V) :- var(V), !.
'$qlg_set_tuple_to_qp'('$set'(L), {}(U)) :- !,
    map('$qlg_set_tuple_to_qp', L, L1),
    '$list2tuple'(L1, U).
'$qlg_set_tuple_to_qp'('$tuple'(L), U) :- !,
    map('$qlg_set_tuple_to_qp', L, L1),
    '$list2tuple'(L1, U).
'$qlg_set_tuple_to_qp'(Comp, Trans) :-
    compound(Comp), !,
    Comp =.. Lst,
    map('$qlg_set_tuple_to_qp', Lst, TLst),
    Trans =.. TLst.
'$qlg_set_tuple_to_qp'(X, X).

read_op_term(T, Stream) :-
    stream_readable_(Stream, read_term(T, Stream)),
    '$stream2qpstream'(Stream, SNum),
    read_qulog_string__(InString, SNum),
    string_concat(InString, ".", DotString),
    open_string(read(DotString), S),
    catch(readR(S, Term1), _, throw(read_op_term_syntax_error(DotString))),
    '$qp_set_tuple_to_qlg'(Term1, Term2),
    '$check_and_eval_input_term'(Term2, T, string(DotString), read_op_term).

'$check_and_eval_input_term'(InTerm, _OutTerm, Kind, Call) :-
    \+has_type2__(InTerm, term, []), !,
    '$input_kind_to_string'(Kind, InString),
    throw(input_term_type_error(Call, InString)).
'$check_and_eval_input_term'(InTerm, OutTerm, Kind, Call) :-
    ip_set('$had_eval', _),
    '$translate_goal'((InTerm = MidTerm), TransGoal),
    ip_lookup('$had_eval', DoesEval),
    (
      var(DoesEval)
    ->
      call(TransGoal),
      '$action_one_sided_unif'(OutTerm, MidTerm, [], Call)
    ;
      '$input_kind_to_string'(Kind, InString),
      throw(input_term_error(Call, InString))
    ).
          
'$input_kind_to_string'(string(S), S) :- !.
'$input_kind_to_string'(term(T, VarList), S) :-
    '$try_name_vars'(VarList),
    term2string(T, S).
                             
% '$read_op_term_aux'(T1, T) :-
%     \+has_type2__(T1, term, []), !,
%     term2string(T1, TString),
%     throw(input_term_type_error(read_op_term(T), TString)).
% '$read_op_term_aux'(T1, T) :-
%     '$translate_goal'((T1 = T2), Call), Call \= (T1 = T2), !,
%     term2string(T1, TString),
%     throw(input_term_error(read_op_term(T), TString)).    
% '$read_op_term_aux'(T1, T) :-
%     '$action_one_sided_unif'(T, T1, [], read_op_term(T)).

%% Interface to read_term(Term, Stream)
'$read_term_'(T, Stream) :-
    %% test and throw an exception if Stream is not readable
    stream_readable_(Stream, read_term(T, Stream)),
    '$stream2qpstream'(Stream, SNum),
    %% read term - fail if not parsed
    '$read_qulog_term'(T1, SNum, VarList), !,
    '$check_and_eval_input_term'(T1, T, term(T1, VarList), read_term).
'$read_term_'(_T, _) :-
    throw(read_term_syntax_error('$none_')).

%% read term is not of type term
% '$readT_aux'(T1, _T, VarList, Call) :-
%     \+has_type2__(T1, term, []), !,
%     '$try_name_vars'(VarList),
%     term2string(T1, TString),
%     throw(input_term_type_error(Call, TString)).
% '$readT_aux'(T1, _T, VarList, Call) :-
%     '$translate_goal'((T1 = T2), UCall), UCall \= (T1 = T2), !,
%     '$try_name_vars'(VarList),
%     term2string(T1, TString),
%     throw(input_term_error(Call, TString)).
% '$readT_aux'(T1, T, VarList, Call) :-
%     '$action_one_sided_unif'(T, T1, VarList, Call).

%% one sided unification with action faiure
'$action_one_sided_unif'(T1, T2, _, _Action) :-
    '=@'(T1, T2), !.
'$action_one_sided_unif'(_T1, _T2, VarList, Action) :-
    '$try_name_vars'(VarList),
    term2string(Action, ActionString),
    throw(action_failure(ActionString)).
    
                     
%% interface for raise(Exception)
raise(Exception) :- throw(Exception).

with_occurs_check_on(Goal) :-
    '$occurs_check_on'(Goal).

%% interface for the function #T  (implemented as the relation '#'(T, N)
%% where N is the result
'#'(T, N) :- string(T), !, string_length(T, N).
'#'('$set'(T), N) :- !, length(T, N).
'#'(T, N) :- closed_list(T), length(T, N).

%% interface for A in B
in(A, B) :-
    string(B), !,
    sub_string(B, _Start, Len, _, A), Len = 1.
in(A, '$set'(B)) :- !, member(A, B).
%%in(A, '$tuple'(B)) :- !, member(A, B).
in(A, B) :- closed_list(B), member(A, B).

%% pedro related interface
disconnect_from_pedro('$none_') :-
    pedro_is_connected, !,
    pedro_disconnect.
%% do nothing if already disconnectd
disconnect_from_pedro('$none_').

%% do nothing if already connected
connect_to_pedro(_Host, _Port) :-
    pedro_is_connected, !.
connect_to_pedro(Host, Port) :-
    pedro_connect(Host, Port), !.
connect_to_pedro(Host, Port) :-
    term2string(connect_to_pedro(Host, Port), AString),
    throw(action_failure_(AString)).

register_with_pedro(_Name) :-
    \+ pedro_is_connected, !,
    throw(pedro_not_connected(register_with_pedro)).
register_with_pedro(Name) :-
    pedro_is_registered, !,
    pedro_deregister,
    pedro_register(Name).
register_with_pedro(Name) :-
    pedro_register(Name), !.
register_with_pedro(Name) :-
    throw(pedro_name_already_registered(Name)).

deregister_from_pedro('$none_') :-
     \+ pedro_is_connected, !,
    throw(pedro_not_connected(deregister_with_pedro)).
deregister_from_pedro('$none_') :-
    pedro_is_registered, !,
    pedro_deregister.
deregister_from_pedro('$none_').

%% the function tolist(T)
tolist(T, R) :-
    string(T), !,
    findall(X, in(X, T), R).
%%tolist(T, R) :-
%%    list(T), !, R = T.
%%tolist(T, R) :-
%%    T = '$tuple'(R), !.
tolist('$set'(R), R).

%%toset(T, T) :- T = '$set'(_), !.
toset(T, '$set'(R)) :- list(T), !, sort(T, R).
toset(T, '$set'(R)) :- tolist(T, L), sort(L, R).


'$map_'(_F, [], []).
'$map_'(F, [H|T], [FH|FT]) :-
     apply_(F, [H], FH),
     '$map_'(F, T, FT).

'$filter_'(_Test, [], []).
'$filter_'(Test, [H|T], [H|FT]) :-
     Test(H), !,
     '$filter_'(Test, T, FT).
'$filter_'(Test, [_|T], FT) :-
     '$filter_'(Test, T, FT).

'$filter_map_'(_Test, _F, [], []).
'$filter_map_'(Test, F, [H|T], [FH|FT]) :-
    Test(H), !,
    apply_(F, [H], FH),
     '$filter_map_'(Test, F, T, FT).
'$filter_map_'(Test, F, [_|T], FT) :-
     '$filter_map_'(Test, F, T, FT).


isa(Term, Type) :-
    '$defined_type'(Type, DType),
    '$isa_aux'(DType, Term, Type).

'$isa_aux'('$union_type'(UType), Term, _) :-
    member(T, UType),
    isa(Term, T).
'$isa_aux'('$enum_type'(TypeE), Term, _) :-
    member(Term, TypeE).
'$isa_aux'((N..M), Term, _) :-
    var(Term), !, between(N, M,  Term).
'$isa_aux'((N..M), Term, _) :-
    integer(Term), between(N, M,  Term).
'$isa_aux'('$constr_enum_type'(UType), Term, Type) :-
    '$non_param_recursive'(Type, UType, []), !,
    member(UT, UType),
    functor(UT, F, N),
    length(Args, N),
    T =..[F|Args],
    UT =.. [_|UTArgs],
    map(isa, Args, UTArgs),
    Term = T.

'$non_param_recursive'(Type, _UType, _Seen) :-
    compound(Type), !, fail.
'$non_param_recursive'(_Type, UType, Seen) :-
    forall(member(UT, UType), '$finite_type'(UT, Seen)).

%%'$finite_type'(Type, Seen) :- errornl('$finite_type'(Type, Seen)), fail.
'$finite_type'(Type, _) :-
    '$defined_type'(Type, '$union_type'(_)), !.
'$finite_type'(Type, _) :-
    '$defined_type'(Type, '$enum_type'(_)), !.
'$finite_type'(Type, _) :-
    '$defined_type'(Type, (_N.._M)), !.
'$finite_type'(Type, Seen) :-
    '$defined_type'(Type, '$constr_enum_type'(CT)), !,
    '$non_param_recursive'(Type, CT, Seen).
'$finite_type'(Type, Seen) :-
    compound(Type), \+member(Type, Seen), !,
    '@..rev'(Type, _, Args),
    forall(member(A, Args), '$finite_type'(A, [Type|Seen])).


pi(A) :- A is pi.
e(A) :- A is e.
now(T) :- gettimeofday(T).
start_time(ST) :- '$start_time'(ST).
exec_time(ET) :- now(T), start_time(ST), ET is T - ST.

random_num(T) :- '$random_seed'(_), !, random(T).
random_num(T) :- srandom(S), assert('$random_seed'(S)), random(T).

random_int(N, M, T) :- '$random_seed'(_), !, random(N, M, T).
random_int(N, M, T) :- srandom(S), assert('$random_seed'(S)), random(N, M, T).

random_seed(S) :-  srandom(S), assert('$random_seed'(S)).

%% sum function compiles to '$sum' relation
'$sum'([], B, B).
'$sum'([X|Xs], A, B) :-
    A1 is A + X,
    '$sum'(Xs, A1, B).
%% ditto prod
'$prod'([], B, B).
'$prod'([X|Xs], A, B) :-
    A1 is A * X,
    '$prod'(Xs, A1, B).

sum(A, B) :- '$sum'(A, 0, B).
prod(A, B) :- '$prod'(A, 1, B).

+(A, B, C) :- C is A+B.
-(A, B, C) :- C is A-B.
*(A, B, C) :- C is A*B.
/(A, B, C) :- B \= 0, !, C is A/B.
/(A, B, _C) :- throw(arithmetic_function_domain_error(A/B)).
>>(A,B,C) :- C is A >> B.
<<(A,B,C) :- C is A << B.
//(A,B,C) :- B \= 0, !, C is A // B.
//(A,B,_C) :- throw(arithmetic_function_domain_error(A//B)).
mod(A,B,C) :-  B \= 0, !, C is A mod B.
mod(A,B,_C) :-  throw(arithmetic_function_domain_error(mod(A, B))).
'\\/'(A,B,C) :-  C is A \/ B.
'/\\'(A,B,C) :-  C is A /\ B.
**(A, B, _C) :- A < 0, \+integer(B), !, throw(arithmetic_function_domain_error(A**B)).
**(A, B, C) :-  C is A**B.

-(A,B) :-  B is -A.
'\\'(A,B) :-  B is \A.
sqrt(A,B) :- A >= 0, !, B is sqrt(A).
sqrt(A,_B) :- throw(arithmetic_function_domain_error(sqrt(A))).
sin(A,B) :-  B is sin(A).
cos(A,B) :-  B is cos(A).
tan(A,B) :-  B is tan(A).
asin(A,B) :-  A >= -1, A =< 1, !, B is asin(A).
asin(A,_B) :-  throw(arithmetic_function_domain_error(asin(A))).
acos(A,B) :-  A >= -1, A =< 1, !,B is acos(A).
acos(A,_B) :-  throw(arithmetic_function_domain_error(acos(A))).
atan(A,B) :-  B is atan(A).
round(A,B) :-  B is round(A).
floor(A,B) :-  B is floor(A).
ceiling(A,B) :-  B is ceiling(A).
abs(A,B) :-  B is abs(A).
log(A,B) :-  A> 0, !, B is log(A).
log(A,_B) :-  throw(arithmetic_function_domain_error(log(A))).
atan2(Y,X,C) :-  '$atan2'(Y,X,C).

max(A, B, C) :- A > B, !, C = A.
max(_, B, B).
min(A, B, C) :- A < B, !, C = A.
min(_, B, B).


'$atan2'(Y,0,C) :-
    Y > 0, !,
    C is pi/2.
'$atan2'(Y,0.0,C) :-
    Y > 0, !,
    C is pi/2.
'$atan2'(Y,0,C) :-
    Y < 0, !,
    C is -pi/2.
'$atan2'(Y,0.0,C) :-
    Y > 0, !,
    C is pi/2.
'$atan2'(Y,X,C) :- 
    X > 0, !,
    C is atan(Y/X).
'$atan2'(Y,X,C) :- 
    Y >= 0, !,
    C is pi + atan(Y/X).
'$atan2'(Y,X,C) :- 
    C is atan(Y/X) - pi.

'=@'(A, B) :-
    freeze_term(B, Vars),
    A = B,
    thaw_term(Vars).


false('$none_') :- fail.
true('$none_').
fail('$none_') :- fail.
'$true'('$none_').

range(X, Start, End, Step) :-
    Step < 0, !,
    NEnd is -Start,
    NStart is -End,
    NStep is -Step,
    range(NX, NEnd, NStart, NStep),
    X is -NX.
range(X, Start, End, Step) :-
    Step = 1, !,
    End1 is End-1,
    between(Start, End1, X).
range(X, Start, End, Step) :-
    Step > 1,
    NEnd is (End - Start - 1)//Step,
    between(0, NEnd, X1),
    X is Start + X1*Step.

%% listof(Result, Pattern :: Goal)
%% By using a thread in which to do the computation and messages with
%% remember_names then Result can contain variables from Goal and Pattern
%% but other variables that are instantiated during execution of Goal
%% in the sub-thread will not be instantiated in the caller
'$pfindall'(Result, Pattern, Goal) :-
    thread_symbol(Me),
    collect_vars(Pattern, PatternVars),
    collect_vars(Goal, GoalVars),
    intersect_list(PatternVars,  GoalVars, BothVars),
    diff_list(PatternVars, BothVars, PatternOnly),
    diff_list(GoalVars, BothVars, GoalOnly),
    append(GoalOnly, PatternOnly, Vars),
    append(Vars, BothVars, AllVars),
    thread_fork(Th, listof_eval__(Me)),
    ipc_send(listof_eval__(AllVars, Pattern, Goal), Th, [remember_names(true)]),
    ipc_recv(M, Th, [remember_names(true)]),
    listof_aux__(M, Result, Th).

%%listof_aux__(A,B,C) :- errornl(listof_aux__(A,B,C)),fail.
listof_aux__(no_more_solutions, [], _Th).
listof_aux__(ans(M), [M|Result], Th) :-
    ipc_recv(M1, Th, [remember_names(true)]),
    listof_aux__(M1, Result, Th).

listof_eval__(Parent) :-
    ipc_recv(listof_eval__(_, Pattern, Goal), Parent, [remember_names(true)]),
    call(Goal),
    ipc_send(ans(Pattern), Parent, [remember_names(true)]),
    fail.
listof_eval__(Parent) :-
    no_more_solutions ->>Parent.




union(A, B, C) :-
    A = '$set'(A1),
    B = '$set'(B1),
    union_list(A1, B1, D1),
    sort(D1, C1), C = '$set'(C1).

inter(A, B, C) :-
    A = '$set'(A1),
    B = '$set'(B1),
    findall(X, (member(X, A1), member(X, B1)), D1),
    sort(D1, C1), C = '$set'(C1).

diff(A, B, C) :-
    A = '$set'(A1),
    B = '$set'(B1),
    findall(X, (member(X, A1), \+member(X, B1)), D1),
    sort(D1, C1), C = '$set'(C1).

%template(X) :- atomic(X), !.
template(X) :-
    compound(X),
    functor(X, F, _),
    atom(F).

string2term(String, Term) :-
    '$read_qulog_term_from_string'(String, Term1, VarList), !,
    '$name_all_vars'(VarList),
    '$check_and_eval_input_term'(Term1, Term, term(Term1, VarList), string2term).
string2term(String, _Term) :-
    throw(string2term_syntax_error(String)).

% '$string2term_aux'( Term1, _Term, _VarList, String) :-
%     \+has_type2__(Term1, term, []), !,
%     throw(string2term_type_error(String)).
% '$string2term_aux'( Term1, Term, VarList, String) :-
%     '$translate_goal'((Term1 = T2), Call), Call \= (Term1 = T2), !,
%     '$try_name_vars'(VarList),
%     throw(input_term_error(string2term(String, Term), String)).    
% '$string2term_aux'(Term1, Term, VarList, _) :-
%     '$action_one_sided_unif'(Term, Term1, VarList, string2term).

%% open(File, RW, Stream)
%% Streams are either stdin, stdout, stderr or stream_(N) where N is a stream
%% number - e.g. stream__(0) is the same stream as stdin
'$open_'(File, RW, Stream) :-
    open(File, RW, Snum),
    '=@'(Stream, stream_(Snum)), !.
'$open_'(File, RW, Stream) :-
    term2string(open(File, RW, Stream), AString),
    throw(action_failure(AString)).

%% support for testing streams for reading/writing
stream_open_(stream_(S), _) :-
    '$get_stream_properties'(S, _), !.
stream_open_(stream_(_S), Call) :-
    !,
    term2string(Call, SCall),
    throw(stream_not_open(SCall)).
stream_open_(_, _).

stream_writeable_(stream_(S), Call) :-
    stream_open_(stream_(S), Call),
    '$get_stream_properties'(S, '$prop'(_, output, _, _, _, _, _)),
    !.
stream_writeable_(stream_(_S), Call) :-
    term2string(Call, SCall),
    throw(stream_not_writeable(SCall)), !.
stream_writeable_(S, _Call) :-
    '$stream2qpstream'(S, SNo),
    '$get_stream_properties'(SNo, '$prop'(_, output, _, _, _, _, _)),
    !.
stream_writeable_(_S, Call) :-
    term2string(Call, SCall),
    throw(stream_not_writeable(SCall)), !.

stream_readable_(stream_(S), Call) :-
    stream_open_(stream_(S), Call),
    '$get_stream_properties'(S, '$prop'(_, input, _, _, _, _, _)),
    !.
stream_readable_(stream_(_S), Call) :-
    term2string(Call, SCall),
    throw(stream_not_readable(SCall)), !.
stream_readable_(S, _Call) :-
    '$stream2qpstream'(S, SNo),
    '$get_stream_properties'(SNo, '$prop'(_, input, _, _, _, _, _)),
    !.
stream_readable_(_S, Call) :-
    term2string(Call, SCall),
    throw(stream_not_readable(SCall)), !.

%% close(Stream)
'$close_'(Stream) :-
    stream_open_(Stream, close(Stream)),
    '$stream2qpstream'(Stream, S),
    close(S).

'$stream2qpstream'(stream_(N), N) :- !.
'$stream2qpstream'(stdin, 0).
'$stream2qpstream'(stdout, 1).
'$stream2qpstream'(stderr, 2).

'$get_line_'(String, Stream) :-
    stream_readable_(Stream, get_line(String, Stream)),
    '$stream2qpstream'(Stream, SNum),
    get_line(SNum, OutString),
    '=@'(String, OutString), !.
'$get_line_'(String, Stream) :-
    term2string(get_line(String, Stream), AString),
    throw(action_failure(AString)).
    
'$put_line_'(String, Stream) :-
    stream_writeable_(Stream, put_line(String, Stream)),
    '$stream2qpstream'(Stream, SNum),
    put_line(SNum, String).

term2string(Term, String) :-
    open_string(write, S),
    writeTerm_(S, Term),
    stream_to_string(S, String).

str(A, B) :- term2string(A,B).


%% for checking call in fork
% '$check_exec'(Code, Kind) :-
%     freeze_term(Code, TVars),
%     name_vars(Code),
%     collect_vars(Code, CodeVars),
%     map('$add_var_names', CodeVars, Vars),
%     open_string(write, Stream),
%     set_std_stream(2, Stream),
%     once(
%          (
%            '$check_goal_expressions'(Code, Code, Vars, Kind),
%            check_query_relation_mode_type__(Code, Vars, _OutVMT),fail
%          ;
%            true
%          )),
%     reset_std_stream(2),
%     stream_to_string(Stream, S2),
%     (
%       S2 = ""
%     ->
%       thaw_term(TVars)
%     ;
%       throw(invalid_fork_arg(S2))
%     ), !.

% '$add_var_names'(V, Map) :-
%     get_var_name(V, Name),
%     Map = (V=Name).


do(Act) :- '$call__'(Act).

'$call__'(C) :-
    '$translate_goal'(C, FullC),
    call(FullC).

fork_light(Call, Name) :-
    Sizes = [choice_size(1),
             env_size(1),
             heap_size(5),
             binding_trail_size(1),
             other_trail_size(1),
             scratchpad_size(1),
             name_table_size(100),
             ip_table_size(100)],
    fork(Call, Name, Sizes).

fork(_, Name, _) :-
    atom(Name), thread_is_thread(Name), !,
    throw(fork_thread_name_exists(Name)).
fork(Call, Name, Sizes) :-
    !,
    add_default_args__(Call, FullCall),
    thread_fork(Name, FullCall, Sizes).

is_thread(Name) :- thread_is_thread(Name).

% fork(Call, Name, Sizes) :-
%     '$default_args'(Call, FullCall), 
%     catch((thread_fork(Root, FullCall, thread, Sizes), Name = Root),
%           _, fork_root_extend__(FullCall, Name, Sizes, Root)), !.
% fork(Call, Name, Sizes) :- 
%     '$default_args'(Call, FullCall),
%     fork_root_extend__(FullCall, Name, Sizes, Root).

% fork_root_extend__(Call, Name, Sizes, Root) :- 
%     thread_fork(Name, Call, Root, Sizes).

'$write_vars'(_S, []).
'$write_vars'(S, [V]) :- !, writeL_(S, [V]).
'$write_vars'(S, [V|Rest]) :- writeL_(S, [V, ", "]), '$write_vars'(S, Rest).



'$remote_query'(Vars, Query, Addr) :-
    open_string(write, S),
    (
      Query = '$num_vars_query'(N, Vs, Q)
    ->
      writeL_(S, [ N, " of "]),
      '$write_vars'(S, Vs),
      writeL_(S, [" :: ", Q])
    ; 
      Query = '$num_query'(N, Q)
    ->
      collect_vars(Query, Vs),
      writeL_(S, [ N, " of ", Q])
    ;
      Query = '$vars_query'(Vs, Q)
    ->
      '$write_vars'(S, Vs),
      writeL_(S, [" :: ", Q])
    ; 
      collect_vars(Query, Vs),
      writeL_(S, [Query])
    ),
    stream_to_string(S, QueryString),
    %% convert query to string and send it to Addr
    to(remote_query(QueryString), Addr),
    %% get first answer
    remote_query_recv_match_(M, Addr),
    %% get the rest
    '$get_remote_answers'(M, AllAns, Addr),
    !,
    %% set up choicepoint to access answers one at a time
    member(Ans, AllAns),
    (
      is_remote_exception_(Ans)
    ->
      raise(Ans)
    ;
      %string2term(AnsS, Ans),
      %% all the answer vars have to be declared vars
      %% all the declared vars have answers and the answer has the right type
      Vs = Ans,
      '$check_ans_vars2'(Vars)
      %bind_ans__(Ans, Vs, Vars)
    ).

is_remote_exception_(remote_query_exception(_)) :- !.
is_remote_exception_(remote_query_ontology_mismatch(_)).

%%'$get_remote_answers'(A,B,C) :- errornl('$get_remote_answers'(A,B,C)),fail.
'$get_remote_answers'(ans(no_more_solutions), [], _AgAddr) :- !.
'$get_remote_answers'(ans(AnsS), AllAns, AgAddr) :-
    string(AnsS),
    !,
    catch(
          (
            string2term(AnsS, Ans),
            remote_query_recv_match_(M, AgAddr),
            '$get_remote_answers'(M, Answers, AgAddr),
            AllAns = [Ans|Answers]
          ),
          remote_query_ontology_mismatch(E),
          AllAns = [remote_query_ontology_mismatch(E)]).
'$get_remote_answers'(ans(remote_query_ontology_mismatch(A)),
                      [remote_query_ontology_mismatch(A)], _AgAddr) :-
    !.
'$get_remote_answers'(ans(remote_query_exception(A)), [remote_query_exception(A)], _AgAddr) :-
    !.

'$get_remote_answers'(_, Answers, AgAddr) :-
    remote_query_recv_match_(M, AgAddr),
    '$get_remote_answers'(M, Answers, AgAddr).

%% check that all declared vars have a binding and the binding is the
%% correct type
%%'$check_ans_vars2'(Vars) :- errornl('$check_ans_vars2'(Vars)),fail.
'$check_ans_vars2'([]).
'$check_ans_vars2'([(Term:MT)|Vars]) :-

    (
      type(Term, MT)
    ->
      true
    ;
      open_string(write, Stream),
      writeL_(Stream, ["Remote Query Error: ", Term, " is not of moded type ", MT, nl_]),
      stream_to_string(Stream, S),
      raise(remote_query_ontology_mismatch(S))
    ),
    '$check_ans_vars2'(Vars).


% '$check_ans_vars2'(Ans, Vars) :-
%     member((Y:MT), Vars),
%     once((member(remote_var_binding_(X, Term), Ans),
%           X == Y)),
%     (
%       type(Term, MT)
%     ->
%       fail
%     ;
%       open_string(write, Stream),
%       writeL_(Stream, ["Remote Query Error: ", Term, " is not of moded type ", MT, nl_]),
%       stream_to_string(Stream, S),
%       raise(remote_query_ontology_mismatch(S))
%     ).
% '$check_ans_vars2'(_, _).


'$get_needed_vars'(Vars, (V = _)) :-
    member_eq(V, Vars).

%% For the agent doing the remote query - compute the answers and return
%% to client
respond_remote_query(QueryString, Client) :-
    thread_symbol(ThreadName),
    concat_atom(['$total_solns_',ThreadName], ThreadNumSolns),
    open_string(write, Stream),
    set_std_stream(2, Stream),
    (
      trans_query__(QueryString, FullQuery, Vars),
      ip_set('$global_vars', []),
      ip_set('$local_vars', []),
      global_state_set(ThreadNumSolns, 20),
      (
        FullQuery = '$num_query'(N, Query)
      ->
        global_state_set(ThreadNumSolns, N),
        collect_vars(FullQuery, FVars)
      ;
        FullQuery = '$vars_query'(FVars, Query)
      ->
        true
      ; 
        FullQuery = '$num_vars_query'(N, FVars, Query)
      ->
        global_state_set(ThreadNumSolns, N)
      ;
        collect_vars(FullQuery, FVars),
        Query = FullQuery
      ),
      '$get_call_kind'(Query, RealQuery, Vars, _Kind),
      %%ip_lookup('$local_vars', Locals),
      %%filter('$ignore'(Locals), Vars, QVars),
      ip_set('$local_vars', []),
      ip_set('$global_vars', [])
    ;
      true
    ), !,
    reset_std_stream(2),
    stream_to_string(Stream, S2),
    (
      S2 = ""
    ->
      call_query_send_answer__(RealQuery, FVars, Client, ThreadNumSolns)
    ;
      %% strip off "exception(" and the closing ")"
      %%Len is After - 1,
      %%sub_string(S2, 10, Len, _, S3),
      to_thread(ans(remote_query_ontology_mismatch(S2)), Client)
    ).

% '$assert_has_allowed_remote_queries'(Addr) :-
%     current_predicate(allowed_remote_query_from/2), !,
%     assert('$has_allowed_remote_queries'(Addr)).
% '$assert_has_allowed_remote_queries'(_Addr).          
   
call_query_send_answer__(Q, Vars, Client, ThreadNumSolns) :-
    catch(call_query_send_answer_aux__(Q, Vars, Client, ThreadNumSolns), E,
         process_exception__(E, Client)).

%process_exception__(A,B) :- errornl(process_exception__(A,B)),fail.
process_exception__(remote_query_in_remote_query, Client) :-
    !,
    to_thread(ans(no_more_solutions), Client).
process_exception__(E, Client) :-
    to_thread(ans(remote_query_exception(E)), Client).

call_query_send_answer_aux__(Q, Vars, Client, ThreadNumSolns) :-
    get_answer__(Q, Vars, VarsString, ThreadNumSolns),
    to_thread(ans(VarsString), Client),
    fail.
call_query_send_answer_aux__(_Q, _Vars, Client, _) :-
    to_thread(ans(no_more_solutions), Client).


get_answer__(Q, Vars, VarsString, ThreadNumSolns) :-
    call(Q),
    ( Vars = [] -> ! ; true),
    gen_ans_string__(Vars, VarsString),
    global_state_decrement(ThreadNumSolns, N),
    (
      N = 0
    ->
      !
    ;
      true
    ).

gen_ans_string__(Vars, VarsString) :-
    open_string(write, Stream),
    write_term(Stream, Vars, [quoted(true)]),
    stream_to_string(Stream, VarsString).

%% parse the query string as a qulog query
trans_query__(QString, Query, VarNamesOut) :-
    retractall('$parser_info'(_, _, _, _)),
    assert('$parser_info'(QString, _StartLine, _EndLine, stdin)),
    string2tokens__(QString, Tokens, VarList, TokenPositions),
    ip_set('$token_positions', t(Tokens, TokenPositions)),
    Tokens \= [],
    '$collect_var_tables'(VarList, _Variables, VarNames, _Singles, []),
    '$unify_on_names'(VarList),
    remove_duplicates(VarNames, VarNamesOut),
    catch(
          (
            phrase(query__(Query), Tokens, [])
          ->
            true
          ;
            report_error__('Can\'t parse', Tokens),
            fail
          ),
          syntax_error,
          fail
         ).



bind_ans__([], _).
bind_ans__([rvb_(X, Term)|Rest], Vars) :-
    member((V:_), Vars), X == V, !,
    X = Term,
    bind_ans__(Rest, Vars).
bind_ans__([_|Rest], Vars) :-
    bind_ans__(Rest, Vars).

writeDebug(Terms) :-
    writeL_(Terms), nl.

%%write_list(A,B) :- errornl(write_list(A,B)),fail.
write_list(TermList, stream_(N)) :-
    !,
    stream_writeable_(stream_(N), write_list(TermList, stream_(N))),
    freeze_term(TermList),
    writeL_(N, TermList),
    thaw_term(TermList).
write_list(TermList, Stream) :-
    stream_writeable_(Stream, write_list(TermList, Stream)),
    freeze_term(TermList),
    writeL_(Stream, TermList),
    thaw_term(TermList).

'$flush_output__'(stream_(N)) :-
    !,
    stream_writeable_(stream_(N), flush_output(stream_(N))),
    flush_output(N).
'$flush_output__'(Stream) :-
    stream_writeable_(Stream, flush_output(Stream)),
    flush_output(Stream).

%interact(Prompt, Answer) :-
%    writeL_(1, [Prompt]),
%    flush_output,
%    get_line(Answer).



write_list_to_string(TermList, String) :-
    open_string(write, Stream),
    writeL_(Stream, TermList),
    stream_to_string(Stream, String).

log_list(Terms) :-
    open_string(write, Stream),
    writeL_(Stream, ["User ::: "|Terms]),
    send_log_message_(Stream).

write_op_term(Term, Stream) :-
    '$qlg_set_tuple_to_qp'(Term, Term1),
    stream_writeable_(Stream, write_op_term(Term, Stream)),
    write(Stream, Term1).

exit_thread(Name) :- thread_exit(Name), !.
exit_thread(_Name). 

':='(A,B) :-
    thread_atomic_goal('$forget_remember_aux'([A(_)], [A(B)])).

'+:='(A,B) :-
    thread_atomic_goal((A(N), N1 is N+B,
                        '$forget_remember_aux'([A(_)], [A(N1)]))).

'-:='(A,B) :-
    thread_atomic_goal((A(N), N1 is N-B,
                        '$forget_remember_aux'([A(_)], [A(N1)]))).

%% sort(L1, L2, Order)
'$sort__'(L1, L2, Order) :-
    freeze_term(L1, Vars),msort(L1, L2, Order),thaw_term(Vars).

%% re_match(RE, String, Match)
'$re_match__'(RE, String, Match) :-
    re_match(RE, String, Match1),
    map('$re2tuple', Match1, Match).

'$re2tuple'((A:B), '$tuple'([A,B])).

'$transform_subterms_'(Higher, From, To) :-
    transform_subterms('$apply_to_2_args_'(Higher), From, To),
    type(To, ??(term)).

'$collect_simple_terms_'(Higher, From, Base, To) :-
    collect_simple_terms('$apply_to_3_args_'(Higher), From, Base, To).


'$apply_to_2_args_'(F,A, B) :- atom(F), !, call(F(A, B)).
'$apply_to_2_args_'(F,A, B) :- apply_(F, [A, B], R) , R = true.

'$apply_to_3_args_'(F, A, B, C) :- atom(F), !, call(F(A, B, C)).
'$apply_to_3_args_'(F, A, B, C) :- apply_(F, [A, B, C], R) , R = true.


term2list(T, L) :- L \= [], closed_list(L), !,
   T1 =.. L, type(T1, ?term), term2list_aux__(T1, T).
term2list(T, L) :- nonvar(T), T =.. L.

term2list_aux__(T, T1) :-
    compound(T), functor(T, F, _),
    '$type_info'(F, fun, _, _, _), !,
    '$translate_goal'((T1 = T), Call), call(Call).
term2list_aux__(T, T).

%% NOTE: because of address types : has a lower precedence than @ in
%% qulog and so is the opposite to that in QP. So the system should never
%% use ->> other than in the defns of to and to_thread.
%% Also shouldn't use <<= or <<- but prefer from and from_thread
%% unless '$same_thread_handle_' is used to transform (see from)
%%to_thread(A, B) :- errornl(to_thread(A, B)),fail.
to_thread(Msg, Addr) :- atom(Addr), !, Msg ->> Addr.
to_thread(Msg, ':'(TID, Addr)) :- atom(Addr), !, Msg ->> TID : Addr.
to_thread(Msg, ':'(TID, '@'(P,M))) :- Msg ->> TID:P@M.


remote_query_recv_match_(Msg, Addr) :-
    '$ipc_peek'(Msg1, Ref, Addr1, block, false),
    '$qp_set_tuple_to_qlg'(Msg1, Msg2),
    '=@'(Msg, Msg2),
    '$same_agent_handle_'(Addr1, Addr),
    !,
    '$ipc_commit'(Ref).

from_thread(Msg, Addr, Test) :-
    '$ipc_peek_and_check_term_or_throw'(Msg1, Ref, Addr1, block, false,
                                        from_thread),
    '=@'(Msg, Msg1),
    '$qp2qlg_handle'(Addr1, QAddr1),
    '$same_thread_handle_'(QAddr1, Addr),
    call(Test),
    !,
    '$ipc_commit'(Ref).

peek_messages(Msg, Addr, -1) :-
    !,
    peek_messages_aux_(Msg, Addr, block).
peek_messages(Msg, Addr, Timeout) :-
    !,
    peek_messages_aux_(Msg, Addr, Timeout).


peek_messages_aux_(Msg, Addr, Timeout) :-
    '$ipc_peek_and_check_term_or_throw'(Msg1, _Ref, Addr1, Timeout, false,
                                        peek_messages),
    Msg1 \= '$$ipc_timeout$$',
    '$qp_set_tuple_to_qlg'(Msg1, Msg2),
    '=@'(Msg, Msg2),
    '$qp2qlg_handle'(Addr1, QAddr1),
    '$same_thread_handle_'(QAddr1, Addr),
    !.
    

    

to(Msg, Addr) :- Addr == pedro, !,
    '$qlg_set_tuple_to_qp'(Msg, Msg1), Msg1 ->> pedro.
to(Msg, Addr) :- atom(Addr) , !,
    '$qlg_set_tuple_to_qp'(Msg, Msg1),
    Msg1 ->> Addr @ localhost.
to(Msg, P@M) :-
    '$qlg_set_tuple_to_qp'(Msg, Msg1),
    Msg1 ->> P@M.

from(Msg, Addr, Test) :-
    '$ipc_peek_and_check_term_or_throw'(Msg1, Ref, Addr1, block, false, from),
    '=@'(Msg, Msg1),
    '$same_agent_handle_'(Addr1, Addr),
    call(Test),
    !,
    '$ipc_commit'(Ref).

%% peek at next message and check it is a valid term otherwise
%% throw and exception and consume the invalid message
'$ipc_peek_and_check_term_or_throw'(Term, Ref, Addr, Block, Remember) :-
    '$ipc_peek_and_check_term_or_throw'(Term, Ref, Addr, Block, Remember,
                                        receive).
'$ipc_peek_and_check_term_or_throw'(Term, Ref, Addr, Block, Remember, Call) :-
    '$ipc_peek'(Term1, Ref, Addr, Block, Remember),
    '$qp_set_tuple_to_qlg'(Term1, Term),
    catch('$check_and_eval_input_term'(Term1, Term, term(Term1, []), Call),
          Patt, ('$ipc_commit'(Ref), throw(Patt))).
    
% '$check_term_or_throw'(Term, Ref, Call) :-
%     has_type2__(Term, term, []), !,
%     (
%       '$translate_goal'((Term = T2), UCall), UCall \= (Term = T2)
%     ->
%       '$ipc_commit'(Ref),
%       term2string(Term, TermString),
%       throw(input_term_error(Call, TermString))
%     ;
%       true
%     ).
% '$check_term_or_throw'(Term, Ref, Call) :-
%     '$ipc_commit'(Ref),
%     term2string(Term, TermString),
%     throw(input_term_type_error(Call, TermString)).

%% test or generate a full agent handle
%% for a pedro notification the handle is pedro
'$same_agent_handle_'(pedro, Addr2) :-
    !, Addr2 = pedro.
%% the full agent handle for an atom has localhost as the host
'$same_agent_handle_'(Addr1, Addr2) :-
    atom(Addr1), !,
    '$same_agent_handle_'(Addr1@localhost, Addr2).
%% For a full handle drop the thread name
'$same_agent_handle_'(Addr1, Addr2) :-
    var(Addr2), 
    Addr1 = '@'(':'(_, A1), A2), !,
    (
     catch(same_handle('@'(A1, localhost), '@'(A1, A2)), _, fail)
    ->
      Addr2 = '@'(A1, localhost)
    ;
      Addr2 = '@'(A1, A2)
    ).
%% If the host is the same as localhost - replace by localhost
'$same_agent_handle_'(Addr1, Addr2) :-
    var(Addr2), !,
    Addr1 = '@'(A1, A2),
    (
     catch(same_handle('@'(A1, localhost), '@'(A1, A2)), _, fail)
    ->
      Addr2 = '@'(A1, localhost)
    ;
      Addr2 = '@'(A1, A2)
    ).
%% reverse when Addr2 is given
'$same_agent_handle_'(Addr1, Addr2) :-
    atom(Addr2), 
    Addr1 = '@'(':'(_, A1), A2), !,
    catch(same_handle('@'(Addr2, localhost), '@'(A1, A2)), _, fail).
'$same_agent_handle_'(Addr1, Addr2) :-
    atom(Addr2), !,
    Addr1 = '@'(A1, A2),
    catch(same_handle('@'(Addr2, localhost), '@'(A1, A2)), _, fail).

'$same_agent_handle_'(Addr1, Addr2) :-
    Addr2 = '@'(X1, Y), var(Y), !,
    same_handle(Addr1, '@'(X, Y1)),
    (
      catch(same_handle('@'(X, localhost), '@'(X, Y1)), _, fail)
    ->
      Y = localhost,
     once((X = ':'(_, X1) ; X = X1))
    ;
      Y = Y1,
     once((X = ':'(_, X1) ; X = X1))
    ).
'$same_agent_handle_'(Addr1, Addr2) :-
    Addr2 = '@'(X, Y), atom(Y), 
    same_handle(Addr1, '@'(':'(_, X), Y1)), !,
    '$same_host'(Y1, Y).
'$same_agent_handle_'(Addr1, Addr2) :-
    Addr2 = '@'(X, Y), atom(Y), !,
    same_handle(Addr1, '@'(X, Y1)),
    '$same_host'(Y1, Y).
'$same_agent_handle_'(Addr1, Addr2) :-
    catch(same_handle(Addr1, Addr2), _, fail).

%% test/generate full handle (including thread name)
%%'$same_thread_handle_'(Addr1, Addr2) :- errornl('$same_thread_handle_'(Addr1, Addr2)),fail.
'$same_thread_handle_'(A, B) :- A == B, !.
'$same_thread_handle_'(pedro, Addr2) :- !, Addr2 = pedro.
'$same_thread_handle_'(Addr1, Addr2) :-
    atom(Addr1), !,
    thread_handle('@'(':'(_, PID), Host)),
    '$same_thread_handle_'(':'(Addr1, '@'(PID, Host)), Addr2).
'$same_thread_handle_'(Addr1, Addr2) :-
    Addr1 = ':'(Thread, PID),
    atom(PID), !,
    thread_handle('@'(':'(_, _), Host)),
    '$same_thread_handle_'(':'(Thread, '@'(PID, Host)), Addr2).    
'$same_thread_handle_'(Addr1, Addr2) :-
    var(Addr2),!,
    Addr1 = ':'(ID, '@'(P, Host)),
    thread_handle('@'(':'(_, PID), _)),
    (
      catch(same_handle('@'(':'(ID, P), localhost), '@'(':'(ID, P), Host)),
            _, fail)
    ->
     (
      P = PID
     ->
       Addr2 = ID
     ;
      Addr2 = ':'(ID, P)
     )
    ;
      Addr2 = ':'(ID, '@'(P, Host))
    ).
'$same_thread_handle_'(Addr1, Addr2) :-
    atom(Addr2),!,
    '$same_thread_handle_'(Addr2, Addr1).
'$same_thread_handle_'(Addr1, Addr2) :-
    Addr2 = ':'(ID, Addr3),
    var(Addr3),!,
    thread_handle('@'(':'(_, PID), _)),
    Addr1 = ':'(ID, '@'(P, Host)),
    (
     catch(same_handle('@'(PID, localhost), '@'(PID, Host)), _, fail)
    ->
     (
      P = PID
     ->
      Addr3 = PID
     ;
      Addr3 = '@'(P, localhost)
     )
    ;
      Addr3 = '@'(P, Host)
    ).    
'$same_thread_handle_'(Addr1, Addr2) :-
    Addr2 = ':'(ID, _Addr3),
    var(ID),!,
    Addr1 = ':'(ID, _),
    '$same_thread_handle_'(Addr2, Addr1).
'$same_thread_handle_'(Addr1, Addr2) :-
    Addr2 = ':'(_X, P), atom(P), !,
    '$same_thread_handle_'(Addr2, Addr1).
'$same_thread_handle_'(Addr1, Address) :-
    Address = ':'(ID, '@'(P, M)),
    Addr1 = ':'(ID, '@'(P, Host)),
    thread_handle('@'(':'(_, PID), _)),
    (
     catch(same_handle('@'(PID, localhost), '@'(PID, Host)), _, fail)
    ->
     M = localhost
    ;
     M = Host
    ).

%% ?(G)
'$rel_escape'(G) :- call(G), !.
'$rel_escape'(G) :-
    term2string('?'(G), AString),
    throw(action_failure(AString)).


%% A = B
%%unify(A, A).
%% A \= B
%%not_unify(A, B) :- A \= B.

not(A) :- \+ A.

this_thread_name(Name) :- thread_symbol(Name1), Name = Name1.
this_process_name(Name) :- thread_handle(_:Name1@_), Name = Name1.


subscribe(Substring, Id) :-
    add_under_to_br_br__(Substring, TransSubstring),
    string_concat(TransSubstring, ".", TermString),
    open_string(read(TermString), Stream),
    catch('$subscribe_as_aux'(Stream, Id1), _, (Id1 = 0)),
    '$action_one_sided_unif'(Id, Id1, [], subscribe(Substring, Id)),
    close(Stream).

add_under_to_br_br__(String, TransString) :-
    re_match(".*?(\\(\\)).*", String, [_, S:E]), !,
    sub_string(String, 0, S, _, First),
    Len is E-S,
    sub_string(String, S, Len, After, _),
    sub_string(String, E, After, _, Rest),
    string_concat(First, "(_)", Bra),
    string_concat(Bra, Rest, String1),
    add_under_to_br_br__(String1, TransString).
add_under_to_br_br__(String, String).
    
unsubscribe(ID) :-
    pedro_unsubscribe(ID), !.
unsubscribe(_).

'$subscribe_as_aux'(_Stream, _Id) :-
    \+ pedro_is_connected, !,
    throw(pedro_not_connected(subscribe_as)).
'$subscribe_as_aux'(Stream, Id) :-
    read_term(Stream, Term, [op_table(subscribe_reader)]),
    (
      compound(Term), functor(Term, F, 2), F == '::'
    ->
      Term = (Patt :: Test)
    ;
      Patt = Term,
      Test = true
    ),
    (
      pedro_subscribe(Patt, Test, Id)
    ->
      true
    ;
      Id = 0
    ).
    

kill_thread(Name) :-
    thread_exit(Name), !.
kill_thread(_).

exit(_) :- thread_exit.

forall(_,C1,C2) :- forall(C1,C2).

this_task_name(Task) :- ip_lookup('$task_name', Task), atom(Task).
    
sub_string(String,Before,Sub,After) :- 
    string(String), string(Sub), 
    string_length(String,StringN), string_length(Sub,SubN),
    SubN =< StringN,
    !,
    sub_string(String, StartSubN, SubN, RemainingN, Sub),
    sub_string(String,0,StartSubN,_,Before),
    (RemainingN =0 -> After="" 
       ;
       AfterBegins is StartSubN+SubN,
       sub_string(String,AfterBegins,_,0,After)). 
sub_string(String,Before,Sub,After) :- 
    string(String), string(Before), string(After),
    !,
    sub_string(String,0,BeforeN,RemN,Before),
    sub_string(String,_,AfterN,_,After),
    ( RemN > AfterN -> sub_string(String, BeforeN, _, AfterN, Sub)
       ;
       Sub="").

%% This takes a list of facts to remember
remember(Cs) :-
    thread_atomic_goal('$rememberBS'(Cs, z)).

%% reset the time fact after all have been remembered given
%% there is at least one to remember
'$rememberBS'(Cs,AorZ) :-
    global_state_set('$db_update_count', 0),
    member(C, Cs),
    '$check_assert'(C,AorZ),
    fail.
'$rememberBS'(_Cs,_AorZ) :-
    global_state_lookup('$db_update_count', 0),
    !.
'$rememberBS'(Cs, _AorZ) :-
    '$infer_and_remember'(Cs, []),
    '$reset_time_fact'.

%% don't remember if already there
'$check_assert'(C,_) :-
    call(C), !.
'$check_assert'(C,z) :- !,
    global_state_increment('$db_update_count', _),
    assert(C).
'$check_assert'(C,a) :- 
    global_state_increment('$db_update_count', _),
    asserta(C).



remember_for(Terms, Secs) :-
    thread_atomic_goal((
                        remember(Terms),
                        create_timer(forget(Terms), Secs, true, _)
                      )).


forget(Cs) :-
    thread_atomic_goal('$forgetBS'(Cs)).

%% reset time fact in same way as for remember
'$forgetBS'(Cs) :-
    global_state_set('$db_update_count', 0),
    member(C, Cs),
    '$check_retract'(C),
    fail.
'$forgetBS'(_Cs) :-
    global_state_lookup('$db_update_count', 0),
    !.
'$forgetBS'(Cs) :-
    '$infer_and_remember'(Cs, []),
    '$reset_time_fact'.


'$check_retract'(C) :-
    \+call(C),
    %% nothing to forget
    !.
'$check_retract'(C) :-
    retractall(C),
    global_state_increment('$db_update_count', _).




forget_after(Term, Secs) :-
    thread_atomic_goal((
                        create_timer(forget(Term), Secs, true, _)
                      )).

forget_remember(Fs, Rs) :-
    thread_atomic_goal('$forget_remember_aux'(Fs, Rs)).
    

%% do the forgets before the remembers
'$forget_remember_aux'(Fs, Rs) :-
    global_state_set('$db_update_count', 0),
    %% only forget the facts that won't then be remembered
    member(C, Fs),
    call(C),
    %% C is instantiated to an instance in the BS
    \+member(C, Rs),
    %% this instance is not to be remembered so retract it
    retract(C),
    global_state_increment('$db_update_count', _),
    fail.
'$forget_remember_aux'(_Fs, Rs) :-
    member(C, Rs),
    '$check_assert'(C,z),
    fail.
'$forget_remember_aux'(_Fs, _Rs) :-
    %% No changes were made to the belief store
    global_state_lookup('$db_update_count', 0), !.
'$forget_remember_aux'(Fs, Rs) :-
    %% Changes were made so infer and remembers relations declared as rem
    '$infer_and_remember'(Fs, Rs),
    '$reset_time_fact'.


%% update rem relations based on the lists of facts
%% forgotten and remembered - each rem relation's cache is only cleared
%% if dependent on remembers anf forgets
'$infer_and_remember'(Dep1, Dep2) :-
    cache_info__(Call, _, PFact, _),
    '$update_inferred'(Call, PFact, Dep1, Dep2),
    fail.
'$infer_and_remember'(_, _).

'$update_inferred'(Call, _PFact, Dep1, Dep2) :-
    %% no dependiences so no update required
    not_dep_overlap_(Call, Dep1, Dep2), !.
'$update_inferred'(_, PFact, _Dep1, _Dep2) :-
    %% remove all PFact facts
    retractall(PFact),
    retractall('$is_cached'(PFact)),
    fail.
'$update_inferred'(_, _PFact, _Dep1, _Dep2).

%% only forget if fact is present and if G is a template forget
%% first fact
'$try_forget'(G) :-
    call(G), !,retract(G).
'$try_forget'(_).

%% only remember if G is not already in the BS
'$try_remember'(G) :-
    call(G), !.
'$try_remember'(G) :-
    assert(G).



%% Used in not_dep_overlap_ which is generated by the compiler
%%'$no_overlap'(A, B, C) :- errornl('$no_overlap'(A, B, C)),fail.
'$no_overlap'(A, _B, C) :-
    member(X, A), member(X, C), !, fail.
'$no_overlap'(_, [], _) :- !.
'$no_overlap'(_, B, C) :-
    member(X, B), member(X, C), !, fail.
'$no_overlap'(_, _, _).

%%cache_call__(C, R, P) :- errornl(cache_call__(C, R, P)),fail.
cache_call__(C, R, P) :-
    \+'$is_cached'(P),
    thread_atomic_goal(cache_call_cache__(C,R,P)),
    fail.
cache_call__(_C, R, _P) :- call(R).

%%cache_call_cache__(C, R, P) :- errornl(cache_call_cache__(C, R, P)),fail.
cache_call_cache__(C, _R, P) :-
    %% if 2 threads get to this point then one will win and in its atomic
    %% call assert is_cached and so the other will fail on the next call
    %% and so not double up on caching
    \+'$is_cached'(P),
    forall(C, assert(P)),
    assert('$is_cached'(P)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% TR specific interface code
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%start_percept_handler('$none_') :- start_percept_handler.

running(A) :- '__running__'(A).
waiting(A) :- '__waiting__'(A).
resources(Task, Res) :-
    '__running__'(Task),
    '__resources__'(Task, Res).
           
time_(A) :- '__time__'(A).



actions(Actions) :-
    task(Task, _), !,
    writeL_(stderr, ["Error: ", actions(Actions),
                    " can't be used when a task is running. ",
                    "Currently" , Task, " is running.", nl_]).
actions(Actions) :-
    thread_atomic_goal((
                        log_manual_actions_(Actions),
                        controls_(Actions)
                       )).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% General library code
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% set the number of answers for the interpreters
answers(N) :-
    retract('$num_solutions'(_)),
    assert('$num_solutions'(N)).

%% build compound from functor and args list being careful to deal
%% with translating empty args list into '$none_' arg so Prolog can cope.
'@..'(',..', Args, R) :- !, Args = [H, T], R = [H|T].
'@..'(F, [], R) :- R= F('$none_'), !.
'@..'(F, Args, R) :- R=..[F|Args].

%% Reverse of above - ie splitting up compound
'@..rev'(Compound, F, Args) :-
    \+list(Compound), !,
    compound(Compound),
    Compound =.. [F|Args1],
    (
      Args1 = [A], A == '$none_'
    ->
      Args = []
    ;
      Args = Args1
    ).
'@..rev'(Compound, F, Args) :-
    Compound = [H|T], !,
    F = ',..',
    Args = [H, T].
'@..rev'(Compound, F, Args) :-
    Compound = [], !,
    F = nil_, Args = [].
'@..rev'(Compound, F, Args) :-
    Compound = nil_(_), !,
    F = nil_, Args = [].

%% Equiv of Prolog functor but can cope with f('$none_')
'@functor'(A,B,C) :-
    compound(A), !,
    '@..rev'(A, B, Args),
    length(Args, C).
'@functor'(A, B, C) :-
    integer(C),!,
    length(Args, C),
    '@..'(B, Args, A).

%% Equiv of =.. but can cope with f('$none_')
'@=..'(A, B) :-
    compound(A), !,
    '@..rev'(A, F, Args),
    B = [F|Args].
'@=..'(A, B) :- B = [F|Args],
    '@..'(F, Args, A).

%% apply_(Functor, ArgsList, Result)  applies Functor to the args list to
%% get Result using the apply predicate for the given functor
%% EG for apply_(+, [2,3],R) we call '+$apply'(+, [2,3], R) which sets R to 5
%% The apply predicate for curry is
%%'curry$apply'(curry, [A], curry(A)) :- 
%%    !.
%%'curry$apply'(curry(A), [B], curry(A)(B)) :- 
%%    !.
%%'curry$apply'(curry(A)(B), [C], D) :- 
%%    !,
%%    apply_(A, [B, C], E),
%%    unify(E, D).
%% Note that the third clause above is where we have all the args for curry
%% and apply is used to apply the given function to its args.

%%apply_(A, B, C) :- errornl(apply_(A, B, C)), fail.
apply_(A, B, C) :-
    atom(A), 
    AC =.. [A|B],
    '$type_info'(_, act, _, AC, _), !,
    C = AC.
apply_(A, B, C) :-
    atom(A), 
    AC =.. [A|B],
    '$type_info'(_, rel, _, AC, _), !,
    C = AC.
apply_(A, Tuple, C) :-
    '$atom_functor'(Tuple),
    Tuple = '$tuple'(B),
    '$generate_apply_name'(A, ApplyName),
    current_predicate(ApplyName/3), !,
    call(ApplyName(A, B, C)). %%% WAS '$eval_goal'(ApplyName(A, B, C), 1, 1). % call without occurs check
apply_(A, B, C) :-
    '$generate_apply_name'(A, ApplyName),
    current_predicate(ApplyName/3), !,
    call(ApplyName(A, B, C)). %%% WAS '$eval_goal'(ApplyName(A, B, C), 1, 1).  %% call without occurs check
apply_(A, B, AC) :-
    AC =.. [A, _|B],
    term2string(undefined_predicate(AC), ExceptStr),
    throw(qp_exception(ExceptStr)).

%% These are intended to be overwritten by users
'$system_user_type'(handle_remote_query_exception).
'$system_user_type'(handle_message).
'$system_user_type'(handle_invalid_message).
'$system_user_type'(handle_template_message).
'$system_user_type'(resources_hook):-
    '$running_teleo'.
'$system_user_type'(init_message_handler):-
    '$running_teleo'.
'$system_user_type'(init_percept_handler):-
    '$running_teleo'.
'$system_user_type'(post_process_percepts):-
    '$running_teleo'.
'$system_user_type'(poll_sensors):-
    '$running_teleo'.
'$system_user_type'(control_device):-
    '$running_teleo'.
'$system_user_type'(send_robotic_message):-
    '$running_teleo'.
'$system_user_type'(overlapping_resources):-
    '$running_teleo'.


%% lookup type info for type checking
'$type_info'(A, B, C, D, E) :-
    '$builtin_type_info'(A, B, C, D, E).
'$type_info'(A, B, C, D, E) :-
    '$user_type_info'(A, B, C, D, E).

%% Add default args if not given
%% Note that if say the second default arg is given then the first must be
%% given as well even if it has its default value
'$default_args'(Call, FullCall) :-
    freeze_term(Call, Vars),
    '$builtin_default_args'(Call, FullCall), !,
    thaw_term(Vars).
'$default_args'(Call, FullCall) :-
    freeze_term(Call, Vars),
    '$user_default_args'(Call, FullCall), !,
    thaw_term(Vars).


%% look up defined types
'$defined_type'(A, B) :-
    '$type_info'(_, defined_type, A, B, _).
'$defined_type'(A, B) :-
    '$type_info'(_, macro_type, A, B, _).
'$user_defined_type'(A, B) :-
    '$user_type_info'(_, defined_type, A, B, _).
'$user_defined_type'(A, B) :-
    '$user_type_info'(_, macro_type, A, B, _).

%% for checking if its OK for a user to define code if the type is builtin
'$dynamic_builtin_type_info'(A, B, C, D, E) :-
    '$builtin_type_info'(A, B, C, D, E),
    '$dynamic_builtin'(A).


%% A is a belief - get the type info
'$belief_type_info'(A,C,D) :-
    '$type_info'(A, _, C, D, _),
    '$belief'(D).

%% A is a percept - get the type info
'$percept_type_info'(A,C,D) :-
    '$type_info'(A, _, C, D, _),
    '$percept'(D).


%%Get the kind of Name -(rel/act/tel/fun but replace rel by belief if a belief
%% and use special name for system (builtin) Names
'$real_kind'(Name, Kind) :-
    '$user_type_info'(Name, Kind0, _, _, _), !,
    (
      '$belief'(Name)
    ->
      Kind = belief
    ;
      Kind = Kind0
    ).
'$real_kind'(Name, Kind) :-
    '$type_info'(Name, Kind0, _, _, _),
    (
      Kind0 = rel
    ->
      Kind = 'system relation'
    ;
      Kind0 = fun
    ->
      Kind = 'system function'
    ;
      Kind0 = act
    ->
      Kind = 'system action'
    ).

%% assert for debugging
check_assert_(Goal) :-
    call(Goal), !.
check_assert_(Goal) :-
    errornl(failed_check_assert___________________(Goal)).




%% for use in the interpreter
'$is_relactfuntr'(T) :-
    compound(T), functor(T, F, N),
    atom(F),
    '$is_relactfuntr_aux'(F, N).
'$is_relactfuntr_aux'(atom_naming,1).
'$is_relactfuntr_aux'(rel,1).
'$is_relactfuntr_aux'(act,1).
'$is_relactfuntr_aux'(tel,1).
'$is_relactfuntr_aux'('->',2).

'$is_atom_or'(Term, alt_types([T|_])) :-
    atom(Term),
    '$is_relactfuntr'(T),
    !.
'$is_atom_or'(Term, T) :-
    atom(Term),
    '$is_relactfuntr'(T),
    !.
    

% get_all_types(Term, Type) :-
%     findall(T, (has_type2__(Term, T1, []), '$strip_naming'(T1,T)),
%             AltTypes),
%     merge_types__(AltTypes, Type).
         
'$strip_naming'(T1, T) :-
    compound(T1), T1 = atom_naming(T), !.
'$strip_naming'(T1, T) :-
    compound(T1), T1 = term_naming(T), !.
'$strip_naming'(T, T).

%% simplify alt types - for use in interpreter
merge_types__(AltTypes, Type) :-
    '$collect_min_types'(AltTypes, [], SimpAltTypes), 
    remove_identical_patterns__(SimpAltTypes, SimpSimpAltTypes),
    (
      SimpSimpAltTypes = [T1]
    ->
      Type = T1
    ;
      Type = alt_types(SimpSimpAltTypes)
    ).

'$collect_min_types'([], _, []).
'$collect_min_types'([T|Ts], Seen, Simp) :-
    member(T1, Ts), type_le(T1, T), !,
    '$collect_min_types'(Ts, Seen, Simp).
'$collect_min_types'([T|Ts], Seen, Simp) :-
    member(T1, Seen), type_le(T1, T), !,
    '$collect_min_types'(Ts, Seen, Simp).
'$collect_min_types'([T|Ts], Seen, Simp) :-
    '$collect_min_types'(Ts, [T|Seen], Simp1),
    Simp = [T|Simp1].


remove_identical_patterns__([], []).
remove_identical_patterns__([X|T], ST) :-
    member(Y, T),
    identical_patterns_(X, Y), !,
    remove_identical_patterns__(T, ST).
remove_identical_patterns__([X|T], [X|ST]) :-
    remove_identical_patterns__(T, ST).

identical_patterns_(X, Y) :-
    freeze_term(X),
    X \= Y, !,
    fail.
identical_patterns_(X, Y) :-
    freeze_term(Y),
    X \= Y, !,
    fail.
identical_patterns_(_, _).


%% EG curry(+)(1)
'$higher_order_term'(T) :-
    compound(T),
    functor(T, F, _N),
    compound(F).

%% EG  T -> T used when we have a type of the form T -> Type
'$higher_order_type'(Type) :-
    nonvar(Type),
    (
      Type = (_ -> _)
    ->
      true
    ;
      '$defined_type'(Type, DType),
      '$higher_order_type'(DType)
    ).


%% EG '$get_ultimate_functor'(curry(+)(1), F)  F = curry
'$get_ultimate_functor'(Term, F) :-
    \+compound(Term), !, F = Term.
'$get_ultimate_functor'(Term, F) :-
    functor(Term, F1, _),
    '$get_ultimate_functor'(F1, F).

%% For use in apply eg '$generate_apply_name'(+, Name)   Name = '+$apply'
'$generate_apply_name'(F, Name) :-
    '$get_ultimate_functor'(F, UF),
    atom(UF),
    atom_concat(UF, '$apply', Name).
    
'$atom_functor'(Term) :-
    atom(Term), !.
'$atom_functor'(Term) :-
    compound(Term),
    functor(Term, F, _),
    atom(F).

%% Vars is a list of form V = VName where V is a variable and VName is an
%% atom - the name of the variable
%% replace all variables in Term with their name wrapped with '$$var$$'
%% Used in pretty printing of declarations and definitions to suppress quotes
%% around variable names
'$vars2names'(Term, Vars) :-
    '$do_unifs'(Vars),
    collect_vars(Term, UVars),
    '$unify_underscore'(UVars).

'$do_unifs'([]).
'$do_unifs'([(A=B)|U]) :-
    A = '$$var$$'(B), !,
    '$do_unifs'(U).
'$do_unifs'([_|U]) :-
    '$do_unifs'(U).

'$unify_underscore'([]).
'$unify_underscore'(['$$var$$'('_')|UV]) :-
    '$unify_underscore'(UV).


%% Term is a function application Args are the args of Term and Types are the
%% corresponding types and ResultType is the result type of the function
%% EG
%% | ?- '$get_function_type_info'(+(X,Y), A, T, R).

%% X = X
%% Y = Y
%% A = ['$tuple'([X, Y])]
%%T = ['$tuple_type'([nat, nat])]
%% R = nat
%% and on backtracking we get int and num types
%% EG
%% | ?- '$get_function_type_info'(curry(+)(1), A, T, R).

%% A = ['$tuple'([1]), '$tuple'([+])]
%% T = ['$tuple_type'([B]), '$tuple_type'([('$tuple_type'([B, C]) -> D)])]
%% R = term_naming(('$tuple_type'([C]) -> D))

%% Note for HO functions the args and types are flattened into flat lists
%% with reversal of blocks of args and their types - i.e. 1 appears before +


'$get_function_type_info'(Term, Args, Types, ResultType) :-
    '$get_ultimate_functor'(Term, F),
    '$type_info'(F, fun, _, _, _), !,
    '$type_info'(F, fun, Type, _, _VarNames),
    '$get_function_type_info_aux'(Term, Type, Args, Types, ResultType).
% '$get_function_type_info'(Term, Args, Types, ResultType) :-
%     '$had_function_clauses'(Term), !, fail,
%     '$gen_type_term_pattern'(Term, F, term, Type),
%     '$get_full_goal_term'(F, Type, GT),
%     assert('$user_type_info'(F, fun, Type, GT, [])),
%     assert('$saved_input_with_vars'((F : Type), [], _)),
%     writeL_(stderr, [nl_, "Warning: undeclared function ",  F,
% 		    nl_, "Declaring with type ", Type, nl_, nl_]),
%     '$get_function_type_info'(Term, Args, Types, ResultType).

'$gen_type_term_pattern'(A, F, InType, Type) :-
    compound(A),!,
    A =.. [F1|Args],
    findall(term, (member(X, Args), X \== '$none_'), Terms),
    Types = '$tuple_type'(Terms),
    '$gen_type_term_pattern'(F1, F, (Types -> InType), Type).
'$gen_type_term_pattern'(A, A, Type, Type).

%% Given the name of a function return the rule head for F
%% TODO maybe we should have a version of the head with just variable
%% args (all distinct) - used in watch and tr_translator
'$get_function_head_for_name'(F, TemplateHead) :-
    '$saved_input_with_vars'(B, _, _),
    (
     B = (Head -> _), '$get_ultimate_functor'(Head, F)
    ;
     B = (Head :: _ -> _), '$get_ultimate_functor'(Head, F)
    ), !,
    '$change_to_template_compound'(Head, TemplateHead).
'$get_function_head_relact_for_name'(F, TemplateHead) :-
    '$saved_input_with_vars'(B, _, _),
    (
     B = (Head <= _), '$get_ultimate_functor'(Head, F)
    ;
     B = (Head ~> _), '$get_ultimate_functor'(Head, F)
    ;
     B = (Head :: _ <= _), '$get_ultimate_functor'(Head, F)
    ;
     B = (Head :: _ <= _), '$get_ultimate_functor'(Head, F)
    ;
     B = Head, '$get_ultimate_functor'(Head, F)
    ), !,
    '$change_to_template_compound'(Head, TemplateHead).

'$change_to_template_compound'(C, Template) :-
    '@..rev'(C, F, Args), atom(F), !,
    length(Args, N),
    length(TempArgs, N),
    '@..'(F, TempArgs, Template).
'$change_to_template_compound'(C, Template) :-
    '@..rev'(C, F, Args),
    '$change_to_template_compound'(F, TempF),
    length(Args, N),
    length(TempArgs, N),
    '@..'(TempF, TempArgs, Template).


%% TODO - if we insist on everything having types then not required?
% '$had_function_clauses'(A) :-
%     nonvar(A),'$get_ultimate_functor'(A, F),
%     '$saved_input_with_vars'(B, _, _),
%     (
%      B = (Head -> _), '$get_ultimate_functor'(Head, F)
%     ;
%      B = (Head :: _ -> _), '$get_ultimate_functor'(Head, F)
%     ), !.

%%'$get_function_type_info_aux'(Term, ResultType, OA, OT, ResultType) :- errornl('$get_function_type_info_aux'(Term, ResultType, OA, OT, ResultType)),fail.
'$get_function_type_info_aux'(Term, ResultType, [], [],
                              term_naming(ResultType)) :-
    \+compound(Term), nonvar(ResultType),
    ResultType = (_ -> _),
    !.
'$get_function_type_info_aux'(Term, ResultType, [], [],
                              term_naming(ResultType)) :-
    \+compound(Term), nonvar(ResultType),
    ResultType = rel(_),
    !.
'$get_function_type_info_aux'(Term, ResultType, [], [],
                              term_naming(ResultType)) :-
    \+compound(Term), nonvar(ResultType),
    ResultType = dyn(_),
    !.
'$get_function_type_info_aux'(Term, ResultType, [], [],
                              term_naming(ResultType)) :-
    \+compound(Term), nonvar(ResultType),
    ResultType = for(_),
    !.
'$get_function_type_info_aux'(Term, ResultType, [], [],
                              term_naming(ResultType)) :-
    \+compound(Term), nonvar(ResultType),
    ResultType = act(_),
    !.
'$get_function_type_info_aux'(Term, ResultType, [], [],
                              term_naming(ResultType)) :-
    \+compound(Term), nonvar(ResultType),
    ResultType = tel(_),
    !.
'$get_function_type_info_aux'(Term, ResultType1, [], [], ResultType) :-
    \+compound(Term), nonvar(ResultType1),
    '$type_info'(_, macro_type, ResultType1, ResultType2, _),
    !,
    '$get_function_type_info_aux'(Term, ResultType2, [], [],
                                  ResultType).

'$get_function_type_info_aux'(Term, ResultType, [], [], ResultType) :-
    \+compound(Term),
    !.
'$get_function_type_info_aux'(Term, ResultType, OutArgs,
                              OutTypes, FinalResultType) :-
    compound(Term), !,
    Term =.. [F|A1],
    %%'$list2tuple'(A, AT),
    (
      A1 = [X], X == '$none_'
    ->
      A = []
    ;
      A = A1
    ),
    Arg = '$tuple'(A),
    (
      ResultType = (T1 -> T2)
    ->
      '$strip_top_fun_bangs'(T1, ST1),
      '$strip_top_fun_bangs'(T2, ST2),
      '$get_function_type_info_aux'(F, ST2, Args, Types,  FinalResultType),
      append(Types, [ST1], OutTypes)
    ;
      '$defined_type'(ResultType, (T1 -> T2))
    ->
      '$get_function_type_info_aux'(F, T2, Args, Types,  FinalResultType),
      append(Types, [T1], OutTypes)
    ;
      '$defined_type'(ResultType, rel(T1))
    ->
      FinalResultType = rel,
      Args = [],
      Types = [],
      append(Types, [T1], OutTypes)
    ;
      ResultType = rel(T1),
      FinalResultType = rel,
      Args = [],
      Types = [],
      append(Types, [T1], OutTypes)
    ),
    OutArgs = [Arg|Args].
'$get_function_type_info_aux'(_Term, ResultType, OutArgs,
                              OutTypes, FinalResultType) :-
    OutArgs = [],
    OutTypes = [],
    FinalResultType = ResultType.

%% This is used for generating type info for functions
%% For most functions, EG curry this will return GT = '$function'
%% and the user type info asserted will have the 4th arg as '$function'
%% but something like curryR (which is a relation with all args given)
%% will have its 4th arg as curryR(A)(B)(C) 
%%'$get_full_goal_term'(FName, Type, GT) :- errornl('$get_full_goal_term'(FName, Type, GT)),fail.
'$get_full_goal_term'(FName, Type, GT) :-
    freeze_term(Type, Vs),
    '$type_info'(_, macro_type, Type, MType, _), !,
    thaw_term(Vs),
    '$get_full_goal_term'(FName, MType, GT).
'$get_full_goal_term'(_FName, Type, GT) :-
    \+compound(Type), !, GT = '$function'.
'$get_full_goal_term'(FName, Type, GT) :-
    functor(Type, F, 2), F == '->', !,
    Type = (T1 -> T2),
    '$tuple_type_to_list'(T1, T1L),
    length(T1L, N),
    length(Args, N),
    NextF =.. [FName|Args],
    '$get_full_goal_term'(NextF, T2, GT).
'$get_full_goal_term'(FName, Type, GT) :-
    functor(Type, F, 1), F == rel, !,
    Type = rel(T1),
    '$tuple_type_to_list'(T1, T1L),
    length(T1L, N),
    length(Args, N),
    GT =.. [FName|Args].
'$get_full_goal_term'(_FName, _Type, GT) :-
    GT = '$function'.


%% Special queries that are type checked specially and so when we do a
%% check_query to make sure a query in the interpreter is valid we
%% use this table to look up special queries
'$builtin_query'(X) :- '$special_builtin_query'(X).
'$builtin_query'('$exists'(_,_)).
'$builtin_query'('$forall'(_,_,_)).
'$builtin_query'('$pfindall'(_,_,_)).
'$builtin_query'('$at_forall'(_,_,_)).
'$builtin_query'('$forall_actions'(_,_,_)).
'$builtin_query'('$at_forall_actions'(_,_,_)).
%%'$builtin_query'(X) :- ground(X), list(X), X = [_|_].
'$builtin_query'('$remote_query'(_, _)).
'$builtin_query'(true).

'$special_builtin_query'(subscribe_as(_, _)).
'$special_builtin_query'(from(_, _, _)).
'$special_builtin_query'(from_thread(_, _, _)).
'$special_builtin_query'(receive(_)).
'$special_builtin_query'(case(_)).
'$special_builtin_query'(wait(_)).
'$special_builtin_query'(wait_case(_)).
'$special_builtin_query'(try_except(_, _)).
'$special_builtin_query'(repeat_until(_, _)).
'$special_builtin_query'(atomic_action(_)).
'$special_builtin_query'(wait(_, _, _)).
'$special_builtin_query'(once(_)).
'$special_builtin_query'('$rel_escape'(_)).
'$special_builtin_query'(type(_, _)).
%'$special_builtin_query'(typeC(_, _)).
%'$special_builtin_query'(typeC(_, _, _)).
'$special_builtin_query'(isa(_, _)).
'$special_builtin_query'(remember(_)).
'$special_builtin_query'(forget_remember(_,_)).
'$special_builtin_query'(remember_for(_,_)).
'$special_builtin_query'(rememberA(_)).
'$special_builtin_query'(rememberA_for(_,_)).
'$special_builtin_query'(forget(_)).
'$special_builtin_query'(forget_after(_,_)).
'$special_builtin_query'(fork_as(_,_)).
'$special_builtin_query'(fork_light_as(_,_)).
'$special_builtin_query'(fork_sizes_as(_,_)).
'$special_builtin_query'('+:='(_, _)).
'$special_builtin_query'('-:='(_, _)).
'$special_builtin_query'(':='(_, _)).
'$special_builtin_query'('=?'(_, _)).
%'$special_builtin_query'('$remote_query'(_, _)).
'$special_builtin_query'('$remote_query'(_, _)).

%% check that Query is a valid query in the interpreter
%%'$check_query'(Query, Vars) :-errornl('$check_query'(Query, Vars)), fail.
'$check_query'(Query, Vars) :-
    var(Query), !,
    '$check_goal_expressions'(Query, Query, Vars, any).
'$check_query'(Query, Vars) :-
    compound(Query), functor(Query, F, _), var(F), !,
    '$check_goal_expressions'(Query, Query, Vars, any).
'$check_query'('$rel_escape'(Q), Vars) :- !,
    '$check_goal_expressions'(Q, '$rel_escape'(Q), Vars, rel).
'$check_query'(show, _) :- !.
'$check_query'(show(_), _) :- !.
'$check_query'(not(Q), Vars) :- !,
    '$check_goal_expressions'(not(Q),  not(Q), Vars, rel).
'$check_query'(Query, Vars) :-
    (
      '$builtin_query'(Query)
    ;
      '$type_info'(_, rel, _, Query, _)
    ;
      '$type_info'(_, act, _, Query, _)
    ;
      '$default_args'(Query, FullQuery),
      '$type_info'(_, rel, _, FullQuery, _)
    ;
      '$default_args'(Query, FullQuery),
      '$type_info'(_, act, _, FullQuery, _)
    ;
      compound(Query), functor(Query, F, _), F == '&'
    ;
      compound(Query), functor(Query, F, _), F == ';'    
    ), !,
    '$check_goal_expressions'(Query, Query, Vars, any).
%% Higher order relations
'$check_query'(Query, Vars) :-
    '$get_function_type_info'(Query, _Args, _Types, RT),
    RT == rel,
    !,
    '$check_expressions'(Query, Query, Vars).
'$check_query'(Query, Vars) :-
    '$check_goal_expressions'(Query, Query, Vars, any),
    fail.
    
%% check and then compile query to QP
'$translate_query'(Query, TQuery) :-
    ip_set('$tel_vars', []),
    freeze_term(Query),
     (
       '$is_goal_term'(Query)
     ;
       Query = (_ & _)
     ;
       Query = (_ ; _)
     ;
      '$builtin_query'(Query)
     ), !,
     '$translate_goal'(Query, TQuery).

%% update the '__time__ fact with the current time - called when the belief
%% changes and is used as a timestamp on BS updates
'$reset_time_fact' :-
    retract('__time__'(_)), !,
    gettimeofday(T),
    %%errornl(reset_time_fact),
    assert('__time__'(T)).
'$reset_time_fact' :-
     gettimeofday(T),
     assert('__time__'(T)).

%% Converta  & or ; sequence to a list
'$goals2list'(G, BL) :-
    compound(G),
    functor(G, F, 2),
    F == '&',
    !,
    G = (B1 & B2),
    '$goals2list'(B1, BL1),
    '$goals2list'(B2, BL2),
    append(BL1, BL2, BL).
'$goals2list'(G, BL) :-
    compound(G),
    functor(G, F, 2),
    F == ';',
    !,
    G = (B1 ; B2),
    '$goals2list'(B1, BL1),
    '$goals2list'(B2, BL2),
    append(BL1, BL2, BL).
'$goals2list'(B, [B]).

%% convert a list of actions to a ; sequence of actions
'$list2actions'([] , '$null_action').
'$list2actions'([A], A) :- !.
'$list2actions'([A|As], (A ; AsA)) :-
    '$list2actions'(As, AsA).

'$flatten_actions'(A, Flat, Clause, VarNames) :-
    '$goals2list'(A, As),
    '$group_alts'(As, AsR, _, Clause, VarNames),
    '$list2actions'(AsR, Flat).

%% For all of the action constructs that have alternative paths we
%% flatten the alternatives so that actions that follow a construct
%% with alternative the trailing actions are pushed into each branch
%% This produces a tree of alternatives that is used by the type checker
%% to check each branch is type correct.
%%'$group_alts'(A, B, C, D, E) :- errornl('$group_alts'(A, B, C, D, E)),fail.
'$group_alts'([], [], _, _, _).
'$group_alts'([Call|Rest], Result, HasAlt, Clause, VarNames) :-
    compound(Call),
    functor(Call, F, _),
    var(F), !,
    '$group_alts'(Rest, GRest, HasAlt, Clause, VarNames),
    Result = [Call|GRest].
'$group_alts'([Receive|Rest], [Result], true, Clause, VarNames) :-
    compound(Receive),
    functor(Receive, receive, _), !,
    '$group_alts'(Rest, GRest, _HasAlt, Clause, VarNames),
    Receive = receive(Alts),
    '$push_into_receive'(Alts, GRest, GAlts, Clause, VarNames),
    Result = receive(GAlts).
'$group_alts'([Wait|Rest], [Result], true, Clause, VarNames) :-
    compound(Wait),
    functor(Wait, wait, _),
    Wait = wait(Goals, Preds, '$wait_timeout'(T, Act)),!,
    '$group_alts'(Rest, GRest, _HasAlt, Clause, VarNames),
    '$list2actions'(GRest, RestA),
    '$flatten_actions'((Act;RestA), GAct, Clause, VarNames),
    '$flatten_actions'(('$rel_escape'(Goals);RestA), G, Clause, VarNames),
    Result = wait(G, Preds, '$wait_timeout'(T, GAct)).
'$group_alts'([TryExcept|Rest], [Result], true, Clause, VarNames) :-
    compound(TryExcept),
    functor(TryExcept, try_except, _), !,
    '$group_alts'(Rest, GRest, _HasAlt, Clause, VarNames),
    TryExcept = try_except(Try, Excepts),
    '$push_into_except'(Excepts, GRest, GExcepts, Clause, VarNames),
    '$list2actions'(Rest, RestA),
    '$flatten_actions'((Try;RestA), GTry, Clause, VarNames),
    Result = try_except(GTry, GExcepts).
'$group_alts'([WaitCase|Rest], [Result], true, Clause, VarNames) :-
    compound(WaitCase),
    functor(WaitCase, wait_case, _), !,
    '$group_alts'(Rest, GRest, _HasAlt, Clause, VarNames),
    WaitCase = wait_case(AltCases),
    '$group_alts'(Rest, GRest, _, Clause, VarNames),
    '$push_into_alts'(AltCases, GRest, GAltCases, Clause, VarNames),
    Result = wait_case(GAltCases).
'$group_alts'([Case|Rest], [Result], true, Clause, VarNames) :-
    compound(Case),
    functor(Case, case, _), !,
    '$group_alts'(Rest, GRest, _HasAlt, Clause, VarNames),
    Case = case(AltCases),
    '$group_alts'(Rest, GRest, _, Clause, VarNames),
    '$push_into_alts'(AltCases, GRest, GAltCases, Clause, VarNames),
    Result = case(GAltCases).
'$group_alts'([Forall|Rest], Result, HasAlt, Clause, VarNames) :-
    Forall = '$forall_actions'(QVars, LHS, RHS), !,
    '$flatten_actions'(RHS, FRHS, Clause, VarNames),
    '$group_alts'([FRHS], _, SubHasAlt, Clause, VarNames),
    (
      var(SubHasAlt)
    ->
      '$group_alts'(Rest, GRest, HasAlt, Clause, VarNames),
      Result = ['$forall_actions'(QVars, LHS, RHS)|GRest]
    ;
      '$display_error'(alt_in_forall(Forall, Clause, VarNames))
    ).
'$group_alts'([Call|Rest], Result, HasAlt, Clause, VarNames) :-
    '$group_alts'(Rest, GRest, HasAlt, Clause, VarNames),
    Result = [Call|GRest].

'$push_into_except'([], _, [], _, _).
'$push_into_except'(['$catch'(Patt, Test, Act)|Excepts], [],
                    ['$catch'(Patt, Test, GAct)|GExcepts], Clause, VarNames) :-
    !,
    '$flatten_actions'(Act, GAct, Clause, VarNames),
    '$push_into_except'(Excepts, [], GExcepts, Clause, VarNames).
'$push_into_except'(['$catch'(Patt, Test, Act)|Excepts], Rest,
                    ['$catch'(Patt, Test, GAct)|GExcepts], Clause, VarNames) :-
    '$list2actions'(Rest, RestA),
    '$flatten_actions'((Act;RestA), GAct, Clause, VarNames),
    '$push_into_except'(Excepts, Rest, GExcepts, Clause, VarNames).

'$push_into_receive'([], _, [], _, _).
'$push_into_receive'(['$normal'('$receive_from'(M, A, T, Act))|Alts], [],
                     ['$normal'('$receive_from'(M, A, T, GAct))|GAlts],
                     Clause, VarNames) :-
    !,
    '$flatten_actions'(Act, GAct, Clause, VarNames),
    '$push_into_receive'(Alts, [], GAlts, Clause, VarNames).
'$push_into_receive'(['$normal'('$receive_from'(M, A, T, Act))|Alts], Rest,
                     ['$normal'('$receive_from'(M, A, T, GAct))|GAlts],
                     Clause, VarNames) :-
    '$list2actions'(Rest, RestA),
    '$flatten_actions'((Act;RestA), GAct, Clause, VarNames),
    '$push_into_receive'(Alts, Rest, GAlts, Clause, VarNames).
'$push_into_receive'([timeout(T, Act)|Alts], [],
                     [timeout(T, GAct)|GAlts], Clause, VarNames) :-
    !,
    '$flatten_actions'(Act, GAct, Clause, VarNames),
    '$push_into_receive'(Alts, [], GAlts, Clause, VarNames).
'$push_into_receive'([timeout(T, Act)|Alts], Rest,
                     [timeout(T, GAct)|GAlts], Clause, VarNames) :-
    '$list2actions'(Rest, RestA),
    '$flatten_actions'((Act;RestA), GAct, Clause, VarNames),
    '$push_into_receive'(Alts, Rest, GAlts, Clause, VarNames).

%%'$push_into_alts'(A, B, C) :- errornl('$push_into_alts'(A, B, C)),fail.
'$push_into_alts'([], _GRest, [], _, _).
'$push_into_alts'([timeout(T, Act)], GRest, [timeout(T, GAct)],
                  Clause, VarNames) :-
    !,
    '$list2actions'(GRest, AGRest),
    '$flatten_actions'((Act;AGRest), GAct, Clause, VarNames).
'$push_into_alts'(['$case_alt'(Test, Act)|AltCases], GRest,
                  ['$case_alt'(Test, GAct)|GAltCases], Clause, VarNames) :-
   '$list2actions'(GRest, AGRest),
   '$flatten_actions'((Act;AGRest), GAct, Clause, VarNames),
   '$push_into_alts'(AltCases, GRest, GAltCases, Clause, VarNames).

%% Qulog wait is compiled into qulog_wait that is implemented below
%%'$qulog_wait'(A, B, C) :- errornl('$qulog_wait'(A, B, C)),fail.
%% waiting on G but without predicate info or timeout
'$qulog_wait'(G, '$empty', '$empty') :-
    !,
    thread_wait_on_goal(G).
%% wait has a timeout
'$qulog_wait'(G, '$empty', '$wait_timeout'(Time, Act)) :-
    !,
    (
      thread_wait_on_goal(G, [wait_for(Time)])
    ->
      %% Goal succeeded
      true
    ;
      %% timout - call timeout goal
      call(Act)
    ).
%% wait with timeout and pred info
'$qulog_wait'(G, Preds, '$wait_timeout'(Time, Act)) :-
    !,
    '$wait_trans_preds'(Preds, TPreds),
    (
      thread_wait_on_goal(G, [wait_preds(TPreds), wait_for(Time)])
    ->
      true
    ;
      call(Act)
    ).
%% wait with pred info
'$qulog_wait'(G, Preds, '$empty') :-
    '$wait_trans_preds'(Preds, TPreds),
    thread_wait_on_goal(G, [wait_preds(TPreds)]).

%% translate qulog pred info - .i.e. '$plus'(P), '$minus'(P), P
%% to +P/N, -P/N, P/N where N is teh arity of P to be used at the
%% Qp level for waiting
'$wait_trans_preds'([], []).
'$wait_trans_preds'(['$plus'(P)|Preds], [(+P/N)|TPreds]) :-
    !,
    '$get_pred_arity'(P, N),
    '$wait_trans_preds'(Preds, TPreds).
'$wait_trans_preds'(['$minus'(P)|Preds], [(-P/N)|TPreds]) :-
    !,
    '$get_pred_arity'(P, N),
    '$wait_trans_preds'(Preds, TPreds).
'$wait_trans_preds'([P|Preds], [(P/N)|TPreds]) :-
    '$get_pred_arity'(P, N),
    '$wait_trans_preds'(Preds, TPreds).

'$get_pred_arity'(P, N) :-
    '$type_info'(P, _, _, Pred, _),
    '@functor'(Pred, _, N).

%%'$add_input'(X, '!'(X)).

%% flatten a type list by unfolding union types 
'$flatten_type_list'([], []).
'$flatten_type_list'([H|T], F) :-
    (
      nonvar(H),H = '$union_type'(AL)
    ->
      '$flatten_type_list'(AL, HF),
      append(HF, F1, F)
    ;
      F = [H|F1]
    ),
    '$flatten_type_list'(T, F1).





%% used in an exists query to remove choice points if no variables in the
%% query other than the quantified variables will be bound.
free_vars_(Term, Vars) :-
    ground(Term), !, Vars = [].
free_vars_(Term, Vars) :-
    var(Term), !,
    Vars = [Term].
free_vars_('$forall'(_, _, _), Vars) :-
    !, Vars = [].
free_vars_('$pfindall'(_, _, _), Vars) :-
    !, Vars = [].
free_vars_('$forall_actions'(_, _, _), Vars) :-
    !, Vars = [].
free_vars_('$exists'(_, _), Vars) :-
    !, Vars = [].
free_vars_('$list_constr'(_, _), Vars) :-
    !, Vars = [].
free_vars_('$set'(_, _), Vars) :-
    !, Vars = [].
free_vars_(List, Vars) :-
    list(List), !,
    free_vars_list(List, Vars).
free_vars_(Compound, Vars) :-
    Compound =.. List,
    free_vars_list(List, Vars).

free_vars_list([], []).
free_vars_list(L, Vars) :-
    var(L), !,
    Vars = [L].
free_vars_list([H|T], Vars) :-
    free_vars_(H, HV),
    free_vars_list(T, TV),
    union_list(HV, TV, Vars).


%% used by the type checker when type checking a query to see if we are
%% type checking an action seq or a relational seq
'$has_action_query'(QueryList) :-
    member(Q0, QueryList),
    add_default_args__(Q0, Q),
    (
    '$type_info'(_, act, _, Q, _)
     ;
     Q = receive(_)
    ;
     Q = case(_)
    ;
     Q = try_except(_, _)
    ;
     Q = repeat_until(_, _)
    ;
     Q = atomic_action(_)
    ;
     Q = from(_,_,_)
    ;
     Q = from_thread(_,_,_)
    ;
      Q = '$rel_escape'(_)
    ;
      Q = wait(_)
    ;
      Q = wait_case(_)
    ;
      Q = '$forall_actions'(_,_,_)
    ;
      Q = '?'(_)
    ),!.



is_nat_(N) :- integer(N), N >= 0.



%% display the current belief store
'$show_bs' :-
    findall(PX, '$belief_type_info'(PX, _, _), PS),
    sort(PS, PSS),
    member(P, PSS),
    show_belief_clause_(P),
    fail.
'$show_bs'.

%% treat atom as a zero arity compound - used in consult
'$real_functor'(C, F, N) :-
    \+compound(C), !, F = C, N = 0.
'$real_functor'(C, F, N) :-
    C = F(X), X == '$none_', !,
    N = 0.
'$real_functor'(C, F, N) :-
    functor(C, F, N).


%% used in write to tell if T is a compound whose functor is an operator
%% e.g.  In A + B  + is an operator
'$has_operator'(T) :-
    compound(T),
    functor(T, F, _),
    open_string(write, S),
    writeT_compound(S, T),
    stream_to_string(S, Text),
    string_to_atom(FS, F),
   \+ (string_concat(FS, Rest, Text), string_concat("(", _, Rest)).

%% used in qlg2ql
'$atom2string'(Atom, String) :-
    string_to_atom(String, Atom).


%% Used in consult to check for overlapping constructor types.

'$not_identical_mod_vars'(T1, T2) :-
    freeze_term(T1), T1 \= T2.
'$not_identical_mod_vars'(T1, T2) :-
    freeze_term(T2), T1 \= T2.


'$type_descr'(A, B, C, D) :- '$user_type_descr'(A, B, C, D).
'$type_descr'(A, B, C, D) :- '$builtin_type_descr'(A, B, C, D).


