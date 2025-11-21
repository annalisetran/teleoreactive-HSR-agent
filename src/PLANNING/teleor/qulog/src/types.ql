% Copyright 2014 Keith Clark, Peter Robinson
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
?- dynamic('$saved_type_info'/1).
?- dynamic(checking_inner_bangs__/0).

%% '$simplify_types'(Types, SimpTypes)
%% When determining the type of a term we often produce a set of types for
%% the term. This predicate takes a list of types (Types) and attempts to
%% simplify the types by removing subtypes and by replacing types by
%% defined types. If the list of types reduce to a single type than that is
%% returned. Otherwise a simplified union type is returned.

'$simplify_types'(Group, T) :-
    %% Because the types can contain type variables we freeze the variables
    %% so they don't get instantiated to other types.
    freeze_term((Group, T), FVars),
    %% recursively flatten any union types
    '$flatten_type_list'(Group, FGroup),
    %% use sort to remove duplicates and to group like constructor types
    sort(FGroup, SGroup),
    %% remove subtypes (using the includes predicate)
    '$remove_includes'(SGroup, SGroup, IGroup),
    %% where possible replace a union type by a defined type
    '$lookForDefinedType'(IGroup, DGroup),
    thaw_term(FVars),
    sort(DGroup, SimpGroup),
    (
    SimpGroup = [T]
    ->
     true
    ;
     %'$list2tuple'(DGroup, Disj),
     T = '$union_type'(SimpGroup)
    ).

'$remove_includes'([], _SGroup, []).
'$remove_includes'([T|G], SGroup, IGroup) :-
    '$remove_includes'(G, SGroup, IGroup1),
    (
      member(T1, SGroup), T \== T1, includes__(T1, T)
    ->
      IGroup = IGroup1
    ;
      IGroup = [T|IGroup1]
    ).

'$lookForDefinedType'(UTypes,[atomic|UTypes4]) :-
    list(UTypes),
    delete(atom, UTypes, UTypes1),
    delete(num, UTypes1, UTypes2),
    delete(string, UTypes2, UTypes3), !,
    '$lookForDefinedType'(UTypes3, UTypes4).
'$lookForDefinedType'(UTypes,RUTypes) :-
    list(UTypes),
    '$defined_type'(Type, '$union_type'(OTypes)),
    forall( member(T,OTypes), member(T,UTypes)),
    !,
    findall(X, (member(X, UTypes), \+member(X, OTypes)), RUTypes0),
    '$lookForDefinedType'([Type|RUTypes0], RUTypes).
'$lookForDefinedType'(UTypes,UTypes). 

%% For recursive polymorphic types (such as list or tree) with an
%% empty constructor then we set the type for occurrences of the empty
%% constructor to be the empty type. EG
%% [] : list('$empty_type')
%% This type is a subtype of every type so if we have a non-empty list
%% then the type of the list is purely based on the types of the elements
%% of the list. However, in the interpreter where we display the type
%% of the values then, for the empty list we prefer
%% [] : list(_) instead of [] : list('$empty_type')
%% The predicate below is used as the relation in a call to
%% transform_simple_terms in the interpreter when printing types
'$strip_empty_type'(A, B) :-
    atom(A), A = '$empty_type', !, B = '$$var$$'('_').
'$strip_empty_type'(A, A).


%% type(Term, Type) when used in the body of a rule tests if Term is of the
%% supplied type.
type(Term, Type) :-
    moded__(Type), !,
    has_type__(Term, Type, [], [], _, [], OLE, [], [], 0, Term, Error),
    bind_type_constraints__(OLE, Error),
    var(Error).
type(Term, Type) :-
    has_type__(Term, '!'(Type), [], [], _, [], OLE, [], [], 0, Term, Error),
    bind_type_constraints__(OLE, Error),
    var(Error).

% type(Term, Type, A) :-
%     moded__(Type), !,
%     has_type__(Term, Type, [], [], A, [], OLE, [], _, 0, Term, Error),
%     bind_type_constraints__(OLE, Error),
%     var(Error).
% type(Term, Type, A) :-
%     has_type__(Term, '!'(Type), [], [], A, [], OLE, [], _, 0, Term, Error),
%     bind_type_constraints__(OLE, Error),
%     var(Error).
      

allowed_rel_call(Term) :-
    has_type__(Term, '!??'(rel_term), [], [], _, [], OLE, [], [], 0, Term, Error),
    bind_type_constraints__(OLE, Error),
    var(Error).
    
allowed_dyn_call(Term) :-
    has_type__(Term, '!??'(dyn_term), [], [], _, [], OLE, [], [], 0, Term, Error),
    bind_type_constraints__(OLE, Error),
    var(Error).

allowed_act_call(Term) :-
    has_type__(Term, '!??'(act_term), [], [], _, [], OLE, [], [], 0, Term, Error),
    bind_type_constraints__(OLE, Error),
    var(Error).



wrap_var_functor__(A, _(A)).


%% '$add_annotations'(InTypes, OutTypes, Annotation)
%% adds mode  Annotation to elements of InTypes to get OutTypes
'$add_annotations'([], [], _).
'$add_annotations'([T|Types], [AT|ATypes], Ann) :-
    (
     var(T)
    ->
     AT = Ann(T)
    ;
     '$has_annotation'(T)
    ->
      '$add_annotations_to_type'(T, AT)
    ;
      '$add_annotations_to_type'(T, AT1),  AT = Ann(AT1)
    ),
    '$add_annotations'(Types, ATypes, Ann).

'$has_annotation'('??'(_)).
'$has_annotation'('?'(_)).
'$has_annotation'('!??'(_)).
%'$has_annotation'('!?'(_)).
'$has_annotation'('+?'(_)).
'$has_annotation'('!'(_)).
%'$has_annotation'('@'(_)).


% '$unify_all'([], _).
% '$unify_all'([X|R], X) :-
%     '$unify_all'(R, X).

%% When managing type LE constraints we sometimes find a cycle of constraints
%% such as  T1 =< T2, T2 =< T3, T3 =< T1
%% When this happens then all the types have to be the same - either unify
%% or represent the same type (e.g. one is a macro for the other).
%% unify_all_types tries to make all the types the same using unification
%% which will bind type variables.
'$unify_all_types'([], _).
'$unify_all_types'([X|R], Y) :-
    '$identical_types'(X, Y), !,
    '$unify_all_types'(R, X).
'$unify_all_types'([X|R], X) :-
    '$unify_all_types'(R, X).


%% Used by the interpreter to check the modes and types of Query
%% Vars are the global variables of the query.
%% Kind TODO
%%check_query_mode_type__(Query, Vars, _Kind) :- errornl(check_query_mode_type__(Query, Vars, _Kind)),fail.
check_query_mode_type__(Query, Vars, _Kind) :-
    var(Query), !,
    global_state_set(num_errors, 0),
    check_query_relation_mode_type__(Query, Vars, _OutVMT).
check_query_mode_type__(Query, Vars, _Kind) :-
    compound(Query), functor(Query, F, _), var(F), !,
    global_state_set(num_errors, 0),
    check_query_relation_mode_type__(Query, Vars, _OutVMT).
check_query_mode_type__(once(Query), Vars, Kind) :-
    !,
    check_query_relation_mode_type__(Query, Vars, OutVMT),
     Kind = rel(OutVMT).
check_query_mode_type__(Query, Vars, Kind) :-
    '$default_args'(Query, FullQuery), !,
    check_query_mode_type__(FullQuery, Vars, Kind).
%check_query_mode_type(wait(_, _, _), _, act(_)) :- !.
check_query_mode_type__(Query, Vars, Kind) :-
    %global_state_set('$error_level', 1),
    global_state_set(num_errors, 0),
    '$goals2list'(Query, QueryList),
    QueryList = [G|_],
    (
      '$has_action_query'(QueryList)
    ->
      check_query_action_mode_type__(Query, Vars, QueryInfo), 
     Kind = act(QueryInfo)
    ;     
      ('$is_goal_term'(G) ; Query = (_ & _) )
    ->
      check_query_relation_mode_type__(Query, Vars, QueryInfo),
      Kind = rel(QueryInfo)
    ),
    global_state_lookup(num_errors, 0).

%% Type check relational queries. Because relations etc can have multiple
%% type/mode declarations then we use findall to try all possibilities
%% some of these will generate type/mode errors. If all alternatives
%% produce errors then we generate a type/mode error.
check_query_relation_mode_type__(Query, VarNames, TypeInfo) :-
    '$goals2list'(Query, QueryL),
    thaw_term(QueryL),
    ip_lookup('$underscore_vars', Unders),
    check_globals_locals__(QueryL, [], _G, [], _L, Unders, full, ErrGL),
    RuleInfo = rule_info(rel_query, Query, VarNames, Unders),
    report_error__(globals_locals, query, QueryL, ErrGL, RuleInfo),
    findall(err__(Error, query, query, QueryL),
            (
              type_check_body__(QueryL, rel, [], _OutVTB, Unders, [], LE, [],
                                _OutUnif, Err, []),
               bind_type_constraints__(LE, Err),
              check_error__(Err, body, Error)
            ),
            Errors),
    bind_error_vars__(Errors, query, QueryL),
    process_errors__(Errors, RuleInfo),
    once((member(err__(E, QueryInfo, _, _), Errors), not_an_error__(E))),
    TypeInfo = QueryInfo.

not_an_error__(E) :- var(E), !.
not_an_error__(body(body_call_error(false( _), false_always_fails(_)))).

%% Ditto for action queries.
check_query_action_mode_type__(Query, VarNames, TypeInfo) :-
    '$goals2list'(Query, QueryL),
    ip_lookup('$underscore_vars', Unders),
    ip_set('$act_seen', []),
    check_globals_locals__(QueryL, [], _G, [], _L, Unders, full, ErrGL),
    RuleInfo = rule_info(act_query, Query, VarNames, Unders),
    report_error__(globals_locals, query, QueryL, ErrGL, RuleInfo),
    findall(err__(Error, query, query, QueryL),
            (
              type_check_body__(QueryL, act, [], _OutVTB, Unders, [], _, [],
                                _OutUnif, Err, []),
              %add_unifs_to_VTB__(OutUnif, OutVTB, OutUnif, VTB, Err),
              check_error__(Err, body, Error)
            ),
            Errors),
    bind_error_vars__(Errors, query, QueryL),
    process_errors__(Errors, RuleInfo),
    %once((member(err__(E, QueryInfo, _, _), Errors), var(E))),
    TypeInfo = query.


%% pcovers__(T1, T2) is true if T1 and T2 are "primitive" types and
%% T1 >= T2
pcovers__(term, _) :- !.
pcovers__(_, '$empty_type') :- !.
pcovers__(term, atomic) :- !.
pcovers__(atom, atom_naming(_)) :- !.
pcovers__(atomic,int) :- !.
pcovers__(atomic,nat) :- !.
pcovers__(atomic,atom) :- !.
pcovers__(atomic,num) :- !.
pcovers__(atomic,string) :- !.
pcovers__(num,int) :- !.
pcovers__(num,nat) :- !.
pcovers__(int,nat) :- !.
pcovers__(dyn_term, tel_percept_term) :- !.
pcovers__(_, bottom) : !.

%% covers__(T1, T2) is true if T1 >= T2
%%covers__(T1, T2) :- errornl(covers__(T1, T2)),fail.
covers__(T1, T2) :- T1 == T2, !.
covers__(T1,_T2) :- var(T1), !, fail.
covers__(_T1,T2) :- var(T2), !, fail.
covers__(T1,T2) :- pcovers__(T1,T2), !.
covers__('$constr_enum_type'(T1), '$constr_enum_type'(T2)) :-
    !,
    forall(member(X, T2), member(X, T1)).
covers__('$enum_type'(T1), '$enum_type'(T2)) :-
    !,
    forall(member(X, T2), member(X, T1)).
covers__(atom, '$enum_type'(List)):-
    List = [A|_],
    atom(A),
    !,
    forall( member(A,List), atom(A)).
covers__(string, '$enum_type'(List)):-
    List = [A|_],
    string(A),
    !,
    forall( member(A,List), string(A)).
covers__(num, '$enum_type'(List)):-
    List = [A|_],
    number(A),
    !,
    forall( member(A,List), number(A)).
covers__(int, '$enum_type'(List)):-
    List = [A|_],
    integer(A),
    !,
    forall( member(A,List), integer(A)).
covers__(nat, '$enum_type'(List)):-
    List = [A|_],
    is_nat_(A),
    !,
    forall( member(A,List), is_nat_(A)).
covers__(A, B) :-
    '$defined_type'(A, AD), !,
    covers__(AD, B).
covers__(A, B) :-
    '$defined_type'(B, BD), !,
    covers__(A, BD).
covers__(A, '$union_type'(List)):-
    is_primitive_or_defined_type_(A),   
    !,
    forall( member(E,List), includes__(A,E) ).
covers__('$union_type'(List), A):-
    is_primitive_or_defined_type_(A),    
    !,
    once(( member(E,List), includes__(E,A) )).
covers__('$union_type'(Types),atomic) :-
    !,
    forall( member(T,[atom,int,num,string]),
             (member(ST,Types),includes__(ST,T))).
covers__('$union_type'(List),A):-
    member(E,List), includes__(E,A). 
covers__('$union_type'(List1),'$union_type'(List2)):-
    forall( member(E2,List2), 
            (member(E1,List1), once(includes__(E1,E2)) )
	  ).
covers__('$tuple_type'(List1), '$tuple_type'(List2)) :-
    covers_list__(List1, List2).
%%covers__('$tuple_type'(List), tuple(T)) :-
%%    covers__('$union_type'(List), T).
covers__((T1A -> T1B), (T2A -> T2B)) :-
    !,
    covers__(T1B, T2B),
    covers__(T2A, T1A).
covers__((N1 .. N2), '$enum_type'(List)) :-
    List = [A|_], integer(A), !,
    forall(member(M, List), (N1 =< M, M =< N2)).
covers__(num,(_ .. _)).
covers__(nat,(A .. _)) :- A >= 0.
covers__(int,(_ .. _)).
covers__(atomic,(_ .. _)).
covers__((N1 .. M1), (N2 .. M2)):-
	N1=<N2,
	M1>=M2.
covers__(T1, T2) :-
    constructor_type__(T1, Constr, Types1), !,
    constructor_type__(T2, Constr, Types2),
    covers_list__(Types1, Types2).


covers_list__([], []).
covers_list__([T1|List1], [T2|List2]) :-
    includes__(T1, T2),
    covers_list(List1, List2).

%% includes__(T1, T2) is true if T1 >= T2
%% this is the same as covers except for unfolding of definitions
%%includes__(T1,T2) :- errornl(includes__(T1,T2)),fail.
includes__(T1,T2) :-
    T1==T2, !.			% identical types. 
includes__(T1,T2) :-  % when T1 and T2 are two instances of same constructor type
    nonvar(T1), nonvar(T2), ( compound(T1) ; compound(T2) ),
    '$defined_type'(T1, _),
    '$defined_type'(T2, _),
    !,
    '@..rev'(T1, F, ArgTs1),
    '@..rev'(T2, F, ArgTs2),
     map(includes__, ArgTs1, ArgTs2).
includes__(T1,T2) :-  % when T1 and T2 both primitive
    nonvar(T1), nonvar(T2),
    pcovers__(T1,T2),
    !. 
includes__(T1,T2) :-  
    %% not same types and not diff instances of same poly. type
    %% get their descriptions if defined types, else use type name
    get_def__(T1,DefT1),
    get_def__(T2,DefT2),
    covers__(DefT1,DefT2).

%% Unfold possible definition
get_def__(T,DefT) :-
    var(T), !, DefT = T.
get_def__(T,DefT) :-
    '$defined_type'(T, DefT),
    !.
get_def__(T,DefT):-
    T=DefT.

%% '$smallest_enum_type_for'(X, EnumType)
%% Provided X is in some enumerated type then EnumType is the smallest
%% such type.
'$smallest_enum_type_for'(X, EnumType) :-
    findall(f(N, Type), '$is_in_enum_type'(X, Type, N), Pairs),
    sort(Pairs, Sorted),
    Sorted = [f(_, EnumType)|_].

%% '$is_in_enum_type'(X, Type, Size) is true if X in the enumerated type Type
%% that contains Size elements
'$is_in_enum_type'(X, Type, Size) :-
    integer(X), '$defined_type'(Type, (N..M)),
    between(N, M, X),
    Size is M - N + 1.
'$is_in_enum_type'(X, Type, Size) :-
    '$defined_type'(Type,'$enum_type'(E)),
    member(X, E),
    length(E, Size).

%% The same as above but for enumerated constructor types
'$smallest_constr_enum_type_for'(X, EnumType) :-
    findall(f(N, Type), '$is_in_constr_enum_type'(X, Type, N), Pairs),
    sort(Pairs, Sorted),
    Sorted = [f(_, EnumType)|_].
'$is_in_constr_enum_type'(X, Type, Size) :-
    '$defined_type'(Type,'$constr_enum_type'(E)),
    member(TX, E),
    functor(X, F, N),
    functor(TX, F, N),
    length(E, Size).

%% has_type2(Term, Type, InVTB) Term has type Type WRT the variable
%% type binding InVTB - used to determine Type (type inference)
has_type2__(Term, Type, InVTB) :-
    ip_lookup('$call_kind', Kind),
    ip_set('$call_kind', rel),
    %% change to rel call_kind for the test
    has_type__(Term, '?'(Type), InVTB, InVTB, _OutVTB, [], OutLE,
               [], [], 0, Term, Error),
    bind_type_constraints__(OutLE, Error),
    %% revert to old call_kind
    ip_set('$call_kind', Kind),
    var(Error).

%% has_moded_type__(Term, Type, MType, CurrVTB, InVTB, InLE, Unif, Unders)
%% Used to generate the correct moded type for Term for producing
%% a mode error. Type is the known unmoded type of Term
has_moded_type__(Term, Type, MType, CurrVTB, InVTB, InLE, Unif, Unders) :-
    strip_modes__(Type, SType),
    try_push_modes_in__(SType, MType),
    ip_lookup('$call_kind', Kind),
    ip_set('$call_kind', rel),
    %% Change to call_kind for test
    has_type__(Term, MType, CurrVTB, InVTB, _, InLE, _, Unif,
               Unders, 0, Term, Error),
    ip_set('$call_kind', Kind),
    %% revert to old call_kind
    var(Error), !.

%% try_push_modes_in__(DT, R), on backtracking generates R as a moded
%% version of type DT. The generated moded types are generated in order
%% from the most restricted mode (!) to the less restricted mode (?)
try_push_modes_in__(DT, R) :-
    compound(DT), functor(DT, F, _), F == '$$var$$', !,
    '$try_push_mode_choice'(M),
    R = M(DT).
try_push_modes_in__(DT, R) :-
    compound(DT), functor(DT, '->', 2), !,
    '$try_push_mode_choice'(M),
    R = M(DT).
try_push_modes_in__(DT, R) :-
    compound(DT), functor(DT, F, 1), '$code_kind'(F), !,
    '$try_push_mode_choice'(M),
    R = M(DT).
try_push_modes_in__(DT, M('$tuple_type'(MTypes))) :-
    compound(DT), DT = '$tuple_type'(Types), !,
    '$try_push_mode_choice'(M),
    try_push_modes_in_iterate__(Types, MTypes).
try_push_modes_in__(DT, M(DType)) :-
    compound(DT), DT \= '$union_type'(_), \+ moded__(DT), !,
    '@=..'(DT, [F|Args]),
    '$try_push_mode_choice'(M),
    try_push_modes_in_iterate__(Args, MArgs),
    '@=..'(DType, [F|MArgs]).
try_push_modes_in__(T, M(T)) :-
    '$try_push_mode_choice'(M).
    
try_push_modes_in_iterate__([], []).
try_push_modes_in_iterate__([A|Args], [MA|MArgs]) :-
    try_push_modes_in__(A,MA),
    try_push_modes_in_iterate__(Args, MArgs).

'$try_push_mode_choice'('!').
'$try_push_mode_choice'('?').

%% '$known_call_type'(Type, ArgTypes, Kind) is true
%% if Type is a code type, ArgTypes is the list of types of the arguments
%% of Type and Kind is the kind of code (rel, act, tel)
%%'$known_call_type'(A,B,C) :- errornl('$known_call_type'(A,B,C)),fail.
'$known_call_type'(VT, T, Kind) :-
    '$type_info'(_, macro_type, VT, MacroVT, _), !,
    '$known_call_type'(MacroVT, T, Kind).
'$known_call_type'(atom_naming(VT), T, Kind) :- !,
    '$known_call_type'(VT, T, Kind).
'$known_call_type'(term_naming(VT), T, Kind) :- !,
    '$known_call_type'(VT, T, Kind).
'$known_call_type'(alt_types(AT), T, Kind) :-
    moded__(AT), !,
    AT = _(AT1),
    member(T1, AT1),
    '$known_call_type'(T1, T, Kind).
'$known_call_type'(alt_types(AT), T, Kind) :-
    !,
    member(T1, AT),
    '$known_call_type'(T1, T, Kind).
'$known_call_type'(VT, T, Kind) :-
    moded__(VT), !,
    VT = _(VT1),
    '$known_call_type'(VT1, T, Kind).
'$known_call_type'(Kind('$tuple_type'(T)), T, Kind).


%% In QP atomic does not include strings but Qulog does
'$atomic_or_string'(A) :- atomic(A), !.
'$atomic_or_string'(A) :- string(A).


%% Used in  redo_type_le for sort order
%% This first clause is this way so that le constraints with the same
%% second element will be put together - this simplifies
%% bind_type_constraints_aux (in particular collect_same_2_le)
order_le__(le(A, B), le(C, D)) :-
    !,
    le(B,A) @< le(D, C).
order_le__(A, B) :-
    A @< B.

%% type_le__(A, B) is true if A =< B without leaving any type constraints behind
type_le__(A, B) :-
    type_le__(A, B, [], []).

%% type_le__(T1, T2, InConstr, OutConstr) tests then adds le(T1, T2) to the
%% constraints InConstr and simplifies to OutConstr
%% If either T1 is definitely not le T2 or the simplification fails then
%% we conclude not T1 =< T2
%%type_le__(T1, T2, InConstr, OutConstr) :- errornl(type_le________(T1, T2, InConstr, OutConstr)),fail.
type_le__(T1, T2, InConstr, OutConstr) :-
    strip_modes__(T1, ST1),
    strip_modes__(T2, ST2),
    type_le_aux__(ST1, ST2, InConstr, MidConstr),
    redo_type_le__(MidConstr, OutConstr).

%%redo_type_le__(InConstr, OutConstr) :- errornl(redo_type_le_______(InConstr, OutConstr)),fail.

%% redo_type_le__(InConstr, OutConstr) repeatedly tries to simplify the
%% constraints InConstr - when no more simplifications are possible we
%% return the update constraints OutConstr. If there are inconsistencies
%% then this predicate fails - i.e. there are no solutions for InConstr
redo_type_le__(InConstr, OutConstr) :-
    sort(InConstr, SConstr, order_le__),
    redo_type_le_aux__(SConstr, MidConstr),
    sort(MidConstr, SMidConstr, order_le__),
    (
     SConstr == SMidConstr
    ->
     OutConstr = SConstr %% stop when no (more) simplification possible
    ;
     redo_type_le__(SMidConstr, OutConstr)
    ).

%% redo_type_le_aux__(InConstr, OutConstr) makes one pass through attempting to
%% simplify InConstr returning the constraints OutConstr.
%% Fails if there are no solutions for the constraints InConstr
%%redo_type_le_aux__(InConstr, OutConstr) :-errornl(redo_type_le_aux__(InConstr, OutConstr)),fail.
redo_type_le_aux__(InConstr, OutConstr) :-
    member(le(T1, T2), InConstr), '$identical_types'(T1, T2), !,
    %% remove essentially le(T, T) constraints
    diff_list(InConstr, [le(T1, T2)], OutConstr).
redo_type_le_aux__(InConstr, OutConstr) :-
    member(le(T1, T2), InConstr),
    find_a_loop__(InConstr, T1, T2, [], Loop), !,
    %% find a loop i.e. [le(T1, T2), le(T2, T3), ..., le(Tn, T1)]
    %% In this case all the types have to be the same - check if that's the
    %% case unifying any type variables
    %% note that these constraints will be removed in the next round
    once(( T1 = T2 ; '$identical_types'(T1, T2))),
    '$unify_all_types'(Loop, T1),
    OutConstr = InConstr.
%% find chains that start from a ground type and end with a ground type
%% note that all interior chain entries are type variables
%% T1 = End - bind all
%% T1 > End or T1 and End incompatible - fail
%% else leave as is but remove le(T1, T2) and bind T2, ... to Tn 
redo_type_le_aux__(InConstr, OutConstr) :-
    member(le(T1, T2), InConstr), ground(T1),
    find_a_chain__(T2, InConstr, Chain, Tn),
    (
      '$identical_types'(T1, Tn)
    ->
      !,
      '$unify_all_types'(Chain, T1),
      OutConstr = InConstr
    ;
      type_le_aux__(Tn, T1, [], _)
    ->
      !,
      fail
    ;
      type_le_aux__(T1, Tn, [], _)
    ->
      T2 = Tn, '$unify_all_types'(Chain, Tn),
      le1_remove_from__(le(T1, T2), InConstr, OutConstr)
    ;
      !,
      fail
    ).
%% remove le(T1, T2) with T1 == T2
redo_type_le_aux__(InConstr, OutConstr) :-
    le1_remove_from__(le(T1, T2), InConstr, OutConstr),
    '$identical_types'(T1, T2), !.
%% le(T, T1), le(T, T2) and T1 and T2 incompatible -> fail
redo_type_le_aux__(InConstr, _OutConstr) :-
    member(le(T, T1), InConstr), ground(T1),
    member(le(Tx, T2), InConstr), T == Tx, ground(T2),
    \+ is_compatible__(T1, T2), !, fail.
%% le(T1, T2), le(T2, T3), le(T1, T3) -> remove le(T1, T3)
redo_type_le_aux__(InConstr, OutConstr) :-
    member(le(T1, T2), InConstr),
    member(le(T2x, T3), InConstr), T2x == T2,
    member(le(T1x, T3x), InConstr), T1 == T1x, T2 \== T3x,
    '$identical_types'(T3x, T3), !,
    le1_remove_from__(le(T1, T3x), InConstr, OutConstr),
    '$identical_types'(T3x, T3),  !.
%% le(T1, T2), le(T1, T3) and T2 =< T3 -> remove le(T1, T3)
redo_type_le_aux__(InConstr, OutConstr) :-
    member(le(T1, T2), InConstr), ground(T2),
    member(le(T1, T3), InConstr), ground(T3), T2 \= T3,
    diff_list(InConstr, [le(T1, T3)], OutConstr),
    %le1_remove_from__(le(T1, T3), InConstr, OutConstr),
    \+'$identical_types'(T2, T3),
    ground(T3), type_le_aux__(T2, T3, [], _), !.
%% le(T2, T1), le(T3, T1) and T2 =< T3 -> remove le(T2, T1)
redo_type_le_aux__(InConstr, OutConstr) :-
    member(le(T3, T1), InConstr), ground(T3),
    le2_remove_from__(le(T2, T1), InConstr, OutConstr),
    \+'$identical_types'(T2, T3),
    ground(T2), type_le_aux__(T2, T3, [], _), !.
%% le(T1, T2) can be simplified
redo_type_le_aux__(InConstr, OutConstr) :-
    delete(le(T1, T2), InConstr, MidConstr),
    (
      type_le_aux__(T1, T2, [], C)
    ->
      true
    ;
      !,fail
    ),
    C \== [le(T1, T2)], !,
    append(C, MidConstr, OutConstr).
%% two atom_naming that are not compatible - change to atom
redo_type_le_aux__(InConstr, OutConstr) :-
  delete(le(T1, TA), InConstr, MidConstr),
  compound(T1), T1 = atom_naming(T1A),
  delete(le(T2, TB), MidConstr, Mid2Constr),
  compound(T2), T2 = atom_naming(T2A),
  '$identical_types'(TA, TB),
  \+ '$compatible_atom_naming'(T1A, T2A),
  !,
  OutConstr = [le(atom, TA)|Mid2Constr].
%% catchall - no change
redo_type_le_aux__(InConstr, InConstr).

%% find a loop T1 <= T2 <= ... <= Tn <= T1
find_a_loop__(_Constr, T1, T2, InLoop, Loop) :-
    '$identical_types'(T1, T2), !, Loop = InLoop.
find_a_loop__(Constr, T1, T2, InLoop, Loop) :-
    member(le(T3, T4), Constr),
    '$identical_types'(T2, T3), \+member_eq(T4, InLoop), !,
    find_a_loop__(Constr, T1, T4, [T4|InLoop], Loop).

%%find_a_chain__(T2, _InConstr, Chain, Tn) :-errornl(find_a_chain___(T2, _InConstr, Chain, Tn)),fail.
find_a_chain__(T2, _InConstr, Chain, Tn) :-
      ground(T2), !,
      Chain = [], Tn = T2.
find_a_chain__(T2, InConstr, [T4|Chain], Tn) :-
    member(le(T3, T4), InConstr), '$identical_types'(T2, T3), !,
    find_a_chain__(T4, InConstr, Chain, Tn).

%% TA and TB are identical types (one might be a macro type for the other 
'$identical_types'(TA, TB) :-
    strip_modes__(TA, STA),
    strip_modes__(TB, STB),
    includes__(STA, STB),
    includes__(STB, STA).

%% type_le_aux__(T1, T2, InConstr, OutConstr) :
%% check if T1 <= T2 - this might update InConstr - a set of le constraints
%% by adding le(T1, T2) to produce OutConstr

%%type_le_aux__(T1, T2, InConstr, OutConstr) :- errornl(type_le_aux__(T1, T2, InConstr, OutConstr)),fail.

%% strip modes
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    nonvar(T1), moded__(T1), !,
    T1 = _(ST1),
    type_le_aux__(ST1, T2, InConstr, OutConstr).
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    nonvar(T2), moded__(T2), !,
    T2 = _(ST2),
    type_le_aux__(T1, ST2, InConstr, OutConstr).
%% '$empty_type' is le all types
type_le_aux__(T1, _T2, InConstr, OutConstr) :-
    T1 == '$empty_type', !, 
    OutConstr = InConstr.
type_le_aux__(T1, T2, InConstr, OutConstr) :-
  T1 == T2, !,
  OutConstr = InConstr.
type_le_aux__(T1, T2, _InConstr, _OutConstr) :-
    var(T1), \+(T1=T2), !,fail.
type_le_aux__(T1, T2, _InConstr, _OutConstr) :-
    var(T2), \+(T1=T2), !,fail.
%% term is ge all types
type_le_aux__(_T1, T2, InConstr, OutConstr) :-
    T2 == term, !,
    OutConstr = InConstr.
%% dyn_term is le rel_term
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    T1 == dyn_term, T2 == rel_term,
    !,
    OutConstr = InConstr.

type_le_aux__(TT1, T2, InConstr, OutConstr) :-
    nonvar(TT1), nonvar(T2),
    TT1 = term_naming(T1), !,
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(T1, TT2, InConstr, OutConstr) :-
    nonvar(TT2), nonvar(T1),
    TT2 = term_naming(T2), !,
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(TT1, T2, InConstr, OutConstr) :-
    nonvar(TT1), nonvar(T2),
    TT1 = atom_naming(T1), !,
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(T1, TT2, InConstr, OutConstr) :-
    nonvar(TT2), nonvar(T1),
    TT2 = atom_naming(T2), !,
    type_le_aux__(T1, T2, InConstr, OutConstr).


%% When type checking rules for relations (say) with polymorphic
%% types we instantiate the type variables to the term '$$var$$'(Name)
%% where Name is the name of the variable (an atom). Such types
%% are not le another type unless the types are the same
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    ground(T1), T1 = '$$var$$'(_), !,
    T1 = T2,
    OutConstr = InConstr.
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    ground(T2), T2 = '$$var$$'(_), !,
    T1 = T2,
    OutConstr = InConstr.
%% When T2 is a variable type we add to the type constraints
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    var(T2), !,
    OutConstr = [le(T1, T2)|InConstr].
%% unfold macro types
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    nonvar(T1),
    freeze_term(T1, V),
    '$type_info'(_, macro_type, T1, DefT1, _),
    thaw_term(V),
    type_le_aux__(DefT1 , T2, InConstr, OutConstr), !.
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    nonvar(T2),
    freeze_term(T2, V),
    '$type_info'(_, macro_type, T2, DefT2, _),
    thaw_term(V),
    type_le_aux__(T1 , DefT2, InConstr, OutConstr), !.
%% "unfold" code types
type_le_aux__(TA, TB, InConstr, OutConstr) :-
    TB = (T2D -> T2R),
    nonvar(TA),
    TA =  term_naming((T1D -> T1R)),
    !,
    type_le_aux__((T1D -> T1R), (T2D -> T2R), InConstr, OutConstr).
type_le_aux__(TA, rel(T2), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  term_naming(rel(T1)),      
    !,
    type_le_aux__(rel(T1), rel(T2), InConstr, OutConstr).
type_le_aux__(TA, act(T2), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  term_naming(act(T1)),      
    !,
    type_le_aux__(act(T1), act(T2), InConstr, OutConstr).
type_le_aux__(TA, tel(T2), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  term_naming(act(T1)),      
    !,
    type_le_aux__(tel(T1), tel(T2), InConstr, OutConstr).
type_le_aux__(TA, term_naming((T2D -> T2R)), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  term_naming((T1D -> T1R)),      
    !,
    type_le_aux__((T1D -> T1R), (T2D -> T2R), InConstr, OutConstr).
type_le_aux__(TA, term_naming((T2D -> T2R)), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  (T1D -> T1R),      
    !,
    type_le_aux__((T1D -> T1R), (T2D -> T2R), InConstr, OutConstr).
type_le_aux__(TA, term_naming(rel(T2)), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  term_naming(rel(T1)),      
    !,
    type_le_aux__(rel(T1), rel(T2), InConstr, OutConstr).
type_le_aux__(TA, term_naming(act(T2)), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  term_naming(act(T1)),      
    !,
    type_le_aux__(act(T1), act(T2), InConstr, OutConstr).
type_le_aux__(TA, term_naming(tel(T2)), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  term_naming(tel(T1)),      
    !,
    type_le_aux__(tel(T1), tel(T2), InConstr, OutConstr).
type_le_aux__(TA, (T2D -> T2R), InConstr, OutConstr) :-
    nonvar(TA),
    TA = atom_naming((T1D -> T1R)),
    !,
    type_le_aux__((T1D -> T1R), (T2D -> T2R), InConstr, OutConstr).
type_le_aux__(TA, rel(T2), InConstr, OutConstr) :-
    nonvar(TA),
    TA = atom_naming(rel(T1)),
    !,
    type_le_aux__(rel(T1), rel(T2), InConstr, OutConstr).
type_le_aux__(TA, act(T2), InConstr, OutConstr) :-
    nonvar(TA),
    TA = atom_naming(act(T1)),
    !,
    type_le_aux__(act(T1), act(T2), InConstr, OutConstr).
type_le_aux__(TA, tel(T2), InConstr, OutConstr) :-
    nonvar(TA),
    TA = atom_naming(tel(T1)),
    !,
    type_le_aux__(tel(T1), tel(T2), InConstr, OutConstr).
type_le_aux__(TA, atom_naming((T2D -> T2R)), InConstr, OutConstr) :-
    nonvar(TA),
    TA =  atom_naming((T1D -> T1R)),      
    !,
    type_le_aux__((T1D -> T1R), (T2D -> T2R), InConstr, OutConstr).
type_le_aux__(T1, (T2D -> T2R), InConstr, OutConstr) :-
    !,
    T1 = (T1D -> T1R), %% T1 must be a function
    '$strip_top_fun_bangs'(T1D, ST1D),
    '$strip_top_fun_bangs'(T2D, ST2D),
    '$strip_top_fun_bangs'(T1R, ST1R),
    '$strip_top_fun_bangs'(T2R, ST2R),
    %% note the order of types is flipped for the domain types
    type_le_aux__(ST2D, ST1D, InConstr, Mid1Constr),
    type_le_aux__(ST1R, ST2R, Mid1Constr, OutConstr).
type_le_aux__(T, rel(T2), InConstr, OutConstr) :-
    var(T),
    !,
    '$tuple_type_to_list'(T2, Types2),
    length(Types2, N),
    length(Types1, N),
    T = rel('$tuple_type'(Types1)), %% T is a rel with the same num of args
    '$add_annotations'(Types2, ATypes2, '!'),
    map(wrap_var_functor__, Types1, ATypes1),
    type_annotated_le__(ATypes1, ATypes2, InConstr, OutConstr).
type_le_aux__(T, act(T2), InConstr, OutConstr) :-
    var(T),
    !,
    '$tuple_type_to_list'(T2, Types2),
    length(Types2, N),
    length(Types1, N),
    T = act('$tuple_type'(Types1)),
    '$add_annotations'(Types2, ATypes2, '!'),
    map(wrap_var_functor__, Types1, ATypes1),
    type_annotated_le__(ATypes1, ATypes2, InConstr, OutConstr).
type_le_aux__(T, tel(T2), InConstr, OutConstr) :-
    var(T),
    !,
    '$tuple_type_to_list'(T2, Types2),
    length(Types2, N),
    length(Types1, N),
    T = tel('$tuple_type'(Types1)),
    '$add_annotations'(Types2, ATypes2, '!'),
    map(wrap_var_functor__, Types1, ATypes1),
    type_annotated_le__(ATypes1, ATypes2, InConstr, OutConstr).
type_le_aux__(T, '$tuple_type'(T2), InConstr, OutConstr) :-
    !,
    T =  '$tuple_type'(T1),
    list_type_le_aux__(T1, T2, InConstr, OutConstr).
% type_le_aux__(TA, TB, InConstr, OutConstr) :-
%     nonvar(TA),
%     TA = '$tuple_type'(T1),
%     nonvar(TB),
%     TB = tuple(T2),
%     nonvar(T2),
%     T2 = '$union_type'(T3),
%     !,
%     list_type_le_aux__(T1, T3, InConstr, OutConstr).
type_le_aux__(TA, TB, InConstr, OutConstr) :-
    nonvar(TA),
    TA = '$tuple_type'(T1),
    nonvar(TB),
    !,
    TB =  '$tuple_type'(T2),
    list_type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(T, set(T2), InConstr, OutConstr) :-
    var(T),
    !,
    T =  set(T1),
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(T, list(T2), InConstr, OutConstr) :-
    var(T),
    !,
    T =  list(T1),
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(T, T2, InConstr, OutConstr) :-
    var(T),
    compound(T2), functor(T2, F, _), atom(F),
    '$defined_type'(T2, '$constr_enum_type'(_)), !,
    functor(T2, C, N),
    functor(T, C, N),
    type_le_aux__(T, T2, InConstr, OutConstr).
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    var(T1), !,
    OutConstr = [le(T1, T2)|InConstr].

%% from here on T1 and T2 are different nonvars
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    constructor_type_aux__(T1, Constr, T1Args),
    constructor_type_aux__(T2, Constr, T2Args),
    !,
    list_type_le_aux__(T1Args, T2Args, InConstr, OutConstr).
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    T1 =.. [Constr1|Args1],
    length(Args1, N),
    length(VArgs, N),
    T1V =.. [Constr1|VArgs],
    T2 =.. [Constr2|Args2],
    T2V =.. [Constr2|VArgs],
    
    '$defined_type'(T1V, '$constr_enum_type'(T1Enum)),
    '$defined_type'(T2V, '$constr_enum_type'(T2Enum)),
    !,
    forall(member(T1C, T1Enum), member(T1C, T2Enum)),
    list_type_le_aux__(Args1, Args2, InConstr, OutConstr).
    
    
type_le_aux__('$enum_type'(T1), '$enum_type'(T2), InConstr, OutConstr) :-
    !,
    forall(member(X, T1), member(X, T2)), 
    OutConstr = InConstr.
type_le_aux__('$enum_type'(T1), atom, InConstr, OutConstr) :-  
    forall( member(A,T1), atom(A)), !, 
    OutConstr = InConstr.
type_le_aux__('$enum_type'(T1), string, InConstr, OutConstr) :-  
    forall( member(A,T1), string(A)), !, 
    OutConstr = InConstr.
type_le_aux__('$enum_type'(T1), nat, InConstr, OutConstr) :-  
    forall( member(A,T1), is_nat_(A)), !, 
    OutConstr = InConstr.
type_le_aux__('$enum_type'(T1), int, InConstr, OutConstr) :-  
    forall( member(A,T1), integer(A)), !, 
    OutConstr = InConstr.
type_le_aux__('$enum_type'(T1), num, InConstr, OutConstr) :-  
    forall( member(A,T1), number(A)), !, 
    OutConstr = InConstr.
type_le_aux__('$enum_type'(T1), T2, InConstr, OutConstr) :-
    T2 == atomic,
    forall( member(A,T1), '$atomic_or_string'(A)), !, 
    OutConstr = InConstr.

type_le_aux__('$union_type'(T1), T2, InConstr, OutConstr) :-
    is_primitive_or_defined_type_(T2), !,
    forall( member(E,T1), type_le_aux__(E,T2, [], []) ), 
    OutConstr = InConstr.
type_le_aux__('$union_type'(T1), '$union_type'(T2), InConstr, OutConstr) :-
    !,
    '$type_union_le'(T1, '$union_type'(T2), InConstr, OutConstr).
type_le_aux__(_T1, '$union_type'([]), _InConstr, _OutConstr) :-
    !,
    fail.
type_le_aux__(T1, '$union_type'([T2]), InConstr, OutConstr) :-
    !,
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(T1, '$union_type'(T2), InConstr, OutConstr) :-
    !,
    (
      member(E2,T2), \+type_le_aux__(T1, E2, [], _)
    ->
      diff_list(T2, [E2], T3),
      type_le_aux__(T1, '$union_type'(T3), InConstr, OutConstr)
    ;
      member(E2,T2), type_le_aux__(T1, E2, [], [])
    ->
      InConstr = OutConstr
    ;
      OutConstr = [le(T1, '$union_type'(T2))|InConstr]
    ).
type_le_aux__('$enum_type'(T1), (N1 .. N2), InConstr, OutConstr) :-
    T1 = [A|_],
    integer(A),
    forall(member(M, T1), (N1 =< M, M =< N2)), 
    OutConstr = InConstr.
type_le_aux__((N1 .. M1), (N2 .. M2), InConstr, OutConstr) :-
    N2 =< N1,
    M2 >= M1, 
    OutConstr = InConstr.
type_le_aux__((A.._), nat, InConstr, InConstr) :- A >= 0, !.
type_le_aux__((_.._), int, InConstr, InConstr).
type_le_aux__((_.._), num, InConstr, InConstr).
type_le_aux__((_.._), atomic, InConstr, InConstr).
type_le_aux__(nat, int, InConstr, InConstr).
type_le_aux__(nat, num, InConstr, InConstr).
type_le_aux__(int, num, InConstr, InConstr).
type_le_aux__(nat, atomic, InConstr, InConstr).
type_le_aux__(int, atomic, InConstr, InConstr).
type_le_aux__(num, atomic, InConstr, InConstr).
type_le_aux__(atom, atomic, InConstr, InConstr).
type_le_aux__(string, atomic, InConstr, InConstr).
type_le_aux__(bottom, _, InConstr, InConstr).
type_le_aux__(T, term, InConstr, InConstr) :- T \= code.
%type_le_aux__(set(T1), [T2], InConstr, OutConstr) :-
%    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(list(T1), list(T2), InConstr, OutConstr) :-
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(set(T1), set(T2), InConstr, OutConstr) :-
    type_le_aux__(T1, T2, InConstr, OutConstr).

type_le_aux__(alt_types(T1s), rel(T2), InConstr, OutConstr) :-
    member(T1, T1s),
    type_le_aux__(T1, rel(T2), InConstr, X), X = InConstr,
    InConstr = OutConstr.
type_le_aux__(alt_types(T1s), (T2D -> T2R), InConstr, OutConstr) :-
    member(T1, T1s),
    type_le_aux__(T1, (T2D -> T2R), InConstr, X), X = InConstr,
    InConstr = OutConstr.
type_le_aux__(alt_types(T1s), alt_types(T2s), InConstr, OutConstr) :-
    forall(member(T2, T2s),
	   (member(T1, T1s),
	    type_le_aux__(T1, T2, InConstr, X), X = InConstr)),
    InConstr = OutConstr.
type_le_aux__(alt_types(T1s), act(T2), InConstr, OutConstr) :-
    member(T1, T1s),
    type_le_aux__(T1, act(T2), InConstr, X), X = InConstr,
    InConstr = OutConstr.
type_le_aux__(alt_types(T1s), tel(T2), InConstr, OutConstr) :-
    member(T1, T1s),
    type_le_aux__(T1, tel(T2), InConstr, X), X = InConstr,
    InConstr = OutConstr.

type_le_aux__(rel(T1), rel(T2), InConstr, OutConstr) :-
    !,
    '$tuple_type_to_list'(T1, Types1),
    '$tuple_type_to_list'(T2, Types2),
    '$add_annotations'(Types1, ATypes1, '!'),
    '$add_annotations'(Types2, ATypes2, '!'),
    type_annotated_le__(ATypes1, ATypes2, InConstr, OutConstr).
type_le_aux__(act(T1), act(T2), InConstr, OutConstr) :-
    !,
    '$tuple_type_to_list'(T1, Types1),
    '$tuple_type_to_list'(T2, Types2),
    '$add_annotations'(Types1, ATypes1, '!'),
    '$add_annotations'(Types2, ATypes2, '!'),
    type_annotated_le__(ATypes1, ATypes2, InConstr, OutConstr).
type_le_aux__(tel(T1), tel(T2), InConstr, OutConstr) :-
    !,
    '$tuple_type_to_list'(T1, Types1),
    '$tuple_type_to_list'(T2, Types2),
    '$add_annotations'(Types1, ATypes1, '!'),
    '$add_annotations'(Types2, ATypes2, '!'),
    type_annotated_le__(ATypes1, ATypes2, InConstr, OutConstr).

type_le_aux__(atom_naming(_), atom, InConstr, OutConstr) :-
    !,
    OutConstr = InConstr.
type_le_aux__(atom_naming(T1), atom_naming(T2), InConstr, OutConstr) :-
    !,
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(atom_naming(T1), term_naming(T2), InConstr, OutConstr) :-
    !,
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(term_naming(_), term, InConstr, OutConstr) :-
    !,
    OutConstr = InConstr.
type_le_aux__(term_naming(T1), term_naming(T2), InConstr, OutConstr) :-
    !,
    type_le_aux__(T1, T2, InConstr, OutConstr).
type_le_aux__(atom_naming(T1), T2, InConstr, OutConstr) :-
    !,
    type_le_aux__( T1, T2, InConstr, OutConstr).
type_le_aux__(term_naming(T1), T2, InConstr, OutConstr) :-
    !,
    type_le_aux__( T1, T2, InConstr, OutConstr).
% type_le_aux__(T1, T2, InConstr, OutConstr) :-
%     compound(T1), compound(T2),
%     T1 =.. [F|T1Args],
%     T2 =.. [F|T2Args], !,
%     list_type_le_aux__(T1Args, T2Args, InConstr, OutConstr).
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    '$defined_type'(T1, DT1),
    type_le_aux__(DT1, T2, InConstr, OutConstr).
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    '$defined_type'(T2, DT2),
    type_le_aux__(T1, DT2, InConstr, OutConstr).
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    '$type_info'(_, macro_type, T1, DT1, _),
    type_le_aux__(DT1, T2, InConstr, OutConstr).
type_le_aux__(T1, T2, InConstr, OutConstr) :-
    '$type_info'(_, macro_type, T2, DT2, _),
    type_le_aux__(T1, DT2, InConstr, OutConstr).

%% '$type_union_le'(T1, T2, InC, OutC) checks if the type union
%% T1 is le type union T2 - i.e. all types in the union T1 are le
%% T2
'$type_union_le'([], _, InConstr, InConstr).
'$type_union_le'([T|Ts], T2, InConstr, OutConstr) :-
    type_le_aux__(T, T2, InConstr, MidConstr),
    '$type_union_le'(Ts, T2, MidConstr, OutConstr).

%% check that the type elements of the first list are pairwise le
%% the elements in the second list
list_type_le_aux__([], [], Constr, Constr).
list_type_le_aux__([T1|T1R], [T2|T2R], InConstr, OutConstr) :-
    type_le_aux__(T1, T2, InConstr, MidConstr),
    list_type_le_aux__(T1R, T2R, MidConstr, OutConstr).

%% remove any top-level ! modes from function types
'$strip_top_fun_bangs'(X, X) :- var(X), !.
'$strip_top_fun_bangs'('!'(T), T) :- !.
'$strip_top_fun_bangs'('$tuple_type'(Ts), '$tuple_type'(STs)) :-
    !,
    map('$strip_top_fun_bangs', Ts, STs).
'$strip_top_fun_bangs'(X, X).

%% min_mode__(M1, M2, M3) is true iff M3 is the min mode of M1 and M2 with
%% ! < ? < ??
min_mode__('!', _, X) :- !, X = '!'.
min_mode__(_, '!', X) :- !, X = '!'.
%min_mode__('@', _, X) :- !, X = '!'.
%min_mode__(_, '@', X) :- !, X = '!'.
min_mode__('?', _, X) :- !, X = '?'.
min_mode__(_, '?', X) :- !, X = '?'.
min_mode__('??', _, X) :- !, X = '??'.
min_mode__(_, '??', X) :- !, X = '??'.
min_mode__(X, X, X).


%% type_member_of__(X:T, VTB) is true if X:T is a member of VTB
type_member_of__(X:T, [Y:TY|_]) :- X == Y, !, T = TY.
type_member_of__(XT, [_|Xs]) :- type_member_of__(XT, Xs).

%% type_remove_from__(X:T, InVTB, OutVTB) is true iff X:T is in INVTB
%% and OutVTB is InVTB without the constraint X:T
type_remove_from__(X:T, [Y:TY|Ys], Z) :- X == Y, !, T = TY, Z = Ys.
type_remove_from__(XT, [Y|Ys], [Y|Z]) :- type_remove_from__(XT, Ys, Z).

%% le1_remove_from__(le(X, T), InLEs, OutLEs) is true iff le(X, T1) is in InLEs
%% and T = T1 and OutLEs is InLEs with le(X, T) removed
le1_remove_from__(le(X, T), [le(Y,TY)|Ys], Z) :- X == Y, !, T = TY, Z = Ys.
le1_remove_from__(XT, [Y|Ys], [Y|Z]) :- le1_remove_from__(XT, Ys, Z).
%% le1_remove_from__(le(T, X), InLEs, OutLEs) is true iff le(T1, X) is in InLEs
%% and T = T1 and OutLEs is InLEs with le(T, X) removed
le2_remove_from__(le(T, X), [le(TY,Y)|Ys], Z) :- X == Y, !, T = TY, Z = Ys.
le2_remove_from__(XT, [Y|Ys], [Y|Z]) :- le2_remove_from__(XT, Ys, Z).


%% Find a solution to a set of le type constraints by binding type variables.
%% We copy the constraints and simplify those first (binding type variables).
%% After that further simplify the types (binding the type variables in
%% the original set of constraints to these simplified types)
%%bind_type_constraints1__(Constr) :- errornl(bind_type_constraints1__(Constr)),fail.
bind_type_constraints1__(Constr) :-
    %% first carry out any residual "forced" constraint simplifications
    redo_type_le__(Constr, SimpConstr),
    copy_term(SimpConstr, SimpConstrCopy),
    bind_type_constraints_aux__(SimpConstrCopy),
    bind_to_copy_with_simp__(SimpConstr, SimpConstrCopy), !.


%%bind_to_copy_with_simp__(A, B) :- errornl(bind_to_copy_with_simp__(A, B)),fail.
bind_to_copy_with_simp__([], []).
bind_to_copy_with_simp__([le(A1, B)|Constr], [le(A2, D)|ConstrCopy]) :-
    '$simplify_the_type'(A1, A1S),
    '$simplify_the_type'(A2, A2S),
    A1S = A2S,
    var(B), !,
    '$simplify_the_type'(D, B),
    bind_to_copy_with_simp__(Constr, ConstrCopy).
bind_to_copy_with_simp__([le(A1,_)|Constr], [le(A2,_)|ConstrCopy]) :-
    '$simplify_the_type'(A1, A1S),
    '$simplify_the_type'(A2, A2S),
    A1S = A2S,
    bind_to_copy_with_simp__(Constr, ConstrCopy).

%%'$simplify_the_type'(T1, T2) simplifies T1 to T2 - this recursively
%% simplifies contructor types bottoming out at union types where it
%% compresses union types into small union types (or a single type)
%% EG '$union_type'([nat, int]) simplifies to '$union_type'([int]) and then
%% to int
%%'$simplify_the_type'(T, V) :- errornl('$simplify_the_type'(T)),fail.
'$simplify_the_type'(V, R) :-
    var(V), !,
    R = V.
'$simplify_the_type'('$union_type'(Types), Type) :-
    !,
    map('$simplify_the_type', Types, SimpTypes),
    sort(SimpTypes, SortedSimpTypes),
    '$collect_simp_types'(SortedSimpTypes, [], SimpSimpTypes),
    (
      SimpSimpTypes = [T1]
    ->
      Type = T1
    ;
      Type = '$union_type'(SimpSimpTypes)
    ).
'$simplify_the_type'('$tuple_type'(Types), '$tuple_type'(SimpTypes)) :-
    !,
    map('$simplify_the_type',Types, SimpTypes).
'$simplify_the_type'(list(T), Type) :-
    !,
    '$simplify_the_type'(T, SimpT),
    Type = list(SimpT).
'$simplify_the_type'(set(T), Type) :-
    !,
    '$simplify_the_type'(T, SimpT),
    Type = set(SimpT).
'$simplify_the_type'(CType, SimpT) :-
    '$defined_type'(CType, '$constr_enum_type'(_)),
    CType =.. [C|Types], !,
    map('$simplify_the_type', Types, STypes),
    SimpT =.. [C|STypes].
'$simplify_the_type'(T, T).

'$collect_simp_types'([], _, []).
'$collect_simp_types'([T|Ts], Seen, Simp) :-
    member(T1, Ts), type_le__(T, T1), !,
    %% a bigger type than T is in the types to be processed - ignore T
    '$collect_simp_types'(Ts, Seen, Simp).
'$collect_simp_types'([T|Ts], Seen, Simp) :-
    member(T1, Seen), type_le__(T, T1), !,
    %% a bigger type than T is in the seen (already processed) types - ignore T
    '$collect_simp_types'(Ts, Seen, Simp).
'$collect_simp_types'([T|Ts], Seen, Simp) :-
    %% T is not LE any other types in the original list of types
    '$collect_simp_types'(Ts, [T|Seen], Simp1),
    Simp = [T|Simp1].

'$collect_simp_alt_types'([], _, []).
'$collect_simp_alt_types'([T|Ts], Seen, Simp) :-
    member(T1, Ts), type_le__(T1, T), !,
    %% a smaller type than T is in the types to be processed - ignore T
    '$collect_simp_alt_types'(Ts, Seen, Simp).
'$collect_simp_alt_types'([T|Ts], Seen, Simp) :-
    member(T1, Seen), type_le__(T1, T), !,
    %% a smaller type than T is in the seen (already processed) types - ignore T
    '$collect_simp_alt_types'(Ts, Seen, Simp).
'$collect_simp_alt_types'([T|Ts], Seen, Simp) :-
    %% T is not GE any other types in the original list of types
    '$collect_simp_alt_types'(Ts, [T|Seen], Simp1),
    Simp = [T|Simp1].

%% Bind type varaibles to solve le constraints.
%% EG take the constraints [le(atom, T), le(int, T)] - we collect as list
%% [atom, int] and then instantiate T to '$union_type'([atom, int])
%% This gives the minimal type binding for T that satisfies the constraints
bind_type_constraints_aux__([]).
bind_type_constraints_aux__([le(T1, T2)|Rest]) :-
    var(T2), !,
    collect_same_2_le__(Rest, T2, Collected, Left),
    '$simplify_types'([T1|Collected], T22),
    T2 = T22,
    bind_type_constraints_aux__(Left).
bind_type_constraints_aux__([le(T, T)|Rest]) :-
    bind_type_constraints_aux__(Rest).

%% collect_same_2_le__(Constraints, T, Collected, Left)
%% Collected is the list of Ti st le(Ti, T) in Constraints and Left
%% the remaining constraints in Constraints.
%% NOTE: we assume the le constraints are sorted on the second arg so that
%% all the le(Ti, T) with the same second args are grouped together.
collect_same_2_le__([], _T2, [], []) :- !.
collect_same_2_le__([le(T1, T1x)|Rest], T2, [T1|Collected], Left) :-
    T1x == T2, !,
    collect_same_2_le__(Rest, T2, Collected, Left).
collect_same_2_le__(Rest, _T2, [], Rest).

%% '$annotated_type_info'(ATypes, Term) is true if ATypes is a (code) type
%% of Term - used in type_check_call__ for a "normal" call.
%% Term has kind rel, act or tel
%% EG if we have the declaration
%% rel p(nat, ?nat)
%% then for
%% '$annotated_type_info'(AType, p(A, B)) we would get
%% AType = rel([!nat, ?nat])
%%'$annotated_type_info'(ATypes, Term) :- errornl('$annotated_type_info'(ATypes, Term)),fail.
'$annotated_type_info'(ATypes, Term) :-
    freeze_term(Term, TVars),
    '$get_ultimate_functor'(Term, F),
    '$type_info'(F, _Kind, Types, _, _),
    thaw_term(TVars),
    '$add_annotations'(Types, ATypes, '!').

%% recursively add mode annotations to code types - the default mode is !
%% EG '$add_annotations_to_type'(!list(rel('$tuple_type'([int, ?nat]))), Y)
%% gives Y = !list(rel('$tuple_type'([!int, ?nat])))
'$add_annotations_to_type'(X, Y) :-
    var(X),
    !,
    X = Y.
'$add_annotations_to_type'(X, Y) :-
    '$has_annotation'(X), !,
    X = Ann(T),
    '$add_annotations_to_type'(T, AT),
    Y = Ann(AT).
'$add_annotations_to_type'('$tuple_type'(X), '$tuple_type'(Y)) :-
    !,
    map('$add_annotations_to_type', X, Y).
'$add_annotations_to_type'(rel('$tuple_type'(Types)),
                           rel('$tuple_type'(ATypes))) :-
    !,
    map('$add_annotations_to_type', Types, ATypes1),
    '$add_annotations'(ATypes1, ATypes, '!').
    %'$list_to_tuple_type'(ATypes, ATuple).
'$add_annotations_to_type'(act(Tuple), act(ATuple)) :-
    !,
    '$tuple_type_to_list'(Tuple, Types),
    map('$add_annotations_to_type', Types, ATypes1),
    '$add_annotations'(ATypes1, ATypes, '!'),
    '$list_to_tuple_type'(ATypes, ATuple).
'$add_annotations_to_type'(tel(Tuple), tel(ATuple)) :-
    !,
    '$tuple_type_to_list'(Tuple, Types),
    map('$add_annotations_to_type', Types, ATypes1),
    '$add_annotations'(ATypes1, ATypes, '!'),
    '$list_to_tuple_type'(ATypes, ATuple).
'$add_annotations_to_type'(Ty, Ty2) :-
    Ty = (Tuple1 -> Type2),
    Ty2 = (ATuple1 -> AType2),
    !,
    '$tuple_type_to_list'(Tuple1, Types1),
    %'$tuple_type_to_list'(Tuple2, Types2),
    map('$add_annotations_to_type', Types1, ATypes1),
    '$add_annotations_to_type'(Type2, AType2),
    '$list_to_tuple_type'(ATypes1, ATuple1).
    %'$list_to_tuple_type'(ATypes2, ATuple2).
'$add_annotations_to_type'(T, T).


%% Used to type check A =? B <>? C  - B or C might also be of the form X <>? Y
%% This recursively collects all the args of <>? into one list and then
%% the type checker checks that each element is of the appripriate list type
extract_append_args__(A, Args) :-
    compound(A),
    A = (A1 <>? A2),
    !,
    extract_append_args__(A1, Args1),
    extract_append_args__(A2, Args2),
    append(Args1, Args2, Args).
extract_append_args__(A, Args) :-
    Args = [A].

%% Ditto for type checking A =? B ++? C but now B and C might be
%% ++? structures and we collect the args for type checking against strings
extract_concat_args__(A, Args) :-
    compound(A),
    A = (A1 ++? A2),
    !,
    extract_concat_args__(A1, Args1),
    extract_concat_args__(A2, Args2),
    append(Args1, Args2, Args).
extract_concat_args__(A, Args) :-
    Args = [A].


%% is_a_constructor_term_of_type__(Term, Type, TermN, ConstrN) is true
%% if Term is a constructed term of the constructor enum type Type where
%% TermN is the arity of Term and ConstrN is the arity of the corresponding
%% enum (normally these arities are the same but if they are different
%% we generate an arity error in the caller).
is_a_constructor_term_of_type__(Term, Type, TermN, ConstrN) :-
    compound(Term),
    '@functor'(Term, C, TermN),
    '$defined_type'(Type, '$constr_enum_type'(Enum)),
    member(T, Enum),
    '@functor'(T, C, ConstrN),
    !.
    

%% A system defined relation
%% type_info(Type, Info) is true if Type is a union type, a tuple type
%% or a constructor type
type_info(Type, Info) :-
    '$type_info'(_, defined_type, Type, TypeDef, _), !,
    '$type_info_aux'(TypeDef, Info).
type_info(Type, Info) :-
    '$type_info'(_, macro_type, Type, TypeDef, _), !,
    '$type_info_aux'(TypeDef, Info).

'$type_info_aux'('$union_type'(T), unionT_(T)).
'$type_info_aux'('$enum_type'(T), enumT_(T)).
'$type_info_aux'('$tuple_type'(T), tupleT_(T)).
'$type_info_aux'('$constr_enum_type'(T), constrT_(T)).
'$type_info_aux'((N..M), rangeT_(N, M)).

%% Used in the negation to rule out 2 atom naming types that are obviously
%% different because of arities
'$compatible_atom_naming'(('$tuple_type'(T1) -> _), ('$tuple_type'(T2) -> _)) :-
    length(T1, N), length(T2, N), !.
'$compatible_atom_naming'(rel('$tuple_type'(T1)), rel('$tuple_type'(T2))) :-
    length(T1, N), length(T2, N), !.
'$compatible_atom_naming'(act('$tuple_type'(T1)), act('$tuple_type'(T2))) :-
    length(T1, N), length(T2, N), !.
'$compatible_atom_naming'(tel('$tuple_type'(T1)), tel('$tuple_type'(T2))) :-
    length(T1, N), length(T2, N), !.


%%
%% All type checking/inference is done through the following call
%% If variables occur in Term then variable type bindings are
%% produced.
%% Example:
%% The call has_type__(tr(A, B, C), !tree(?T), [], [], VTB, [], LE, [], [],
%%                     1, p(tr(A, B, C)), Error)
%% where we are type checking the first argument of p(tr(A, B, C))
%% where def tree(T) ::= tr(tree(T), T, tree(T)) | empty() 
%% will produce
%% VTB = [A : !tree(?T), B : ?T, C : !tree(?T)]
%% LE = []
%% Error = Error
%% Also if Type is a variable then we flip to doing type inference
%% rather than type checking. It's possible to flip from type checking
%% to type inference as in the above example.
%% Most of the type checking related predicates have an Error argument
%% that is a variable unless an error has occurred in which case
%% Error is instantiated to an Error term.
%% These predicates typically first test if the incoming Error arg is
%% a nonvar and in which case the call succeeds - this means that
%% if a call generates an error then any subsequent calls in the check
%% will succeed and then this Error term will then later be tested and
%% an error output generated


%% For checking the extra constraint on ?/?? args for actions - i.e. those
%% variables have to be new variables at call time, we use two implicit
%% parameters:
%% '$call_kind' - set to act if we are checking as an action and rel OW
%% '$act_seen' - the list of variables seen in the call body (no matter
%%   what kind the code is).

%%has_type__(T, MT, CurrVMT, InVTB, OutVTB, InLE, OutLE, Unif, Unders, I,PT,Err) :- errornl(has_type__(T, MT, CurrVMT, InVTB, OutVTB, InLE, OutLE, Unif, Unders, I,PT,Err)),ip_lookup('$act_seen', Seen), errornl(act_seen___(Seen)),fail.
has_type__(_Term, _MT, _CurrVTB, InVTB, OutVTB, InLE, OutLE, _Unif,
           _Unders, _Index, _PTerm, Error) :-
    nonvar(Error), !,
    %% an error has occurred - exit
    OutLE = InLE,
    OutVTB = InVTB.
has_type__(_Term, MT, _CurrVTB, InVTB, OutVTB, InLE, OutLE, _Unif,
           _Unders, _Index, _PTerm, _Error) :-
    checking_inner_bangs__, MT \= '!'(_), !,
    %% an error has occurred - exit
    OutLE = InLE,
    OutVTB = InVTB.
has_type__(Term, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
           Unders, Index, PTerm, Error) :-
    var(Type), !,
    Type = '!'(_),
    %% Type has no mode - give it ! mode
    has_type__(Term, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
               Unders, Index, PTerm, Error).
has_type__(Term, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
           Unders, Index, PTerm, Error) :-
    atom(Type), !,
    %% Type has no mode - give it ! mode
    has_type__(Term, '!'(Type), CurrVTB, InVTB, OutVTB, InLE, OutLE,
               Unif, Unders, Index, PTerm, Error).
has_type__(Term, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
           Unders, Index, PTerm, Error) :-
    compound(Type), functor(Type, Mode, _), nonvar(Mode), \+moded__(Type), !,
    %% Type has no mode - give it ! mode
    has_type__(Term, '!'(Type), CurrVTB, InVTB, OutVTB, InLE, OutLE,
               Unif, Unders, Index, PTerm, Error).
has_type__(Term, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders,
           Index, PTerm, Error) :-
    var(Term), ( CurrVTB = head ; CurrVTB = unif ),
    in_unif__((Term = UTerm), Unif),!,
    has_type__(UTerm, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
               Unders, Index, PTerm, Error).
has_type__(Term, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, _, _, _, _, _) :-
    var(Term), ( CurrVTB = head ; CurrVTB = unif ), !,
    %% we are in the head with a variable argument - just add a VTB constraint
    push_modes_in__(Type, MType),
    OutLE = InLE,
    OutVTB = [Term:MType|InVTB].
% has_type__(Term, '@'(term), CurrVTB, InVTB, OutVTB, InLE, OutLE,
%            _, _, Index, PTerm, Error) :-
%     CurrVTB = head, !,
%     %% we are in the head with a non-variable argument in a @term position
%     %% mode error
%     Error = mode_error(Index, PTerm, Term, '@'(term), at_term_mode_error),
%     OutLE = InLE,
%     OutVTB = InVTB.

has_type__(Term, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, _Unif,
           _Unders, Index, PTerm, Error) :-
    var(Term), type_member_of__(Term:MT, CurrVTB), !,
    %% Term is a variable appearing in current VTB context
    has_type_VTB__(MT, Term, Type, InVTB, OutVTB, InLE, OutLE,
                   Index, PTerm, Error),
    %% add the var Term to act_seen
    '$update_act_seen'(Term).

has_type__(Term, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
           Unders, Index, PTerm, Error) :-
    var(Term), in_unif__((Term = UTerm), Unif), !,
    %% Term is a variable that would have been unified with UTerm earlier
    %% in the call - replace Term by UTerm and call has_type__ on UTerm
    has_type__(UTerm, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
               Unders, Index, PTerm, Err),
    strip_modes__(Type, SType),
    (
      var(Err)
    ->
      true
    ;
      Err = type_error(I, P, _, _, Reason, DecI)
    ->
      Error = type_error(I, P, Term, SType, Reason, DecI)
    ;
      functor(Err, mode_error, _)
    ->
      has_moded_type__(UTerm, Type, OType, CurrVTB, InVTB, InLE, Unif, Unders),
      Error = mode_error(Index, Term, Term, Type, OType)
    ;
      functor(Err, act_input_error, _)
    ->
      Error = Err
    ;
      functor(Err, unify_err, _)
    ->
      Error = Err
    ;
      Error = unify_err(Term, UTerm, Type, Err)
    ),
    %% add the var Term to act_seen
    '$update_act_seen'(Term).
has_type__(Term, MType, _CurrVTB, InVTB, OutVTB, InLE, OutLE, _, _,
           Index, PTerm, Error) :-
    var(Term), MType = M(_), M == '!', !,
    %% Term is required to be in ! mode but is a new variable
    Error = mode_error(Index, PTerm, Term, MType, new_variable),
    OutLE = InLE,
    OutVTB = InVTB.
has_type__(Term, MType, _CurrVTB, InVTB, OutVTB, InLE, OutLE, _, _,
           Index, PTerm, Error) :-
    var(Term), MType = M(dyn_term), '$dyn_term_mode'(M), !,
    %% Term is required to be in partial ! mode (i.e. ground functor)
    %% but is a new variable
    Error = mode_error(Index, PTerm, Term, M, dyn_term_mode_error),
    OutLE = InLE,
    OutVTB = InVTB.
has_type__(Term, MType, _CurrVTB, InVTB, OutVTB, InLE, OutLE, _, _, _, _, _) :-
    var(Term), !,
    %% add the var Term to act_seen
    '$update_act_seen'(Term),
    %% Term is a new variable and the mode is not ! so add Term:MType to VTB
    OutLE = InLE,
    OutVTB = [Term:MType|InVTB].
has_type__(Term, Mode(Type), CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
           Unders, Index, PTerm, Error) :-
    Type == term, !,
    has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                         Unif, Unders, Index, PTerm, Error).
%% In Qulog we have, for example, empty() which is represented in
%% QuPolog as empty('$none_') and so '$none_' is not a real term and
%% type checking it can be ignored
has_type__('$none_', _, _, InVTB, InVTB, InLE, InLE, _, _, _, _, _) :- !.
has_type__(Term, Mode(Type), CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
           Unders, Index, PTerm, Error) :-
    var(Type), !,
    %% Term is not a variable but Type is
    type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type__(Term, Mode(Type), CurrVTB, InVTB, OutVTB, InLE, OutLE,
           Unif, Unders, Index, PTerm, Error) :-
    %% Neither Term nor Type are variables
    has_type_term__(Type, Mode, Term,
                    CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                    Index, PTerm, Error).

%% add the var Term to act_seen
'$update_act_seen'(Term) :-
    ip_lookup('$act_seen', Seen),
    list(Seen), !,
    ip_set('$act_seen', [Term|Seen]).
'$update_act_seen'(_).

%%
%% has_type_VTB__(MT, Term, Type, InVTB, OutVTB, InLE, OutLE,
%%                Index, PTerm, Error)
%% Term is a variable that appears in CurrVTB as Term:MT
%% Check that Type is consistent with MT
%% During the check, a new VTB constraint on Term might be added to InVTB
%% to produce OutVTB. Type LE constraints may also be updated

%%has_type_VTB__(MT, Term, Type, InVTB, OutVTB, InLE, OutLE, Index, PTerm, Error) :- errornl(has_type_VTB__(MT, Term, Type, InVTB, OutVTB, InLE, OutLE, Index, PTerm,  Error)),fail.
% has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
%                Index, PTerm, Error) :-
%     MT2 = Mode(_),
%     ip_lookup('$call_kind', Kind), Kind == act,
%     var(Term), '$dyn_term_mode_modify'(Mode, _), %either ? or ??
%     ip_lookup('$act_seen', Seen),
%     list(Seen),
%     member_eq(Term, Seen), !,
%     %% We are processing an action and Term is a variable that has appeared
%     %% before in the body of a rule (or a query) and so is an error.
%     Error = act_input_error(Index, PTerm, Term, MT1),
%     OutLE = InLE,
%     OutVTB = InVTB.    
has_type_VTB__(_MT1, _Term, MT2, InVTB, OutVTB, InLE, OutLE,
               _Index, _PTerm, _Error) :-
    checking_inner_bangs__, MT2 \= '!'(_), !,
    OutLE = InLE,
    OutVTB = InVTB.
% has_type_VTB__(_MT1, _Term, '@'(term), InVTB, OutVTB, InLE, OutLE,
%                _Index, _PTerm, _Error) :-
%     !,
%     %% The required moded type is @term - nothing to do
%     OutLE = InLE,
%     OutVTB = InVTB.
% has_type_VTB__('@'(term), Term, MT, InVTB, OutVTB, InLE, OutLE,
%                Index, PTerm, Error) :-
%     !,
%     %% The given moded type is @term but the required is not - mode error
%     Error = mode_error(Index, PTerm, Term, MT, body_at_term_mode_error),
%     OutLE = InLE,
%     OutVTB = InVTB.
has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
               Index, PTerm, Error) :-
    MT2 = '!'(_), MT1 = M1(_), M1 \= '!', !,
    %% Term is required to be in ! mode but is not in VTB
    Error = mode_error(Index, PTerm, Term, MT2, MT1),
    OutLE = InLE,
    OutVTB = InVTB.
has_type_VTB__(M1(T1), _Term, M2(T2), InVTB, OutVTB,
               InLE, OutLE, _, _, _) :-
    T1 == dyn_term, T2 == dyn_term, 
    '$dyn_term_mode'(M1), 
    '$dyn_term_mode'(M2),
    !,
    %% both types are dyn_term with ground functor
    OutLE = InLE,
    OutVTB = InVTB.
has_type_VTB__(_M1(T1), Term, M2(T2), InVTB, OutVTB,
               InLE, OutLE, Index, PTerm, Error) :-
   T1 == dyn_term, T2 == dyn_term, 
    '$dyn_term_mode'(M2),
    !,
    %% Term is required to have a ground functor but does not
    Error =  mode_error(Index, PTerm, Term, M2, dyn_term_mode_error),
    OutLE = InLE,
    OutVTB = InVTB.
has_type_VTB__(M1(T1), Term, MT, InVTB, OutVTB,
               InLE, OutLE, Index, PTerm, Error) :-
    T1 == dyn_term,
    '$dyn_term_mode'(M1),
    !,
    %% the mode in VTB is more restrictive than needs be - recheck
    %% with less restrictive mode
    '$dyn_term_mode_modify'(M, M1),
    has_type_VTB__(M(dyn_term), Term, MT, InVTB, OutVTB,
                   InLE, OutLE, Index, PTerm, Error).
has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE, Index, PTerm, Error) :-
    MT1 = _M1(T1),
    MT2 = _M2(T2),
    constructor_type__(T1, Constr, _),
    constructor_type__(T2, Constr, _),
    !,
    %% EG MT1 = !list(int),  MT2 = !list(?int) then we compare
    %% [!int] and [?int]
    push_modes_in__(MT1, _(ModedT1)),
    push_modes_in__(MT2, M(ModedT2)),
    constructor_type__(ModedT1, Constr, SubT1),
    constructor_type__(ModedT2, Constr, SubT2),
    has_type_VTB_constructors__(SubT1, SubT2, Term, Index, PTerm, InLE, OutLE,
                                AddVTB, Err),
    (
      var(Err), SubT1 = []
    ->
      OutVTB = [Term:M(ModedT2)|InVTB]
    ;
      var(Err), AddVTB == true
    ->
      OutVTB = [Term:M(ModedT2)|InVTB]
    ;
      var(Err)
    ->
      OutVTB = InVTB
    ;
      Err = mode_error(_, V1, V2, _, _)
    ->
      Error = mode_error(Index, V1, V2,MT2, MT1)
    ;
      Err = type_error(_, _, _, _, conflict(_), DecI)
    ->
      strip_modes__(T1, ST1),
      strip_modes__(T2, ST2),
      Error = type_error(Index, PTerm, Term, ST2, conflict(ST1), DecI)
    ;
      Error = Err,
      OutVTB = InVTB                  
    ).
      
has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
               Index, PTerm, Error) :-
    MT1 = '!'(T1), !,
    %% The given mode is ! so we just need the give type is LE required type
    MT2 = Mode(T2),
    type_le_or_err__(Mode, Term, T1, T2, InLE, OutLE, Index, PTerm, Error),
    OutVTB = InVTB.
has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
               Index, PTerm, Error) :-
    PTerm == saved, !,
    MT1 = _(T1),
    %% We are checking saved VTB so just the given type is LE required type
    MT2 = _(T2),
    type_le_or_err__('!', Term, T1, T2, InLE, OutLE, Index, PTerm, Error),
    OutVTB = InVTB.

% has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
%                Index, PTerm, Error) :-
%     MT1 = '!?'(T1), MT2 = '!?'(T2), !,
%     %% The given mode is !? so we just need the give type is LE required type
%     %% the types need to be equal unless dyn_term and rel_term
%     type_le_or_err__('!', Term, T1, T2, InLE, OutLE, Index, PTerm, Error),
%     OutVTB = [Term:'!'(T1)|InVTB].
has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
               Index, PTerm, Error) :-
    MT1 = '!??'(T1), MT2 = '!??'(T2), !,
    %% The given mode is !?? so we just need the give type is LE required type
    %% the types need to be equal unless dyn_term and rel_term
    type_le_or_err__('!', Term, T1, T2, InLE, OutLE, Index, PTerm, Error),
    OutVTB = InVTB.
has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
               Index, PTerm, Error) :-
    MT1 = '??'(T1), MT2 = '!??'(T2), !,
    %% The given mode is !? so we just need the give type is LE required type
    %% the types need to be equal unless dyn_term and rel_term
    type_le_or_err__('!', Term, T1, T2, InLE, OutLE, Index, PTerm, Error),
    OutVTB = [Term:'!??'(T1)|InVTB].
% has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
%                Index, PTerm, Error) :-
%     MT1 = '!??'(_T1), MT2 = '!?'(_T2), !,
%     %% Mode error
%     Error = mode_error(Index, PTerm, Term, MT2, MT1),
%     InLE = OutLE,
%     OutVTB = InVTB.

%% below neither modes are !
has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
               Index, PTerm, Error) :-
    strip_modes__(MT1, T1),
    strip_modes__(MT2, T2),
    ground(T1),
    ground(T2),
    !,
    %% Two ground non ! moded types - must be equal
    push_modes_in__(MT2, ModedT2),
    OutVTB = [Term:ModedT2|InVTB],
    OutLE = InLE,
    (
      T1 = T2
    ->
      true
    ;
      Error = type_error(Index, PTerm, Term, T2, conflict(T1), _)
    ).
%% below neither mode is ! and there is at least one type variable in the types
has_type_VTB__(MT1, Term, MT2, InVTB, OutVTB, InLE, OutLE,
               Index, PTerm, Error) :-
    strip_modes__(MT1, T1),
    strip_modes__(MT2, T2),
    !,
    push_modes_in__(MT2, ModedT2),
    OutVTB = [Term:ModedT2|InVTB],
    (
      T1 = T2, check_bind_type_constraints__(InLE, E), var(E)
    ->
      redo_type_le__(InLE, OutLE)
    ;
      OutLE = InLE,
      bind_type_constraints1__(OutLE),
      Error = type_error(Index, PTerm, Term, T2, conflict(T1), _)
    ).

%%
%% has_type_VTB_constructors__(KnownTypes, RequiredTypes, Term, Index,
%%                             PTerm, InLE, OutLE, AddVTB, Error)
%% Compare the list KnownTypes with RequiredTypes for Term at Index in PTerm
%% adding any LE constraints to InLE to get OutLE.
%% AddVTB is set to true if the original required type constraint
%% should be added to VTB
%% If an error occurs Error is bound to the required error term.

%%has_type_VTB_constructors__(M1s, M2s, Term, Index, PTerm, InLE, OutLE, AddVTB,  Error) :-errornl(has_type_VTB_constructors__(M1s, M2s, Term, Index, PTerm, InLE, OutLE, AddVTB, Error)),fail.
has_type_VTB_constructors__([], [], _, _, _Term, InLE, InLE, _AddVTB, _Error).
has_type_VTB_constructors__([_|_], [], _SubT2, _, _Term, _InLE, _OutLE, _,
                            _Error) :-
    !,
    errornl(unmatched_type_arities(has_type_VTB_constructors)),
    fail.
has_type_VTB_constructors__([], [_|_], _SubT2, _, _Term, _InLE, _OutLE, _,
                            _Error) :-
    !,
    errornl(unmatched_type_arities(has_type_VTB_constructors)),
    fail.
has_type_VTB_constructors__([_|_], [_|_], _SubT2, _, _Term, InLE, OutLE, _,
                            Error) :-
    nonvar(Error), !,
    OutLE = InLE.
has_type_VTB_constructors__([M1(T1)|SubT1], [M2(T2)|SubT2], Term,Index,
                            PTerm, InLE, OutLE, AddVTB, Error) :-
    has_type_VTB_constructors_aux__(M1, T1, M2, T2, Term, Index, PTerm,
                                    InLE, MidLE, AddVTB, Error),
    Index1 is Index + 1,
    has_type_VTB_constructors__(SubT1, SubT2, Term, Index1, PTerm,
                                MidLE, OutLE, AddVTB, Error1),
    (
      var(Error1)
    ->
      true
    ;
      Error1 = type_error(_, _, _, _, conflict(_), DecI)
    ->
      strip_modes__(T1, ST1),
      strip_modes__(T2, ST2),
      Error = type_error(Index, PTerm, Term, ST2, conflict(ST1), DecI)
    ;
      Error = Error1
    ).
%% The same as has_type_VTB_constructors__ except working on a single
%% pair of types
%%has_type_VTB_constructors_aux__(A,B,C,D,E,F,G,H,I,J,K) :- errornl(has_type_VTB_constructors_aux__(A,B,C,D,E,F,G,H,I,J,K)),fail.
% has_type_VTB_constructors_aux__(_M, _T1, '@', term, _Term, _Index, _PTerm,
%                                 InLE, OutLE, _AddVTB,  _Error) :-
%     !,
%     %% required type is @term - nothing to do
%     OutLE = InLE.
% has_type_VTB_constructors_aux__('@', term, M, T, Term, Index, PTerm,
%                                 InLE, OutLE, _AddVTB,  Error) :-
%     !,
%     %% given type is @term - mode error
%     Error = mode_error(Index, PTerm, Term, M(T), body_at_term_mode_error),
%     OutLE = InLE.
has_type_VTB_constructors_aux__(_, _T1, M, _T2, _Term, _Index, _PTerm,
                                InLE, OutLE, _AddVTB,  _Error) :-
    checking_inner_bangs__, M \= '!', !,
    OutLE = InLE.
has_type_VTB_constructors_aux__('!', T1, _M2, T2, Term, Index, PTerm,
                                InLE, OutLE, AddVTB, Error) :-
    constructor_type__(T1, Constr, SubT1),
    constructor_type__(T2, Constr, SubT2), !,
    has_type_VTB_constructors__(SubT1, SubT2, Term, Index, PTerm,
                                InLE, OutLE, AddVTB, Error).
has_type_VTB_constructors_aux__('!', T1, M2, T2, Term, Index, PTerm,
                                InLE, OutLE, _AddVTB, Error) :-
    !,
    %% The given type has ! mode and so we simply check if
    %% the required type is LE the given type
    type_le_or_err__(M2, Term, T1, T2, InLE, OutLE, Index, PTerm, Error).
has_type_VTB_constructors_aux__(M, T1, '!', T2, Term, Index, PTerm,
                                InLE, OutLE, _AddVTB,  Error) :-
    !,
    %% The required mode is ! but the given mode is not - mode error
    OutLE = InLE,
    Error = mode_error(Index, PTerm, Term, '!'(T2), M(T1)).
has_type_VTB_constructors_aux__(_, T1, _, T2, Term, Index, PTerm,
                                InLE, OutLE, _AddVTB, Error) :-
    %% we are checking the saved VTB so simply check for type_le
    PTerm == saved,
    !,
    type_le_or_err__('!', Term, T1, T2, InLE, OutLE, Index, PTerm, Error).
    
has_type_VTB_constructors_aux__(_, T1, _, T2, Term, Index, PTerm,
                                InLE, OutLE, AddVTB, Error) :-
    %% Neither mode is ! and both types ground
    %% so the types must be equal 
    strip_modes__(T1, ST1),
    strip_modes__(T2, ST2),
    ground(ST1), ground(ST2), !,
    OutLE = InLE,
    (
      ST1 = ST2
    ->
      AddVTB = true
    ;
      Error = type_error(Index, PTerm, Term, T2, conflict(T1), _)
    ).
has_type_VTB_constructors_aux__(_, MT1, _, MT2, Term, Index, PTerm,
                                InLE, OutLE, AddVTB, Error) :-
    %% Neither mode is ! and so the types must be equal with at least
    %% one non-ground type - we add a qeq constraint
    strip_modes__(MT1, T1),
    strip_modes__(MT2, T2),
    (
      T1 = T2, check_bind_type_constraints__(InLE, E), var(E)
    ->
      redo_type_le__(InLE, OutLE),
      AddVTB = true
    ;
      OutLE = InLE,
      bind_type_constraints1__(OutLE),
      Error = type_error(Index, PTerm, Term, T2, conflict(T1), _)
    ).



%% has_type_term__(Type, Mode, Term, CurrVTB, InVTB, OutVTB,
%%                   InLE, OutLE, Unif, Unders, Index, PTerm, Error)        
%% Here both Type and Term are novars
%% Check to see if Term has the given type and mode
%%has_type_term__(Type, Mode, Term, CurrVTB, InVTB, _OutVTB, InLE, _OutLE, Unif, Unders, Index, PTerm, Error) :- errornl(has_type_term__(Type, Mode, Term, CurrVTB, InVTB, InLE, Unif, Unders, Index, PTerm, Error)),fail.

has_type_term__(_Type, _Mode, _Term, _CurrVTB, InVTB, OutVTB,
                InLE, OutLE, _Unif, _Unders, _Index, _PTerm, Error) :-
    nonvar(Error), !,
    OutVTB = InVTB,
    OutLE = InLE.
has_type_term__(_Type, Mode, _Term, _CurrVTB, InVTB, OutVTB,
                InLE, OutLE, _Unif, _Unders, _Index, _PTerm, _Error) :-
    checking_inner_bangs__, Mode \= '!', !,
    OutVTB = InVTB,
    OutLE = InLE.
%% basic builtin types
has_type_term__(term, Mode, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                Unif, Unders, Index, PTerm, Error) :-
    !,
    has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                         Unif, Unders, Index, PTerm, Error).
has_type_term__(atomic, _, Term, _, VTB, VTB, InLE, InLE, _, _, _, _, _Error) :-
    atomic(Term), !.
has_type_term__(atomic, _, Term, _, VTB, VTB, InLE, InLE, _, _, _, _, _Error) :-
    string(Term), !.
has_type_term__(num, _, Term, _, VTB, VTB, InLE, InLE, _, _, _, _, _Error) :-
    number(Term), !.
has_type_term__(int, _, Term, _, VTB, VTB, InLE, InLE, _, _, _, _, _Error) :-
    integer(Term), !.
has_type_term__(nat, _, Term, _, VTB, VTB, InLE, InLE, _, _, _, _, _Error) :-
    integer(Term), Term >= 0, !.
has_type_term__(atom, _, Term, _, VTB, VTB, InLE, InLE, _, _, _, _, _Error) :-
    atom(Term), !.
has_type_term__(string, _, Term, _, VTB, VTB, InLE, InLE, _, _, _, _, _Error) :-
    string(Term), !.
has_type_term__(T, _Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '$get_ultimate_functor'(Term, F),
    atom(F),
    '$type_info'(F, fun, _, _, _), !,
    add_default_args__(Term, FullTerm),
    strip_modes__(T, ST),
    has_type_term_function_call__(ST, FullTerm, CurrVTB, InVTB, OutVTB, InLE,
                                  OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(Type, _Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '$get_ultimate_functor'(Term, F),
    var(F), !,
    %% Term is compound and in order to name code the ultimate functor
    %% must an appropriate higher-order function
    (
      type_member_of__(F:M(T), CurrVTB), M = '!'
    ->
      has_type_term_var_function_call__(T, Type, Term, CurrVTB, InVTB, OutVTB,
                                        InLE, OutLE, Unif, Unders,
                                        Index, PTerm, Error)
    ;
      Error = mode_error(Index, PTerm, Term, '!', non_ground_var_functor(F))
    ).
%% list type
has_type_term__(list(_MT), _M, [], _, VTB, VTB, InLE, InLE, _, _, _, _, _) :- !.
has_type_term__(list(MT), M, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                Unif, Unders, Index, PTerm, Error) :-
    !,
    ( 
      Term = [_A|_B]
    ->
      push_modes_in__(M(list(MT)), M(list(InnerT))),
      has_type_list__(Term, M(list(InnerT)), CurrVTB, InVTB, OutVTB,
                      InLE, OutLE, Unif, Unders, Index, PTerm, Error)
    ;
      Term = '$list_constr'(_Pattern, _Goal)
    ->
      OutVTB = InVTB,
      has_type_comprehension_term__(Term, MT, CurrVTB, Unders,
                                    InLE, OutLE, Unif, Index, PTerm, Error)
    ;
      Error = type_error(Index, PTerm, Term, list(MT), default, _)
    ).
%% set type 
has_type_term__(set(T), _Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    push_modes_in__('!'(set(T)), '!'(set(MT))),
    (
      Term = '$set_enum'(Elems)
    ->
      has_type_term_iterate_one__(Elems, MT, CurrVTB, InVTB, OutVTB, InLE,
                                  OutLE, Unif, Unders, PTerm, Error)
    ;
      Term = '$set'(Elems)
    ->
      has_type_term_iterate_one__(Elems, MT, CurrVTB, InVTB, OutVTB, InLE,
                                  OutLE, Unif, Unders, PTerm, Error)
    ;
      Term = '$set_constr'(_Pattern, _Goal)
    ->
      has_type_comprehension_term__(Term, MT, CurrVTB, Unders,
                                    InLE, OutLE, Unif, Index, PTerm, Error)
    ;
      Error = type_error(Index, Term, Term, set(MT), default, _)
    ).
has_type_term__(Type, Mode, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                Unif, Unders, Index, PTerm, Error) :-
    nonvar(Type),
    '$type_info'(_, macro_type, Type, DefType, _), !,
    has_defined_type_term__(macro_type(DefType), Type, Mode, Term, CurrVTB,
                            InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                            Index, PTerm, Error).
has_type_term__(Type, Mode, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                Unif, Unders, Index, PTerm, Error) :-
    nonvar(Type),
    '$defined_type'(Type, DefType), !,
    has_defined_type_term__(DefType, Type, Mode, Term, CurrVTB, InVTB, OutVTB,
                            InLE, OutLE, Unif, Unders, Index, PTerm, Error).
% has_type_term__(tuple(T), Mode, '$tuple'(Elems), CurrVTB, InVTB, OutVTB,
%                 InLE, OutLE, Unif, Unders, _Index, PTerm, Error) :-
%     !,
%     push_modes_in__(Mode(tuple(T)), Mode(tuple(MT))),
%     has_type_term_iterate_one__(Elems, MT, CurrVTB, InVTB, OutVTB,
%                                 InLE, OutLE, Unif, Unders, PTerm, Error).
has_type_term__('$tuple_type'(T), Mode, '$tuple'(Elems), CurrVTB,  InVTB,
                OutVTB, InLE, OutLE, Unif, Unders, _Index, PTerm, Error) :-
    !,
    (
      length(T, N), length(Elems, N)
    ->
      push_modes_in__(Mode('$tuple_type'(T)), Mode('$tuple_type'(MT))),
      has_type_term_iterate__(MT, Elems, CurrVTB, InVTB, OutVTB,
                              InLE, OutLE, Unif, Unders, 1, PTerm, Error)
    ;
      Error = arity_error('$tuple'(Elems), '$tuple_type'(T))
    ).
has_type_term__('$union_type'(Types), Mode, Term, CurrVTB,  InVTB,
                OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    (
      member(T, Types),
      has_type_term__(T, Mode, Term, CurrVTB,  InVTB, OutVTB, InLE, OutLE,
                      Unif, Unders, Index, PTerm, Error),
      var(Error)
    ->
      true
    ;
      Error = type_error(Index, PTerm, Term, '$union_type'(Types), default, _)
    ).    
has_type_term__(typeE(T), _Mode, Term, _CurrVTB, InVTB, InVTB,
                InLE, InLE, _Unif, _Unders, Index, PTerm, Error) :-
    '$is_a_type'(Term),
    !,
    strip_modes__(T, ST),
    (
      Term = ST
    ->
      true
    ;
      Error = type_error(Index, PTerm, Term, typeE(T), default, _)
    ).
has_type_term__(atom_naming(dyn(T)), Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    '$get_code_types'(dyn, Term, dyn(T1)),
    strip_modes__(T1, T),
    has_type_term__(atom_naming(rel(T1)), Mode, Term, CurrVTB, InVTB, OutVTB,
                    InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(atom_naming(T), Mode, Atom, _CurrVTB, InVTB, InVTB,
                InLE, OutLE, _Unif, _Unders, Index, PTerm, Error) :-
    atom(Atom), !,
    (
      '$type_info'(Atom, Kind, _, _, _),
      '$code_kind'(Kind)
    ->
      has_type_term_atom_naming__(T, Mode, Atom, InLE, OutLE,
                                  Index, PTerm, Error)
    ;
      Error = type_error(Index, PTerm, Atom, atom_naming(T), default, _)
    ).
has_type_term__(term_naming(T), _Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    atom(Term), !,
    has_type_term__(atom_naming(T), _Mode, Term, CurrVTB, InVTB, OutVTB,
                    InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(dyn(T), Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    '$get_code_types'(dyn, Term, dyn(T1)),
    strip_modes__(T1, T),
    has_type_term__(atom_naming(rel(T1)), Mode, Term, CurrVTB, InVTB, OutVTB,
                    InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(rel(T), Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    has_type_term__(term_naming(rel(T)), Mode, Term, CurrVTB, InVTB, OutVTB,
                    InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(act(T), Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    has_type_term__(term_naming(act(T)), Mode, Term, CurrVTB, InVTB, OutVTB,
                    InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(tel(T), Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    has_type_term__(term_naming(tel(T)), Mode, Term, CurrVTB, InVTB, OutVTB,
                    InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__('->'(DT, RT), Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    has_type_term__(term_naming('->'(DT, RT)), Mode, Term, CurrVTB,
                    InVTB, OutVTB,
                    InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(tel_percept_term, Mode, Term, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
     compound(Term),
     '@functor'(Term, F, _),
     atom(F), !,
     has_type_code_term__(tel_percept_term, F, Mode, Term, CurrVTB, InVTB,
                          OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(dyn_term, Mode, Term, CurrVTB, InVTB, OutVTB,
                 InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
     compound(Term),
     '@functor'(Term, F, _),
     atom(F), !,
     has_type_code_term__(dyn_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                          InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(rel_term, Mode, Term, CurrVTB, InVTB, OutVTB,
                 InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
     compound(Term),
     '@functor'(Term, F, _),
     atom(F), !,
     has_type_code_term__(rel_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                          InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(act_term, Mode, Term, CurrVTB, InVTB, OutVTB,
                 InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
     compound(Term),
     '@functor'(Term, F, _),
     atom(F), !,
     has_type_code_term__(act_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                          InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term__(tel_term, Mode, Term, CurrVTB, InVTB, OutVTB,
                 InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
     compound(Term),
     '@functor'(Term, F, _),
     atom(F), !,
     has_type_code_term__(tel_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                          InLE, OutLE, Unif, Unders, Index, PTerm, Error).

has_type_term__(_Type, _Mode, Term, CurrVTB, InVTB, _OutVTB, InLE, _OutLE,
                Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    ip_set('$call_kind', rel),
    has_type__(Term, '?'(_), CurrVTB, InVTB, _, InLE, _, Unif,
               Unders, Index, PTerm, Error),
    nonvar(Error), !.
has_type_term__(Type, _Mode, Term, _, _InVTB, _OutVTB, _InLE, _OutLE,
                _Unif, _Unders, Index, PTerm, Error) :-
    Error = type_error(Index, PTerm, Term, Type, default, _).

'$code_kind'(fun).
'$code_kind'(rel).
'$code_kind'(act).
'$code_kind'(tel).

%%has_defined_type_term__(DefType, Type, Mode, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error) :- errornl(has_defined_type_term__(DefType, Type, Mode, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error)),fail.

has_defined_type_term__('$enum_type'(Enum), _Type, _Mode, Term, _CurrVTB,
                        InVTB, InVTB, InLE, InLE, _, _, _, _, _Error) :-
    member(Term, Enum), !.
has_defined_type_term__('$union_type'(Union), _Type, Mode, Term, CurrVTB,
                        InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                        Index, PTerm, Error) :-
    is_a_constructor_term_of_type__(Term, CType, TermN, ConstrN), !,
    (
      \+includes__('$union_type'(Union), CType)
    ->
      Error = type_error(Index, PTerm, Term, '$union_type'(Union), default, _)
    ;
      TermN = ConstrN
    ->
      has_type__(Term, Mode(CType), CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error)
    ;
      Error = arity_error(Term, CType)
    ).
has_defined_type_term__('$union_type'(Union), _Type, Mode, Term, CurrVTB,
                        InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                        Index, PTerm, Error) :-
     !,
    (
      member(T, Union),
      has_type__(Term, Mode(T), CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Err), 
      var(Err)
    ->
      true
    ;
      Error = type_error(Index, PTerm, Term, '$union_type'(Union), default, _)
    ).    
has_defined_type_term__('..'(N1, N2), _Type, _Mode, Term, _CurrVTB,
                        InVTB, InVTB, InLE, InLE, _, _, _, _, _Error) :-
      integer(Term),
      N1 =< Term, Term =< N2, !.
has_defined_type_term__('$constr_enum_type'(ConstrEnum), Type, Mode, Term,
                        CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
                        Unders, _Index, _PTerm, Error) :-
    compound(Term),
    '@functor'(Term, F, NF),
    member(T, ConstrEnum),
    '@functor'(T, F, NT),
    (
      NT = NF
    ->
      push_modes_in__(Mode(T), _(MT)),
      '@=..'(MT, [F|ArgTypes]),
      '@=..'(Term, [F|Args]),
      has_type_term_index_iterate__(ArgTypes, Args, 1, Term, CurrVTB,
                              InVTB, OutVTB, InLE, OutLE, Unif, Unders, Error)
    ;
      Error = arity_error(Term, Type)
    ),!.
has_defined_type_term__(macro_type('$union_type'(Union)), Type, Mode, Term,
                        CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
                        Unders, Index, PTerm, Error) :-
    !,
    has_defined_type_term__('$union_type'(Union), Type, Mode, Term,
                            CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
                            Unders, Index, PTerm, Err),
    '$get_enum_type_error'(Err, Error, Term,
                           '$union_type'(Union), Type).

has_defined_type_term__(macro_type(DefType), Type, Mode, Term, CurrVTB, InVTB,
                        OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    !,
    %% macro type
    has_type_term__(DefType, Mode, Term, CurrVTB, InVTB, OutVTB,
                    InLE, OutLE, Unif, Unders, Index, PTerm, Err),
    '$get_enum_type_error'(Err, Error, Term, DefType, Type).
has_defined_type_term__(_, Type, _Mode, Term, _CurrVTB,
                        InVTB, InVTB, InLE, InLE, _Unif, _, Index, PTerm, Error) :-
    Error = type_error(Index, PTerm, Term, Type, default, _).

%%'$get_enum_type_error'(A,B,C,D,E) :- errornl('$get_enum_type_error'(A,B,C,D,E)),fail.
'$get_enum_type_error'(Err, _Error, _Term, _DefType, _Type) :-
    var(Err), !.
'$get_enum_type_error'(type_error(Index, PTerm, Term, _, Reason, DecI),
                       Error, Term, DefType, _Type) :-
      !, Error = type_error(Index, PTerm, Term, DefType, Reason, DecI).
'$get_enum_type_error'(Err, Err, _, _, _).
    
    
%% Check to see if Term is of type term. 
%%has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error) :- errornl(has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error)),fail.
has_type_term_term__(_Term, _Mode, _CurrVTB, InVTB, InVTB, InLE, InLE,
                     _Unif, _Unders, _Index, _PTerm, Error) :-
    nonvar(Error), !.
has_type_term_term__(_Term, Mode, _CurrVTB, InVTB, InVTB, InLE, InLE,
                     _Unif, _Unders, _Index, _PTerm, _Error) :-
     checking_inner_bangs__, Mode \= '!', !.
has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, Index, PTerm, Error) :-
    var(Term), !, %% produced by a recursive call via @..
    has_type__(Term, Mode(term), CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, Index, PTerm, Error).
has_type_term_term__(Term, _Mode, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     _, _, _, _, _Error) :-
    atomic(Term), !,
    OutVTB = InVTB,
    OutLE = InLE.
has_type_term_term__(Term, _Mode, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     _Unif, _, _, _, _Error) :-
    string(Term), !,
    OutVTB = InVTB,
    OutLE = InLE.
has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, _Index, PTerm, Error) :-
    list(Term), Term = [H|T],
    !,
    has_type_term_term__(H, Mode, CurrVTB, InVTB, MidVTB, InLE, MidLE,
                         Unif, Unders, 0, PTerm, Error),
    has_type_term_term__(T, Mode, CurrVTB, MidVTB, OutVTB,
                         MidLE, OutLE, Unif, Unders, 0, PTerm, Error).
has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, _Index, PTerm, Error) :-
    Term = '$list_constr'(_Pattern, _Goal), !,
    OutVTB = InVTB,
    has_type_comprehension_term__(Term, Mode(term), CurrVTB, Unders,
                                  InLE, OutLE, Unif, 0, PTerm, Error).
has_type_term_term__(Term, _Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, _Index, PTerm, Error) :-
    Term = '$set_enum'(Elems), !,
    has_type_term_term__(Elems, '!', CurrVTB, InVTB, OutVTB, InLE, OutLE,
                         Unif, Unders, 0, PTerm, Error).
has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, _Index, PTerm, Error) :-
    Term = '$set'(Elems), !,
    has_type_term_term__(Elems, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                         Unif, Unders, 0, PTerm, Error).
has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, _Index, PTerm, Error) :-
    Term = '$set_constr'(_Pattern, _Goal), !,
    OutVTB = InVTB,
    has_type_comprehension_term__(Term, Mode(term), CurrVTB, Unders,
                                  InLE, OutLE, Unif, 0, PTerm, Error).
has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, _Index, PTerm, Error) :-
    Term = '$tuple'(Elems), !,
    has_type_term_term__(Elems, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                         Unif, Unders, 0, PTerm, Error).
has_type_term_term__(Term, _Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '$get_ultimate_functor'(Term, F),
    atom(F),
    '$type_info'(F, fun, _, _, _), !,
    has_type_term_function_call__(term, Term, CurrVTB, InVTB, OutVTB, InLE,
                                  OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term_term__(Term, _Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '$get_ultimate_functor'(Term, F),
    var(F), !,
    (
      
      type_member_of__(F:M(T), CurrVTB), M = '!'
    ->
      has_type_term_var_function_call__(T, term, Term, CurrVTB, InVTB, OutVTB,
                                        InLE, OutLE, Unif, Unders,
                                        Index, PTerm, Error)
    ;
      Error = mode_error(Index, PTerm, Term, '!', non_ground_var_functor(F))
    ).
has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '@functor'(Term, F, _NF),
    '$defined_type'(CType, '$constr_enum_type'(Types)), 
    member(T, Types),
    '@functor'(T, F, _NT),
    !,
    has_defined_type_term__('$constr_enum_type'(Types), CType, Mode, Term,
                            CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
                            Unders, Index, PTerm, Error).
has_type_term_term__(Term, Mode, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '@functor'(Term, F, _),
    atom(F),
    '$type_info'(F, _, _, _, _), !,
    has_type_code_term__(_TermType, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                         InLE, OutLE, Unif, Unders, Index, PTerm, Error).
has_type_term_term__(Term, _Mode, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     _Unif, _Unders, Index, PTerm, Error) :-
    compound(Term), !,
    Error = type_error(Index, PTerm, Term, term, not_a_term, _),
    OutVTB = InVTB,
    OutLE = InLE.
has_type_term_term__(Term, _Mode, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                     _Unif, _Unders, Index, PTerm, Error) :-
    Error = type_error(Index, PTerm, Term, term, default, _),
    OutVTB = InVTB,
    OutLE = InLE.

%%  has_type_term_iterate__(ArgTypes, Args, CurrVTB, InVTB, OutVTB,
%%                          InLE, OutLE, Unif, Unders, Index, PTerm, Error)
%% Same as has_typ__ except pairwise iterating over list of types
%% and list of args
%%has_type_term_iterate__(ArgTypes, Args, CurrVTB, MidVTB, OutVTB, MidLE, OutLE, Unif, Unders, Index, PTerm, Error) :- errornl(has_type_term_iterate__(ArgTypes, Args, CurrVTB, MidVTB, OutVTB, MidLE, OutLE, Unif, Unders, Index, PTerm, Error)),fail.
has_type_term_iterate__(_, _, _CurrVTB, InVTB, InVTB, InLE, InLE,
                        _Unif, _, _Index, _PTerm, Error) :-
    nonvar(Error), !.
has_type_term_iterate__([], [], _CurrVTB, InVTB, InVTB, InLE, InLE,
                        _Unif, _, _, _, _Error).
has_type_term_iterate__([], [_|_], _CurrVTB, _InVTB, _OutVTB, _InLE, _OutLE,
                        _Unif, _, _, _, _Error) :-
    !,
    errornl(unmatched_type_arities(has_type_term_iterate)),
    fail.
has_type_term_iterate__([_|_], [], _CurrVTB, _InVTB, _OutVTB, _InLE, _OutLE,
                        _Unif, _, _, _, _Error) :-
    !,
    errornl(unmatched_type_arities(has_type_term_iterate)),
    fail.
has_type_term_iterate__([AT|ArgTypes], [A|Args], CurrVTB, InVTB, OutVTB,
                        InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    has_type__(A, AT, CurrVTB, InVTB, MidVTB, InLE, MidLE, Unif, Unders,
               Index, PTerm, Error),
    Index1 is Index+1,
    has_type_term_iterate__(ArgTypes, Args, CurrVTB, MidVTB, OutVTB,
                            MidLE, OutLE, Unif, Unders, Index1, PTerm, Error).
%% has_type_term_iterate_one__(Args, MType, CurrVTB, InVTB, OutVTB,
%%                            InLE, OutLE, Unif, Unders, PTerm, Error)
%% Similar to has_type_term_iterate__ except all args have the same type
has_type_term_iterate_one__([], _, _CurrVTB, InVTB, InVTB, InLE, InLE,
                            _Unif, _, _, _Error).
has_type_term_iterate_one__([_|_], _, _CurrVTB, InVTB, InVTB,
                            InLE, InLE, _Unif, _, _, Error) :-
    nonvar(Error), !.
has_type_term_iterate_one__([A|Args], MT, CurrVTB, InVTB, OutVTB,
                            InLE, OutLE, Unif, Unders, PTerm, Error) :-
    has_type__(A, MT, CurrVTB, InVTB, MidVTB, InLE, MidLE, Unif, Unders,
               0, PTerm, Error),
    has_type_term_iterate_one__(Args, MT, CurrVTB, MidVTB, OutVTB,
                                MidLE, OutLE, Unif, Unders, PTerm, Error).

%%has_type_list__(Term, MT, CurrVTB, InVTB, OutVTB,
%%                InLE, OutLE, Unif, Unders, Index, PTerm, Error)
%% Checks that Term is of moded type MT where Term is known ot be a list
%% Assumes the modes have been pushed in to MT
%%has_type_list__(A,B,C,D,E,F,G,H,I,J,K, L) :- errornl(has_type_list__(A,B,C,D,E,F,G,H,I,J,K, L)),fail.
has_type_list__(_Term, _MT, _CurrVTB, InVTB, InVTB,
                InLE, InLE, _Unif, _Unders, _Index, _PTerm, Error) :-
    nonvar(Error), !.
has_type_list__(Term, MT, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    var(Term), !,
    has_type__(Term, MT, CurrVTB, InVTB, OutVTB, InLE, OutLE,
               Unif, Unders, Index, PTerm, Error).
has_type_list__([], _, _CurrVTB, InVTB, InVTB, InLE, InLE,
                            _Unif, _, _, _, _Error).
has_type_list__([_|_], _, _CurrVTB, InVTB, InVTB,
                InLE, InLE, _Unif, _, _, _, Error) :-
    nonvar(Error), !.
has_type_list__(Term, MT, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    \+list(Term), !,
    has_type__(Term, MT, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif,
           Unders, Index, PTerm, Error).
has_type_list__([A|As], MT, CurrVTB, InVTB, OutVTB,
                InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    MT = _(list(MTA)),
    has_type__(A, MTA, CurrVTB, InVTB, MidVTB, InLE, MidLE, Unif, Unders,
               Index, PTerm, Error),
    has_type_list__(As, MT, CurrVTB, MidVTB, OutVTB,
                    MidLE, OutLE, Unif, Unders, Index, PTerm, Error).

%% has_type_term_index_iterate__(ArgTypes, Args, Index, Term,
%%                              CurrVTB, InVTB,
%%                              OutVTB, InLE, OutLE, Unif, Unders, Error)
%% Used for checking the arguments of a compound term - typeically a
%% function call that keeps track of the index of the arg being checked
%% The index is used if an error occurs to identify the argument
%%has_type_term_index_iterate__(ArgTypes, Args, Index, Term, CurrVTB, MidVTB, OutVTB, MidLE, OutLE, Unif, Unders, Error) :- errornl(has_type_term_index_iterate__(ArgTypes, Args, Index, Term, CurrVTB, MidVTB, OutVTB, MidLE, OutLE, Unif, Unders, Error)),fail.
has_type_term_index_iterate__(_, _, _, _, _CurrVTB, InVTB, InVTB, InLE, InLE,
                        _Unif, _, Error) :-
    nonvar(Error), !.
has_type_term_index_iterate__([], [], _,_,  _CurrVTB, InVTB, InVTB, InLE, InLE,
                        _Unif, _, _Error).
has_type_term_index_iterate__([], [_|_], _,_,  _CurrVTB, _InVTB, _OutVTB,
                              _InLE, _OutLE, _Unif, _, _Error) :-
    !,
    errornl(unmatched_type_arities(has_type_term_index_iterate)),
    fail.
has_type_term_index_iterate__([_|_], [], _,_,  _CurrVTB, _InVTB, _OutVTB,
                              _InLE, _OutLE, _Unif, _, _Error) :-
    !,
    errornl(unmatched_type_arities(has_type_term_index_iterate)),
    fail.
has_type_term_index_iterate__([AT|ArgTypes], [A|Args], Index, Term,
                              CurrVTB, InVTB,
                              OutVTB, InLE, OutLE, Unif, Unders, Error) :-
    has_type__(A, AT, CurrVTB, InVTB, MidVTB, InLE, MidLE, Unif, Unders,
               Index, Term, Error),
    Index1 is Index+1,
    has_type_term_index_iterate__(ArgTypes, Args, Index1, Term, CurrVTB, MidVTB,
                                  OutVTB, MidLE, OutLE, Unif, Unders, Error).


%% Type is a variable, Term is a non-variable
%% Used when the type is a variable - the type is inferred and we add a LE
%% constraint that the infered type is LE Type
%%type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index,PTerm,Err) :- errornl(type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index,PTerm, Err)),fail.

type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    Term = '$list_constr'(_Pattern, _Goal), !,
    (
      var(Mode)
    ->
      Mode = '!'
    ;
      true
    ),
    has_type__(Term, '!'(list('!'(TermElemType))), CurrVTB, InVTB, OutVTB,
               InLE, MidLE, Unif, Unders, Index, PTerm, Error),
    type_le_or_err__(Mode, Term, list(TermElemType), Type, MidLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    Term = '$set_constr'(_Pattern, _Goal), !,
    (
      var(Mode)
    ->
      Mode = '!'
    ;
      true
    ),
    has_type__(Term, '!'(set('!'(TermElemType))), CurrVTB, InVTB, OutVTB,
               InLE, MidLE, Unif, Unders, Index, PTerm, Error),
    type_le_or_err__(Mode, Term, set(TermElemType), Type, MidLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '$get_ultimate_functor'(Term, F),
    atom(F),
    '$type_info'(F, fun, _, _, _), !,
    add_default_args__(Term, FullTerm),
    (
      var(Mode)
    ->
      Mode = '!'
    ;
      true
    ),
    '$get_ho_types'(FullTerm, TermType, CurrVTB, Unif, Unders, Error),
    OutVTB = InVTB,
    type_le_or_err__(Mode, FullTerm, TermType, Type, InLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '$get_ultimate_functor'(Term, F),
    var(F), !,
    (
      type_member_of__(F:M(T), CurrVTB), M = '!',
      has_type_term_var_function_call__(T, TermType, Term, CurrVTB, InVTB,
                                        OutVTB, InLE, MidLE, Unif,
                                        Unders, Index, PTerm, Error),
      type_le_or_err__(Mode, Term, TermType, Type, MidLE, OutLE,
                       Index, PTerm, Error),
      var(Error)
    ->
      true
    ;
      Error = type_error(Index, PTerm, Term, term, not_a_term, _)
    ).

type_inference__([], Mode, Type, _CurrVTB, InVTB, OutVTB,
                 InLE, OutLE, _Unif, _, Index, PTerm, Error) :-
    !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, [], list('$empty_type'), Type, InLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB,
                 InLE, OutLE, _Unif, _, Index, PTerm, Error) :-
    '$is_in_enum_type'(Term, _, _), !,
    '$smallest_enum_type_for'(Term, TermType),
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, TermType, Type, InLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _, Index, PTerm, Error) :-
    '$is_a_type'(Term),
    !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, typeE(Term), Type, InLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, _Mode, Type, _CurrVTB, InVTB, InVTB,
                 InLE, InLE, _Unif, _, _Index, _PTerm, Error) :-
    compound(Term),
    functor(Term, F, _),
    '$user_type_info'(F, defined_type, _, _, _), !,
    Error = arity_error(Term, Type).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _, Index, PTerm, Error) :-
    atom(Term),
    '$type_info'(Term, _, _, _, _),
    '$belief_type_info'(Term,_,_),
    !,
    '$get_code_types'(dyn, Term, CodeType),
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, atom_naming(CodeType), Type,
                     InLE, OutLE, Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _, Index, PTerm, Error) :-
    atom(Term),
    '$type_info'(Term, Kind, _, _, _),
    '$code_kind'(Kind),
    !,
    '$get_code_types'(Kind, Term, CodeType),
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, atom_naming(CodeType), Type,
                     InLE, OutLE, Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _, Index, PTerm, Error) :-
    atom(Term), !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, atom, Type, InLE, OutLE, Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _, Index, PTerm, Error) :-
    string(Term), !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, string, Type, InLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _, Index, PTerm, Error) :-
    integer(Term), Term >= 0, !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, nat, Type, InLE, OutLE, Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _, Index, PTerm, Error) :-
    integer(Term), !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, int, Type, InLE, OutLE, Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _, Index, PTerm, Error) :-
    number(Term), !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, num, Type, InLE, OutLE, Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _Unders, Index, PTerm, Error) :-
    Term = '$set_enum'([]), !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, set('$empty_type'), Type, InLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    Term = '$set_enum'(Elems), !,
    type_inference__(Elems, '!', list(SType), CurrVTB, InVTB, OutVTB, InLE,
                     MidLE, Unif, Unders, 0, PTerm, Error),
    type_le_or_err__(Mode, Term, set(SType), Type, MidLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _Unders, Index, PTerm, Error) :-
    Term = '$set'([]), !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, Term, set('$empty_type'), Type, InLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    Term = '$set'(Elems), !,
    type_inference__(Elems, '!', list(SType), CurrVTB, InVTB, OutVTB, InLE,
                     MidLE, Unif, Unders, Index, PTerm, Error),
    type_le_or_err__(Mode, Term, set(SType), Type, MidLE, OutLE,
                     Index, PTerm, Error).

type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    list(Term), !,
    has_type__(Term, Mode(list(Mode(TermElemType))), CurrVTB, InVTB, OutVTB,
               InLE, MidLE, Unif, Unders, Index, PTerm, Error),
    type_le_or_err__(Mode, Term, list(TermElemType), Type, MidLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    '$functor_is_constr_enum_type'(Term, CTerm), !,
    '$smallest_constr_enum_type_for'(CTerm, TermType),
    push_modes_in__(Mode(TermType), ModedType),
    has_type__(Term, ModedType, CurrVTB, InVTB, OutVTB, InLE, MidLE,
               Unif, Unders, Index, PTerm, Error),
    type_le_or_err__(Mode, Term, TermType, Type, MidLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '@functor'(Term, F, _),
    atom(F),
    '$type_info'(F, _, _, _, _), !,
    has_type_code_term__(TermType, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                         InLE, MidLE, Unif, Unders, Index, PTerm, Error),
    type_le_or_err__(Mode, Term, TermType, Type, MidLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, Mode, Type, _CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 _Unif, _Unders, Index, PTerm, Error) :-
    compound(Term),
    '@functor'(Term, F, _),
    F == '$tuple',
    var(Type),
    Term = '$tuple'([]), !,
    OutVTB = InVTB,
    type_le_or_err__(Mode, [], tuple('$empty_type'), Type, InLE, OutLE,
                     Index, PTerm, Error).

type_inference__(Term, Mode, Type, CurrVTB, InVTB, OutVTB, InLE, OutLE,
                 Unif, Unders, Index, PTerm, Error) :-
    compound(Term),
    '@functor'(Term, F, _),
    F == '$tuple', !,
    Term = '$tuple'(Args),
    length(Args, N),
    length(Types, N),
    TType = '$tuple_type'(Types),
    has_type__(Term, Mode(TType), CurrVTB, InVTB, OutVTB, InLE, MidLE,
               Unif, Unders, Index, PTerm, Error),
    type_le_or_err__(Mode, Term, TType, Type, MidLE, OutLE,
                     Index, PTerm, Error).
type_inference__(Term, _Mode, _Type, _CurrVTB, InVTB, InVTB, InLE, InLE,
                 _Unif, _Unders, Index, PTerm, Error) :-
    compound(Term), !,
    Error = type_error(Index, PTerm, Term, term, not_a_term, _).
type_inference__(Term, _Mode, _Type, _CurrVTB, InVTB, InVTB,
                 InLE, InLE, _Unif, _, Index, PTerm, Error) :-
    Error = type_error(Index, PTerm, Term, term, default, _).

%% has_type_code_term__(CodeType, F, Mode, Term, CurrVTB, InVTB, OutVTB,
%%                      InLE, OutLE, Unif, Unders, Index, PTerm, Error)
%% CodeType is one of tel_percept_term, dyn_term, rel_term, act_term,
%% tel_term
%% Checks that Term is a term representing a "callable term" with
%% appropriate argument types - the modes of the arguments are not considered.
%%has_type_code_term__(Type, F, Mode, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error) :- errornl(has_type_code_term__(Type, F, Mode, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error)),fail.
has_type_code_term__(tel_percept_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    '$percept_type_info'(F,Types,_), !,
    '@=..'(Term, [F|Args]),
    strip_modes__(Types, StrippedTypes),
    '$check_arities'(['$tuple'(Args)], ['$tuple_type'(StrippedTypes)],
                     Term, Error),
    has_type_term__('$tuple_type'(StrippedTypes), Mode, '$tuple'(Args),
                    CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                    Index, PTerm, Error).
has_type_code_term__(dyn_term, F, '!??', Term, CurrVTB, _InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, _Index, _PTerm, Error) :-
    '$belief_type_info'(F,_,_), !,
    type_check_call__(Term, rel, CurrVTB, OutVTB, Unders, InLE, OutLE,
                      Unif, _, Error, _).    
has_type_code_term__(dyn_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    '$belief_type_info'(F,Types,_), !,
    '@=..'(Term, [F|Args]),
    strip_modes__(Types, StrippedTypes),
    '$check_arities'(['$tuple'(Args)], ['$tuple_type'(StrippedTypes)],
                     Term, Error),
    has_type_term__('$tuple_type'(StrippedTypes), Mode, '$tuple'(Args),
                    CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                    Index, PTerm, Error). %, var(Error), !.
has_type_code_term__(rel_term, _F, '!??', Term, CurrVTB, _InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, _Index, _PTerm, Error) :-
    !,
    type_check_call__(Term, rel, CurrVTB, OutVTB, Unders, InLE, OutLE,
                      Unif, _, Error, _).
has_type_code_term__(rel_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    '$type_info'(F, rel, Types, _, _), 
    add_default_args__(Term, FullTerm),
    '@=..'(FullTerm, [F|Args]),
    strip_modes__(Types, StrippedTypes),
    '$check_arities'(['$tuple'(Args)], ['$tuple_type'(StrippedTypes)],
                     Term, Error),
    has_type_term__('$tuple_type'(StrippedTypes), Mode, '$tuple'(Args),
                    CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                    Index, PTerm, Error),
    var(Error), !.
has_type_code_term__(act_term, F, '!??', Term, CurrVTB, InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    nonvar(F),
    '$type_info'(F, act, Types, _, _), !,
    add_default_args__(Term, FullTerm),
    '@=..'(FullTerm, [F|Args]),
    '$check_arities'(['$tuple'(Args)], ['$tuple_type'(Types)],
                     Term, Error),
    has_type__('$tuple'(Args), '$tuple_type'(Types), 
               CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders,
               Index, PTerm, Error).
has_type_code_term__(act_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    '$type_info'(F, act, Types, _, _),
    add_default_args__(Term, FullTerm),
    '@=..'(FullTerm, [F|Args]),
    strip_modes__(Types, StrippedTypes),
    '$check_arities'(['$tuple'(Args)], ['$tuple_type'(StrippedTypes)],
                     Term, Error),
    has_type_term__('$tuple_type'(StrippedTypes), Mode, '$tuple'(Args),
                    CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                    Index, PTerm, Error),
    var(Error), !.
has_type_code_term__(tel_term, F, Mode, Term, CurrVTB, InVTB, OutVTB,
                     InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    '$type_info'(F, tel, Types, _, _), 
    add_default_args__(Term, FullTerm), !,
    '@=..'(FullTerm, [F|Args]),
    strip_modes__(Types, StrippedTypes),
    '$check_arities'(['$tuple'(Args)], ['$tuple_type'(StrippedTypes)],
                     Term, Error),
    has_type_term__('$tuple_type'(StrippedTypes), Mode, '$tuple'(Args),
                    CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders,
                    Index, PTerm, Error).
has_type_code_term__(TermType, _F, _Mode, Term, _CurrVTB, InVTB, InVTB,
                     InLE, InLE, _Unif, _, Index, PTerm, Error) :-
    Error = type_error(Index, PTerm, Term, TermType, default, _).

%% has_type_term_atom_naming__(T, Mode, Atom, InLE, OutLE, Index, PTerm, Error)
%% Atom is a given atom that names code, T is a type and InLE will be updated
%% to OutLE by adding a T LE code_type_of_Atom
has_type_term_atom_naming__(T, Mode, Atom, InLE, OutLE, Index, PTerm, Error) :-
    %% there might be multiple declarations for Atom - type check each
    %% against T and collect the results
    findall(type_info__(CodeType, Err, Atom),
            has_type_term_atom_naming_aux__(T, CodeType, Atom, InLE,
                                            Index, PTerm, Err),
            TypeInfo),
    filter('$var_err_type_info__', TypeInfo, AltTypes),
    %% AltTypes is the list of type_info__ terms that don't contain type errors
    (
      AltTypes = []
    ->
      %% All possibilities lead to type errors - so there is a type error
      %% to be generate
      OutLE = InLE,
      gen_atom_naming_error__(Atom, TypeInfo, Index, PTerm, Error)
    ;
      AltTypes = [type_info__(T0, _, _)]
    ->
      %% There is only one valid code type so choose that type
      type_le_or_err__(Mode, Atom, T0, T, InLE, OutLE, Index, PTerm, Error)
    ;
      collect_vars(T, TVars), TVars = []
    ->
      %% T contains no type variables and so no LE constraints are required
      OutLE = InLE
    ;
      %% Otherwise collect all consistent types and add an le_alts constraints
      %% for all the alternative types.
      findall(TX, member(type_info__(TX, _Error, _),AltTypes), AllTX),
      OutLE = [le_alts(Atom, AllTX, T)|InLE]
    ).

%% gen_atom_naming_error__(Atom, Errors, Index, PTerm, Error)
%% If all errors are then same then this is the error returned
%% OW there are different errors and so all we can do is be generic about the
%% error
%%gen_atom_naming_error__(Atom, Errors, Index, PTerm, Error) :- errornl(gen_atom_naming_error__(Atom, Errors, Index, PTerm, Error)),fail.
gen_atom_naming_error__(Atom, Errors, Index, PTerm, Error) :-
    findall(TE, member(type_info__(_, TE, _), Errors), Types),
    Types = [E1|Rest],
    (
      forall(member(E, Rest), E = E1)
    ->
      Error = E1
    ;
      Error = type_error(Index, Atom, PTerm, no_suitable_moded_type, multidecl,_)
    ).
%%has_type_term_atom_naming_aux__(T, CT, Atom, LE, Index, PTerm, Err) :- errornl(has_type_term_atom_naming_aux__(T, CT, Atom, LE, Index, PTerm, Err)),fail.    
has_type_term_atom_naming_aux__(T, CT, Atom, LE, Index, PTerm, Err) :-
    '$type_info'(Atom, fun, _, _, _), !,
    '$type_info'(Atom, _, CT, _, _),
    type_le_or_err__('!', Atom, CT, T, LE, _, Index, PTerm, Err).
has_type_term_atom_naming_aux__(T, CT, Atom, LE, Index, PTerm, Err) :-
    '$type_info'(Atom, Kind, CT0, _, _),
    '$code_kind'(Kind),
    CT =  Kind('$tuple_type'(CT0)),
    type_le_or_err__('!', Atom, CT, T, LE, _, Index, PTerm, Err).

%% has_type_term_function_call__(Type, Term, CurrVTB, InVTB, OutVTB, InLE,
%%                              OutLE, Unif, Unders, Index, PTerm, Error)
%% Similar to all the other type check calls except Term looks like a function
%% call and this check tests that it is indeed a function call with
%% the appropriatly types args and whose return type is Type

%%has_type_term_function_call__(Type, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-errornl(has_type_term_function_call__(Type, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error)),fail.

has_type_term_function_call__(_Type, Term, CurrVTB, InVTB, OutVTB, InLE,
                              OutLE, _Unif, _Unders, _Index, _PTerm, Error) :-
    CurrVTB = head, !,
    %% Function calls not allowed in the head of a rule
    Error = function_call_in_head(Term),
    InLE = OutLE,
    InVTB = OutVTB.
has_type_term_function_call__(Type, Term, CurrVTB, InVTB, OutVTB,
                              InLE, OutLE, Unif, Unders, Index, PTerm, Error) :-
    findall(term_type(Term, FType),
            ('$get_ultimate_functor'(Term, F),
                '$type_info'(F, fun, FType, _, _)),
            AllFTypes),
    has_type_term_function_call_types__(AllFTypes, Type, Term, CurrVTB,
                                        InVTB, OutVTB, InLE, OutLE, Unif,
                                        Unders, Index, PTerm, Error).

has_type_term_function_call_types__(AllFTypes, _Type, Term, CurrVTB, InVTB,
                                    InVTB, InLE, InLE, Unif, Unders,
                                    _Index, _PTerm, Error) :-
    once((
          AllFTypes = member(term_type(Term, FType),AllFTypes),
          '$get_function_type_info_aux'(Term, FType, Args, ArgTypes, _ResultType0),
          check_function_arities__(Args, ArgTypes, Term, Err),
          var(Err)
         )),
    reverse(Args, RevArgs),
    reverse(ArgTypes, RevArgTypes),
    flatten_tuple_args_list__(RevArgs, RevArgTypes,
                              AllArgs, _AllTypes, Error),
    ip_set('$call_kind', rel),
    '$check_valid_terms'(AllArgs, 1, '!', Term, CurrVTB, Unif, Unders, Error),
    nonvar(Error), !.
has_type_term_function_call_types__(AllFTypes, Type, Term, CurrVTB, InVTB,
                                    OutVTB, InLE, OutLE, Unif, Unders,
                                    Index, PTerm, Error) :-
    ground(Type), !,
    findall(te__(Type, Err, Term, []),
            (member(term_type(Term, FType), AllFTypes),
                FType = (_DT -> RT),
                '$atom_type_then_le'(RT, Type),
                has_type_term_function_call_aux1__(Type, FType, Term, CurrVTB,
                                                   InVTB, InLE, Unif,
                                                   Unders, Err)),
            TypeInfo),
    filter('$var_err__', TypeInfo, Types),
    (
      TypeInfo = []
    ->
      Error = type_error(Index, PTerm, Term, Type, default, _)
    ;
      Types = []
    ->
      get_type_error__(Term, TypeInfo, Error)
    ;
      sort(Types, SortedTypes, te_order__),
      member(te__(Type, _Error, Term, _), SortedTypes),
      %% Note: on backtracking this will return suitable types
      %% in type order (smallest first)
      InLE = OutLE,
      InVTB = OutVTB
    ).
    
has_type_term_function_call_types__(AllFTypes, Type, Term, CurrVTB,
                                    InVTB, OutVTB, InLE, OutLE, Unif,
                                    Unders, _Index, _PTerm, Error) :-
    findall(type_info__(Type, Err, Term),
            (member(term_type(Term, FType), AllFTypes),
                has_type_term_function_call_aux1__(Type, FType, Term, CurrVTB,
                                                   InVTB, InLE, Unif,
                                                   Unders, Err)),
            TypeErr),
    filter('$var_err_type_info__', TypeErr, Types),
    (
      Types = []
    ->
      %%freeze_term((Term, TypeErr), FVars),
      get_type_error__(Term, TypeErr, Error) %%,
      %%thaw_term(FVars)
    ;
      Types = [type_info__(Type, _Error, Term)]
    ->
      InLE = OutLE,
      InVTB = OutVTB
    ;
      collect_vars(Type, TVars), TVars = []
    ->
      InLE = OutLE,
      InVTB = OutVTB
    ;
    %   Types = [type_info__(_, Type, _Error, Term)|_],
    %   compound(Type), '@functor'(Type, F, _), '$code_kind'(F),
    %   TypeOutVars = []
    % ->
      
     Types = [type_info__(Type1, _Error, Term)|_],
      compound(Type1), '@functor'(Type1, F, _), '$code_kind'(F)
    ->
      %%sort(Types, SortedTypes, bind_alts_le__),
      findall(TX, member(type_info__(TX, _Error, Term),Types), AllTX),
      OutLE = [le_alts(Term, AllTX, Type)|InLE],
      InVTB = OutVTB
    ;
      findall(te__(TX, _, Term, _), member(type_info__(TX, _Error, Term), Types),
              XTypes),
      sort(XTypes, SortedXTypes, te_order__),
      member(te__(Type, _Error, Term, _), SortedXTypes),
      InLE = OutLE,
      InVTB = OutVTB
    ).

'$atom_type_then_le'(T1, T2) :- atom(T2), !, type_le__(T1, T2, [], _).
'$atom_type_then_le'(_, _).


%% Used above for filtering out type/mode errors
'$var_err__'(te__(_, Err, _, _)) :- var(Err).
'$var_err_type_info__'(type_info__(_, Err, _)) :- var(Err).

%%has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error) :- errornl(has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB, InLE, OutLE, Unif, Unders, Index, PTerm, Error)),fail.
has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB,
                                  InLE, OutLE, Unif, Unders, Index, PTerm,
                                  Error) :-
    nonvar(FType),
    '$type_info'(_,macro_type,FType,FullFType,_), !,
    has_type_term_var_function_call__(FullFType, Type, Term, CurrVTB,
                                      InVTB, OutVTB, InLE, OutLE, Unif,
                                      Unders, Index, PTerm, Error).
has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB,
                                  InLE, OutLE, Unif, Unders, Index, PTerm,
                                  Error) :-
    FType = atom_naming(_(RType)), !,
    has_type_term_var_function_call__(RType, Type, Term, CurrVTB, InVTB, OutVTB,
                                      InLE, OutLE, Unif, Unders, Index, PTerm,
                                      Error).
has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB,
                                  InLE, OutLE, Unif, Unders, Index, PTerm,
                                  Error) :-
    FType = term_naming(_(RType)), !,
    has_type_term_var_function_call__(RType, Type, Term, CurrVTB, InVTB, OutVTB,
                                      InLE, OutLE, Unif, Unders, Index, PTerm,
                                      Error).
has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB,
                                  InLE, OutLE, Unif, Unders, Index, PTerm,
                                  Error) :-
    compound(FType), functor(FType, '->', 2), !,
    has_type_term_var_function_call_aux__([FType], Type, Term, CurrVTB, InVTB,
                                          OutVTB, InLE, OutLE, Unif,
                                          Unders, Index, PTerm, Error).
has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB,
                                  InLE, OutLE, Unif, Unders, Index, PTerm,
                                  Error) :-
    compound(FType), 
    FType = term_naming(FType1),
    compound(FType1), functor(FType1, '->', 2), !,
    has_type_term_var_function_call_aux__([FType1], Type, Term, CurrVTB, InVTB,
                                          OutVTB, InLE, OutLE, Unif,
                                          Unders, Index, PTerm, Error).
has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB,
                                  InLE, OutLE, Unif, Unders, Index, PTerm,
                                  Error) :-
    compound(FType), 
    FType = atom_naming(FType1),
    compound(FType1), functor(FType1, '->', 2), !,
    has_type_term_var_function_call_aux__([FType1], Type, Term, CurrVTB, InVTB,
                                          OutVTB, InLE, OutLE, Unif,
                                          Unders, Index, PTerm, Error).
has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB,
                                  InLE, OutLE, Unif, Unders, Index, PTerm,
                                  Error) :-
    compound(FType),
    FType = atom_naming(FType1),
    compound(FType1), FType1 = alt_types(FType2),
    FType2 = [FType3|_],
    compound(FType3), functor(FType3, '->', 2), !,
    has_type_term_var_function_call_aux__(FType2, Type, Term, CurrVTB, InVTB,
                                          OutVTB, InLE, OutLE, Unif,
                                          Unders, Index, PTerm, Error).
has_type_term_var_function_call__(FType, Type, Term, CurrVTB, InVTB, OutVTB,
                                  InLE, OutLE, Unif, Unders, Index, PTerm,
                                  Error) :-
    compound(FType),
    FType = term_naming(FType1),
    compound(FType1), FType1 = alt_types(FType2),
    FType2 = [FType3|_],
    compound(FType3), functor(FType3, '->', 2), !,
    has_type_term_var_function_call_aux__(FType2, Type, Term, CurrVTB, InVTB,
                                          OutVTB, InLE, OutLE, Unif,
                                          Unders, Index, PTerm, Error).
has_type_term_var_function_call__(_FType, _Type, Term, _CurrVTB, _InVTB,
                                  _OutVTB, _InLE, _OutLE, _Unif, _,
                                  Index, PTerm, Error) :-
    Error = type_error(Index, PTerm, Term, term, not_a_function_call, _).

has_type_term_var_function_call_aux__(FTypes, Type, Term, CurrVTB, InVTB,
                                      OutVTB, InLE, OutLE, Unif,
                                      Unders, _Index, _PTerm, Error) :-
    ground(Type), !,
    findall(te__(Type, Err, Term, []),
            (
              member(FType, FTypes),
              %'$get_function_type_info_aux'(Term, FType, _Args, _ArgTypes, RT),
              %type_le__(RT, Type),
              has_type_term_function_call_aux1__(Type, FType, Term, CurrVTB,
                                                 InVTB, InLE, Unif, Unders, Err)
            ),
            TypeErr),
    filter('$var_err__', TypeErr, Types),
    (
      Types = []
    ->
      %%freeze_term((Term, TypeErr), FVars),
      get_type_error__(Term, TypeErr, Error) %%,
      %%thaw_term(FVars)
    ;
      member(te__(Type, Error, Term, _), Types),
      InLE = OutLE,
      InVTB = OutVTB
    ).
has_type_term_var_function_call_aux__(FTypes, Type, Term, CurrVTB, InVTB,
                                      OutVTB, InLE, OutLE, Unif,
                                      Unders, _Index, _PTerm, Error) :-
    collect_vars(FTypes, FTypesVars),
    findall(te__(Type, Err, Term, FTypesVars),
            (
              member(FType, FTypes),
              has_type_term_function_call_aux1__(Type, FType, Term, CurrVTB,
                                                 InVTB, InLE, Unif, Unders, Err)
            ),
            TypeErr),
    filter('$var_err__', TypeErr, Types),
    (
      Types = []
    ->
      %%freeze_term((Term, TypeErr), FVars),
      get_type_error__(Term, TypeErr, Error) %%,
      %%thaw_term(FVars)
    ;
      member(te__(Type, Error, Term, FTypesVars), Types),
      InLE = OutLE,
      InVTB = OutVTB
    ).

%%has_type_term_function_call_aux1__(ReqType, FType, Term, CurrVTB, InVTB,InLE, Unif, Unders,  Error) :-errornl(has_type_term_function_call_aux1__(ReqType, FType, Term, CurrVTB, InVTB,InLE, Unif, Unders, Error)),fail.
has_type_term_function_call_aux1__(_ReqType, _FType, _Term, _CurrVTB, _InVTB,
                                   _InLE, _Unif, _Unders, Error) :-
    nonvar(Error), !.
has_type_term_function_call_aux1__(ReqType, FType, Term, CurrVTB, InVTB,
                                   InLE, Unif, Unders, Error) :-
    '$get_function_type_info_aux'(Term, FType, Args, ArgTypes, ResultType0),
    '$strip_term_naming'(ResultType0, ResultType),
    reverse(Args, RevArgs),
    reverse(ArgTypes, RevArgTypes),
    check_function_arities__(Args, ArgTypes, Term, Err),
    flatten_tuple_args_list__(RevArgs, RevArgTypes,
                              AllArgs, AllTypes, Err),
    '$add_annotations'(AllTypes, AnnTypes, '!'),
    has_type_term_index_iterate__(AnnTypes, AllArgs, 1, Term, CurrVTB, InVTB, _,
                                  InLE, OutLE, Unif, Unders, Err),
    (
      nonvar(Err)
    ->
      Error = Err
    ;
      type_le_or_err__('!', Term, ResultType,ReqType, OutLE, LE,
                       1, Term, Error),
      bind_type_constraints__(LE, Error) %, var(Error)
    ).


%% add declared default args to call
add_default_args__(Call, FullCall) :-
    '$default_args'(Call, FullCall), !.
add_default_args__(Call, Call).

%% Recursively push the outer mode into the inner types
%% EG for push_modes_in__(!list(int), R)  R = !list(!int)
%% and push_modes_in__(!list(?int), R)  R = !list(?int)
%%push_modes_in__(A,B) :- errornl(push_modes_in__(A,B)),fail.
push_modes_in__(M(DT), R) :-
    moded__(DT), DT = M(DT1), !,
    push_modes_in__(M(DT1), R).
push_modes_in__(M(DT), R) :-
    compound(DT), functor(DT, F, _), F == '$$var$$', !,
    %% The inner type is '$$var$$'(T) where T is the name of a type variable
    %% This happens when we are type checking a rule with an arg that has
    %% a poly type - the type variable is replaced by this $$var$$ term
    %% and so we have hit the bottom of recursion
    R = M(DT).
push_modes_in__(_M(DT), R) :-
    compound(DT), functor(DT, V, _), var(V), !,
    R = DT.
push_modes_in__(M(DT), R) :-
    compound(DT), functor(DT, '->', 2), !,
    R = M(DT).
push_modes_in__(M(DT), R) :-
    compound(DT), functor(DT, F, 1), '$code_kind'(F), !,
    R = M(DT).
push_modes_in__(M(DT), R) :-
    compound(DT), functor(DT, F, 1), F == atom_naming, !,
    R = M(DT).
push_modes_in__(M(DT), R) :-
    compound(DT), functor(DT, F, 1), F == term_naming, !,
    R = M(DT).
push_modes_in__(M(DT), R) :-
    compound(DT), functor(DT, F, 1), F == alt_types, !,
    R = M(DT).
push_modes_in__(M(DT), M('$tuple_type'(MTypes))) :-
    compound(DT), DT = '$tuple_type'(Types), !,
    push_modes_in_iterate__(Types, M, MTypes).
push_modes_in__(M(DT), M(DType)) :-
    compound(DT), DT \= '$union_type'(_), \+ moded__(DT), !,
    '@=..'(DT, [F|Args]),
    push_modes_in_iterate__(Args, M, MArgs),
    '@=..'(DType, [F|MArgs]).
push_modes_in__(_M(DT), PushedDT) :-
    compound(DT), DT \= '$union_type'(_), moded__(DT), !,
    push_modes_in__(DT, PushedDT).
push_modes_in__(MT, MT).

%% push modes in for all elements of the first arg
push_modes_in_iterate__([], _M, []).
push_modes_in_iterate__([A|Args], M, [MA|MArgs]) :-
    push_modes_in__(M(A),MA),
    push_modes_in_iterate__(Args, M, MArgs).

%% true iff T is a moded type i.e M(T1) where M is a mode
moded__(T) :- var(T), !, fail.
moded__(T) :- compound(T), functor(T, F, _), var(F), !, fail.
moded__('!'(_)).    
moded__('?'(_)).    
moded__('??'(_)).
%moded__('!?'(_)).    
moded__('!??'(_)).
%moded__('@'(_)).

%% remove (if there) a term_naming wrapper around a type
'$strip_term_naming'(Type0, Type) :-
    compound(Type0),
    functor(Type0, F, 1), F == term_naming, !,
    Type0 = F(Type).
'$strip_term_naming'(Type, Type).

%% used to filter out arity errors when constructing error terms
is_not_arity_error__(te__(_, AE, _, _)) :-
    AE \= arity_error(_, _).

%% get_type_error__(Term, Errs, Error) takes a list of terms containing
%% error terms for term Term and returns a suitable Error term.
%%get_type_error__(Term, TypeErr, Error) :- errornl(get_type_error__(Term, TypeErr, Error)),fail.
get_type_error__(Term, Errs, Error) :-
    member(te__(_, Err, Term, _), Errs),
    Err = type_error(_, _, _, _, not_a_term, _), !,
    Error = Err.
get_type_error__(Term, Errs, Error) :-
    member(te__(_, AE, Term, _), Errs),
    AE = arity_error(_, _), !,
    %% At least one error is an arity error - if all are arity errors
    %% then return that error else get the best error from the remainig errors
    filter(is_not_arity_error__, Errs, Errs1),
    (
      Errs1 = []
    ->
      Error = AE
    ;
      get_type_error__(Term, Errs1, Error)
    ).
get_type_error__(Term, [te__(_, Err, Term, _)], Error) :-
    functor(Err, type_error, _),
    !,
    %% single type error
    Error = Err.
get_type_error__(Term, [te__(_, Err, Term, _)], Error) :-
    functor(Err, mode_error, _),
    !,
    %% single mode error
    Error = Err.
get_type_error__(Term, TypeErr, Error) :-
    member(te__(_, mode_error(Index, PTerm, TermI, TypeI, Reason), Term, _),
           TypeErr),
    forall(member(te__(_, E, Term, _), TypeErr),
           ( E = mode_error(Index, PTerm, TermI, TI, _),
               type_le__(TI, TypeI))
          ),
    !,
    %% All errors are mode errors of the same kind - choose the one
    %% with the smallest type
    Error = mode_error(Index, PTerm, TermI, TypeI, Reason).
get_type_error__(Term, TypeErr, Error) :-
    member(te__(_, type_error(Index, PTerm, TermI, TypeI, Reason, DecI),
                Term, _), TypeErr),
    forall(member(te__(_, E, Term, _), TypeErr),
           ( E = type_error(Index, PTerm, TermI, TI, _, _),
               type_le__(TI, TypeI))
          ),
    !,
    %% All errors are type errors of the same kind - choose the one
    %% with the smallest type    
    Error = type_error(Index, PTerm, TermI, TypeI, Reason, DecI).
get_type_error__(Term, TypeErr, Error) :-
    member(te__(_, Err, Term, _), TypeErr),
    Err = functor(Err, mode_error, _),
    forall(member(E1, TypeErr), E1 == Err),
    !,
    %% All errors are the same mode error
    Error = Err.
get_type_error__(Term, TypeErr, Error) :-
    member(te__(_, Err, Term, _), TypeErr),
    Err = type_error(Index, PTerm, STerm, _, _, _),
    forall(member(te__(_, E, Term, _), TypeErr),
           E = type_error(Index, PTerm, STerm, _, _, _)),
    !,
    %% all are moded type errors about the same index into PTerm (but
    %% with no minimal moded type
    Error = type_error(Index, PTerm, STerm, no_suitable_moded_type, default, _).
get_type_error__(Term, _, Error) :-
    %% not able to distil down to any focussed error
    Error = type_error(0, Term, Term, no_suitable_moded_type, default, _).


%% Term is a function call with result type Type -
%% collect terms of the form ft(RT, Term, Args, ArgTypes)
%% where Args are the flattened arguments of Term and ArgTypes are
%% their corresponding types - RT is the return type.
%% EG
%% collect_matching_function_calls_(curry(+)(3), T, M).
%% M = [ft(('$tuple_type'([A]) -> B), curry(+)(3),
%%      ['$tuple'([3]), '$tuple'([+])],
%%      ['$tuple_type'([C]), '$tuple_type'([('$tuple_type'([C, A]) -> B)])])]
%% and
%% collect_matching_function_calls__(+(X,Y), T, M).
%% M = [ft(num, A + B, ['$tuple'([A, B])], ['$tuple_type'([num, num])]),
%%      ft(int, C + D, ['$tuple'([C, D])], ['$tuple_type'([int, int])]),
%%      ft(nat, E + F, ['$tuple'([E, F])], ['$tuple_type'([nat, nat])])]
%% Note that these terms are sorted on the type in the first arg
%% (the return type) with biggest first. 
collect_matching_function_calls__(Term, Type, Matches) :-
    findall(ft(RT, Term, Args, ArgTypes),
            ('$get_function_type_info'(Term, Args, ArgTypes, ResultType),
                (
                  compound(ResultType), ResultType = term_naming(RT)
                ->
                  true
                ;
                  compound(ResultType), ResultType = atom_naming(RT)
                ->
                  true
                ;
                  RT = ResultType
                ),
                type_le__(RT, Type, [], LE),
                bind_type_constraints__(LE, Err), var(Err)
            ),
            AllMatches),
    sort(AllMatches, Matches, ft_order__).

%% order for sort on first arg (biggest type first)
ft_order__(ft(Type1, _, _, _), ft(Type2, _, _, _)) :-
    Type1 == Type2, !.
ft_order__(ft(Type1, _, _, _), ft(Type2, _, _, _)) :- \+type_le__(Type1, Type2).

%% order for sort on first arg (smallest type first)
te_order__(te__(Type1, _, _, _), te__(Type2, _, _, _)) :-
    \+type_le__(Type2, Type1).

%% order for sort  (smallest type first)
le_order__(Type1, Type2) :- \+type_le__(Type2, Type1).

%% get_function_call_types__(Term, ResultType, Args, ArgTypes)
%% Term is a function call - on backtracking gives back
%% ResultType, Args, ArgTypes for type checking in largest to
%% smallest ResultType order
%%get_function_call_types__(Term, ResultType, Args, ArgTypes) :- errornl(get_function_call_types__(Term, ResultType, Args, ArgTypes)),fail.
get_function_call_types__(Term, ResultType, Args, ArgTypes) :-
    collect_matching_function_calls__(Term, ResultType, Matches),
    member(ft(ResultType, Term, Args, ArgTypes), Matches).


%% has_type_comprehension_term__(Term, MT, CurrVTB, Unders, InLE, OutLE,
%%                               Unif, Index, PTerm, Error)
%% For type checking a list/set comprehension term
%% (similar to other type checks)
%% Term is [Pattern :: Goal] or {Pattern :: Goal}
%% MT is the moded type of the list/set elements
%%has_type_comprehension_term__(Term, MT, CurrVTB, Unders, InLE, OutLE, Unif, Index, PTerm, Error) :- errornl(has_type_comprehension_term__(Term, MT, CurrVTB, Unders, InLE, OutLE,Unif, Index, PTerm, Error)),fail.
has_type_comprehension_term__(Term, MT, CurrVTB, Unders, InLE, OutLE,
                              Unif, _Index, _PTerm, Error) :-
    Term = _(Pattern, Goal),
    strip_modes__(list(MT), list(Stripped)),
    push_modes_in__('!'(Stripped), GType),
    '$goals2list'(Goal,GoalList),
    type_check_body__(GoalList, rel, CurrVTB, OutVTB, Unders, InLE, MidLE,
                      Unif, OutUnif, Err, inner),
    %% checking the goal can generate new VTB constraints, new LE constraints
    %% and new unify constraints - these updated constraints are used
    %% to check the type of Pattern
    has_type_comprehension_term_aux__(Pattern, GType, OutVTB, Unders, OutUnif,
                                      MidLE, OutLE, Err, Error).

%%has_type_comprehension_term_aux__(A,B,C,D,E,F,G,H,I) :- errornl(has_type_comprehension_term_aux__(A,B,C,D,E,F,G,H,I)),fail.
has_type_comprehension_term_aux__(_Pattern, _GType, _VTB, _Unders, _OutUnif,
                                   InLE, OutLE, Err, Error) :-
    nonvar(Err), !,
    %% an error occurred when type checking the goal of the comprehension
    OutLE = InLE,
    Error = Err.
has_type_comprehension_term_aux__(Pattern, Type, VTB, Unders, Unif,
                                  InLE, OutLE, _Err, Error) :-
    has_type__(Pattern, Type, VTB, [], _, InLE, OutLE, Unif,
               Unders, 0, Pattern, Err),
    (
      var(Err)
    ->
      true
    ;
      Err = mode_error(I,Patt, Var, T, new_variable)
    ->
      Error = mode_error(I,Patt, Var, T, new_variable_compr)
    ;
      Error = Err
    ).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Merge VTB in head
%% merge_head_VTB__(VTB, MergedVTB, Error)
%% When type checking the head of a rule we generate VTB for variables
%% in the head arguments - if we generate more than one constraint for
%% the same variable we need to check these constraints are consistent
%% and if so merge the multiple constraints into a single constraint
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%merge_head_VTB__(VTB, MergedVTB, Error) :- errornl(merge_head_VTB__(VTB, MergedVTB, Error)),fail.
merge_head_VTB__(_VTB, [], Error) :-
    nonvar(Error), !.
% merge_head_VTB__(VTB, MergedVTB, Error) :-
%     append(First, [(X:MT)|Rest], VTB),
%     MT == '@'(term),
%     append(First, Rest, AllOther),
%     member((Y:_), AllOther), Y == X, !,
%     Error = mode_error(0, X, X, '@'(term), repeated_at_term_mode_error),
%     MergedVTB = [].
merge_head_VTB__(VTB, MergedVTB, Error) :-
    msort(VTB, SortedVTB, '@=<'),
    %% SortedVTB will group all the same variables in the constraints together
    merge_head_VTB_aux__(SortedVTB, MergedVTB, Error).

%%merge_head_VTB_aux__(A,B,C) :- errornl(merge_head_VTB_aux__(A,B,C)),fail.
merge_head_VTB_aux__([], [], _).
merge_head_VTB_aux__([C], [C], _) :- !.
merge_head_VTB_aux__([X1:TX1, X2:TX2|Rest], MergedVTB, Error) :-
    X1 == X2, !,
    %% two constraints on the same variable
    push_modes_in__(TX1, PushedTX1),
    push_modes_in__(TX2, PushedTX2),
    check_and_merge_types__(PushedTX1, PushedTX2, Merge, Err),
    %%TX1 = M1(T1),
    %%TX2 = M2(T2),
    %%head_merge__(M1, T1, M2, T2, Merge, Err),
    (
      var(Err)
    ->
      merge_head_VTB_aux__([(X1:Merge)|Rest], MergedVTB, Error)
    ;
      Error = type_error(0, X1, X1, TX1, head_merge_conflict(TX2), _),
      MergedVTB = []
    ).
merge_head_VTB_aux__([Head|SortedVTB], [Head|MergedVTB], Error) :-
    merge_head_VTB_aux__(SortedVTB, MergedVTB, Error).

%%head_merge__(M1, TX1, M2, TX2,  Merge, Err) :- errornl(head_merge__(M1, TX1, M2, TX2,  Merge, Err)),fail.
head_merge__(M1, T1, M2, T2, Merge, _Err) :-
    T1 == T2, !,
    %% The types are the same so merge modes - e.g. if M1 = ! and M2 = ?
    %% then MinM = !
    min_mode__(M1, M2, MinM),
    Merge = MinM(T1).
head_merge__(M1, T1, M2, T2, Merge, Error) :-
    constructor_type__(T1, Constr, SubT1),
    constructor_type__(T2, Constr, SubT2), !,
    %% merge constructor types
    head_merge_constructor_types__(SubT1,SubT2,Merged, Error),
    NewConstr =.. [Constr|Merged],
    min_mode__(M1, M2, MinM),
    Merge = MinM(NewConstr).
head_merge__('!', T1, _M2, T2, Merge, _Err) :-
    type_le__(T1, T2, [], LE), bind_type_constraints1__(LE), !,
    %% T1 with ! mode is LE T2 and so !T1 is the merged type
    Merge = '!'(T1).
head_merge__(_M1, T1, '!', T2, Merge, _Err) :-
    type_le__(T2, T1, [], LE), bind_type_constraints1__(LE), !,
    %% ditto for T2
    Merge = '!'(T2).
head_merge__(M1, T1, M2, T2, M1(T1), Err) :-
    %% One case is when the types are incompatible but another is, for example
    %% head_merge__(?, int, ?, num, Merge, Err)
    %% Neither ?int nor ?num is a suitable merge - ?int won't do as a num
    %% is allowed to be input and ?num won't do as the output is restricted to
    %% an int
    Err = unmatched_moded_types(M1(T1), M2(T2)).

head_merge_constructor_types__(A, _, A, Error) :-
    nonvar(Error), !.
head_merge_constructor_types__([], [], [], _Error).
head_merge_constructor_types__([M1(T1)|T1Rest], [M2(T2)|T2Rest],
                               [MergedT12|MergedT12Rest], Error) :-
    head_merge__(M1, T1, M2, T2, MergedT12, Error),
    head_merge_constructor_types__(T1Rest, T2Rest, MergedT12Rest, Error).

%% constructor_type__(Type, Name, Args) is true if Type is
%% a "constructor type" that includes list, set, tuple and  tuple_type
%% and Name is the name of the type and Args is the list of type args
%% (might be the empty list)
constructor_type__(X, _, _) :- var(X), !, fail.
constructor_type__(list(T), list, [T]) :- !.
constructor_type__(set(T), set, [T]) :- !.
%constructor_type__(tuple(T), tuple, [T]) :- !.
%constructor_type__('$tuple_type'(T), '$tuple_type', T) :- !.
constructor_type__(Type, Constr, Args) :-
    freeze_term(Type, Vars),
    constructor_type_aux__(Type, Constr, Args),
    thaw_term(Vars).

constructor_type_aux__(Type, Constr, Args) :-
    '$defined_type'(Type, '$constr_enum_type'(_)), !,
    Type =.. [Constr|Args].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Merge VTB in body type check
%% merge_type_check_VTB__(VTB, MergedVTB, Error)
%% Note: modes are only ? or ?? (somewhere if in constructor type)
%% The same idea as merge_head_VTB__ but for merging constraints from
%% type checking in the body of a rule
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%merge_type_check_VTB__(VTB, MergedVTB, Error) :- errornl(merge_type_check_VTB__(VTB, MergedVTB, Error)),fail.
merge_type_check_VTB__(_VTB, MergedVTB, Error) :-
    nonvar(Error), !,
    MergedVTB = [].
merge_type_check_VTB__(VTB, MergedVTB, Error) :-
    msort(VTB, SortedVTB, '@=<'),
    merge_type_check_VTB_aux__(SortedVTB, MergedVTB, Error).

merge_type_check_VTB_aux__([], [], _).
merge_type_check_VTB_aux__([C], [C], _) :- !.
merge_type_check_VTB_aux__([X1:TX1, X2:TX2|Rest], MergedVTB, Error) :-
    X1 == X2, !,
    TX1 = M1(T1),
    TX2 = M2(T2),
    type_check_merge__(M1, T1, M2, T2, Merge, Err),
    (
      var(Err)
    ->
      merge_type_check_VTB_aux__([X1:Merge|Rest], MergedVTB, Error)
    ;
      Error = body_merge_conflict(X1, TX1, TX2, Err)
    ).
merge_type_check_VTB_aux__([Head|SortedVTB], [Head|MergedVTB], Error) :-
    merge_type_check_VTB_aux__(SortedVTB, MergedVTB, Error).

%%type_check_merge__(M1, T1, M2, T2, Merge, Error) :-errornl(type_check_merge__(M1, T1, M2, T2, Merge, Error)),fail.
type_check_merge__(M1, T1, M2, T2, Merge, _Error) :-
    T1 == T2, !,
    min_mode__(M1, M2, Min),
    Merge = Min(T1).
type_check_merge__(M1, T1, M2, T2, Merge, Error) :-
    constructor_type__(T1, Constr, SubT1),
    constructor_type__(T2, Constr, SubT2), !,
    type_check_merge_constructor_types__(SubT1,SubT2,Merged, Error),
    NewConstr =.. [Constr|Merged],
    min_mode__(M1, M2, MinM),
    Merge = MinM(NewConstr).
% type_check_merge__('!', T1, _M2, _T2, Merge, _Error) :-
%     !,
%     Merge = '!'(T1).
% type_check_merge__(_, _T1, '!', T2, Merge, _Error) :-
%     !,
%     Merge = '!'(T2).
type_check_merge__(M1, T1, M2, T2, T1, Error) :-
    Error = unmatched_moded_types(M1(T1), M2(T2)).

%%type_check_merge_constructor_types__(SubT1,SubT2,Merged, Error) :-errornl(type_check_merge_constructor_types__(SubT1,SubT2,Merged, Error)),fail.
type_check_merge_constructor_types__(A, _, A, Error) :-
    nonvar(Error), !.
type_check_merge_constructor_types__([], [], [], _Error).
type_check_merge_constructor_types__([M1(T1)|T1Rest], [M2(T2)|T2Rest],
                                     [MergedT12|MergedT12Rest], Error) :-
    type_check_merge__(M1, T1, M2, T2, MergedT12, Error),
    type_check_merge_constructor_types__(T1Rest, T2Rest, MergedT12Rest, Error).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Merge merged VTB from body call with the current VTB
%% to produce a new current VTB
%% merge_VTB_currentVTB__(VTB, Current, MergedVTB)
%% Note: modes are only ? or ??
%% In the type checking that generated VTB, any variable that appears there
%% and also in Current must have the same type.
%% The possible cases (this is simplified - the full story needs to deal with
%% modes imbedded in constructor types):
%% 1. X:MT in Current but X does not appear in VTB - add X:MT to MergedVTB
%% 2. X:MT is in VTB but not in Current (i.e. a new variable) - if
%%    the mode is ?? then add X:MT to MergedVTB, if the mode is ? then change
%%    the mode to ! and add to MergedVTB
%% 3. X:M1(T) is in VTB and X:M2(T) is in Current (with M1 and M2 in [?, ??]) -
%%    cases:
%%    a. M1 = ? - add !(T) to MergedVTB
%%    b. OW add M2(T) to MergedVTB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%merge_VTB_currentVTB__(VTB, Current, MergedVTB) :- errornl(merge_VTB_currentVTB__(VTB, Current, MergedVTB)),fail.
merge_VTB_currentVTB__([], Current, Current) :- !.
merge_VTB_currentVTB__(VTB, Current, MergedVTB) :-
    vtb_split__(Current, VTB, CurrentDiff, CurrentCommon),
    %% CurrentDiff contains constraints on all vars in Current but not in VTB
    %% CurrentCommon contains constraints on variables appearing in both
    in2out__(VTB, OutVTB),
    %% OutVTB is the same as VTB except all ? modes are replaced by !
    vtb_split__(OutVTB, CurrentCommon, VTBDiff, VTBCommon),
    %% VTBDiff contains constraints on all vars in CurrentCommon but not in
    %% OutVTB and VTBCommon contains constraints on variables appearing in
    %% both OutVTB and CurrentCommon (i.e. in both Common and VTB)  
    append(CurrentDiff, VTBDiff, AllDiff),
    %% AllDiff are all the constraints that are in exactly one of Current and
    %% VTB with all ? modes in VTB replaced by !
    merge_VTB_currentVTB_aux__(VTBCommon, CurrentCommon, AllDiff, MergedVTB).

%%merge_VTB_currentVTB_aux__(VTBCommon, CurrentCommon, AllDiff, MergedVTB) :- errornl(merge_VTB_currentVTB_aux__(VTBCommon, CurrentCommon, AllDiff, MergedVTB)),fail.

merge_VTB_currentVTB_aux__([], _CurrentCommon, MergedVTB, MergedVTB).
merge_VTB_currentVTB_aux__([(X:MT)|VTB], CurrentCommon, AllDiff, MergedVTB) :-
    type_member_of__(X:CMT, CurrentCommon),
    get_best_merged_mode_type__(MT, CMT, Best),
    %% pick the best (minimum) mode - the types are the same
    merge_VTB_currentVTB_aux__(VTB, CurrentCommon,
                               [(X:Best)|AllDiff], MergedVTB).
    

%% Split the constraints into common and difference constraints
%%vtb_split__(A,B,C,D) :- errornl(vtb_split__(A,B,C,D)),fail.
vtb_split__([], _, [], []).
vtb_split__([(X:XT)|Rest], VTB, Diff, [(X:XT)|Common]) :-
    type_member_of__(X:_, VTB), !,
    vtb_split__(Rest, VTB, Diff, Common).
vtb_split__([XT|Rest], VTB, [XT|Diff], Common) :-
    vtb_split__(Rest, VTB, Diff, Common).

%% change any ? modes to ! modes
%%in2out__(A,B) :- errornl(in2out__(A,B)),fail.
in2out__([], []).
in2out__([(X:MT)|Rest], [(X:UMT)|RestOut]) :-
    update_modes__(MT, UMT),
    in2out__(Rest, RestOut).

update_modes__(Lst, BList) :- list(Lst), !,
    map(update_modes__, Lst, BList).
update_modes__('!'(T), '!'(UT)) :-
    constructor_type__(T, Constr, SubT1), !,
    !,
    map(update_modes__,  SubT1, Update),
    UT =.. [Constr|Update].
update_modes__('?'(T), '!'(UT)) :-
    constructor_type__(T, Constr, SubT1), !,
    !,
    map(update_modes__,  SubT1, Update),
    UT =.. [Constr|Update].
update_modes__(M(T), M(UT)) :-
    constructor_type__(T, Constr, SubT1), !,
    !,
    map(update_modes__,  SubT1, Update),
    UT =.. [Constr|Update].
update_modes__('?'(T), '!'(T)) :- !.
%update_modes__('!?'(T), '!'(T)) :- !.
update_modes__(MT, MT).

%% get_best_merged_mode_type__(MT1, MT2, Best) takes two moded types
%% MT1 and MT2 that have the same non-moded type and sets Best to the
%% Moded type.
%% EG get_best_merged_mode_type__(!list(??int), ?list(?int), Best)
%% Best = !list(?int)
%%get_best_merged_mode_type__(A,B,C) :- errornl(get_best_merged_mode_type__(A,B,C)),fail.
get_best_merged_mode_type__(M1(T1), M2(T2), Best) :-
    constructor_type__(T1, Constr, InnerT1),!,
    constructor_type__(T2, Constr, InnerT2),
    map(get_best_merged_mode_type__, [InnerT1, InnerT2, InnerBest]),
    min_mode__(M1, M2, MinM),
    NewConstr =.. [Constr|InnerBest],
    Best = MinM(NewConstr).
get_best_merged_mode_type__(M1(T1), M2(_T2), Best) :-
    min_mode__(M1, M2, MinM),
    Best = MinM(T1).

%% get_type_error_le_constraints__(Term, ActType, Type, InLE, Error) 
%% Adding ActType =< Type (where Type is a variable) causes the type LE
%% constraints to become unsolvable. Determine an appropriate type error.
%%get_type_error_le_constraints__(Term, ActType, Type, InLE, Index, PTerm, Error) :- errornl(get_type_error_le_constraints__(Term, ActType, Type, InLE, Index, PTerm, Error)),fail.
get_type_error_le_constraints__(Term, ActType, Type, InLE,
                                Index, PTerm, Error) :-
    findall(Err,
            (
              bind_type_constraints__(InLE, _),
               check_type_against_type_bindings__(ActType, Type, Err)
            ),
            AllErrors),
    filter(nonvar, AllErrors, RealErrors),
    (
      RealErrors = []
    ->
      errornl(should_not_get_here(get_type_error_le_constraints__(Term, ActType, Type, InLE, Index, PTerm, Error))), thread_exit
    ;
      map('$strip_e__', RealErrors, ETypes),
      '$simplify_types'(ETypes, SimpTypes),
      get_type_error__(Term, '!', ActType, SimpTypes, Index, PTerm, Error)
    ).

'$strip_e__'(e__(T), T).

%% In theory, since this is only called from within
%% get_type_error_le_constraints__ which is in turn is only called
%% from within type_le_or_err__ when there are no solution for
%% the LE constraints when ActType =< Type is added to the constraints
%% then only the 3rd clause should be chosen
check_type_against_type_bindings__(_ActType, Type, _Err) :-
    var(Type), !.
check_type_against_type_bindings__(ActType, Type, _Err) :-
    type_le__(ActType, Type, [], _), !.
check_type_against_type_bindings__(_ActType, Type, e__(Type)).


%% type_le_or_err__(Mode, Term, ActType, Type, InLE, OutLE, Index, PTerm, Error)
%% Check if ActType =< Type, possibly updating the LE constraints or
%% generating a type error
%% The mode of ActType is always ! and Mode is the mode of the required type
%% If Mode = ! then we simply ActType =< Type to the LE constraints
%% However if Mode = ?/?? then we don't want to simply add to the constraints
%% as shown in the following example
%% rel q(!list(int))
%% q(X) <= append([1], [a], X)
%% then type checking append with declaration
%% append(!list(T), !list(T), ?list(T))
%% we get le(nat, T), le(atom, T) and if we also add le(int, T) then
%% we have a soln for T = int || atom
%% Clearly this is not correct - want we should do is leave le(int, T)
%% to the side and solve for T without this constraint to get T = nat || atom
%% and now if we look at the other constraint we get le(int, nat || atom)
%% which correctly fails.
%% For this we use the constraint qle(Term, ActType, Type) which will be treated
%% in a later phase when solving for Type

%%type_le_or_err__(Mode, Term, ActType, Type, InLE, OutLE, Index, PTerm, Error) :- errornl(type_le_or_err__(Mode, Term, ActType, Type, InLE, OutLE, Index, PTerm, Error)),fail.
type_le_or_err__(_Mode, _Term, _ActType, _Type, InLE, InLE, _, _, Error) :-
    nonvar(Error), !.
type_le_or_err__('!', Term, ActType, Type, InLE, OutLE, Index, PTerm, Error) :-
    !,
    %% We are in ! mode and are comparing a Term whose type is known to be
    %% ActType and we want to add the constraint that ActType =< Type
    %% If we add this constraint and then check to see if it is consistent
    %% with the other constraints then we are OK.
    %% If not then adding the constraint causes a type error and so we
    %% compute the type error.
    (
      type_le__(ActType, Type, InLE, OutLE),
      check_bind_type_constraints__(OutLE)
    ->
       true
    ;
      get_type_error_le_constraints__(Term, ActType, Type, InLE,
                                      Index, PTerm, Error)
    ).
type_le_or_err__(Mode, Term, ActType, Type, InLE, OutLE, Index, PTerm, Error) :-
    Mode \= '!', ground(ActType), ground(Type), !,
    (
      type_le__(ActType, Type, [], _)
    ->
      OutLE = InLE
    ;
      get_type_error__(Term, '?', ActType, Type, Index, PTerm, Error)
    ).
type_le_or_err__(Mode, Term, ActType, Type, InLE, OutLE, Index, PTerm, Error) :-
    Mode \= '!',  !,
    OutLE = [qle(Term, ActType, Type)|InLE],
    (
      check_bind_type_constraints__(OutLE)
    ->
      true
    ;
      bind_type_constraints__(InLE, Error),
      get_type_error__(Term, '?', ActType, Type, Index, PTerm, Error)
    ).
type_le_or_err__(_, Term, ActType, Type, InLE, OutLE, Index, PTerm, Error) :-
    type_le__(ActType, Type, InLE, OutLE), !,
    (
      check_bind_type_constraints__(OutLE)
    ->
      true
    ;
      get_type_error__(Term, '!', ActType, Type, Index, PTerm, Error)
    ).
type_le_or_err__(_, Term, ActType, Type, InLE, InLE, Index, PTerm, Error) :-
    get_type_error__(Term, '!', ActType, Type, Index, PTerm, Error).



%% type_annotated_le__(MTList1, MTList2, InLE, OutLE)
%% Used for comparing types (including modes) in code types
%% The type LE check is pairwise between the moded types in MTList1 and MTList2
%% LE constraints in inLE are updated to OutLE
%%
%% Testing if CodeType1 =< CodeType2 is the same as asking if
%% the call of type CodeType1 can be used anywhere a call of type CodeType2
%% can be used.
%% EG1 Is rel(!int) =< rel(?int)? Clearly a call whose arg has type ?int
%% can be used when the argument is a new variable whereas this is not the case
%% for a call with type !int. We can therefore conclude that for a pair
%% M1(T1), M2(T2) from MTList1, MTList2 with M1 = ! then M2 must be ! OW
%% the check will fail.
%% EG2 Is rel(M1(T1)) =< rel(!T2)? From a mode point of view this is OK
%% for any mode - i.e. a call whose arg has mode M1 can be used instead
%% of a call that has mode !. From a type point of view we can only make
%% such a call if T2 =< T1. IE we can make a call whose arg has a less
%% restrictive type than the call whose arg is in ! mode with a more
%% restrictive type.
%% EG3  Is rel(?T1) =< rel(?T2)? In this case we can only replace one call
%% by the other if T1 = T2
%%
%% EG4 Is rel(??int) =< rel(?int)?  No : using a call that doesn't guarantee
%% grounded output can't be used instead of one that grounds the output.

%%type_annotated_le__(A, B, C, D) :- errornl(type_annotated_le__(A, B, C, D)),fail.
type_annotated_le__([], [], InConstr, InConstr).
type_annotated_le__([T1|T1R], [T2|T2R], InConstr, OutConstr) :-
    push_modes_in__(T1, T1M),
    push_modes_in__(T2, T2M),
    type_annotated_le_aux__(T1M, T2M, InConstr, MidConstr),
    type_annotated_le__(T1R, T2R, MidConstr, OutConstr).

%%type_annotated_le_aux__(A,B,C,D) :- errornl(type_annotated_le_aux__(A,B,C,D)),fail.
type_annotated_le_aux__('!'(_), M(_), _, _) :-
    M \= '!', !, fail.
type_annotated_le_aux__('??'(_), '?'(_), _, _) :-
    !, fail.
type_annotated_le_aux__(M1(T1), M2(T2), InConstr, OutConstr) :-
    constructor_type__(T1, Constr, Args), var(T2), !,
    length(Args, N),
    length(T2Args, N),
    T2 =.. [Constr|T2Args],
    push_modes_in__(M2(T2), T2M),
    type_annotated_le_aux__(M1(T1), T2M, InConstr, OutConstr).
type_annotated_le_aux__(M1(T1), M2(T2), InConstr, OutConstr) :-
    constructor_type__(T2, Constr, Args), var(T1), !,
    length(Args, N),
    length(T1Args, N),
    T1 =.. [Constr|T1Args],
    push_modes_in__(M1(T1), T1M),
    type_annotated_le_aux__(T1M, M2(T2), InConstr, OutConstr).
type_annotated_le_aux__(_(T1), '!'(T2), InConstr, OutConstr) :-
    constructor_type__(T1, Constr, T1Args), !,
    constructor_type__(T2, Constr, T2Args),
    type_annotated_le__(T1Args, T2Args, InConstr, OutConstr).
type_annotated_le_aux__(_(T1), '!'(T2), InConstr, OutConstr) :-
    constructor_type__(T2, Constr, T2Args), !,
    constructor_type__(T1, Constr, T1Args),
    type_annotated_le__(T1Args, T2Args, InConstr, OutConstr).
type_annotated_le_aux__(_(T1), '!'(T2), InConstr, OutConstr) :-
    !,
    type_le_aux__(T2, T1, InConstr, OutConstr).
type_annotated_le_aux__(_(T1), _(T2), InConstr, InConstr) :-
    T1 = T2.

%% Strip modes off moded types - exception - leave modes inside code types.
%%strip_modes__(Term, StrippedTerm) :- errornl(strip_modes__(Term, StrippedTerm)),fail.
strip_modes__(Term, StrippedTerm) :-
    atomic(Term), !, StrippedTerm = Term.
strip_modes__(Term, StrippedTerm) :-
    string(Term), !, StrippedTerm = Term.
strip_modes__(Term, StrippedTerm) :-
    var(Term), !, StrippedTerm = Term.
strip_modes__(Term, StrippedTerm) :-
    moded__(Term), !,
    Term = _(Term1),
    strip_modes__(Term1, StrippedTerm).
strip_modes__(Term, StrippedTerm) :-
    compound(Term),
    functor(Term, F, 1), F == rel, !,
    StrippedTerm = Term.
strip_modes__(Term, StrippedTerm) :-
    compound(Term),
    functor(Term, F, 1), F == act, !,
    StrippedTerm = Term.
strip_modes__([H|T], [StrippedH|StrippedT]) :-
    !,
    strip_modes__(H, StrippedH),
    strip_modes__(T, StrippedT).
strip_modes__(Term, StrippedTerm) :-
    Term =.. All,
    strip_modes__(All, AllStripped),
    StrippedTerm =.. AllStripped.

%% '$functor_is_constr_enum_type'(Term, CTerm) checks if Term
%% is a term of a constructor enumerator type and CTerm is
%% Term with the args of the constructor replaced by new variables
'$functor_is_constr_enum_type'(Term, CTerm) :-
    compound(Term),
    '@functor'(Term, F, _NF),
    '$defined_type'(_Type,'$constr_enum_type'(ConstrEnum)),
    member(T, ConstrEnum),
    '@functor'(T, F, NT),
    '@functor'(CTerm, F, NT).

%% '$get_code_types'(Kind, Term, Type) - given the Kind of Term
%% (i.e. fun, rel, act, tel) and Term then Type is the code type of
%% Term - wild be alt_type(Types) if Term has multiple declarations
%% '$get_code_types'(Kind, Term, Type) Type is the code type of Term
%% of kind Kind (fun, rel, act, tel)
'$get_code_types'(fun, Term, Type) :- !,
    findall(T, '$type_info'(Term, _, T, _, _),
            AllTypes),
    (
      AllTypes = [Type]
    ->
      true
    ;      
      '$collect_simp_alt_types'(AllTypes, [], SimpAllTypes),
      Type = alt_types(SimpAllTypes)
    ).
'$get_code_types'(Kind,Term, Type) :-
    findall(Kind('$tuple_type'(T)), '$type_info'(Term, _, T, _, _),
            AllTypes),
    (
      AllTypes = [Type]
    ->
      true
    ;
      '$collect_simp_alt_types'(AllTypes, [], SimpAllTypes),
      Type = alt_types(SimpAllTypes)
    ).

%% '$get_ho_types'(Term, Type, CurrVTB, Unif, Unders, Error)
%% Given a higher-order term Term and CurrVTB, Unif, Unders determine
%% Term's type or generate an error term
%% EG for '$get_ho_types'(curry(+)(1), Type, [], [], [], E).
%% we get
%% Type = term_naming(alt_types([('$tuple_type'([int]) -> int),
%%                               ('$tuple_type'([num]) -> num),
%%                               ('$tuple_type'([nat]) -> nat)]))

%%'$get_ho_types'(Term, Type, CurrVTB, Unif, Unders, Error) :- errornl('$get_ho_types'(Term, Type, CurrVTB, Unif, Unders, Error)),fail.
%% Treat '-' as a special case as it is the only "code" that has two arities
'$get_ho_types'(Term, Type, CurrVTB, Unif, Unders, Error) :-
    Term = F(Arg), F == '-', !,
    findall(ho_types_(ResultType,Args, ArgTypes, Term),
            get_function_call_types__(Term, ResultType, Args, ArgTypes),
            AllTypes),
    ip_set('$call_kind', rel),
    '$check_valid_terms'([Arg], 1, '!', Term, CurrVTB, Unif, Unders, Error),
    '$get_ho_types_aux_inner'(Term, Type, AllTypes, CurrVTB,
                              Unif, Unders, Error).
'$get_ho_types'(Term, Type, CurrVTB, Unif, Unders, Error) :-
    Term = F(Arg1, Arg2), F == '-', !,
    findall(ho_types_(ResultType,Args, ArgTypes, Term),
            get_function_call_types__(Term, ResultType, Args, ArgTypes),
            AllTypes),
    ip_set('$call_kind', rel),
    '$check_valid_terms'([Arg1, Arg2], 1, '!', Term, CurrVTB, Unif, Unders, Error),
    '$get_ho_types_aux_inner'(Term, Type, AllTypes, CurrVTB,
                              Unif, Unders, Error).
'$get_ho_types'(Term, Type, _CurrVTB, _Unif, _Unders, Error) :-
    compound(Term),
    functor(Term, F, _), F == '-', !,
    Error = arity_error(Term, Type).
'$get_ho_types'(Term, Type, CurrVTB, Unif, Unders, Error) :-
    findall(ho_types_(ResultType,Args, ArgTypes, Term),
            get_function_call_types__(Term, ResultType, Args, ArgTypes),
            AllTypes),
    '$get_ho_types_aux'(Term, Type, AllTypes, CurrVTB, Unif, Unders, Error).

'$get_ho_types_aux'(Term, Type, AllTypes, CurrVTB, Unif, Unders, Error) :-
    AllTypes = [_],
    !,
    '$get_ho_types_aux_inner'(Term, Type, AllTypes, CurrVTB, Unif, Unders, Error).
'$get_ho_types_aux'(Term, _Type, AllHOTypes, CurrVTB, Unif, Unders, Error) :-
    AllHOTypes = [ho_types_(_ResultType,Args, ArgTypes, Term)|_],
    check_function_arities__(Args, ArgTypes, Term, Err),
    (
      var(Err)
    ->
      reverse(Args, RevArgs),
      reverse(ArgTypes, RevArgTypes),
      flatten_tuple_args_list__(RevArgs, RevArgTypes,
                                AllArgs, _AllTypes, Error),
      ip_set('$call_kind', rel),
      '$check_valid_terms'(AllArgs, 1, '!', Term, CurrVTB, Unif, Unders, Error)
    ;
      Error = Err
    ),
    nonvar(Error), !.
'$get_ho_types_aux'(Term, Type, AllTypes, CurrVTB, Unif, Unders, Error) :-
    '$get_ho_types_aux_inner'(Term, Type, AllTypes, CurrVTB,
                              Unif, Unders, Error).
              
    
%%'$get_ho_types_aux_inner'(Term, Type, AllTypes, CurrVTB, Unif, Unders, Error) :-errornl('$get_ho_types_aux_inner'(Term, Type, AllTypes, CurrVTB, Unif, Unders, Error)),fail.
'$get_ho_types_aux_inner'(Term, Type, AllHOTypes, CurrVTB, Unif, Unders, Error) :-
    findall(te__(ResultType,Err, Term, []),
            (
              member(ho_types_(ResultType,Args, ArgTypes, Term), AllHOTypes),
              %% ResultType is the type of the HO function application
              %% Args and ArgTypes are lists with each element being
              %% about the args/types of one part of the HO term
              %% on backtracking alternative types are produced
              check_function_arities__(Args, ArgTypes, Term, Err),
              %% make sure the args have the correct arities relative to the
              %% required types
              reverse(Args, RevArgs),
              reverse(ArgTypes, RevArgTypes),
              %% get_function_call_types__ returns the args and types
              %% in right to left order - we reverse them so indexing will
              %% pick the correct arg when generating error messages
              flatten_tuple_args_list__(RevArgs, RevArgTypes,
                                        AllArgs, AllTypes, Err),
              %% flatten out the args and types into simple lists
              '$add_annotations'(AllTypes, AnnTypes, '!'),
              %% add '!' modes as function types are all in ! mode
              has_type_term_index_iterate__(AnnTypes, AllArgs, 1, Term, CurrVTB,
                                            [], _, [], LE, Unif, Unders, Err),
              %% Do the type checking
              bind_type_constraints__(LE, Err)
              %% solve the type LE constraints
            ),
            TypeErr),
    %% TypeErr is a list of te__(ResultType,Err, Term, [])
    %% where Err is a variable if the particular type choice was type correct
    length(TypeErr, Num),
    '$get_valid_types'(TypeErr, ResultTypes, Term),
    %% ResultTypes are all the result types for type choices that lead
    %% to type correctness
    (
      ResultTypes = [], Num = 1
    ->
      %% There is only one term and there is an error -
      %% Error becomes the overall error
      TypeErr = [te__(_, Error, Term, _)]
    ;
      ResultTypes = []
    ->
      %% All choices lead to type errors - compute a type error term
      get_type_error__(Term, TypeErr, Error)
    ;
      %% Compute the best type for the HO term
      sort(ResultTypes,  SortedResultTypes, le_order__),
      '$get_ho_types_aux'(SortedResultTypes, Type)
    ).

'$check_valid_terms'(_AllArgs, _, _Mode, _Term, _CurrVTB, _Unif, _Unders, Error) :-
    nonvar(Error), !.
'$check_valid_terms'([], _, _Mode, _Term, _CurrVTB, _Unif, _Unders, _Error).
'$check_valid_terms'([Arg|AllArgs], Index, Mode, PTerm, CurrVTB,
                     Unif, Unders, Error) :-
    has_type__(Arg, Mode(_), CurrVTB, [], _, [], _, Unif,
               Unders, Index, PTerm, Error),
    Index1 is Index+1,
    '$check_valid_terms'(AllArgs, Index1, Mode, PTerm, CurrVTB, Unif, Unders, Error).

%% '$get_ho_types_aux'(Types, Type) - given a list of types generatea type
%% for this collection of types
%%'$get_ho_types_aux'(Types, Type) :-errornl('$get_ho_types_aux'(Types, Type)),fail.
'$get_ho_types_aux'(Types, Type) :-
    Types = [Type1],
    Type1 = (_ -> _),
    !,
    %% a single function type
    Type = term_naming(Type1).
'$get_ho_types_aux'(Types, Type) :-
    Types = [(_ -> _)|_],
    !,
    %% multiple function types
    '$collect_simp_alt_types'(Types, [], SimpTypes),
    Type = term_naming(alt_types(SimpTypes)).
'$get_ho_types_aux'(Types, Type) :-
    Types = [T],
    compound(T),
    '@functor'(T, F, _),
    '$code_kind'(F),
    !,
    %% single code type
    Type = term_naming(T).
'$get_ho_types_aux'(Types, Type) :-
    Types = [T|_],
    compound(T),
    '@functor'(T, F, _),
    '$code_kind'(F),
    !,
    %% multiple code types
    '$collect_simp_alt_types'(Types, [], SimpTypes),
    Type = term_naming(alt_types(SimpTypes)).
'$get_ho_types_aux'([Type|_], Type).
%% Not a code type - use the smalles type (the first) - note use of sort
%% in '$get_ho_types'

%% '$extract_ho_alts'(HOTypes, Types, Call, Kind, Error)
%% Given HOTypes (code_types) extract  the list of types and the code Kind
%% or generate an error
%%'$extract_ho_alts'(A,B,C,D,E) :- errornl('$extract_ho_alts'(A,B,C,D,E)),fail.
'$extract_ho_alts'(_, [], _, _, Error) :- nonvar(Error), !.
'$extract_ho_alts'(atom_naming(T), Types, Call, Kind, Error) :-
    '$type_info'(_, macro_type, T, DefT, _), !,
    '$extract_ho_alts'(atom_naming(DefT), Types, Call, Kind, Error).
'$extract_ho_alts'(term_naming(T), Types, Call, Kind, Error) :-
    '$type_info'(_, macro_type, T, DefT, _), !,
    '$extract_ho_alts'(term_naming(DefT), Types, Call, Kind, Error).
'$extract_ho_alts'(term_naming(alt_types(Types)), Types, _, Kind, _Error) :-
    Types = [Kind(_)|_], !.
'$extract_ho_alts'(term_naming(alt_types(Types)), Types, Call, _Kind, Error) :-
    !,
    Error = call_error(Call, not_a_code_type).
'$extract_ho_alts'(term_naming(Type), [Type], _, Kind, _Error) :-
    Type = Kind(_), !.
'$extract_ho_alts'(term_naming(Type), [Type], Call, _Kind, Error) :-
    Error = call_error(Call, not_a_code_type).
'$extract_ho_alts'(atom_naming(alt_types(Types)), Types, _, Kind, _Error) :-
    Types = [Kind(_)|_], !.
'$extract_ho_alts'(atom_naming(alt_types(Types)), Types, Call, _Kind, Error) :-
    !,
    Error = call_error(Call, not_a_code_type).
'$extract_ho_alts'(atom_naming(Type), [Type], _, Kind, _Error) :-
    Type = Kind(_), !.
'$extract_ho_alts'(atom_naming(Type), [Type], Call, _Kind, Error) :-
    Error = call_error(Call, not_a_code_type).
'$extract_ho_alts'(term_naming(Type), [Type], _Kind, _).

%% Used in '$get_ho_types' to extract valid return types
%%'$get_valid_types'(A,B,C) :- errornl('$get_valid_types'(A,B,C)),fail.
'$get_valid_types'([], [], _).
'$get_valid_types'([te__(_T, Err, Term, _)|Rest], Out, Term) :-
    nonvar(Err), !,
    '$get_valid_types'(Rest, Out, Term).
'$get_valid_types'([te__(T, _Err, Term, _)|Rest], [T|Out], Term) :-
    '$get_valid_types'(Rest, Out, Term).

%% in_unif__((X = T), Unif) is true if X = T is in the list of unifications
%% for some T
in_unif__((X = T), Unif) :-
    member((Y = T), Unif), X == Y, !.

%% Used to replace a variable functor F of a compound term where
%% F = UF is in the collection of unifications - used in
%% '$function_call_or_comp'
'$replace_ultimate_functor'(Term, F, UF, UTerm) :-
    Term == F, !,
    UTerm = UF.
'$replace_ultimate_functor'(Term, F, UF, UTerm) :-
    Term =.. [FTerm|Args],
    '$replace_ultimate_functor'(FTerm, F, UF, UFTerm),
    UTerm =.. [UFTerm|Args].


%% type_check_rule__(Rule,  VarNames, Unders)
%% Rule: the rule being type checked
%% VarNames: The names of variables in Rule - used to display type errors
%% Unders: The underscore vars in the rule
%%type_check_rule__(A, B, C) :- errornl(type_check_rule__(A, B, C)),fail.
type_check_rule__((Head :: Guard <= Body),  VarNames, Unders) :-
    !,
    '$goals2list'((Guard & Body), BodyList),
    type_check_rel_act_rule__(Head, BodyList, [],
                              rule_info(rel,(Head :: Guard <= Body),
                                        VarNames, Unders)).
type_check_rule__((Head <= Body),  VarNames, Unders) :-
    !,
    '$goals2list'(Body, BodyList),
    type_check_rel_act_rule__(Head, BodyList, [], 
                              rule_info(rel, (Head <= Body),
                                        VarNames, Unders)).
type_check_rule__((Head :: Guard ~> Body),  VarNames, Unders) :-
    !,
    '$goals2list'(Guard, GuardList),
    '$goals2list'(Body, BodyList),
    type_check_rel_act_rule__(Head, GuardList, BodyList, 
                              rule_info(act, (Head :: Guard ~> Body),
                                        VarNames, Unders)).
type_check_rule__((Head :: Guard -> Expr),  VarNames, Unders) :-
    !,
    '$goals2list'(Guard, BodyList),
    type_check_function_rule__(Head, BodyList, Expr,
                               rule_info(fun, (Head :: Guard -> Expr),
                                         VarNames, Unders)).
type_check_rule__((Head :: Guard),  VarNames, Unders) :-
    !,
    '$goals2list'(Guard, BodyList),
    type_check_rel_act_rule__(Head, BodyList, [], 
                              rule_info(rel, (Head :: Guard),
                                        VarNames, Unders)).
type_check_rule__((Head ~> Body),  VarNames, Unders) :-
    !,
    '$goals2list'(Body, BodyList),
    type_check_rel_act_rule__(Head, [], BodyList, 
                              rule_info(act, (Head ~> Body),
                                        VarNames, Unders)).
type_check_rule__((Head -> Expr),  VarNames, Unders) :-
    !,
    type_check_function_rule__(Head, [], Expr,
                               rule_info(fun, (Head -> Expr),
                                         VarNames, Unders)).
type_check_rule__(Head,  VarNames, Unders) :-
    !,
    type_check_rel_act_rule__(Head, [], [],
                              rule_info(rel, Head, VarNames, Unders)).

%% type_check_rel_act_rule__(Head, RelBody, ActBody, RuleInfo)
%% Type check rule
%% Head : The head of the rule
%% RelBody : the relational part of the body (the guard if an action rule)
%% RuleInfo = rule_info(Kind, Rule, VarNames, Unders)
type_check_rel_act_rule__(Head, RelBody, ActBody, RuleInfo) :-
    %% First check that locals are not being used as globals
    RuleInfo = rule_info(Kind, _, _, Unders),
    freeze_term(Head, FVars),
    append(RelBody, ActBody, Body),
    check_globals_locals__([Head|Body], [], _G, [], _L, Unders, full, ErrGL),
    thaw_term(FVars),
    report_error__(globals_locals, Head, Body, ErrGL, RuleInfo),
    %% at this point there are no global/local errors
    %% Type check rule and report any errors
    '$get_ultimate_functor'(Head, F),
    findall(err__(Error, Kind(F(Types)), Head, Body),
            (
              type_check_rule_aux__(Head, RelBody, ActBody, Kind, Types,
                                    Unders, Error)
                ),
            Errors),
    bind_error_vars__(Errors, Head, Body),
    process_errors__(Errors, RuleInfo).

%% Same as above for function rules
type_check_function_rule__(Head, Body, Expr, RuleInfo) :-
    RuleInfo = rule_info(_, _, _, Unders),
    append(Body, [_ = Expr], GL),
    freeze_term([Head|GL], FVars),
    check_globals_locals__([Head|GL], [], _G, [], _L, Unders, full, ErrGL),
    thaw_term(FVars),
    report_error__(globals_locals, Head, Body, ErrGL, RuleInfo),
    '$get_ultimate_functor'(Head, F),
    findall(err__(Error, fun(FunType), Head, (Body, Expr)),
            (
              type_check_function_rule_aux__(Head, Body, Expr, Unders,
                                             Error, FT),
              FT = (D -> R),
              FunType = (F(D) -> R)        
            ),
            Errors),
    bind_error_vars__(Errors, Head, (Body, Expr)),
    process_errors__(Errors, RuleInfo).

%% Type check the rule returning an error if required.
%% Note: if there are multiple declarations then, on backtracking, we
%% check the rule against all declarations (in findall above)
%%type_check_rule_aux__(Head, RelBody, ActBody, Kind, Types,Unders,  Error) :- errornl(type_check_rule_aux__(Head, RelBody, ActBody, Kind,Types, Unders, Error)),fail.
type_check_rule_aux__(Head, _RelBody, _ActBody, Kind, _Types, _Unders,  Error) :-
    '$get_ultimate_functor'(Head, F),
    '$type_info'(F, Kind1, _, _, _), Kind \= Kind1,
    (Kind = act ; Kind = rel),(Kind1 = act ; Kind1 = rel),!,
    Error = head(head_type_mismatch).
type_check_rule_aux__(Head, RelBody, ActBody, Kind, Types, Unders,  Error) :-
    '$get_ultimate_functor'(Head, F),
    '$type_info'(F, Kind, _, _, _), !,
    '$type_info'(F, Kind, HeadTypes, _, VarNames), 
    '$set_var_names'(HeadTypes, VarNames, _),
    '$add_annotations'(HeadTypes, Types, '!'),
    '@..rev'(Head, _, Args),
    (
      length(Args, N), length(Types, N)
    ->
      true
    ;
      Err1 = arity_error(Head, Types)
    ),
    has_type_term_index_iterate__(Types, Args, 1, Head, head, [], VTB, [],
                                  _LE, [],  Unders, Err1),
    %% Check types in the head of the rule - generate a variable type binding
    %% VTB and bind Err1 to an error term if required
    check_error__(Err1, head, Error),
    %% VTB might contain multiple constraints on the same variable - attempt
    %% to merge the constraints
    merge_head_VTB__(VTB, HeadVTB, Err1),
    check_error__(Err1, head, Error),
    in2out__(HeadVTB, SavedVTB),
    %% Initialize act_seen
    ip_set('$act_seen', []),
    %% SavedVTB is HeadVTB with all ? modes replaced by ! modes so that,
    %% at the end of the rule we check that the body has updated the head
    %% ? modes to ! modes
    type_check_body__(RelBody, rel, HeadVTB, RelOutVTB,Unders,
                      [], _, [], RelOutUnif, Err1, []),
    %% The rel part of the body has been type checked with RelOutVTB
    %% the VTBs generated by the rel part of the body check - RelOutUnif
    %% is the list of unification constrains Err1 is set if an error occurs
    '$group_alts'(ActBody, FlatBody, _, _, _),
    %% actions include "choice" actions like receive, try_except, case
    %% In order to simplify the type checking of each alternative we
    %% push later alts into each branch of earlier alts
    type_check_body__(FlatBody, act, RelOutVTB, OutVTB,Unders,
                      [], _, RelOutUnif, OutUnif, Err1, SavedVTB),
    check_error__(Err1, body, Error),
    %% OutVTB is the VTB constraints at the end of the body - now check
    %% that all the ? moded vars in the head have been replaced by !
    %% uses SavedVTB from earlier
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
    check_savedVTB__(SavedVTB, OutVTB, OutUnif, Unders, Err1),
     ip_set('$call_kind', CallKind),
    check_error__(Err1, saved, Error).
type_check_rule_aux__(Head, RelBody, ActBody, Kind, FunType, Unders,  Error) :-
    get_function_rule_head_types__(Head, FunType, Kind, Args, Types,
                                        _ResultType, Err1),
    has_type_term_index_iterate__(Types, Args, 1, Head, head, [], VTB, [],
                                  _LE, [], Unders, Err1),
    check_error__(Err1, head, Error),
    merge_head_VTB__(VTB, HeadVTB, Err1),
    check_error__(Err1, head, Error),
    in2out__(HeadVTB, SavedVTB),
    %% Initialize act_seen
    ip_set('$act_seen', []),
    type_check_body__(RelBody, rel, HeadVTB, RelOutVTB,Unders,
                      [], _, [], RelOutUnif, Err1, []),
    '$group_alts'(ActBody, FlatBody, _, _, _),
    type_check_body__(FlatBody, act, RelOutVTB, OutVTB,Unders,
                      [], _, RelOutUnif, OutUnif, Err1, SavedVTB),
    check_error__(Err1, body, Error),
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
    check_savedVTB__(SavedVTB, OutVTB, OutUnif, Unders, Err1),
     ip_set('$call_kind', CallKind),
    check_error__(Err1, saved, Error).

%% Ditto for type checking a function rule (no actions in the body)
%% Since the head args are all in ! mode we don't need to check that ?
%% modes are converted to ! modes
type_check_function_rule_aux__(Head, Body, Expr, Unders, Error, FunType) :-
    get_function_rule_head_types__(Head, FunType, fun, Args, Types, ResultType,
                                   Err1),
    has_type_term_index_iterate__(Types, Args, 1, Head, head, [], VTB, [],
                                  _LE, [], Unders, Err1),
    check_error__(Err1, head, Error),
    merge_head_VTB__(VTB, HeadVTB, Err1),
    check_error__(Err1, head, Error),
    type_check_body__(Body, rel, HeadVTB, OutVTB,Unders, [], _, [],
                      OutUnif, Err1, []),
    check_error__(Err1, body, Error),
    has_type__(Expr, '!'(ResultType), OutVTB, [], _,
               [], _, OutUnif, Unders, 1, Expr, Err1),
    check_error__(Err1, fun_result, Error).

%% type_check_body__(Calls, Kind, InVTB, OutVTB, Unders,
%%                   InLE, OutLE, InUnif, OutUnif, Error, SavedVTB)
%% Calls : the list of body calls
%% Kind : the kind of the calls (rel or act)
%% InVTB : The incomming VTB (from the head)
%% OutVTB : the VTB update to InVTB generated by the calls in the body
%% Unders: the underscore vars
%% InLE : the initial type LE constraints ([] when first called)
%% OutLE : InLE updated by type checking Calls
%% InUnif : the initial unification constraints ([] when first called)
%% OutUnif : InUnif updated with any unification constraints in Calls
%%
%% We step through each call, type checking the call and updating
%% the VTB, LE, and Unif
%%type_check_body__(A,B,C,D,E,F,G,H,I,J,K) :- write(type_check_body__(A,B,C,D,E,F,G, H,I,J,K)), nl,fail.
type_check_body__(_, _, VTB, VTB, _, LE, LE, U, U, Error, _) :-
    nonvar(Error), !.
type_check_body__([], _, VTB, VTB,_,  LE, LE, InUnif, InUnif, _, _).
type_check_body__([Call|Rest], Kind, CurrentVTB, OutVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, SVTB) :-
    ip_set('$call_kind', Kind),
    add_default_args__(Call, FullCall),
    %% default arguments are added to the call as necessary
    type_check_call__(FullCall, Kind, CurrentVTB, NextVTB, Unders,
                      InLE, MidLE, InUnif, MidUnif, Err, SVTB),
    (
      var(Err)
    ->
      %% Call does not produce any errors so continue checking the rest
      %% of the body
      type_check_body__(Rest, Kind, NextVTB, OutVTB, Unders,
                        MidLE, OutLE, MidUnif, OutUnif, Error, SVTB)
    ;
      %% an error has occurred - terminate checking
      '$get_body_call_error'(Err, Call, Error)
    ).

%% Add a body_call_error wrapper around error term (unless already there)
%%'$get_body_call_error'(A,B,C) :- errornl('$get_body_call_error'(A,B,C)),fail.
'$get_body_call_error'(E, _, _) :-
    var(E), !.
'$get_body_call_error'(error_at_index(Index, Err), Call, Error) :-
    !,
    Error = body_call_error(Call, error_at_index(Index, Err)).
'$get_body_call_error'(saved(BCE), _Call, Error) :-
    !,
    '$get_body_call_error'(BCE, _, Error).
'$get_body_call_error'(body_call_error(Call1,Err), _Call, Error) :-
    !,
    Error = body_call_error(Call1,  Err).
'$get_body_call_error'(body(body_call_error(Call1,Err)), _Call, Error) :-
    act_alt_error__(Err),
    !,
    Error = body_call_error(Call1,  Err).
'$get_body_call_error'(Err, Call, Error) :-
    !,
    Error = body_call_error(Call, Err).

%% type_check_call__(Call, Kind, CurrentVTB, NextVTB, Unders,
%%                   InLE, OutLE, InUnif, OutUnif,Error, SVTB)
%% This type checks the Call of kind Kind (rel or act) updating
%% CurrentVTB to NextVTB
%% and
%% InLE to OutLE
%% and
%% InUnif to OutUnif
%%type_check_call__(Call, Kind, CurrentVTB, NextVTB, Unders, InLE,OutLE,InU, OutU, Error, Out) :-write(type_check_call__(Call, Kind, CurrentVTB, NextVTB, Unders, InLE,OutLE,InU, OutU, Error, Out)), nl,fail.
type_check_call__(Call, Kind, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif,Error, SVTB) :-
    var(Call),
    in_unif__((Call = RealCall), InUnif), !,
    %% replace call variable with RealCall (previously unified with Call)
    type_check_call__(RealCall, Kind, CurrentVTB, NextVTB, Unders,
                      InLE, OutLE, InUnif, OutUnif,Error, SVTB).
type_check_call__(Call, _Kind, CurrentVTB, NextVTB, _Unders,
                   InLE, OutLE, InUnif, OutUnif,Error, _) :-
    var(Call), !,
    %% A variable call is an error 
    NextVTB = [],
    NextVTB = CurrentVTB,
    OutUnif = InUnif,
    OutLE = InLE,
    Error = var_call_error(Call).
type_check_call__(Call, Kind, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif,Error, SVTB) :-
    compound(Call),
    Call =.. [F|Args],
    var(F), 
    in_unif__((F = UF), InUnif), !,
    %% The call has a variable functor that is in the unification constraints
    UCall =.. [UF|Args],
    type_check_call__(UCall, Kind, CurrentVTB, NextVTB, Unders,
                      InLE, OutLE, InUnif, OutUnif,Err, SVTB),
    (
      var(Err)
    ->
      true
    ;
      Error = call_unify_err(Call, UCall, Err)
    ).
%% call with variable functor - must be a code type
type_check_call__(Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif,
                  OutUnif,Error, _) :-
    compound(Call),
    functor(Call, F, _),
    var(F), !,
    %% The functor is a variable - it is required that the functor
    %% is in CurrentVTB constrainted to be a suitably typed code type
    type_check_call_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders,
                                  InLE, OutLE, InUnif, OutUnif,Error).
%% call with HO variable functor - must be a HO function returning a relation
type_check_call__(Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif,
                  OutUnif,Error, _) :-
    compound(Call),
    '$get_ultimate_functor'(Call, F),
    var(F), !,
    InUnif = OutUnif,
    type_check_call_ho_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders,
                                     InLE, OutLE, InUnif,Error).
%% call with compound functor - must be a HO function call that
%% returns a relation
type_check_call__(Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif,
                  OutUnif,Error, _) :-
    compound(Call),
    functor(Call, F, _),
    compound(F), !,
    %% Similar to above but whose ultimate functor is an atom (HO function)
    InUnif = OutUnif,
    type_check_call_ho_functor__(Call, Kind, CurrentVTB, NextVTB, Unders,
                                 InLE, OutLE, InUnif,Error).
%% unification
type_check_call__((A = B), _, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif,
                  OutUnif,Error, _) :-
    !,
    (
      A == B
    ->
      CurrentVTB = NextVTB,
      OutLE = InLE,
      InUnif = OutUnif
    ;
      type_check_unif__((A = B), CurrentVTB, NextVTB,
                        InLE, OutLE, InUnif, OutUnif, Unders, Error)
    ).
%% one sided unification
type_check_call__('=@'(A, B), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif,Error, _) :-
    !,
    (
      A == B
    ->
      CurrentVTB = NextVTB,
      OutLE = InLE,
      InUnif = OutUnif
    ;
      type_check_unif__((A = B), CurrentVTB, NextVTB,
                        InLE, OutLE, InUnif, OutUnif, Unders, Error)
    ).

%% type check call
type_check_call__(type(A, B), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    moded__(B),
    !,
    OutUnif = InUnif,
    push_modes_in__(B, BM),
    OutLE = InLE, 
    type_check_type_call__(type(A, BM), CurrentVTB, NextVTB,
                            InUnif, Error). 
%% type check call
type_check_call__(type(A, B), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    OutUnif = InUnif,
    %% add ! mode
    push_modes_in__('!'(B), BM),
    OutLE = InLE, 
    type_check_type_call__(type(A, BM), CurrentVTB, NextVTB,
                            InUnif, Error). 
%% isa call
type_check_call__(isa(A, B), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    OutUnif = InUnif,
    OutLE = InLE, 
    type_check_type_call__(isa(A, B), CurrentVTB, NextVTB,
                           InUnif, Error).
%% check if arg is ground
type_check_call__(ground(Term), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    has_type_term_term__(Term, '?', [], [], _, InLE, _,
                         InUnif, Unders, 1, ground(Term), Error),
    OutUnif = InUnif,
    OutLE = InLE,
    collect_vars(Term, TermVars),
    set_vars_to_ground__(TermVars, Term, CurrentVTB, NextVTB, InUnif, Error).
%% once call as in Prolog
type_check_call__(once(Call), _Kind, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    '$goals2list'(Call, CallList),
    type_check_body__(CallList, rel, CurrentVTB, NextVTB, Unders,
                      InLE, OutLE, InUnif, OutUnif, Error, []).
%% not call
type_check_call__(not(Call), _Kind, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    '$goals2list'(Call, CallList),
    check_globals_locals__(CallList, [], G, [], _L,
                           Unders, part, _E),
    diff_list(G, Unders, RealG),
    non_ground__(RealG, CurrentVTB, InUnif, NonGround),
    %% Find the non-ground non-underscore global variables
    (
      NonGround = []
    ->
      %% All globals (other than underscores) are ground
      type_check_body__(CallList, rel, CurrentVTB, NextVTB, Unders,
                        InLE, _, InUnif, _, Error, []),
      OutLE = InLE,
      OutUnif = InUnif

    ;
      Error = non_ground_call(NonGround, not(Call))
    ).
%% ?(Call) in action body
type_check_call__('$rel_escape'(Call), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    '$goals2list'(Call, CallList),
    type_check_body__(CallList, rel, CurrentVTB, NextVTB, Unders,
                      InLE, OutLE, InUnif, OutUnif, Error, []).
%% wait call - simply check the call
type_check_call__(wait(Call), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    '$goals2list'(Call, CallList),
    type_check_body__(CallList, rel, CurrentVTB, NextVTB, Unders,
                      InLE, OutLE, InUnif, OutUnif, Error, []).
%% =? is used to match against strings or lists
type_check_call__('=?'(B, A), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    compound(A), A = (_ ++? _), !,
    has_type__(B, '!'(string), CurrentVTB, [], _OutVTB, InLE, MidLE,
               InUnif, Unders, 1, '=?'(B, A), Error),
    check_bind_type_constraints__(MidLE, Error),
    type_check_string_concat__(A, '=?'(B, A), CurrentVTB, NextVTB, Unders,
                               MidLE, OutLE, InUnif, OutUnif, Error).
type_check_call__('=?'(B, A), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    compound(A), A = (_ <>? _), !,
    has_type__(B, '!'(list(T)), CurrentVTB, [], _OutVTB, InLE, MidLE,
               InUnif, Unders, 1, '=?'(B, A), Error),
    check_bind_type_constraints__(MidLE, Error),
    type_check_list_concat__(A, T, '=?'(B, A), CurrentVTB, NextVTB, Unders,
                             MidLE, OutLE, InUnif, OutUnif, Error).
type_check_call__('$pfindall'(Ans, Patt, Body), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    OutUnif = InUnif,
    OutLE = InLE,
    '$goals2list'(Body, BodyList),
    type_check_body__(BodyList, rel, CurrentVTB, NextVTB1, Unders,
                      InLE, OutLE1, InUnif, OutUnif1, Error, inner),
    check_bind_type_constraints__(OutLE1, Error),
    has_type__(Patt, '??'(T), NextVTB1, [], VTB, [], OutLE1,
               OutUnif1, Unders, 0, Patt, Error),
    has_type__(Ans, '?'(list('??'(T))), NextVTB1, VTB, VTB1, [], OutLE1,
               OutUnif1, Unders, 0, Ans, Error),
    merge_type_check_VTB__(VTB1, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB).
               
    
type_check_call__('$exists'(Vars, Body), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    '$goals2list'(Body, BodyList),
    type_check_body__(BodyList, rel, CurrentVTB, NextVTB1, Unders,
                      InLE, OutLE1, InUnif, OutUnif1, Error, []),
    (
      var(Error)
    ->
      collect_vars((NextVTB1,OutUnif1), AllVars),
      diff_list(AllVars, Vars, GVars),
      copy_term((GVars, NextVTB1,OutLE1,OutUnif1),
                (GVars, NextVTB,OutLE,OutUnif))
    %% The purpose of the copy_term is to "hide" the bound variables
    %% so that later checks will not confuse information about these
    %% occurrences of the bound variables with later instances of
    %% bound variables with the same name
    ;
      true
    ).
type_check_call__('$forall'(QVars, LHS, RHS), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    NextVTB = CurrentVTB,
    OutUnif = InUnif,
    %% since forall is just a test (and doesn't instantiate any global
    %% variables) then the VTB and Unif constraints are unchanged by the call
    check_globals_locals__(['$forall'(QVars, LHS, RHS)], [], G, [], _L,
                           Unders, part, _E),
    diff_list(G, Unders, RealG),
    non_ground__(RealG, CurrentVTB, InUnif, NonGround),
    %% The non-underscore global variables are required to be ground
    %% at the time of call
    (
      NonGround = []
    ->
      '$goals2list'(LHS, LHSGoals),
      type_check_body__(LHSGoals, rel, CurrentVTB, LHSVTB, Unders,
                        InLE, MidLE, InUnif, LHSUnif, Error, inner),
      '$goals2list'(RHS, RHSGoals),
      type_check_body__(RHSGoals, rel, LHSVTB, _, Unders,
                        MidLE, OutLE, LHSUnif, _, Error, inner)
    ;
      Error = non_ground_globals(NonGround, '$forall'(QVars, LHS, RHS))
    ).
type_check_call__('$forall_actions'(QVars, LHS, RHS), _, CurrentVTB, NextVTB,
                  Unders, InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    NextVTB = CurrentVTB,
    OutUnif = InUnif,
    check_globals_locals__(['$forall_actions'(QVars, LHS, RHS)], [], G, [], _L,
                           Unders, part, _E),
    diff_list(G, Unders, RealG),
    non_ground__(RealG, CurrentVTB, InUnif, NonGround),
    (
      NonGround = []
    ->
      '$goals2list'(LHS, LHSGoals),
      type_check_body__(LHSGoals, rel, CurrentVTB, LHSVTB, Unders,
                        InLE, MidLE, InUnif, LHSUnif, Error, inner),
      '$goals2list'(RHS, RHSGoals),
      type_check_body__(RHSGoals, act, LHSVTB, _, Unders,
                        MidLE, OutLE, LHSUnif, _, Error, inner)
    ;
      Error = non_ground_globals(NonGround, '$forall_actions'(QVars, LHS, RHS))
    ).
type_check_call__(nonvar(Term), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, _Error, _) :-
    nonvar(Term), !,
    %% If Term is already a nonvar then nothing changes
    OutUnif = InUnif,
    OutLE = InLE,
    NextVTB = CurrentVTB.
type_check_call__(nonvar(Term), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, _Error, _) :-
    var(Term), type_member_of__(Term:M(T), CurrentVTB), !,
    OutUnif = InUnif,
    OutLE = InLE,
    (
      T = dyn_term, '$dyn_term_mode'(M)
    ->
      %% A dynamic term type with a dyn_term_mode - means functor is given
      %% and so is a nonvar
      NextVTB = CurrentVTB
    ;
      T = dyn_term
    ->
      %% The type is a dynamic term and so testing if it is a nonvar
      %% must change the mode so that the functor is given
      '$dyn_term_mode_modify'(M, ModM),
      diff_list(CurrentVTB, [Term:M(T)], VTB),
      NextVTB = [Term:ModM(T)|VTB]
    ;
      type_le__(T, atomic), M \= '!'
    ->
      %% an atomic nonvar is the same as a ground atomic
      diff_list(CurrentVTB, [Term:M(T)], VTB),
      NextVTB = [Term:'!'(T)|VTB]
    ;
      %% in all other cases, where the term is a known variable
      %% we leave unchanged.
      NextVTB = CurrentVTB
    ).
type_check_call__(nonvar(Term), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    var(Term), !,
    %% A new variable cannot possibly be a nonvar
    Error = nonvar_always_fails(nonvar(Term)),
    OutLE = InLE,
    OutUnif = InUnif,
    NextVTB = CurrentVTB.
type_check_call__(false(X), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    Error = false_always_fails(false(X)),
    OutLE = InLE,
    OutUnif = InUnif,
    NextVTB = CurrentVTB.
type_check_call__(raise(X), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    has_type__(X, '!'(exception), CurrentVTB, [], _,
                    InLE, OutLE, InUnif, [], 1, raise(X), Error),
    (
      var(Error)
    ->
      Error = raise_always_exits(raise(X))
    ;
      true
    ),
    OutLE = InLE,
    OutUnif = InUnif,
    NextVTB = CurrentVTB.
type_check_call__(remember(As), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    %% remember a list of ground dyn_term
    has_type_term__(list('!'(dyn_term)), '!', As, CurrentVTB, [], _,
                    InLE, OutLE, InUnif, Unders, 1, remember(As), Error),
    check_bind_type_constraints__(OutLE, Error),
    OutUnif = InUnif,
    NextVTB = CurrentVTB.

type_check_call__(allowed_rel_call(C), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    type_check_allowed_call__(C, rel, CurrentVTB, NextVTB, InLE, OutLE,
                              InUnif, Error),
    check_bind_type_constraints__(OutLE, Error),
    OutUnif = InUnif.
type_check_call__(allowed_dyn_call(C), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    type_check_allowed_call__(C, dyn, CurrentVTB, NextVTB, InLE, OutLE,
                              InUnif, Error),
    check_bind_type_constraints__(OutLE, Error),
    OutUnif = InUnif.
type_check_call__(allowed_act_call(C), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    type_check_allowed_call__(C, act, CurrentVTB, NextVTB, InLE, OutLE,
                              InUnif, Error),
    check_bind_type_constraints__(OutLE, Error),
    OutUnif = InUnif.
    
type_check_call__(call(Call), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    type_check_call_call__(Call, CurrentVTB, NextVTB, 
                           InLE, OutLE, InUnif, Error),
    OutUnif = InUnif.
type_check_call__(do(Call), _, CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    type_check_do_call__(Call, CurrentVTB, NextVTB, 
                           InLE, OutLE, InUnif, Error),
    OutUnif = InUnif.
type_check_call__(remember_for(As, T), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    %% same as above with a ground num as a time
    has_type_term__(list('!'(dyn_term)), '!', As, CurrentVTB, [], _,
                    InLE, MidLE, InUnif, Unders, 1, remember_for(As, T), Error),
    check_bind_type_constraints__(MidLE, Error),
    has_type__(T, '!'(num), CurrentVTB, [], _, MidLE, OutLE, InUnif, Unders, 2,
               remember_for(As, T), Error),
    check_bind_type_constraints__(OutLE, Error),
    NextVTB = CurrentVTB,
    OutUnif = InUnif.
type_check_call__(forget(As), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    %% for forget we have a list of dyn_term but the mode is the special
    %% internal mode !?? that means that the functor is given but
    %% the arguments do not need to be given and might not be instantiated
    %% by the call
    ip_lookup('$call_kind', Kind),
    ip_set('$call_kind', rel),
    has_type_term__(list('!??'(dyn_term)), '!', As, CurrentVTB, [], _,
                    InLE, OutLE, InUnif, Unders, 1, forget(As), Error),
    ip_set('$call_kind', Kind),
    check_bind_type_constraints__(OutLE, Error), 
    OutUnif = InUnif,
    NextVTB = CurrentVTB.
type_check_call__(forget_remember(As, Bs), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    !,
    %% forget As first and then remember Bs - type checking is as above
    ip_lookup('$call_kind', Kind),
    ip_set('$call_kind', rel),
    has_type_term__(list('!??'(dyn_term)), '!', As, CurrentVTB, [], _,
                    InLE, MidLE, InUnif, Unders, 1, forget_remember(As, Bs),
                    Error),
    ip_set('$call_kind', Kind),
    check_bind_type_constraints__(MidLE, Error),
    has_type_term__(list('!'(dyn_term)), '!', Bs, CurrentVTB, [], _,
                    MidLE, OutLE, InUnif, Unders, 2, forget_remember(As, Bs),
                    Error),
    check_bind_type_constraints__(OutLE, Error),
    OutUnif = InUnif,
    NextVTB = CurrentVTB.
type_check_call__(atomic_action(A), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    %% A is a sequence of actions to be called atomically - type checked
    %% as a normal sequence of actions
    '$goals2list'(A, AL),
     type_check_body__(AL, act, CurrentVTB, NextVTB, Unders,
                       InLE, OutLE, InUnif, OutUnif, Error1, SVTB),
     (
       nonvar(Error1)
     ->
       Error1 = body_call_error(_, Err),
       Error = body_call_error(atomic_action(A), Err)
     ;
       true
     ).
type_check_call__(repeat_until(A, '$fail'), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    %% A is a sequence of actions to be called atomically - type checked
    %% as a normal sequence of actions
    '$goals2list'(A, AL),
     type_check_body__(AL, act, CurrentVTB, NextVTB, Unders,
                       InLE, OutLE, InUnif, OutUnif, Error1, SVTB),
     Error1 = Error.
     % (
     %   nonvar(Error1)
     % ->
     %   Error1 = body_call_error(_, Err),
     %   Error = body_call_error(repeat_until(A, '$fail'), Err)
     % ;
     %   true
     % ).
type_check_call__(repeat_until(A, C), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    %% A is a sequence of actions to be called atomically - type checked
    %% as a normal sequence of actions
    '$goals2list'(A, AL),
    '$goals2list'(C, CL),
     type_check_body__(AL, act, CurrentVTB, MidVTB, Unders,
                       InLE, MidLE, InUnif, MidUnif, Error1, SVTB),
     type_check_body__(CL, rel, MidVTB, NextVTB, Unders,
                       MidLE, OutLE, MidUnif, OutUnif, Error1, SVTB),
     Error1 = Error.
     
     % errornl(eee___(Error1)),
     % (
     %   nonvar(Error1)
     % ->
     %   Error1 = body_call_error(_, Err),
     %   Error = body_call_error(repeat_until(A, C), Err)
     % ;
     %   true
     % ).
type_check_call__(from(Msg, Addr, Test), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    %% message receive: Msg is a term - all we can assume is it is a legal term
    %% (that might be a variable or contain variables)
    %% Addr can be given or will be instantiated to an agent_handle
    %% Test is a conjunction of relation calls
    ip_lookup('$call_kind', CallKind),
    ip_set('$call_kind', rel),
    has_type__(Msg, '??'(term), CurrentVTB, [], MidVTB,
               InLE, MidLE, InUnif, Unders, 1, from(Msg, Addr, Test), Error),
    check_bind_type_constraints__(MidLE, Error),
    has_type__(Addr, '?'(agent_handle), CurrentVTB, MidVTB, OutVTB,
               MidLE, Mid2LE, InUnif, Unders, 2, from(Msg, Addr, Test), Error),
    ip_set('$call_kind', CallKind),
    check_bind_type_constraints__(Mid2LE, Error),
    merge_type_check_VTB__(OutVTB, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, MidNextVTB),
    '$goals2list'(Test, Goals),
    type_check_body__(Goals, rel, MidNextVTB, NextVTB, Unders,
                      Mid2LE, OutLE, InUnif, OutUnif, Error, inner).
type_check_call__(from_thread(Msg, Addr, Test), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    %% same as above except Addr is the address of a thread (thread_handle)
    ip_lookup('$call_kind', CallKind),
    ip_set('$call_kind', rel),
    has_type__(Msg, '??'(term), CurrentVTB, [], MidVTB,
               InLE, MidLE, InUnif, Unders, 1, from(Msg, Addr, Test), Error),
    check_bind_type_constraints__(MidLE, Error),
    has_type__(Addr, '?'(thread_handle), CurrentVTB, MidVTB, OutVTB,
               MidLE, Mid2LE, InUnif, Unders, 2, from(Msg, Addr, Test), Error),
    ip_set('$call_kind', CallKind),
    check_bind_type_constraints__(Mid2LE, Error),
    merge_type_check_VTB__(OutVTB, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, MidNextVTB),
    '$goals2list'(Test, Goals),
    type_check_body__(Goals, rel, MidNextVTB, NextVTB, Unders,
                      Mid2LE, OutLE, InUnif, OutUnif, Error, inner).

type_check_call__(case(ActAlts), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    %% Type check each alt to make sure each type checks correctly
    %% Note that in these alt actions OutLE and NextVTB is
    %% strictly not needed as all the tests are done inside the
    %% alt analysis  - it's needed as the saved VTB test is redone
    %% at the top-level.
    OutLE = InLE,
    type_check_act_alts__(ActAlts, case_error, CurrentVTB, NextVTB, Unders,
                          InLE, InUnif, OutUnif, Error, SVTB).
type_check_call__(try_except(Try, Excepts), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    OutLE = InLE,
    %% The exceptions should be treated in a similar way to case
    type_check_excepts__(Excepts, CurrentVTB, Unders,
                         InLE, InUnif, Error, SVTB),
    %% The try should also type check
    type_check_try__(Try, CurrentVTB, NextVTB, Unders,
                     InLE, InUnif, OutUnif, Error, SVTB).    
type_check_call__(wait_case(ActAlts), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    OutLE = InLE,
    %% Ditto wait_case
    type_check_act_alts__(ActAlts, wait_case_error, CurrentVTB, NextVTB, Unders,
                          InLE, InUnif, OutUnif, Error, SVTB).
type_check_call__(receive(ActAlts), _, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    OutLE = InLE,
    %% Ditto receive
    type_check_receive_alts__(ActAlts, CurrentVTB, NextVTB, Unders,
                              InLE, InUnif, OutUnif, Error, SVTB).
type_check_call__('$remote_query'('$num_vars_query'(N, Vars,Query), Addr), _,
                  CurrentVTB, NextVTB,
                  Unders, InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    %NextVTB = CurrentVTB,
    InLE = OutLE,
    InUnif = OutUnif,
    '$goals2list'(Query, QueryList),
    has_type_term__(nat, '!', N, CurrentVTB, [], _, InLE, _,
                    InUnif, _, 0, N, Error),
    has_type__(Addr, '!'(agent_handle), CurrentVTB, [], _, InLE, _,
                    InUnif, _, 0, Addr, Error),
    type_check_body__(QueryList, rel, CurrentVTB, OutVTB,Unders,
                      InLE, _, InUnif, MidUnif, Error, inner),
    check_globals_locals__([Query], [], _V, [], _L,
                           Unders, part, Error),
    type_check_remote_query_vars__(Vars, OutVTB, VarsVTB, MidUnif, Error),
    check_bind_type_constraints__(OutLE, Error),
    remember_remote_query_types__('$remote_query'('$num_vars_query'(N, Vars, Query), Addr), VarsVTB, Error),
    merge_VTB_currentVTB__(VarsVTB, CurrentVTB, NextVTB).
type_check_call__('$remote_query'('$num_query'(N, Query), Addr), _,
                  CurrentVTB, NextVTB,
                  Unders, InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    %NextVTB = CurrentVTB,
    InLE = OutLE,
    InUnif = OutUnif,
    '$goals2list'(Query, QueryList),
    has_type_term__(nat, '!', N, CurrentVTB, [], _, InLE, _,
                    InUnif, _, 0, N, Error),
    has_type__(Addr, '!'(agent_handle), CurrentVTB, [], _, InLE, _,
                    InUnif, _, 0, Addr, Error),
    type_check_body__(QueryList, rel, CurrentVTB, OutVTB,Unders,
                      InLE, _, InUnif, MidUnif, Error, inner),
    check_globals_locals__([Query], [], Vars, [], _L,
                           Unders, part, Error),
    check_bind_type_constraints__(OutLE, Error),
    type_check_remote_query_vars__(Vars, OutVTB, VarsVTB, MidUnif, Error),
    remember_remote_query_types__('$remote_query'('$num_query'(N, Query),
                                                  Addr),
                                  VarsVTB, Error),
    merge_VTB_currentVTB__(VarsVTB, CurrentVTB, NextVTB).
type_check_call__('$remote_query'('$vars_query'(Vars,Query), Addr), _,
                  CurrentVTB, NextVTB,
                  Unders, InLE, OutLE, InUnif, OutUnif, Error1, _SVTB) :-
    !,
    %NextVTB = CurrentVTB,
    InLE = OutLE,
    InUnif = OutUnif,
    '$goals2list'(Query, QueryList),
    has_type__(Addr, '!'(agent_handle), CurrentVTB, [], _, InLE, _,
               InUnif, _, 0, Addr, Error),
    type_check_body__(QueryList, rel, CurrentVTB, OutVTB,Unders,
                      InLE, _, InUnif, MidUnif, Error, _),
    check_globals_locals__([Query], [], _V, [], _L,
                           Unders, part, Error),
    type_check_remote_query_vars__(Vars, OutVTB, VarsVTB, MidUnif, Error),
    remember_remote_query_types__('$remote_query'('$vars_query'(Vars,Query), Addr), VarsVTB, Error1),
    merge_VTB_currentVTB__(VarsVTB, CurrentVTB, NextVTB).
type_check_call__('$remote_query'(Query, Addr), _, CurrentVTB, NextVTB,
                  Unders, InLE, OutLE, InUnif, OutUnif, Error, _SVTB) :-
    !,
    %NextVTB = CurrentVTB,
    InLE = OutLE,
    InUnif = OutUnif,
    '$goals2list'(Query, QueryList),
    has_type__(Addr, '!'(agent_handle), CurrentVTB, [], _, InLE, _,
                    InUnif, _, 0, Addr, Error),
    type_check_body__(QueryList, rel, CurrentVTB, OutVTB,Unders,
                      InLE, _, InUnif, MidUnif, Error, inner),
    check_globals_locals__([Query], [], Vars, [], _L,
                           Unders, part, Error),
    type_check_remote_query_vars__(Vars, OutVTB, VarsVTB, MidUnif, Error),
    remember_remote_query_types__('$remote_query'(Query, Addr),
                                  VarsVTB, Error),
    check_bind_type_constraints__(OutLE, Error),
    merge_VTB_currentVTB__(VarsVTB, CurrentVTB, NextVTB).
    
type_check_call__('$null_action', _,  CurrentVTB, NextVTB, _Unders,
                  InLE, OutLE, InUnif, OutUnif, _Error, _) :-
    !,
    %% NOOP action - do nothing
    OutUnif = InUnif,
    OutLE = InLE,
    CurrentVTB = NextVTB.
type_check_call__(Call, _Kind, CurrentVTB, NextVTB, Unders,
                  InLE, OutLE, InUnif, OutUnif, Error, _) :-
    '$annotated_type_info'(_Types, Call), !,
    %% "normal" call
    InUnif = OutUnif,
    findall(T, '$annotated_type_info'(T, Call), AllTypes),
    '@=..'(Call, [_F|Args]),
    type_check_call_aux__(AllTypes, Call, Args, CurrentVTB, NextVTB, Unders,
                          InLE, OutLE, InUnif, Error).
type_check_call__(Call, _, VTB, VTB, _Unders, LE, LE, Unif, Unif, Error, _) :-
    %% Shouldn't get here - call to an undefined relation/action
    Error = unprocessed_call(Call).

%% Type check each alternative - all the alternative sequences of actions
%% "go to the end of the rule" so each check is the same as checking the
%% given alternative is the body of the rule. This includes checking
%% that any ? moded vars in the head have become ! moded at the end
%% via the check_savedVTB__ call
%%type_check_act_alts__(A,B,C,D,E,F,G,H,I,J) :- errornl(type_check_act_alts__(A,B,C,D,E,F,G,H,I,J)),fail.
type_check_act_alts__(_ActAlts, _, CurrentVTB, NextVTB, _Unders,
                      _InLE, InUnif, OutUnif, Error, _SVTB) :-
    nonvar(Error), !,
    OutUnif = InUnif,
    NextVTB = CurrentVTB.
type_check_act_alts__([timeout(T, Act)], ErrorKind, CurrentVTB, NextVTB, Unders,
                      InLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    %% The last action is a timeout that takes a number as the first arg
     has_type__(T, '!'(num), CurrentVTB, InLE, MidLE, 
                [], _, InUnif, Unders, 0, T, Err1),
     check_bind_type_constraints__(MidLE, Err1),
     '$get_body_call_error'(Err1, T, ErrB1),
    check_error__(ErrB1, body, Error1),
     '$goals2list'(Act, ActList),
     type_check_body__(ActList, act, CurrentVTB, NextVTB,Unders,
                       MidLE, _OutLE, InUnif, OutUnif, Err1, SVTB),
     check_error__(Err1, body, Error1),
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
     check_savedVTB__(SVTB, NextVTB, OutUnif, Unders, Err1),
     ip_set('$call_kind', CallKind),
     check_error__(Err1, saved, Error1),
     update_act_alt_error__(Error1, ErrorKind, timeout(T, Act), Error).
type_check_act_alts__(['$case_alt'(Test, Act)], ErrorKind, CurrentVTB, NextVTB,
                      Unders, InLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    %% the last alt is not a timeout
    '$goals2list'(Test, TestList),
    '$goals2list'(Act, ActList),
    %% check the test as a relation
    type_check_body__(TestList, rel, CurrentVTB, MidVTB,Unders,
                      InLE, MidLE, InUnif, MidUnif, Err1, []),
    %% check Act as an action
    type_check_body__(ActList, act, MidVTB, NextVTB,Unders,
                      MidLE, _OutLE, MidUnif, OutUnif, Err1, SVTB),
    check_error__(Err1, body, Error1),
    ip_lookup('$call_kind', CallKind),
    ip_set('$call_kind', rel),
    check_savedVTB__(SVTB, NextVTB, OutUnif, Unders, Err1),
    ip_set('$call_kind', CallKind),
    check_error__(Err1, saved, Error1),
    update_act_alt_error__(Error1, ErrorKind, '$case_alt'(Test, Act), Error).
type_check_act_alts__(['$case_alt'(Test, Act)|Rest], ErrorKind, CurrentVTB,
                      NextVTB, Unders, InLE, InUnif, OutUnif, Error, SVTB) :-
    %% The recursive case
    %% Since we are checking the alternatives by recursion we need to save
    %% act_seen so it can be reset for the next alternative
    ip_lookup('$act_seen', SavedSeen),
    '$goals2list'(Test, TestList),
    '$goals2list'(Act, ActList),
    type_check_body__(TestList, rel, CurrentVTB, MidVTB,Unders,
                      InLE, MidLE, InUnif, MidUnif, Err1, []),   
    type_check_body__(ActList, act, MidVTB, NextVTB0,Unders,
                      MidLE, _OutLE, MidUnif, OutUnif1, Err1, SVTB),
    check_error__(Err1, body, Error1),
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
    check_savedVTB__(SVTB, NextVTB0, OutUnif1, Unders, Err1),
     ip_set('$call_kind', CallKind),
    check_error__(Err1, saved, Error1),
    update_act_alt_error__(Error1,  ErrorKind, '$case_alt'(Test, Act), Error),
    ip_set('$act_seen', SavedSeen),
    type_check_act_alts__(Rest,  ErrorKind, CurrentVTB, NextVTB, Unders,
                          InLE, InUnif, OutUnif, Error, SVTB).
%% Same as above except for checking the try part of a try-except
type_check_try__(_Try, CurrentVTB, NextVTB, _Unders,
                 _InLE, InUnif, OutUnif, Error, _SVTB) :-
    nonvar(Error), !,
    NextVTB = CurrentVTB,
    OutUnif = InUnif.
type_check_try__(Try, CurrentVTB, NextVTB, Unders,
                 InLE, InUnif, OutUnif, Error, SVTB) :-
    '$goals2list'(Try, TryList),
     type_check_body__(TryList, act, CurrentVTB, NextVTB,Unders,
                       InLE, _, InUnif, OutUnif, Err1, SVTB),
     check_error__(Err1, body, Error1),
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
     check_savedVTB__(SVTB, NextVTB, OutUnif, Unders, Err1),
     ip_set('$call_kind', CallKind),
     check_error__(Err1, saved, Error1),
     update_act_alt_error__(Error1, try_error, Try, Error).

%% Same as above except for checking the except part of a try-except
%%type_check_excepts__(A,B,C,D,E,F,G) :- errornl(type_check_excepts__(A,B,C,D,E,F,G)),fail.
type_check_excepts__(_Excepts, _CurrentVTB, _Unders,
                     _InLE, _InUnif, Error, _SVTB) :-
    nonvar(Error), !.
type_check_excepts__([], _CurrentVTB, _Unders,
                     _InLE, _InUnif, _Error, _SVTB).
type_check_excepts__(['$catch'(Patt, '$empty', Act)|Excepts], CurrentVTB,
                     Unders, InLE, InUnif, Error, SVTB) :-
    !,
    %% no test - just matching against an exception pattern
    ip_set('$call_kind', rel),
    ip_lookup('$act_seen', SavedSeen),
    has_type__(Patt, '?'(exception), CurrentVTB, [], VTB, InLE, MidLE,
               InUnif, Unders, 1, '$catch'(Patt, '$empty', Act), Error),
    check_bind_type_constraints__(MidLE, Error),
    merge_type_check_VTB__(VTB, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, MidCurrentVTB),
    '$goals2list'(Act, ActList),
    type_check_body__(ActList, act, MidCurrentVTB, NextVTB, Unders,
                      MidLE, _, InUnif, OutUnif, Err1, SVTB),
    check_error__(Err1, body, Error1),
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
    check_savedVTB__(SVTB, NextVTB, OutUnif, Unders, Err1),
     ip_set('$call_kind', CallKind),
    check_error__(Err1, saved, Error1),
    update_act_alt_error__(Error1, except_error,
                           '$catch'(Patt, '$empty', Act), Error),
    ip_set('$act_seen', SavedSeen),
    type_check_excepts__(Excepts, CurrentVTB, Unders,
                         InLE, InUnif, Error, SVTB).
type_check_excepts__(['$catch'(Patt, Test, Act)|Excepts], CurrentVTB, Unders,
                     InLE, InUnif, Error, SVTB) :-
    %% matching against an exception pattern with test
    ip_set('$call_kind', rel),
    ip_lookup('$act_seen', SavedSeen),
    '$goals2list'(Act, ActList),
    has_type__(Patt, '?'(exception), CurrentVTB, [], VTB, InLE, MidLE,
               InUnif, Unders, 0, Patt, Err1 ),
    check_bind_type_constraints__(MidLE, Err1 ),
    merge_type_check_VTB__(VTB, MergedVTB, Err1 ),
    '$get_body_call_error'(Err1, Patt, ErrB1),
    check_error__(ErrB1, body, Error1),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, MidCurrentVTB),
    '$goals2list'(Test, TestList),
    '$goals2list'(Act, ActList),
    type_check_body__(TestList, rel, MidCurrentVTB, MidVTB,Unders,
                      MidLE, Mid2LE, InUnif, MidUnif, Err1, SVTB),
    type_check_body__(ActList, act, MidVTB, NextVTB, Unders,
                      Mid2LE, _, MidUnif, OutUnif, Err1, SVTB),
    check_error__(Err1, body, Error1),
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
    check_savedVTB__(SVTB, NextVTB, OutUnif, Unders, Err1),
     ip_set('$call_kind', CallKind),
    check_error__(Err1, saved, Error1),
    update_act_alt_error__(Error1, except_error,
                           '$catch'(Patt, Test, Act), Error),
    ip_set('$act_seen', SavedSeen),
    type_check_excepts__(Excepts, CurrentVTB, Unders,
                         InLE, InUnif, Error, SVTB).

%% Same as above except for checking receive
%%type_check_receive_alts__(A,B,C,D,E,F,G,H,I) :- errornl(type_check_receive_alts__(A,B,C,D,E,F,G,H,I)),fail.
type_check_receive_alts__(_ActAlts, CurrentVTB, NextVTB, _Unders,
                          _InLE, InUnif, OutUnif, Error, _SVTB) :-
    nonvar(Error), !,
    OutUnif = InUnif,
    NextVTB = CurrentVTB.
type_check_receive_alts__([timeout(T, Act)], CurrentVTB, NextVTB, Unders,
                          InLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    %% last case is a timeout
    has_type__(T, '!'(num), CurrentVTB, [], _,
               InLE, MidLE, InUnif, Unders, 1, timeout(T, Act), Err1),
    check_bind_type_constraints__(MidLE, Err1),
    '$get_body_call_error'(Err1, T, ErrB1),
    check_error__(ErrB1, body, Error1),
    '$goals2list'(Act, ActList),
    type_check_body__(ActList, act, CurrentVTB, NextVTB,Unders,
                      MidLE, _, InUnif, OutUnif, Err1, SVTB),
    check_error__(Err1, body, Error1),
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
    check_savedVTB__(SVTB, NextVTB, OutUnif, Unders, Err1),
     ip_set('$call_kind', CallKind),
    check_error__(Err1, saved, Error1),
    update_act_alt_error__(Error1, receive_error,
                           timeout(T, Act), Error).
type_check_receive_alts__([Alt], CurrentVTB, NextVTB, Unders,
                          InLE, InUnif, OutUnif, Error, SVTB) :-
    !,
    %% last case is a receive pattern/action
    type_check_receive_alts_aux__(Alt, CurrentVTB, NextVTB, Unders,
                                  InLE, InUnif, OutUnif, Error, SVTB).
type_check_receive_alts__([Alt|ActAlts], CurrentVTB, NextVTB, Unders,
                          InLE, InUnif, OutUnif, Error, SVTB) :-
    %% recursive case
    ip_lookup('$act_seen', SavedSeen),    
    type_check_receive_alts_aux__(Alt, CurrentVTB, _NextVTB, Unders,
                                  InLE, InUnif, _OutUnif, Error, SVTB),
    ip_set('$act_seen', SavedSeen),
    type_check_receive_alts__(ActAlts, CurrentVTB, NextVTB, Unders,
                              InLE, InUnif, OutUnif, Error, SVTB).

type_check_receive_alts_aux__('$normal'('$receive_from'(M, from(A),
                                                        '$empty', Act)),
                              CurrentVTB, NextVTB, Unders,
                              InLE, InUnif, OutUnif, Error, SVTB) :-    
    !,
    %% M from A ~> Act
    Call = '$normal'('$receive_from'(M, from(A),
                                     '$empty', Act)),
    type_check_receive_alts_aux_1__(M, true('$none_'), A, Act, agent_handle,
                                    Call, CurrentVTB, NextVTB, Unders,
                                    InLE, InUnif, OutUnif, Error, SVTB).

type_check_receive_alts_aux__('$normal'('$receive_from'(M, from_thread(A),
                                                        '$empty', Act)),
                              CurrentVTB, NextVTB, Unders,
                              InLE, InUnif, OutUnif, Error, SVTB) :-    
    !,
    %% M from_thread A ~> Act
    Call = '$normal'('$receive_from'(M, from_thread(A),
                                     '$empty', Act)),
    type_check_receive_alts_aux_1__(M, true('$none_'), A, Act, thread_handle,
                                    Call,  CurrentVTB, NextVTB, Unders,
                                    InLE, InUnif, OutUnif, Error, SVTB).
type_check_receive_alts_aux__('$normal'('$receive_from'(M, from(A),
                                                        Test, Act)),
                              CurrentVTB, NextVTB, Unders,
                              InLE, InUnif, OutUnif, Error, SVTB) :-    
    !,
    %% M from A :: Test ~> Act
    Call = '$normal'('$receive_from'(M, from(A),
                                     Test, Act)),
    type_check_receive_alts_aux_1__(M, Test, A, Act, agent_handle, Call,
                                    CurrentVTB, NextVTB, Unders,
                                    InLE, InUnif, OutUnif, Error, SVTB).

type_check_receive_alts_aux__('$normal'('$receive_from'(M, from_thread(A),
                                                        Test, Act)),
                              CurrentVTB, NextVTB, Unders,
                              InLE, InUnif, OutUnif, Error, SVTB) :-    
    !,
    %% M from_thread A :: Test ~> Act
    Call = '$normal'('$receive_from'(M, from_thread(A),
                                     Test, Act)),
    type_check_receive_alts_aux_1__(M, Test, A, Act, thread_handle, Call,
                                    CurrentVTB, NextVTB, Unders,
                                    InLE, InUnif, OutUnif, Error, SVTB).

type_check_receive_alts_aux_1__(M, Test, A, Act, AddrType, Call,
                                CurrentVTB, NextVTB, Unders,
                                InLE, InUnif, OutUnif, Error, SVTB) :-
    %% the message is a valid term
    ip_set('$call_kind', rel),
    has_type_term__(term, '??', M, CurrentVTB, [], MVTB, InLE, MLE,
                    InUnif, _, 0, M, Err1 ),
    check_bind_type_constraints__(MLE, Err1),
    '$get_body_call_error'(Err1, M, ErrB1),
    check_error__(ErrB1, body, Error1),
    %% the address is of type agent_handle or thread_handle
    has_type_term__(AddrType, '?', A, CurrentVTB, MVTB, VTB, MLE, M2LE,
                    InUnif, _, 0, A, Err1 ),
    check_bind_type_constraints__(MLE, Err1 ),
    merge_type_check_VTB__(VTB, MergedVTB, Err1 ),
    '$get_body_call_error'(Err1, A, ErrB2),
    check_error__(ErrB2, body, Error1),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, MidCurrentVTB),
    '$goals2list'(Test, TestList),
    '$goals2list'(Act, ActList),
    %% the test is a valid relation call
    type_check_body__(TestList, rel, MidCurrentVTB, MidVTB,Unders,
                      M2LE, M3LE, InUnif, MidUnif, Err1, []),
    type_check_body__(ActList, act, MidVTB, NextVTB, Unders,
                      M3LE, _, MidUnif, OutUnif, Err1, SVTB),
    check_error__(Err1, body, Error1),
     ip_lookup('$call_kind', CallKind),
     ip_set('$call_kind', rel),
    check_savedVTB__(SVTB, NextVTB, OutUnif, Unders, Err1),
     ip_set('$call_kind', CallKind),
    check_error__(Err1, saved, Error1),
    update_act_alt_error__(Error1, receive_error, Call, Error).

%% For the alternatives above the error (if it is an error) is wrapped
%% with the information about which alternative was taken so that
%% this info can be displayed as paty of the error message
%%update_act_alt_error__(A,B,C,D) :- errornl(update_act_alt_error__(A,B,C,D)),fail.
update_act_alt_error__(E, _, _, _) :-
    var(E), !.
update_act_alt_error__(body(body_call_error(Call, E)), Kind, Case, Error) :-
    !,
    Error = body(body_call_error(Call,Kind(Case, E))).
update_act_alt_error__(saved(E), Kind, Case, Error) :-
    !,
    Error = Kind(Case, E).

%% type_check_call_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders,
%%                              InLE, OutLE, InUnif, OutUnif,Error)
%% F is a variable and the functor of Call - check that
%% F is in CurrentVTB and is a code type of kind Kind (rel/act/tel)
%%type_check_call_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif, OutUnif,Error) :- errornl(type_check_call_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif, OutUnif,Error)),fail.
type_check_call_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders,
                              InLE, OutLE, InUnif, OutUnif,Error1) :-
    type_member_of__(F:'!'(FT), CurrentVTB),
    '$known_call_type'(FT, ArgsType, Kind), !,
    %% F is in CurrentVTB and is of type FT which is of the appropriate code
    %% kind - check the Args of Call are of type ArgsTypes
    %'$known_call_type'(FT, ArgsType, Kind),
    '@..rev'(Call, _, Args),
    (
      length(Args, N), length(ArgsType, N)
    ->
      true
    ;
      Error = arity_error(Call, ArgsType)
    ),
    has_type_term_iterate__(ArgsType, Args, CurrentVTB, [], OutVTB,
                            InLE, OutLE, InUnif, Unders, 1, Call, Error),
    check_bind_type_constraints__(OutLE, Error),
    merge_type_check_VTB__(OutVTB, MergedVTB, Error),
    (
      var(Error)
    ->
      true
    ;
      %% There are mode/type errors in the Args
      Error1 = call_error(Call, FT, Error)
    ),
    OutUnif = InUnif,
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB).
type_check_call_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, _Unders,
                              InLE, OutLE, InUnif, OutUnif,Error) :-
    type_member_of__(F:'!'(FT), CurrentVTB), 
    '$known_call_type'(FT, _ArgsType, FTKind), !,
    %% F is in CurrentVTB and is of type FT that is a code type but not
    %% the correct kind
    OutUnif = InUnif,
    OutLE = InLE,
    NextVTB = CurrentVTB,
    Error = kind_error(Call, Kind, FTKind). 
type_check_call_var_functor__(F, Call, _, CurrentVTB, NextVTB, _Unders,
                              InLE, OutLE, InUnif, OutUnif,Error) :-
    type_member_of__(F:M(FT), CurrentVTB), M \= '!', !,
    %% F is in CurrentVTB but not ground
    OutUnif = InUnif,
    OutLE = InLE,
    NextVTB = CurrentVTB,
    Error = call_error(Call, mode_error(F, M(FT))).
type_check_call_var_functor__(F, Call, _, CurrentVTB, NextVTB, _Unders,
                              InLE, OutLE, InUnif, OutUnif,Error) :-
    type_member_of__(F:MT, CurrentVTB), !,
    %% F is in CurrentVTB (and ground) but not a code type
    OutUnif = InUnif,
    OutLE = InLE,
    NextVTB = CurrentVTB,
    Error = call_error(Call, not_a_code_type(F, MT)).
type_check_call_var_functor__(F, _Call, _, CurrentVTB, NextVTB, _Unders,
                              InLE, OutLE, InUnif, OutUnif,Error) :-
    %% F is not in CurrentVTB
    OutUnif = InUnif,
    OutLE = InLE,
    NextVTB = CurrentVTB,
    Error = call_error(F, new_variable).

%% Similar to above but in this case F is the ultimate functor of a higher-order
%% term (and therefore a function call)
%%type_check_call_ho_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE,InUnif,Error) :- errornl(type_check_call_ho_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE,InUnif, Error)),fail.
type_check_call_ho_var_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders,
                                 InLE, OutLE,InUnif,Error) :-
    type_member_of__(F:M(Type), CurrentVTB), M = '!', !,
    %% F is in CurrentVTB and is ground
    %% check if Type
    get_function_code_types__(Call, Type, Kind, [], ArgsList, ArgTypesList,
                             Error),
    check_function_arities__(ArgsList, ArgTypesList, Call, Error),
    flatten_args_list__(ArgsList, ArgTypesList, Args, ArgTypes, Error),
    '$add_annotations'(ArgTypes, ATypes, '!'),
    once((
           has_type_term_index_iterate__(ATypes, Args, 1, Call, CurrentVTB,
                                         [], CallVTB, InLE, OutLE, InUnif,
                                         Unders, Error),
           check_bind_type_constraints__(OutLE, Error),
           merge_type_check_VTB__(CallVTB, MergedVTB, Error),
          merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB)
          )).

type_check_call_ho_var_functor__(F, Call, _, CurrentVTB, NextVTB, _Unders,
                                 InLE, OutLE, _InUnif,Error) :-
    type_member_of__(F:M(FT), CurrentVTB), !,
    NextVTB = CurrentVTB,
    OutLE = InLE,
    Error = call_error(Call, mode_error(F, M(FT))).
type_check_call_ho_var_functor__(_F, Call, _, CurrentVTB, NextVTB, _Unders,
                                 InLE, OutLE,_InUnif,Error) :-
    NextVTB = CurrentVTB,
    OutLE = InLE,
    Error = call_error(Call, not_a_code_type).

%%type_check_call_ho_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE,InUnif,Error) :- errornl(type_check_call_ho_functor__(F, Call, Kind, CurrentVTB, NextVTB, Unders, InLE, OutLE,InUnif, Error)),fail.
type_check_call_ho_functor__(Call, Kind, CurrentVTB, NextVTB, Unders,
                             InLE, OutLE, InUnif,Error) :-
    '@=..'(Call, [Code|Args]),
    '$get_ho_types'(Code, Type, CurrentVTB, InUnif, Unders, Error),
    '$extract_ho_alts'(Type, AllCodeTypes, Call, Kind, Error),
    %% Assuming no errors are found AllCodeTypes is a list of alternatives
    %% for the types of Args
    (
      var(Error)
    ->
      '$extract_tuple_types'(AllCodeTypes, AllTypes),
      %% now try to type check Args based on the alternatives in AllTypes
      type_check_call_aux__(AllTypes, Call, Args, CurrentVTB, NextVTB, Unders,
                            InLE, OutLE, InUnif, Error)
    ;
      NextVTB = CurrentVTB
    ).

'$extract_tuple_types'([], []).
'$extract_tuple_types'([_(_(TupleT))|Rest], [TupleT|RestTuples]) :-
    '$extract_tuple_types'(Rest, RestTuples).

% Type check call(Call)
type_check_call_call__(Call, CurrentVTB, NextVTB, 
                       InLE, OutLE, Unif, Error) :-
    var(Call),
    in_unif__((Call = UCall), Unif),!,
    type_check_call_call__(UCall, CurrentVTB, NextVTB, 
                           InLE, OutLE, Unif, Error).
type_check_call_call__(Call, CurrentVTB, NextVTB, 
                       InLE, OutLE, Unif, Error) :-
    var(Call),
    type_remove_from__(Call:M(_), CurrentVTB, Rest), !,
    has_type__(Call, '!??'(rel_term), CurrentVTB, [], OutVTB, 
               InLE, OutLE, Unif, [], 1, call(Call), Error),
    (
      M = '!'
    ->
      M1 = '!'
    ;
      M1 = '!??'
    ),
    merge_type_check_VTB__(OutVTB, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, [Call:M1(rel_term)|Rest], NextVTB).
type_check_call_call__(Call, CurrentVTB, NextVTB, 
                       InLE, OutLE, _Unif, Error) :-
    var(Call), !,
    NextVTB = CurrentVTB,
    OutLE = InLE,
    Error = new_var_call(call(Call)).
type_check_call_call__(Call, CurrentVTB, NextVTB, 
                       InLE, OutLE, Unif, Error) :-
    has_type__(Call, '!??'(rel_term), CurrentVTB, [], OutVTB, 
               InLE, OutLE, Unif, [], 1, call(Call), Error),
    merge_type_check_VTB__(OutVTB, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB).


% Type check do(Call)
type_check_do_call__(Call, CurrentVTB, NextVTB, 
                       InLE, OutLE, Unif, Error) :-
    var(Call),
    in_unif__((Call = UCall), Unif),!,
    type_check_do_call__(UCall, CurrentVTB, NextVTB, 
                           InLE, OutLE, Unif, Error).
type_check_do_call__(Call, CurrentVTB, NextVTB, 
                       InLE, OutLE, Unif, Error) :-
    var(Call),
    type_remove_from__(Call:M(_), CurrentVTB, Rest), !,
    has_type__(Call, '!??'(act_term), CurrentVTB, [], OutVTB, 
               InLE, OutLE, Unif, [], 1, call(Call), Error),
    (
      M = '!'
    ->
      M1 = '!'
    ;
      M1 = '!??'
    ),
    merge_type_check_VTB__(OutVTB, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, [Call:M1(act_term)|Rest], NextVTB).
type_check_do_call__(Call, CurrentVTB, NextVTB, 
                       InLE, OutLE, _Unif, Error) :-
    var(Call), !,
    NextVTB = CurrentVTB,
    OutLE = InLE,
    Error = new_var_call(do(Call)).
type_check_do_call__(Call, CurrentVTB, NextVTB, 
                       InLE, OutLE, Unif, Error) :-
    has_type__(Call, '!??'(rel_term), CurrentVTB, [], OutVTB, 
               InLE, OutLE, Unif, [], 1, call(Call), Error),
    merge_type_check_VTB__(OutVTB, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB).

    
%%type_check_allowed_call__(Call, Kind, CurrentVTB, NextVTB, InLE, OutLE, Unif, Error) :- errornl(type_check_allowed_call__(Call, Kind, CurrentVTB, NextVTB, InLE, OutLE, Unif, Error)),fail.
type_check_allowed_call__(Call, Kind, CurrentVTB, NextVTB, 
                          InLE, OutLE, Unif, Error) :-
    var(Call),
    in_unif__((Call = UCall), Unif),!,
    type_check_allowed_call__(UCall, Kind, CurrentVTB, NextVTB, 
                              InLE, OutLE, Unif, Error).
type_check_allowed_call__(Call, Kind, CurrentVTB, NextVTB, 
                          InLE, OutLE, _Unif, Error) :-
    var(Call), 
    type_remove_from__(Call:M(T), CurrentVTB, RestVTB), !,
    '$allowed2allowed_call_'(Kind, Call, ACall, Type),
    type_le_or_err__('!', Call, Type,  T, InLE, OutLE, 1, ACall, Error),
    (
      M = '!'
    ->
      M1 = '!'
    ;
      M1 = '!??'
    ),
    NextVTB = [Call:M1(Type)|RestVTB].
type_check_allowed_call__(Call, Kind, CurrentVTB, NextVTB, 
                          InLE, OutLE, _Unif, Error) :-
    var(Call), !,
    '$allowed2allowed_call_'(Kind, Call, ACall, _Type),
    NextVTB = CurrentVTB,
    OutLE = InLE,
    Error = new_var_allowed_call(ACall).
type_check_allowed_call__(Call, Kind, CurrentVTB, NextVTB, 
                          InLE, OutLE, Unif, Error) :-
    '$allowed2allowed_call_'(Kind, Call, ACall, Type),
    has_type__(Call, '!??'(Type), CurrentVTB, [], OutVTB, 
               InLE, OutLE, Unif, [], 1, ACall, Error),
    merge_type_check_VTB__(OutVTB, MergedVTB, Error),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB).
 

'$allowed2allowed_call_'(dyn, Call, allowed_dyn_call(Call), dyn_term).
'$allowed2allowed_call_'(rel, Call, allowed_rel_call(Call), rel_term).
'$allowed2allowed_call_'(act, Call, allowed_act_call(Call), act_term).


%% type_check_unif__((A = B), CurrVTB, NextVTB, InLE, OutLE,
%%                  InUnif, OutUnif, Unders, Error)
%% Type check the unification A = B. In doing so we typically update
%% VTB and the unification constraints
%%type_check_unif__(Call, CurrentVTB, NextVTB, InLE, OutLE, InUnif, OutUnif, Unders, Error) :- errornl(type_check_unif__(Call, CurrentVTB, NextVTB, InLE, OutLE, InUnif, OutUnif, Unders, Error)),fail.
% type_check_unif__(AeqB, CurrentVTB, NextVTB, InLE, OutLE,
%                   InUnif, OutUnif, _, Error) :-
%     collect_vars(AeqB, Vars),
%     member(X, Vars),
%     type_member_of__(X:MTA, CurrentVTB),
%     MTA == '@'(term),
%     !,
%     %% The unification contains a known variable of type @term - since
%     %% @term variables are not allowed to be instantiated we produce
%     %% an error (even in the case X = X for simplicity)
%     Error = unify_contains_at_term_var(X),
%     CurrentVTB = NextVTB,
%     OutLE = InLE,
%     InUnif = OutUnif.
%% A and B are function calls or list (set) comprehensions
type_check_unif__((A = B), CurrVTB, NextVTB, InLE, OutLE,
                  InUnif, InUnif, Unders, Error) :-
    '$function_call_or_comp'(A, CurrVTB, InUnif), 
    '$function_call_or_comp'(B, CurrVTB, InUnif), !,
    type_inference__(A, '!', AType, CurrVTB, [], _, InLE, MidLE,
                     InUnif, Unders, 0, A, Error),
    check_bind_type_constraints__(MidLE, Error),
    has_type__(B, '!'(AType), CurrVTB, [], _, MidLE, OutLE,
               InUnif, Unders, 0, B, Error),
    check_bind_type_constraints__(OutLE, Error),
    NextVTB = CurrVTB.
%% A is a function call or list (set) comprehension
type_check_unif__((A = B), CurrVTB, NextVTB, InLE, OutLE,
                  InUnif, InUnif, Unders, Error) :-
    '$function_call_or_comp'(A, CurrVTB, InUnif), !,
    type_inference__(A, '!', AType, CurrVTB, [], _, InLE, MidLE,
                     InUnif, Unders, 0, A, Error1),
    %% AType is the infered type of A
    check_bind_type_constraints__(MidLE, Error1),
    has_type__(B, '!'(AType), unif, CurrVTB, BVTB, MidLE, OutLE,
               InUnif, Unders, 0, B, Error1),
    %% B is type checked against ?AType
    check_bind_type_constraints__(OutLE, Error1),
    merge_head_VTB__(BVTB, NextVTB, Error1),
    head_error_to_unif_error__(Error1, Error).
%% B is a function call or list (set) comprehension - reverse of above
type_check_unif__((A = B), CurrVTB, NextVTB, InLE, OutLE,
                  InUnif, InUnif, Unders, Error) :-
    '$function_call_or_comp'(B, CurrVTB, InUnif), !,
    type_inference__(B, '!', BType, CurrVTB, [], _, InLE, MidLE,
                     InUnif, Unders, 0, B, Error1),
    check_bind_type_constraints__(MidLE, Error1),
    has_type__(A, '!'(BType), unif, CurrVTB, AVTB, MidLE, OutLE, 
               InUnif, Unders, 0, A, Error1),
    check_bind_type_constraints__(OutLE, Error1),
    merge_head_VTB__(AVTB, NextVTB, Error1),
    head_error_to_unif_error__(Error1, Error).
%% Unification always succeeds
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, InLE,
                  InUnif, OutUnif, _, _Error) :-
    A == B, !,
    CurrentVTB = NextVTB,
    InUnif = OutUnif.
%% Unification always fails
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, _, Error) :-
    '$atomic_or_string'(A),
    A \= B, !,
    Error = unify_always_fails(A, B),
    CurrentVTB = NextVTB,
    OutLE = InLE,
    InUnif = OutUnif.
%% Unification always fails 
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, _, Error) :-
    var(A),
    thaw_vars((A=B)),
    (A \= B), !,    
    Error = unify_always_fails(A, B),
    CurrentVTB = NextVTB,
    OutLE = InLE,
    InUnif = OutUnif.

%% Flip so var on LHS
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, Unders, Error) :-
    var(B), \+ var(A), !,
    type_check_unif__((B = A), CurrentVTB, NextVTB, InLE, OutLE,
                      InUnif, OutUnif, Unders, Error).
%% Below here if either side is a variable then the first arg is a variable
%% A is a variable and A = UA is in InUnify - replace A by UA
type_check_unif__((B = A), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, Unders, Error) :-
    var(A), in_unif__((A = UA), InUnif), !,
    type_check_unif__((B = UA), CurrentVTB, NextVTB, InLE, OutLE,
                      InUnif, OutUnif,  Unders, Err),
    (
      var(Err)
    ->
      true
    ;
      Error = unify_unify_err(A, UA, Err)
    ).
%% ditto for reverse
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, Unders, Error) :-
    var(A), in_unif__((A = UA), InUnif), !,
    type_check_unif__((UA = B), CurrentVTB, NextVTB, InLE, OutLE,
                      InUnif, OutUnif, Unders, Err),
    (
      var(Err)
    ->
      true
    ;
      Error = unify_unify_err(A, UA, Err)
    ).
%% Both are vars that have appeared before
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, _Unders, Error) :-
    var(A), var(B),
    type_member_of__(A:MTA, CurrentVTB),
    type_member_of__(B:MTB, CurrentVTB),
    push_modes_in__(MTA, PushedMTA),
    push_modes_in__(MTB, PushedMTB),
    !,
    %% XXXXXX
    check_and_merge_types__(PushedMTA, PushedMTB, MergedType, Err),
    (
      var(Err)
    ->
      diff_list(CurrentVTB, [A:MTA, B:MTB], MidVTB),
      NextVTB = [A:MergedType|MidVTB],
      OutUnif = [(B = A)|InUnif],
      OutLE = InLE
    ;
      Err = unify_conflict(_, _)
    ->
      Error = type_error(0, (A=B), A, PushedMTA, unify_conflict(PushedMTB), _)
    ;
      Error = Err
    ).
%%type_check_unif_vars__(A, PushedMTA, B, PushedMTB, CurrentVTB, NextVTB,
%%                       InLE, OutLE, InUnif, OutUnif, Unders, Error).
%% At least one new var - we forbid such unifications as they are pointless
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, _, Error) :-
    var(A), var(B),
    !,
    CurrentVTB = NextVTB,
    InUnif = OutUnif,
    OutLE = InLE, 
    Error = unify_new_var(A, B).
%% LHS is an existing var, RHS isn't a var
%% TODO what about either side containing ?term
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, Unders, Error) :-
    var(A), type_remove_from__(A:MTA, CurrentVTB, InVTB), !,
    %% type check B against MTA (the moded type of A) - this is done as
    %% though it was in the head of a rule as we are less strict about
    %% comparing types when the mode is not !
    has_type__(B, MTA, unif, InVTB, OutVTB, InLE, OutLE, InUnif,
               Unders, 0, B, Error0),
    check_bind_type_constraints__(OutLE, Error0),
    (
      var(Error0)
    ->
      %% no errors in the type check
      %% merge the VTB constraints as though the type test was in the head of
      %% a rule (see above comment)
      merge_head_VTB__(OutVTB, NextVTB, Error1),
      %% any errors produced will refer to the head of the rule - change this
      %% to a unification error
      head_error_to_unif_error__(Error1, Error)
    ;
      Error0 = type_error(I, Call, Term, Type, _, DecI)
    ->
      %% a type error was found - convert to a unification error
      Error = type_error(I, Call, Term, Type, unify_var_term_error, DecI)
    ;
      Error0 = arity_error(_,_)
    ->
      %% arity error - leave as is
      Error = Error0
    ;
      %% add unification error wrapper to any other error
      Error = unify_var_term_error(A, B, Error0)
    ),
    %% add to unification delays
    OutUnif = [(A = B)|InUnif].
%% LHS is a new var, RHS isn't a var
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, Unders, Error) :-
    var(A), !,
    NextVTB = CurrentVTB,
    has_type__(B, '?'(_T), CurrentVTB, [], _, InLE, OutLE, InUnif, Unders,
               0, B, Err),
    check_bind_type_constraints__(OutLE, Err),
    %% the above checks that B has some valid type
    (
      var(Err)
    ->
      %% add to unification delays
      OutUnif = [(A = B)|InUnif]
    ;
      Err = type_error(I, Call, Term, Type, not_a_term, DecI)
    ->
      Error = Err,
      OutUnif = InUnif
    ;
      Err = type_error(I, Call, Term, Type, _, DecI)
    ->
      Error = type_error(I, Call, Term, Type, unify_var_term_error, DecI),
      OutUnif = InUnif
    ;
      Err = arity_error(_,_)
    ->
      Error = Err
    ;
      Error = unify_var_term_error(A, B, Err),
      OutUnif = InUnif
    ).
%% Neither A nor B is a variable but can be unified but not identical so
%% contains vars - so unfold unification
type_check_unif__((A = B), CurrentVTB, NextVTB, InLE, OutLE,
                  InUnif, OutUnif, Unders, Error) :-
    '@..rev'(A, FA, ArgsA),
    '@..rev'(B, FB, ArgsB),
    type_check_unif_list__([FA|ArgsA], [FB|ArgsB], CurrentVTB, NextVTB,
                           InLE, OutLE, InUnif, OutUnif, Unders, Error).

%% type check pairwise unifications - called above
type_check_unif_list__(_, _, CurrentVTB, CurrentVTB,  InLE, InLE,
                       InUnif, InUnif,  _, Error) :-
    nonvar(Error), !.
type_check_unif_list__([], [], CurrentVTB, CurrentVTB,  InLE, InLE,
                     InUnif, InUnif,  _, _Error).
type_check_unif_list__([A|As], [B|Bs], CurrentVTB, NextVTB, InLE, OutLE,
                       InUnif, OutUnif, Unders, Error) :-
    type_check_unif__((A = B), CurrentVTB, MidVTB, InLE, MidLE,
                      InUnif, MidUnif, Unders, Error),
    type_check_unif_list__(As, Bs, MidVTB, NextVTB,  MidLE, OutLE,
                           MidUnif, OutUnif, Unders, Error).
    
%% type_check_unif_vars__(A,  MA(TA), B, MB(TB), CurrentVTB, NextVTB,
%%                       InLE, OutLE, InUnif, OutUnif, Unders, Error)
%% Check the unification of A = B with A : MA(TA), B: MB(TB)
%%type_check_unif_vars__(A, MTA, B, MTB, CurrentVTB, NextVTB, InLE, OutLE, InUnif, OutUnif, Unders, Error) :- errornl(type_check_unif_vars__(A, MTA, B, MTB, CurrentVTB, NextVTB, InLE, OutLE, InUnif, OutUnif, Unders, Error)),fail.    
type_check_unif_vars__(A,  MA(TA), B, MB(TB), CurrentVTB, NextVTB,
                       InLE, OutLE, InUnif, OutUnif, Unders, Error) :-
    MA \= '!', MB = '!', !,
    %% flip the unification problem if MB = !, MA \= !
    type_check_unif_vars__(B, MB(TB), A, MA(TA), CurrentVTB, NextVTB,
                           InLE, OutLE, InUnif, OutUnif, Unders, Error).
type_check_unif_vars__(A, MTA, B, MTB, CurrentVTB, NextVTB,
                       InLE, OutLE, InUnif, OutUnif, Unders, Error) :-
    errornl(oooo(A, MTA, B, MTB)),
    type_check_unif_vars_aux__(MTA, MTB, NewT, InLE, OutLE, Unders, Err),
    (
      nonvar(Err)
    ->
      Err = unify_conflict(TA, TB),
      Error = type_error(0, (A=B), A, TA, unify_conflict(TB), _)
    ;
      diff_list(CurrentVTB, [A:MTA, B:MTB], MidVTB),
      NextVTB = [A:NewT|MidVTB],
      OutUnif = [(B = A)|InUnif]
    ).


%% 
%%type_check_unif_vars_list__(MTA, MTB, NewMTA, InLE, OutLE, Unders, Error) :- errornl( type_check_unif_vars_list__(MTA, MTB, NewMTA, InLE, OutLE, Unders, Error)), fail.
type_check_unif_vars_list__(MTA, _MTB, NewMTA, InLE, OutLE, _, Error) :-
    nonvar(Error), !, NewMTA = MTA, OutLE = InLE.
type_check_unif_vars_list__([], [], [], InLE, OutLE, _, _Error) :- OutLE = InLE.
type_check_unif_vars_list__([MTA|MTAs], [MTB|MTBs], [NewMTA|NewMTAs],
                            InLE, OutLE, Unders, Error) :-
    type_check_unif_vars_aux__(MTA, MTB, NewMTA, InLE, MidLE, Unders, Error),
    type_check_unif_vars_list__(MTAs, MTBs, NewMTAs, MidLE, OutLE,
                                Unders, Error).

%%type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE,
%%                           Unders, Error)
%% Two variables that are unified have moded types MA(TA) and MB(TB)
%% check if the types are compatible and if so return NewMTA that is the
%% merged moded type that would result by unifying the 2 variables
%%type_check_unif_vars_aux__(MTA, MTB, NewMTA, InLE, OutLE, Unders, Error) :- errornl( type_check_unif_vars_aux__(MTA, MTB, NewMTA, InLE, OutLE, Unders, Error)), fail.
%% both modes \= ! so types must be equal
type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE, _, Error) :-
    %%XXX
    MA \= '!', MB \= '!', !,
    strip_modes__(MA(TA), InnerTA),
    strip_modes__(MB(TB), InnerTB),
    OutLE = InLE,
    (
      InnerTA = InnerTB
    ->
      %% the check of the unification of 2 vars with neither in ! mode should
      %% only succeed if the types are identical
      NewMTA = MA(TA)
    ;
      Error = unify_conflict(TA, TB)
    ).
%% Flip if required so first is !
type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE,
                           Unders, Error) :-
    MA \= '!', MB = '!', !,
    type_check_unif_vars_aux__(MB(TB), MA(TA), NewMTA, InLE, OutLE,
                               Unders, Error).
%% MA is ! from here
%% MB \= ! then error if not type_le
type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE,
                           _Unders, Error) :-
    MB \= '!', !,
    strip_modes__(MA(TA), InnerTA),
    strip_modes__(MB(TB), InnerTB),
    (
      type_le__(InnerTA, InnerTB, InLE, OutLE),
      check_bind_type_constraints__(OutLE)
    ->
      NewMTA = MA(TA)
    ;
      OutLE = InLE,
      Error = unify_conflict(InnerTA, InnerTB),
      NewMTA = MA(TA)
    ).
%% below both MA and MB are !
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% These 2 rules replace the following commented rules

type_check_unif_vars_aux__('!'(TA), '!'(TB), NewMTA, InLE, OutLE, _, _Error) :-
    strip_modes__('!'(TA), InnerTA),
    strip_modes__('!'(TB), InnerTB), 
    intersection_of__(InnerTA, InnerTB, Int), !,
    OutLE = InLE,
    push_modes_in__('!'(Int), NewMTA).
type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE, _, Error) :-
    OutLE = InLE,
    strip_modes__(MA(TA), InnerTA),
    strip_modes__(MB(TB), InnerTB), 
    Error = unify_conflict(InnerTA, InnerTB),
    NewMTA = MA(TA).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% If the types are not compatible then error
% type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE, _, Error) :-
%     strip_modes__(MA(TA), InnerTA),
%     strip_modes__(MB(TB), InnerTB),
%     \+ is_compatible__(InnerTA, InnerTB), !,
%     OutLE = InLE,
%     Error = unify_conflict(InnerTA, InnerTB),
%     NewMTA = MA(TA).
% type_check_unif_vars_aux__('!'(TA), '!'(TB), NewMTA, InLE, OutLE, _, _Error) :-
%     %ground(TA), ground(TB),
%     is_compatible__(TA, TB), !,
%     strip_modes__('!'(TA), InnerTA),
%     strip_modes__('!'(TB), InnerTB), 
%     intersection_of__(InnerTA, InnerTB, Int),
%     OutLE = InLE,
%     push_modes_in__('!'(Int), NewMTA).
% %% below here types must be compatible
% %% neither constructors so use smaller (both !)
% type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE, _, _Error) :-
%     \+ constructor_type__(TA, _, _),
%     \+ constructor_type__(TB, _, _),
%     strip_modes__(MA(TA), InnerTA),
%     strip_modes__(MB(TB), InnerTB), !,
%     (
%       type_le__(InnerTA, InnerTB, InLE, OutLE),
%       check_bind_type_constraints__(OutLE)
%     ->
%       NewMTA = MA(TA)
%     ;
%       NewMTA = MB(TB)
%     ),
%     OutLE = InLE.
% %% one is a constructor type and the other is a macro - unfold macro
% type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE, _, _Error) :-
%     constructor_type__(TA, _, _),
%     '$type_info'(_,macro_type,TB,MTB,_), !,
%     push_modes_in__(MB(MTB), PTB),
%     type_check_unif_vars_aux__(MA(TA), PTB, NewMTA, InLE, OutLE, _, _Error).
% type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE, _, _Error) :-
%     constructor_type__(TB, _, _),
%     '$type_info'(_,macro_type,TA,MTA,_), !,
%     push_modes_in__(MA(MTA), PTA),
%     type_check_unif_vars_aux__(PTA, MB(TB), NewMTA, InLE, OutLE, _, _Error).
% %% TA is a constructor but TB isn't so TB must be term
% %% MB is not ! so no change
% % type_check_unif_vars_aux__(MA(TA), MB(TB), NewMTA, InLE, OutLE, _, _Error) :-
% %     constructor_type__(TA, _, _),
% %     \+ constructor_type__(TB, _, _),
% %     MB \= '!', !,
% %     OutLE = InLE,
% %     NewMTA = MA(TA).
% %% TA is a constructor but TB isn't so TB must be term
% %% MB is ! so make TA ground
% type_check_unif_vars_aux__(MA(TA), _MB(TB), NewMTA, InLE, OutLE, _, _Error) :-
%     constructor_type__(TA, _, _),
%     \+ constructor_type__(TB, _, _), !,
%     OutLE = InLE,
%     strip_modes__(MA(TA), InnerTA),
%     push_modes_in__('!'(InnerTA), NewMTA).
% %% TB is a constructor but TA isn't so TA must be term
% %% and MA is ! so make TB ground
% type_check_unif_vars_aux__(_MA(TA), MB(TB), NewMTA, InLE, OutLE, _, _Error) :-
%     \+ constructor_type__(TA, _, _),
%     constructor_type__(TB, _, _), !,
%     OutLE = InLE,
%     strip_modes__(MB(TB), InnerTB),
%     push_modes_in__('!'(InnerTB), NewMTA).
% %% Both are the same constructor type
% type_check_unif_vars_aux__(_MA(TA), _MB(TB), NewMTA, InLE, OutLE, Unders, Error) :-
%     TA =.. [F|TAArgs],
%     TB =.. [F|TBArgs],
%     type_check_unif_vars_list__(TAArgs, TBArgs, NewTAArgs, InLE, OutLE,
%                                 Unders, Error),
%     NewTA =.. [F|NewTAArgs],
%     NewMTA = '!'(NewTA).

%%
%% Assume both types have all their modes explicit
%%
%%check_and_merge_types__(MT1, MT2, MergedType, Error) :- errornl(check_and_merge_types__(MT1, MT2, MergedType, Error)),fail.

%% an error has occurred - terminate
check_and_merge_types__(_MT1, _MT2, _MergedType, Error) :-
    nonvar(Error), !.
check_and_merge_types__(M1(term), M2(term), ModedMergedType, _Error) :-
    M1 \= '!', M2 \= '!', !,
    min_mode__(M1, M2, MinM),
    ModedMergedType = MinM(term).
check_and_merge_types__(_M(term), '!'(T2), ModedMergedType, _Error) :-
    \+ constructor_type__(T2, _, _), !,
    ModedMergedType = '!'(T2).
check_and_merge_types__('!'(T1), _M(term), ModedMergedType, _Error) :-
    \+ constructor_type__(T1, _, _), !,
    ModedMergedType = '!'(T1).
%% if the types are equal then we get success with merged modes
check_and_merge_types__(MT1, MT2, MergedType, _Error) :-
    strip_modes__(MT1, T1),
    strip_modes__(MT2, T2),
    \+ contains_term_type__(T1),
    \+ contains_term_type__(T2),
    T1 = T2, !,
    merge_modes__(MT1,  MT2, MergedType).
%% unfold macros
check_and_merge_types__(M1(T1), M2(T2), ModedMergedType, Error) :-
    '$type_info'(_, macro_type, T2, MacroT2, _), !,
    push_modes_in__(M2(MacroT2), PT2),
    check_and_merge_types__(M1(T1), PT2, ModedMergedType, Error).
check_and_merge_types__(M1(T1), M2(T2), ModedMergedType, Error) :-
    '$type_info'(_, macro_type, T1, MacroT1, _), !,
    push_modes_in__(M1(MacroT1), PT1),
    check_and_merge_types__(M2(T2), PT1, ModedMergedType, Error).
%% if the types are not compound and not compatible then failure
check_and_merge_types__(MT1, MT2, _MergedType, Error) :-
    strip_modes__(MT1, T1),
    \+ constructor_type__(T1, _, _),
    strip_modes__(MT2, T2),
    \+ contains_term_type__(T1),
    \+ contains_term_type__(T2),
    \+ is_compatible__(T1, T2), !,
    Error = unify_conflict(MT1, MT2).
%% no inner ! modes and so types have to be equal - covered by rule 3 above
%% so this rule should yield failure
check_and_merge_types__(MT1, MT2, _MergedType, Error) :-
    \+has_inner_bang_aux__(MT1),
    \+has_inner_bang_aux__(MT2),
    strip_modes__(MT1, T1),
    \+ contains_term_type__(T1),
    strip_modes__(MT2, T2),
    \+ contains_term_type__(T2),
    !,
    Error = unify_conflict(MT1, MT2).
%% base case - neither types are constructor types
check_and_merge_types__(M1(T1), M2(T2), MergedType, Error) :-
    \+ constructor_type__(T1, _, _),
    \+ constructor_type__(T2, _, _), !,
    (
      is_compatible__(T1, T2), M1 = '!', M2 = '!'
    ->
      strip_modes__('!'(T1), InnerT1),
      strip_modes__('!'(T2), InnerT2), 
      intersection_of__(InnerT1, InnerT2, Int),
      push_modes_in__('!'(Int),  MergedType)
    ;
      M1 = '!', type_le__(T1, T2, [], _)
    ->
      MergedType = M1(T1)
    ;
      M2 = '!', type_le__(T2, T1, [], _)
    ->
      MergedType = M2(T2)
    ;
      %% rule 3 deals with M1 \= '!', M2 \= '!' so failure
      Error = unify_conflict(M1(T1), M2(T2))
    ).
%% both constructor types
check_and_merge_types__(M1(T1), M2(T2), ModedMergedType, Error) :-
    constructor_type__(T1, Constr, T1Args),
    constructor_type__(T2, _, T2Args),
    !,
    %% The constructors have to be the same  with the same number
    %% of args (rule 2 above)
    %% macros have been unfolded by earlier rules
    check_and_merge_types_list__(T1Args, T2Args, MergedArgs, Error),
    MergedType =.. [Constr|MergedArgs],
    min_mode__(M1, M2, MinM),
    ModedMergedType = MinM(MergedType).
check_and_merge_types__('!'(term), _M(term), ModedMergedType, _Error) :-
    !,
    ModedMergedType = '!'(term).
check_and_merge_types__(_M(term), '!'(term), ModedMergedType, _Error) :-
    !,
    ModedMergedType = '!'(term).
check_and_merge_types__(M(term), '!'(T2), ModedMergedType, Error) :-
    constructor_type__(T2, Constr, T2Args), !,
    length(T2Args, N),
    length(T1Args, N),
    '$unify_all_types'(T1Args, M(term)),
    check_and_merge_types_list__(T1Args, T2Args, MergedArgs, Error),
    MergedType =.. [Constr|MergedArgs],
    ModedMergedType = '!'(MergedType).
check_and_merge_types__('!'(T1), M(term), ModedMergedType, Error) :-
    constructor_type__(T1, Constr, T1Args), !,
    length(T1Args, N),
    length(T2Args, N),
    '$unify_all_types'(T2Args, M(term)),
    check_and_merge_types_list__(T1Args, T2Args, MergedArgs, Error),
    MergedType =.. [Constr|MergedArgs],
    ModedMergedType = '!'(MergedType).
check_and_merge_types__(M1(term), M2(term), ModedMergedType, Error) :-
    M1 \= '!', M2 \= '!', !,
    ModedMergedType = M1(term),
    Error = unify_conflict(M1(term), M2(term)).
    
%% catchall - failure
check_and_merge_types__(MT1, MT2, _MergedType, Error) :-
    Error = unify_conflict(MT1, MT2).

check_and_merge_types_list__(T1Args, _T2Args, T1Args, Error) :-
    nonvar(Error), !.
check_and_merge_types_list__([], [], [], _Error).
check_and_merge_types_list__([TA1|T1Args], [TA2|T2Args],
                             [Merged|MergedArgs], Error) :-
    check_and_merge_types__(TA1, TA2, Merged, Error),
    check_and_merge_types_list__(T1Args, T2Args, MergedArgs, Error).

%% the types without modes are equal
%%merge_modes__(A, B, C) :- errornl(merge_modes__(A, B, C)),fail.
merge_modes__(M1(T1),  M2(T2), Min(T1)) :-
    \+ constructor_type__(T1, _, _),
    \+ constructor_type__(T2, _, _), !,
    min_mode__(M1, M2, Min).
merge_modes__(M1(T1),  M2(T2), Min(MergedType)) :-
    constructor_type__(T1, Constr, T1Args),
    constructor_type__(T2, _, T2Args),
    min_mode__(M1, M2, Min),
    merge_modes_list__(T1Args, T2Args, MergedArgs),
    MergedType =.. [Constr|MergedArgs].

merge_modes_list__([], [], []). 
merge_modes_list__([T1|T1Args], [T2|T2Args], [Merged|MergedArgs]) :-
    merge_modes__(T1, T2, Merged),
    merge_modes_list__(T1Args, T2Args, MergedArgs).    

contains_term_type__(term).
contains_term_type__(T) :-
    constructor_type__(T, _, TArgs),
    member(T1, TArgs),
    contains_term_type__(T1), !.
    

%% type_check_type_call__(Type(Term, MT), CurrentVTB, NextVTB, Unif, Error)
%% Type(Term, MT) is either type(Term, MT) or isa(Term, MT)
%% This checks if this call is consistent with CurrentVTB and if it is
%% update it to NextVTB
%%type_check_type_call__(A,B,C,D,E) :- errornl(type_check_type_call__(A,B,C,D,E)),fail.
type_check_type_call__(Type(Term, MT), CurrentVTB, NextVTB, Unif, Error) :-
    compound(Term), Term = F(Terms), F == '$tuple', !,
    %% Term is a tuple - check MT is a typle type of the correct arity
    %% normally we insist Term is a vaiable but we allow this one exception as
    %% it allows us to do multiple tests in one go.
    length(Terms, N),
    (
      MT = '$tuple_type'(Types),
      length(Types, N)
    ->
      %% pairwise check type elements against tuple type elements
      type_check_type_call_aux__(Terms, Types, CurrentVTB, NextVTB, Unif, Error)
    ;
      Error = non_matching_tuples_type_test(Type(Term, MT))
    ).
type_check_type_call__(_Type(Term, MT), CurrentVTB, NextVTB, _, _Error) :-
    type_remove_from__(Term:TermType, CurrentVTB, VTB),
    (MT = '!'(_) ; TermType = '!'(_)),
    strip_modes__(TermType, InnerTermType),
    strip_modes__(MT, InnerMT),
    is_compatible__(InnerTermType,  InnerMT),
    !,
    intersection_of__(InnerTermType,  InnerMT, Int),
    push_modes_in__('!'(Int), MInt),
    NextVTB = [(Term:MInt)|VTB].
type_check_type_call__(Type(Term, MT), CurrentVTB, NextVTB, _, Error) :-
    nonvar(Term), !,
    %% error - only use dto type check variables
    NextVTB = CurrentVTB,
    Error = non_var_type_test(Type(Term, MT)).
type_check_type_call__(Type(Term, MT), CurrentVTB, NextVTB, Unif, Error) :-
    in_unif__((Term = UTerm), Unif), !,
    %% replace unification problem
    type_check_type_call__(Type(UTerm, MT), CurrentVTB, NextVTB, Unif, Error).
type_check_type_call__(TypeF(Term, MT), CurrentVTB, NextVTB, _, Error) :-
    type_member_of__(Term:TermType, CurrentVTB),
    strip_modes__(TermType, InnerTermType),
    strip_modes__(MT, InnerMT),
    \+ type_le__(InnerMT, InnerTermType), !,
    %% MT is not a restriction of the  given type of Term
    Error = not_le_type_test(TypeF(Term, MT), TermType),
    NextVTB = CurrentVTB.
type_check_type_call__(isa(Term, T), CurrentVTB, NextVTB, _, _Error) :-
    type_remove_from__(Term:TermType, CurrentVTB, VTB), !,
    %% extract the constraint of Term (a var) in CurrentVTB
    type_check_type_call_type__('!'(T), TermType, MergedType),
    %% MergedType is the merge of the two types (really a restriction)
    NextVTB = [(Term:MergedType)|VTB].
type_check_type_call__(isa(Term, T), CurrentVTB, NextVTB, _, _Error) :-
    !,
    NextVTB = [(Term:'!'(T))|CurrentVTB].
type_check_type_call__(_Type(Term, MT), CurrentVTB, NextVTB, _, _Error) :-
    type_remove_from__(Term:TermType, CurrentVTB, VTB), !,
    type_check_type_call_type__(MT, TermType, MergedType),
    %% Ditto of above for type test
    NextVTB = [(Term:MergedType)|VTB].
type_check_type_call__(Type(Term, MT), CurrentVTB, NextVTB, _, Error) :-
    %% a new variable - an error as this test makes no sense
    NextVTB = CurrentVTB,
    Error = new_var_type_test(Type(Term, MT)).

%% type_check_type_call_type__(TestType, VarType, OutType)
%% VarType is the type of the variable being tested, TestType is the
%% type in the type check call and OutType will be instantiated
%% to Type for the variable after the type call.
%% TestType is type_le VarType
%%type_check_type_call_type__(A,B,C) :- errornl(type_check_type_call_type__(A,B,C)),fail.
type_check_type_call_type__(T, T2, MT) :-
    \+ moded__(T) , !,
    %% add ! mode if none given
    type_check_type_call_type__('!'(T), T2, MT).
% type_check_type_call_type__(M(_), MT, MT) :-
%     %% If the test type mode is not ! then we return the vars type
%     %% IE the type test call has no effect
                                %     M \= '!', !.
type_check_type_call_type__(M1(T1),M2(_), M3(T1)) :-
    %% If we don't have a constructor type as the test type then
    %% we output that type
    \+ constructor_type__(T1, _, _), !,
    min_mode__(M1, M2, M3).
type_check_type_call_type__(M1(T1), M2(T2), MT) :-
    constructor_type__(T1, C, SubT1),
    constructor_type__(T2, C, SubT2),
    !,
    %% If both are constructors then we recursively process teh arg types.
    type_check_type_call_type_aux__(SubT1, SubT2, SubMerged),
    MTT =.. [C|SubMerged],
    min_mode__(M1, M2, M3),
    MT = M3(MTT).
%% the only other possibility is the second type is term and the first is a
%% constructor type - in this case we replace term by the constructor type
%% all of whose args are term and recursively call the check
type_check_type_call_type__(M1(T1), M2(term), MT) :-
    constructor_type__(T1, C, SubT1),
    length(SubT1, N),
    findall(M2(term), between(1, N, _), SubT2),
    MTT =.. [C|SubT2],
    type_check_type_call_type__(M1(T1), M2(MTT), MT).

%type_check_type_call_type_aux__(SubT1, SubT2, SubMerged) :- errornl(type_check_type_call_type_aux__(SubT1, SubT2, SubMerged)),fail.
type_check_type_call_type_aux__([], [], []).    
type_check_type_call_type_aux__([T1|SubT1], [T2|SubT2], [Merged|SubMerged]) :-
    type_check_type_call_type__(T1, T2, Merged),
    type_check_type_call_type_aux__(SubT1, SubT2, SubMerged).

type_check_type_call_aux__(_Terms, _Types, CurrentVTB, NextVTB, _, Error) :-
    nonvar(Error), !,
    CurrentVTB = NextVTB.
type_check_type_call_aux__([], [], InVTB, InVTB, _, _Error).
type_check_type_call_aux__([Term|Terms], [MT|Types], InVTB, OutVTB,
                           Unif, Error) :-
    type_check_type_call__(type(Term, MT), InVTB, MidVTB, Unif, Error),
    type_check_type_call_aux__(Terms, Types, MidVTB, OutVTB, Unif, Error).

%%type_check_call_aux__(AllTypes, Call, Args, CurrentVTB, NextVTB, Unders,
%%                          InLE, OutLE, InUnif, Error)
%% Called from type_check_call__ where Call is a "non-special" call.
%% AllTypes is a list of type lists - each inner list represents a possible
%% list of moded types for the args of Call and Args are the args of Call
%% The outer list is the list of all possible types for Call
%%type_check_call_aux__(A,B,C,D,E,F,G,H,I,J) :- errornl(type_check_call_aux__(A,B,C,D,E,F,G,H,I,J)),fail.
type_check_call_aux__(_AllTypes, _Call, _Args, _CurrentVTB, _NextVTB, _Unders,
                      _, _, _Unify, Error) :-
    %% An error has occurred - finish returning Error
    nonvar(Error), !.
type_check_call_aux__([Types], Call, Args, CurrentVTB, NextVTB, Unders,
                      InLE, OutLE, Unify, Error) :-
    !,
    %% There is only possible collection of types for call
    once((
          has_type_term_index_iterate__(Types, Args, 1, Call, CurrentVTB, [],
                                        CallVTB, InLE, OutLE, Unify, Unders,
                                        Error),
          %% if Error is a var then Args type check, OutLE is the update
          %% type_le constraints and Call VTB is the set of variable type
          %% bindings generated by the test
          check_bind_type_constraints__(OutLE, Error),
          %% Check if OutLE is solvable
          merge_type_check_VTB__(CallVTB, MergedVTB, Error),
          %% there may be repeated VTB's for a variable - merge them into a
          %% single constraint and then merge those constraints into
          %% CurrentVTB to get NextVTB (for the next call)
          merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB)
         )).
%% Multiple declarations
%% As an example to explain what is going on below consider the call
%% append([1], [2], [1,a])
%% If we look at the declaration append(!list(T), !list(T), ?list(T))
%% and just look at the ! moded args then we get the LE constraint nat le T
%% Now we solve for T to get T = nat. When we type check the third arg
%% we get a type error.
%% Similarly for the call
%% append([1], [a], [1,2])
%% and we look at the declaration  append(?list(T), ?list(T), !list(T))
%% and process the third arg (in ! mode) we get T = nat which gives a type
%% error for the 2nd arg.
%% So the approach is to divide each type in AllTypes into those with an
%% inner ! mode and the others. For each declaration that has at least one
%% inner ! mode we first type check those and solve for polymorphic variables.
%% If that succeeds we then type check the rest of the args -
%% if an error occurs then the call has a type error.
%% If there are no inner ! moded types or all of those ! moded type checks
%% fail then we look to see if any declarations match - if so then we succeed
%% If none succeed then we fail the type check
type_check_call_aux__(_, Call, Args, CurrentVTB, _NextVTB, Unders,
                      _InLE, _OutLE, Unify, Error) :-
    %% before doing the "real" type check we first check that at least all
    %% the terms have a valid type
    ip_set('$call_kind', rel),
    '$check_valid_terms'(Args, 1, '?', Call, CurrentVTB, Unify, Unders, Error),
    nonvar(Error), !.
type_check_call_aux__(AllTypes, Call, Args, CurrentVTB, NextVTB, Unders,
                      InLE, OutLE, Unify, Error) :-
    split_on_inner_bangs__(AllTypes, 1, Bangs, NonBangs),
    assert(checking_inner_bangs__),
    %% We assert checking_inner_bangs__ (and then retract after the
    %% following test) so that the type checker knows we
    %% are in this special case of type checking
    type_check_call_part_bangs__(Bangs, Args, Call, CurrentVTB, _, Unders,
                               InLE, Unify, OKBangs),
    retract(checking_inner_bangs__),
    (
      OKBangs = [], NonBangs = []
    ->
      %% no ! moded check succeeded and there were no declarations
      %% with non inner ! modes
      Error = type_error(0, Call, Call, no_suitable_moded_type, multidecl, _)
    ;
      OKBangs = []
    ->
      %% no ! moded check succeeded so check declarations with only
      %% non-inner ! moded types
      type_check_call_part_aux__(NonBangs, Call, Args, CurrentVTB,
                                 NextVTB, Unders, InLE, OutLE, Unify, Error),
      check_bind_type_constraints__(OutLE, Error)
    ;
      %% there is at least one that type checks for inner ! moded args
      %% find the best types
      type_check_call_bangs_list__(OKBangs, Call, Args, CurrentVTB, NextVTB,
                                   Unders, InLE, OutLE, Unify, Error)
    ).
 

type_check_call_part_bangs__(Bangs, Args, Call, CurrentVTB, _, Unders,
                             InLE, Unify, OKBangs) :-
    %% find all the type declarations with at least one inner ! mode that
    %% correctly type check
    findall(TNT,
            type_check_call_part_bangs_aux__(TNT, Bangs, Args, Call, CurrentVTB,
                                             Unders, InLE, Unify),
            OKBangs).

type_check_call_part_bangs_aux__(TNT, Bangs, Args, Call, CurrentVTB, Unders,
                                 InLE, Unify) :-
    member(TNT, Bangs),
    TNT = t(_, Types),
    has_type_term_iterate__(Types, Args, CurrentVTB, [], _, InLE, OutLE,
                            Unify, Unders, 1, Call, Error),
    check_bind_type_constraints__(OutLE, Error),
    var(Error).

type_check_call_bangs_list__(Bangs, Call, Args, CurrentVTB, NextVTB,
                             Unders, InLE, OutLE, Unify, Error) :-
    type_check_call_aux_list__(Bangs, Call, Args, CurrentVTB, AllCallVTB,
                               Unders, InLE, AllLE, Unify, Unders, AllError),
    (
      member(Error, AllError), nonvar(Error)
    ->
      true
    ;
      %% all declarations pass the type check
      %%get_all_type_correct__(AllError, AllCallVTB, AllOKVTB, AllLE, AllOKLE),
      map(sort, AllCallVTB, SortedVTB),
      sort(SortedVTB, [OutVTB|_], '$vtb_order'),
      %% OutVTB will be the VTB with the smallest moded types for the variables
      %% and hence the best choice for the variable binding constraints
      extract_LE_from_VTB__(SortedVTB, AllLE, OutVTB, OutLE),
      %% OutLE is the list of type_le constraints associated with OutVTB
      merge_type_check_VTB__(OutVTB, MergedVTB, Error),
      merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB)
    ).


type_check_call_part_aux__([], _Call, _Args, CurrentVTB,
                           CurrentVTB, _Unders, InLE, InLE, _Unify, _Error) :-
    !.
type_check_call_part_aux__(AllTypes, Call, Args, CurrentVTB,
                           NextVTB, Unders, InLE, OutLE, Unify, Error) :-
    type_check_call_aux_list__(AllTypes, Call, Args, CurrentVTB, AllCallVTB,
                               Unders, InLE, AllLE, Unify, Unders, AllError),
    %% type check with AllTypes
    get_all_type_correct__(AllError, AllCallVTB, AllOKVTB, AllLE, AllOKLE),
    (
      AllOKVTB = [], AllError = [E], nonvar(E)
    ->
      %% Only one error found - return that error
      Error = E
    ;
      AllOKVTB = []
    ->
      %% multiple errors
      Error = type_error(0, Call, Call, no_suitable_moded_type, multidecl, _)
    ;
      %% at least one success - find best VTB (As above)
      map(sort, AllOKVTB, SortedVTB),
      sort(SortedVTB, [OutVTB|_], '$vtb_order'),
      extract_LE_from_VTB__(SortedVTB, AllOKLE, OutVTB, OutLE),
      merge_type_check_VTB__(OutVTB, MergedVTB, Error),
      merge_VTB_currentVTB__(MergedVTB, CurrentVTB, NextVTB)
    ), !.

extract_LE_from_VTB__([VTB|_SortedVTB], [OutLE|_AllOKLE], OutVTB, OutLE) :-
    VTB == OutVTB, !.
extract_LE_from_VTB__([_|SortedVTB], [_|AllOKLE], OutVTB, OutLE) :-
    extract_LE_from_VTB__(SortedVTB, AllOKLE, OutVTB, OutLE).

%%type_check_call_aux_list__(A, B, C, D, E, F, H, I, J, K, L) :- errornl(type_check_call_aux_list__(A, B, C, D, E, F, H, I, J, K, L)),fail.
type_check_call_aux_list__([], _Call, _Args, _CurrentVTB, [],
                           _Unders, _, [], _Unify, _Unders, []).
type_check_call_aux_list__([t(DecIndex, Types)|AllTypes], Call, Args,
                           CurrentVTB, [CallVTB|AllVTB], Unders, InLE,
                           [OutLE|AllLE], Unify, Unders, [Error|AllErrors]) :-
    ip_lookup('$act_seen', Seen),
    has_type_term_iterate__(Types, Args, CurrentVTB, [], CallVTB, InLE, OutLE,
                            Unify, Unders, 1, Call, Error1),
    check_bind_type_constraints__(OutLE, Error1),
    (
      var(Error1)
    ->
      true
    ;
      Error1 = type_error(I, PTerm, Term, T1, R, _)
    ->
      Error = type_error(I, PTerm, Term, T1, R, DecIndex)
    ;
      Error = Error1
    ),
    ip_set('$act_seen', Seen),
    type_check_call_aux_list__(AllTypes, Call, Args, 
                               CurrentVTB, AllVTB, Unders, InLE, AllLE,
                               Unify, Unders, AllErrors).

%% type_check_string_concat__(StringTerm, Call, CurrentVTB, NextVTB, Unders,
%%                           InLE, OutLE, InUnif, OutUnif, Error)
%% type check StringTerm (from the RHS of =?) 
%%type_check_string_concat__(A, Call, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif, OutUnif, Error) :- errornl(type_check_string_concat__(A, Call, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif, OutUnif, Error)),fail.
type_check_string_concat__(A, _Call, CurrentVTB, NextVTB, Unders,
                           InLE, OutLE, InUnif, OutUnif, Error) :-
    extract_concat_args__(A, AArgs),
    %%AArgs is the list of args to a sequence of ++ applications
    type_check_string_concat_aux__(AArgs, CurrentVTB, NextVTB, InLE, OutLE,
                                   InUnif, OutUnif, Unders, 1, A, Error).

%%type_check_string_concat_aux__(Args, CurrentVTB, NextVTB, InLE, OutLE, InUnif, OutUnif, Unders, N, A, Err) :- errornl(type_check_string_concat_aux__(Args, CurrentVTB, NextVTB, InLE, OutLE, InUnif, OutUnif, Unders, N, A, Err)),fail.
type_check_string_concat_aux__(_, CurrentVTB, CurrentVTB, InLE, InLE,
                               InUnif, InUnif, _Unders, _, _, Err) :-
    nonvar(Err), !.
type_check_string_concat_aux__([], CurrentVTB, CurrentVTB, InLE, InLE, 
                               InUnif, InUnif, _Unders, _, _, _Err).
type_check_string_concat_aux__([A|Args], CurrentVTB, NextVTB, InLE, OutLE,
                               InUnif, OutUnif, Unders, N, Parent, Err) :-
    compound(A), A = (RETest :: Pred),
    compound(RETest), RETest = '$re'(T,RE), !,
    %% A is of the form T/RE :: Pred
    %% RE is a string representing a regular expression
    has_type__(RE, '!'(string), CurrentVTB, [], _OutVTB, InLE, MidLE,
                    InUnif, Unders, 2, RETest, Err), 
    check_bind_type_constraints__(MidLE, Err),
    %% T is a variable of type ?string
    has_type__(T, '?'(string), CurrentVTB, [], OutVTB, MidLE, Mid2LE,
               InUnif, Unders, 1, RETest, Err),
    check_bind_type_constraints__(Mid2LE, Err),
    merge_type_check_VTB__(OutVTB, MergedVTB, Err),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, Next0VTB),
    %% Pred is a type correct sequence of relation calls
    '$goals2list'(Pred, PredList),
    type_check_body__(PredList, rel, Next0VTB, MidCurrVTB, Unders, Mid2LE,
                      Mid3LE, InUnif, MidUnif, Err, inner),
    N1 is N+1,
    type_check_string_concat_aux__(Args, MidCurrVTB, NextVTB, Mid3LE, OutLE,
                                   MidUnif, OutUnif, Unders, N1, Parent, Err).
type_check_string_concat_aux__([A|Args], CurrentVTB, NextVTB, InLE, OutLE,
                               InUnif, OutUnif, Unders, N, Parent, Err) :-
    compound(A), A = '$re'(T,RE), !,
    %% A is of the form T/RE
    has_type__(RE, '!'(string), CurrentVTB, [], _OutVTB, InLE, MidLE,
                    InUnif, Unders, 2, A, Err), 
    check_bind_type_constraints__(MidLE, Err),
    has_type__(T, '?'(string), CurrentVTB, [], OutVTB, MidLE, Mid2LE,
               InUnif, Unders, 1, A, Err),
    check_bind_type_constraints__(Mid2LE, Err),
    merge_type_check_VTB__(OutVTB, MergedVTB, Err),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, Next0VTB),
    N1 is N+1,
    type_check_string_concat_aux__(Args, Next0VTB, NextVTB, Mid2LE, OutLE,
                                   InUnif, OutUnif, Unders, N1, Parent, Err).
type_check_string_concat_aux__([A|Args], CurrentVTB, NextVTB, InLE, OutLE,
                               InUnif, OutUnif, Unders, N, Parent, Err) :-
    compound(A), A = (T :: Pred), !,
    %% A is of the form T :: Pred
    has_type__(T, '?'(string), CurrentVTB, [], OutVTB, InLE, MidLE,
               InUnif, Unders, 1, A, Err),
    check_bind_type_constraints__(MidLE, Err),
    merge_type_check_VTB__(OutVTB, MergedVTB, Err),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, Next0VTB),
    '$goals2list'(Pred, PredList),
    type_check_body__(PredList, rel, Next0VTB, MidCurrVTB, Unders,
                      MidLE, MidLE2, InUnif, MidUnif, Err, inner),
    N1 is N+1,
    type_check_string_concat_aux__(Args, MidCurrVTB, NextVTB, MidLE2, OutLE,
                                   MidUnif, OutUnif, Unders, N1, Parent, Err).
type_check_string_concat_aux__([A|Args], CurrentVTB, NextVTB, InLE, OutLE, 
                               InUnif, OutUnif, Unders, N, Parent, Err) :-
    %% A is just a variable
    has_type__(A, '?'(string), CurrentVTB, [], OutVTB, InLE, MidLE,
               InUnif, Unders, N, Parent, Err),
    check_bind_type_constraints__(MidLE, Err),
    merge_type_check_VTB__(OutVTB, MergedVTB, Err),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, Next0VTB),
    N1 is N+1,
    type_check_string_concat_aux__(Args, Next0VTB, NextVTB, MidLE, OutLE, 
                                   InUnif, OutUnif, Unders, N1, Parent, Err).

%% type_check_list_concat__(ListTerm, Call, CurrentVTB, NextVTB, Unders,
%%                           InLE, OutLE, InUnif, OutUnif, Error)
%% type check ListTerm (from the RHS of =?)
%%type_check_list_concat__(A, ListType, Call, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif, OutUnif, Error) :- errornl(type_check_list_concat__(A, ListType, Call, CurrentVTB, NextVTB, Unders, InLE, OutLE, InUnif, OutUnif, Error)),fail.
type_check_list_concat__(A, ListType, _Call, CurrentVTB, NextVTB, Unders,
                         InLE, OutLE, InUnif, OutUnif, Error) :-
    extract_append_args__(A, AArgs),
    %%AArgs is the list of args to a sequence of <> applications
    type_check_list_concat_aux__(AArgs, ListType, CurrentVTB, NextVTB,InLE,
                                 OutLE, InUnif, OutUnif, Unders, 1, A, Error).

%%type_check_list_concat_aux__(AArgs, ListType, CurrentVTB, NextVTB, InLE, OutLE, InUnif, OutUnif, Unders, N, A, Err) :- errornl(type_check_list_concat_aux__(AArgs, ListType, CurrentVTB, NextVTB, InLE, OutLE, InUnif, OutUnif, Unders, N, A, Err)),fail.

type_check_list_concat_aux__(_AArgs, _ListType, CurrentVTB, CurrentVTB,
                             InLE, InLE, InUnif, InUnif, _Unders, _, _, Err) :-
    nonvar(Err), !.
type_check_list_concat_aux__([], _ListType, CurrentVTB, CurrentVTB,
                             InLE, InLE, InUnif, InUnif, _Unders, _, _, _Err).
type_check_list_concat_aux__([A|Args], ListType, CurrentVTB, NextVTB, InLE,
                             OutLE, InUnif, OutUnif, Unders, N, Parent, Err) :-
    compound(A), A = (T :: Pred), !,
    %% A is of the form T :: Pred
    push_modes_in__('?'(list(ListType)), MT),
    has_type__(T, MT, CurrentVTB, [], OutVTB, InLE, MidLE,
               InUnif, Unders, 1, A, Err),
    check_bind_type_constraints__(MidLE, Err),
    merge_type_check_VTB__(OutVTB, MergedVTB, Err),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, Next0VTB),
    '$goals2list'(Pred, PredList),
    type_check_body__(PredList, rel, Next0VTB, MidCurrVTB, Unders,
                      MidLE, Mid2LE, InUnif, MidUnif, Err, inner),
    N1 is N+1,
    type_check_list_concat_aux__(Args, ListType, MidCurrVTB, NextVTB,
                                 Mid2LE, OutLE,
                                 MidUnif, OutUnif, Unders, N1, Parent, Err).

type_check_list_concat_aux__([A|Args], ListType, CurrentVTB, NextVTB, InLE,
                             OutLE, InUnif, OutUnif, Unders, N, Parent, Err) :-
    %% A is just a variable
    push_modes_in__('?'(list(ListType)), MA),
    has_type__(A, MA, CurrentVTB, [], OutVTB, InLE, MidLE, InUnif,
               Unders, N, Parent, Err),
    check_bind_type_constraints__(MidLE, Err),
    merge_type_check_VTB__(OutVTB, MergedVTB, Err),
    merge_VTB_currentVTB__(MergedVTB, CurrentVTB, Next0VTB),
    N1 is N+1,
    type_check_list_concat_aux__(Args, ListType, Next0VTB, NextVTB, MidLE,
                                 OutLE, InUnif, OutUnif,
                                 Unders, N1, Parent, Err).
%% Used by type_check_call_part_aux__ to extract all
%% variable type bindings and corresponding type_le constraints
%% for problems that are type correct (the error term is a variable)
get_all_type_correct__([], [], [], [], []).
get_all_type_correct__([E|AllError], [VTB|AllCallVTB], [VTB|AllOKVTB],
                       [LE|AllLE], [LE|AllOKLE]) :-
    var(E), !,
    get_all_type_correct__(AllError, AllCallVTB, AllOKVTB, AllLE, AllOKLE).
get_all_type_correct__([_|AllError], [_|AllCallVTB], AllOKVTB,
                       [_|AllLE], AllOKLE) :-
    get_all_type_correct__(AllError, AllCallVTB, AllOKVTB, AllLE, AllOKLE).

%% Used when type checking the call ground(Term) - all the variables
%% mentioned in Term need to have there VTB entries updated to be in ! mode
%%set_vars_to_ground__(A,B,C,D,E,F) :- errornl(set_vars_to_ground__(A,B,C,D,E,F)),fail.
set_vars_to_ground__(_, _, VTB, VTB, _, Error) :-
    nonvar(Error), !.
set_vars_to_ground__([], _, VTB, VTB, _, _).
set_vars_to_ground__([V|TermVars], Term, CurrentVTB, NextVTB, Unif, Error) :-
    in_unif__((V = UV), Unif), !,
    collect_vars((UV, TermVars), NewTermVars),
    set_vars_to_ground__(NewTermVars, Term, CurrentVTB, NextVTB, Unif, Error).
set_vars_to_ground__([V|TermVars], Term, CurrentVTB, NextVTB, Unif, Error) :-
    type_remove_from__(V:MTermType, CurrentVTB, VTB), !,
    transform_simple_terms(modes_to_bangs__, MTermType, UpdatedType),
    set_vars_to_ground__(TermVars,  Term, [(V:UpdatedType)|VTB],
                         NextVTB, Unif, Error).
set_vars_to_ground__([V|_], Term, VTB, VTB, _, Error) :-
    %% a new variable can never pass the ground test so we make this an error
    Error = new_var_error(ground(Term), new_var(V)).

modes_to_bangs__('?', '!') :- !.
modes_to_bangs__('??', '!') :- !.
%modes_to_bangs__('!?', '!') :- !.
modes_to_bangs__(X, X).

%%type_check_remote_query_vars__(A,B,C,D,E) :- errornl(type_check_remote_query_vars__(A,B,C,D,E)),fail.
type_check_remote_query_vars__(_Vars, _CurrentVTB, [], _Unif, Error) :-
    nonvar(Error), !.
type_check_remote_query_vars__([], _CurrentVTB, [], _Unif, _Error).

type_check_remote_query_vars__([V|Vars], CurrentVTB, [(V:GMT)|OutVTB],
                               Unif, Error) :- !,
    (
      type_member_of__(V:MT, CurrentVTB)
    ->
      strip_modes__(MT, StrippedMT),
      push_modes_in__('!'(StrippedMT), GMT)
    ;
      in_unif__((V = UV), Unif)
    ->
      has_type__(UV, MT, CurrentVTB, CurrentVTB, _, [], _, Unif,
                 [], 0, UV, Error),
      strip_modes__(MT, StrippedMT),
      push_modes_in__('!'(StrippedMT), GMT)

    ;
      Error = mode_error(0, V, V, MT, MT)
    ),
      
    type_check_remote_query_vars__(Vars, CurrentVTB,  OutVTB,
                                   Unif, Error).
%%XXXX TODO    
type_check_remote_query_vars__([V|Vars], CurrentVTB, [(V:MT)|OutVTB],
                               Unif, Error) :-
    (
      type_member_of__(V:MT, CurrentVTB)
    ->
      (
        fully_ground__(MT)
      ->
        true
      ;
        Error = mode_error(0, V, V, MT, MT)
      )
    ;
      in_unif__((V = UV), Unif)
    ->
      has_type__(UV, MT, CurrentVTB, CurrentVTB, _, [], _, Unif,
                 [], 0, UV, Error),
      (
        var(Error), fully_ground__(MT)
      ->
        true
      ;
        Error = mode_error(0, V, V, MT, MT)
      )
    ;
      Error = mode_error(0, V, V, MT, MT)
    ),
      
    type_check_remote_query_vars__(Vars, CurrentVTB,  OutVTB,
                                   Unif, Error).

%%remember_remote_query_types__(A,B,C) :- errornl(remember_remote_query_types__(A,B,C)),fail.
remember_remote_query_types__(_RemoteQ, _VTB, Error) :-
    nonvar(Error), !.    
remember_remote_query_types__(RemoteQ, VTB, _Error) :-
    assert('$remote_query_info'(RemoteQ, VTB)).

%% Used in type checking as the order predicate for sorting
%% variable type bindings
'$vtb_order'([], []).
'$vtb_order'([V1:M1(T1)|VMT1], [V2:M2(T2)|VMT2]) :-
    check_assert_(V1 == V2),
    strip_modes__(M1(T1), InnerT1),
    strip_modes__(M2(T2), InnerT2),
    (
      InnerT1 == InnerT2
    ->
      '$vtb_mode_order'(M1(T1), M2(T2)),
      '$vtb_order'(VMT1, VMT2)
    ;
      type_le__(InnerT1, InnerT2)
    ).

'$vtb_mode_order'(M1(T1), M2(T2)) :-
    min_mode__(M1, M2, Min), Min = M1,
    '$vtb_mode_order_aux'(T1, T2).

'$vtb_mode_order_aux'(T1, T2) :-
    constructor_type__(T1, _, SubT1), !,
    constructor_type__(T2, _, SubT2),
    map('$vtb_mode_order', SubT1, SubT2).
'$vtb_mode_order_aux'(_T1, _T2).



%% Used to check that the variable type bindings at the end of a rule
%% is consistent with VTB produced in the head. This is used to check
%% that any ? moded variables have been changed to ! moded
%%check_savedVTB__(A, B, C, D, E) :- errornl(check_savedVTB__(A, B, C, D, E)),fail.
%% An error has alread occurred - ignore
check_savedVTB__(_, _, _, _, Error) :-
    nonvar(Error), !.
%% We are type checking an internal sequence of calls - e.g. a forall
%% and so we ignore this check
check_savedVTB__(inner, _OutVTB, _, _, _Error).
check_savedVTB__([], _OutVTB, _, _, _Error).
%% check the moded type is consistent
check_savedVTB__([(X:MT)|Rest], OutVTB, Unif, Unders, Error) :-
    has_type__(X, MT, OutVTB, [], _, [], _LE, Unif, Unders, 0, saved, Error),
    check_savedVTB__(Rest, OutVTB, Unif, Unders, Error).

%% check_globals_locals__(Terms, Globals, OutGlobals, Locals, OutLocals,
%%                        Unders, Check, Error)
%% Given a list of Terms and and input list of Globals and Locals (variables)
%% and Unders (underscore variables) check that no globals are used as locals
%% and visa versa and return an update list of globals and locals
%% Check is either full (if this test is used at the outer level and
%% part (if used inside, for example, a forall). Used to determine if a
%% new global variable should be considered as an error.
%%check_globals_locals__(A,B,C,D,E,F,G, H) :- errornl(check_globals_locals__(A,B,C,D,E,F,G, H)),fail.
check_globals_locals__(_, Globals, Globals, Locals, Locals, _, _, Error) :-
    nonvar(Error), !.
check_globals_locals__([], Globals, Globals, Locals, Locals, _, _, _Error).
check_globals_locals__([Term|Terms], Globals, OutGlobals, Locals, OutLocals,
                       Unders, Check, Error) :-
    check_globals_locals_aux__(Term, Globals, MidGlobals, Locals, MidLocals,
                               Unders, Check, Error),
    !,
    check_globals_locals__(Terms, MidGlobals, OutGlobals, MidLocals, OutLocals,
                            Unders, Check, Error).

%%check_globals_locals_aux__(A,B,C,D,E,F,G,H) :- errornl(check_globals_locals_aux__(A,B,C,D,E,F,G,H)),fail.
check_globals_locals_aux__(Term, Globals, OutGlobals, Locals, OutLocals,
                           _Unders, _, _Error) :-
    ground(Term), !,
    %% Term contains no variables - nothng to do
    OutGlobals = Globals,
    OutLocals = Locals.
check_globals_locals_aux__(Term, Globals, OutGlobals, Locals, OutLocals,
                            _Unders, _, _Error) :-
    var(Term), !,
    %% Term is a variable - update globals
    union_list([Term], Globals, OutGlobals),
    OutLocals = Locals.
check_globals_locals_aux__(Term, Globals, OutGlobals, Locals, OutLocals,
                            Unders, Check, Error) :-
    Term = F(_, Query, _),  F == '$remote_query', !,
    %% A remote query - recursively call on Query part of the term
    '$goals2list'(Query, QueryList),
    check_globals_locals__(QueryList, Globals, OutGlobals, Locals, OutLocals,
                            Unders, Check, Error).
check_globals_locals_aux__(Term, Globals, OutGlobals, Locals, OutLocals,
                            Unders, Check, Error) :-
    compound(Term),
    functor(Term, F, 3),
    ( F == '$forall' ; F == '$forall_actions' ), !,
    %% a forall
    Term = _(V, Q1, Q2),
    collect_vars(Q1, Q1Vars),
    collect_vars(Q2, Q2Vars),
    diff_list(Q2Vars, Q1Vars, Q2Only),
    intersect_list(Q2Only, V, QVRHSOnly),
    intersect_list(Globals, V, Inter),
   (
      QVRHSOnly \= []
   ->
     %% at last on variable in V appears in Q2 but not Q1 - error
      Error = forall_quant_vars(QVRHSOnly, Term)
    ;
      Inter = []
   ->
     %% the globals and V do not overlap
      union_list(Globals, V, SubGlobals),
     '$goals2list'('&'(Q1, Q2), Q),
     %% For the body of the forall consider the variables in V as global
     %%check_globals_locals__(Q, SubGlobals, SubGlobalsOut, Locals, SubLocals,
     %%                       Unders, Check, Error),
     check_globals_locals__(Q, SubGlobals, SubGlobalsOut, [], SubLocals,
                            Unders, Check, Error),
     union_list(SubLocals, V, VandSubLocals),
     union_list(VandSubLocals, Locals, OutLocals),
     %% OutLocals are the InLocals, V and the locals found in Q
     diff_list(SubGlobalsOut, V, OutGlobals),
     %% OutGlobals are the globals passed into the check for Q that included V
     %% together with any globals found in Q - we need to remove V
     diff_list(OutGlobals, Globals, NewGlobals),
     diff_list(NewGlobals, Unders, New),
     check_new_globals__(Check, New, Term, Error)
   ;
     %% A global variable used as a quantified (local) variable
      Error = globals_locals(Inter, Term)
    ).
check_globals_locals_aux__(Term, Globals, OutGlobals, Locals, OutLocals,
                            Unders, Check, Error) :-
    Term = F(Result, Pattern, Goal), F == '$pfindall', !,
    check_globals_locals_aux__(Result, Globals, MidGlobals, Locals, MidLocals,
                               Unders, Check, Error),
    %% treat as a list constructor
    check_globals_locals_aux__('$list_constr'(Pattern, Goal), MidGlobals,
                               OutGlobals, MidLocals, OutLocals,
                               Unders, Check, Err),
    (
      var(Err)
    ->
      true
    ;
      Err = E(A, _),
      Error = E(A, Term)
    ).
check_globals_locals_aux__(Term, Globals, OutGlobals, Locals, OutLocals,
                            Unders, Check, Error) :-
    compound(Term),
    functor(Term, F, 2),
    F == '$exists', !,
    Term = _(V, Q1),
    '$goals2list'(Q1, Q),
    intersect_list(Globals, V, Inter),
    (
      Inter = []
    ->
      %% similar to forall
      union_list(Globals, V, SubGlobals),
      %%check_globals_locals__(Q, SubGlobals, SubGlobalsOut, Locals, SubLocals,
      %%                       Unders, Check, Error),
      check_globals_locals__(Q, SubGlobals, SubGlobalsOut, [], SubLocals,
                             Unders, Check, Error),
      union_list(Locals, V, VLocals),
      union_list(SubLocals, VLocals, OutLocals),
      diff_list(SubGlobalsOut, V, OutGlobals)
    ;
      Globals = OutGlobals,
      Locals = OutLocals,
      Error = locals_globals(Inter, Term)
    ).
check_globals_locals_aux__(Term, Globals, OutGlobals, Locals, OutLocals,
                            Unders, Check, Error) :-
    Term = F(Pattern, Body), 
    ( F == '$list_constr' ; F == '$set_constr' ), !,
    '$goals2list'(Body, BodyList),
    %% In a comprehension like [Pattern :: Body] we need to determine
    %% what are the "quantified" variables - these are globals in
    %% Pattern that are not global to the term - note that Pattern can,
    %% for example, be another (or include another) comprehension term
    %%check_globals_locals_aux__(Pattern, Globals, PattGlobals,
    %%                           Locals, _PattLocals,  Unders, Check, Error),
    check_globals_locals_aux__(Pattern, Globals, PattGlobals,
                               [], _PattLocals,  Unders, Check, Error),
    diff_list(PattGlobals, Globals, PattQuant),
    %%check_globals_locals__(BodyList, PattGlobals, MidGlobals,
    %%                       Locals, MidLocals,  Unders, Check, Error),
    check_globals_locals__(BodyList, PattGlobals, MidGlobals,
                           [], MidLocals,  Unders, Check, Error),
    diff_list(MidGlobals, PattQuant, OutGlobals),
    union_list(PattQuant, MidLocals, Mid2Locals),
    union_list(Locals, Mid2Locals, OutLocals).    
check_globals_locals_aux__(Term, Globals, OutGlobals, Locals, OutLocals,
                           Unders, Check, Error) :-
    compound(Term), !,
    %% delfault - a compund term - test funtor and all args
    Term =.. Parts,
    check_globals_locals__(Parts, Globals, OutGlobals, Locals, OutLocals,
                           Unders, Check, Error0),
    intersect_list(OutGlobals, OutLocals, Both),
    (
      var(Error0), Both \= []
    ->
      Error = global_local_error(Term, Both)
      ;
      Error = Error0
    ).

%%check_new_globals__(A,B,C,D) :- errornl(check_new_globals__(A,B,C,D)),fail.
check_new_globals__(part, _, _, _) :- !.
check_new_globals__(_, [], _Term, _Error) :- !.
check_new_globals__(_, New, Term, Error) :-
    Error = quantifer_new_globals(New, Term).

%% check_globals_locals__ can produce errors of various kinds.
%% report_error__ does some minor translations of the error term and
%% displays the required error message
report_error__(_Where, _Head, _Body, Err, _RuleInfo) :-
    var(Err), !.
report_error__(globals_locals, _Head, _Body, global_local_error(Term, Vars),
               RuleInfo) :-
    !,
    bind_vars__(RuleInfo),
    RuleInfo = rule_info(Kind, Clause, _Names, _Unders),
    '$display_error'(globals_locals_error(Vars, Kind, Term, Clause)),
    fail.
report_error__(globals_locals, _Head, _Body, globals_locals(Vars, Term),
               RuleInfo) :-
    !,
    bind_vars__(RuleInfo),
    RuleInfo = rule_info(Kind, Clause, _Names, _Unders),
    '$display_error'(globals_locals_error(Vars, Kind, Term, Clause)),
    fail.
report_error__(globals_locals, _Head, _Body, locals_globals(Vars, Term),
               RuleInfo) :-
    !,
    bind_vars__(RuleInfo),
    RuleInfo = rule_info(Kind, Clause, _Names, _Unders),
    '$display_error'(locals_globals_error(Vars, Kind, Term, Clause)),
    fail.
report_error__(globals_locals, _Head, _Body, quantifer_new_globals(Vars, Term),
               RuleInfo) :-
    !,
    bind_vars__(RuleInfo),
    RuleInfo = rule_info(Kind, Clause, _Names, _Unders),
    '$display_error'(not_underscore_mode_error(Vars, Kind, Term, Clause)),
    fail.
report_error__(globals_locals, _Head, _Body, forall_quant_vars(Vars, Term),
               RuleInfo) :-
    !,
    bind_vars__(RuleInfo),
    RuleInfo = rule_info(Kind, Clause, _Names, _Unders),
    '$display_error'(forall_quant_vars_error(Vars, Kind, Term, Clause)),
    fail.
report_error__(Where, Head, Body, Err, RuleInfo) :-
    errornl(error_____(Where, Head, Body, Err, RuleInfo)),
    fail.

%% During type checking involving polymorphic types various constraints
%% on these types can be generated. The aim of the code below is
%% to find "the best" solution for these constraints - this consists of finding
%% instantiations for these polymorphic type variables.
%% The possible constraints are:
%% le(T1, T2) : T1 type_le T2
%% le_alts(Name, AltTypes, Type) : Name is the name of some code type involving
%% alternative type declarations - AltTypes are extracted from these
%% alternatives and we constrain these alt types to be le Type
%% qle(Term, T1, T2): (see comment at the beginning of type_le_or_err__)
%% that come from where both types are in ?/?? modes.
%% To solve these constraints we do this in the order:
%% find a solution for le_alts (on backtracking we find another solution)
%% find a solution for le constraints
%% find a solution for qle constraints
%% As an example of the interaction between le and le_alts constraints consider
%% rel p(((T,T) -> T), T, T, ?T)
%% p(F, X, Y, Z) <= Z = F(X, Y)
%% and the call
%% p(+, 2, 3.4, X)
%% (noting + : (nat, nat) -> nat, (int, int) -> int, (num, num) -> num
%% after the first phase of type checking we get  constraints
%% [le(num, T1),
%%  le_alts(+, [($tuple_type([nat, nat]) -> nat),
%%              ($tuple_type([int, int]) -> int),
%%              ($tuple_type([num, num]) -> num)],
%%          ($tuple_type([T1, T1]) -> T1))])
%% where T1 is the instance of T used to type check this call.
%% Note we don't include le(nat, T1) as this is subsumed by le(num, T1)
%% The first step is to solve for the le_alts constraints. This leads to
%% T1 = nat but then the constraint le(num, nat) fails.
%% On backtracking we resolve for le_alts to get T1 = int and le(num, int)
%% fails. On the next backtracking we succeed with T1 = num

%%bind_type_constraints__(LE, Error) :- errornl(bind_type_constraints__(LE, Error)),fail.
bind_type_constraints__(_LE, Error) :-
    nonvar(Error), !.
bind_type_constraints__(LE, Error) :-
    split_le__(LE, LE1, LE2, LE3),
    bind_alts__(LE2, Error),
    bind_type_constraints1__(LE1),
    bind_type_constraints_leq__(LE3, [], LE5, Error),
    bind_type_constraints1__(LE5).

%% Test if LE has a solution without binding any type variables
%%check_bind_type_constraints__(LE) :- errornl(check_bind_type_constraints__(LE)),fail.
check_bind_type_constraints__(LE) :-
    not_check_bind_type_constraints__(LE), !,fail.
check_bind_type_constraints__(_).

%%not_check_bind_type_constraints__(LE) :- errornl(not_check_bind_type_constraints__(LE)),fail.
not_check_bind_type_constraints__(LE) :-
    check_bind_type_constraints__(LE, Err1), var(Err1), !,fail.
not_check_bind_type_constraints__(_).

%% Test if LE has a solution - instantiate Error if error found
check_bind_type_constraints__(_LE, Error) :-
    nonvar(Error), !.
check_bind_type_constraints__(LE, Error) :-
    split_le__(LE, LE1, LE2, LE3),
    bind_alts__(LE2, Error),
    bind_type_constraints__(LE1, Error),
    check_bind_qle_type_constraints__(LE3, Error).

%% Test if QLE constraint has a solution - instantiate Error if error found
%%check_bind_qle_type_constraints__(LE, Error) :- errornl(check_bind_qle_type_constraints__(LE, Error)),fail.
check_bind_qle_type_constraints__(_LE, Error) :-
    nonvar(Error), !.
check_bind_qle_type_constraints__([], _Error).
check_bind_qle_type_constraints__([qle(Term, T1, T2)|_LE], Error) :-
    \+compatible__(T1, T2), !,
    get_type_error__(Term, '?', T1, T2, 0, Term, Error).
check_bind_qle_type_constraints__([_|LE], Error) :-
    check_bind_qle_type_constraints__(LE, Error).


%% split constraints into le, le_alts and qle constraints
split_le__([], [], [], []).
split_le__([H|T], [H|LE1], LE2, LE3) :-
    H = le(_, _), !,
    split_le__(T, LE1, LE2, LE3).
split_le__([H|T], LE1, [H|LE2], LE3) :-
    H = le_alts(_, _, _), !,
    split_le__(T, LE1, LE2, LE3).
split_le__([H|T], LE1, LE2, [H|LE3]) :-
    split_le__(T, LE1, LE2, LE3).


%% solve qle constraints by binding type variables (or return an error)
%%bind_type_constraints_leq__(A,B,C,D) :- errornl(bind_type_constraints_leq__(A,B,C,D)),fail.
bind_type_constraints_leq__(_, LE, LE, Error) :-
    nonvar(Error), !.
bind_type_constraints_leq__([], LE, LE, _Error). 
bind_type_constraints_leq__([qle(Term, T1, T2)|Rest],  InLE, OutLE, Error) :-
    !,
    type_le_or_err__('!', Term, T1, T2, InLE, MidLE, 0, Term, Error),
    bind_type_constraints_leq__(Rest, MidLE, OutLE, Error).
bind_type_constraints_leq__([C|Constr],  InLE, OutLE, Error) :-
    errornl(should_not_get_here(bind_type_constraints_leq__([C|Constr],  InLE, OutLE, Error))).

%% Find solutions to le_alts constraints - allow backtracking to find
%% alternative solutions
%%bind_alts__(Alts, Error) :- errornl(bind_alts__(Alts, Error)), fail.
bind_alts__(_, Error) :- nonvar(Error), !.
bind_alts__([], _) :- !.
bind_alts__(Alts,  _Error) :- 
    bind_backtrack_alts__(Alts).


bind_backtrack_alts__([]).
bind_backtrack_alts__([le_alts(Term, As, T)|Rest]) :-
    member(AT, As),
    type_le_or_err__('!', Term, AT, T, [], LE, 0, bind_alts, Err),
    var(Err),
    bind_type_constraints1__(LE),
    bind_backtrack_alts__(Rest).


%% construct a type error term - if Term is a variable we call it a type
%% conflict otherwise we simply say its a type error
%%get_type_error__(Term, Mode, ActType, WantedType, Index, PTerm, Error) :-errornl(get_type_error__(Term, Mode, ActType, WantedType, Index, PTerm, Error)),fail.
get_type_error__(Term, _Mode, ActType, WantedType, Index, PTerm, Error) :-
    var(Term), !,
    Error = type_error(Index, PTerm, Term, WantedType, conflict(ActType), _).
get_type_error__(Term, _Mode, _ActType, WantedType, Index, PTerm, Error) :-
     Error = type_error(Index, PTerm, Term, WantedType, default, _).

%% '$function_call_or_comp'(T, VTB, Unif) is true if T is a compound term
%% whise ultimate functor is either  a non-variable
%% and T is a list or set comprehension or a function call
%% or is a variable that is of type function (or is unified with a variable
%% of type function)
'$function_call_or_comp'(T, _, _) :-
    compound(T), functor(T, F, 2), F == '$list_constr', !.
'$function_call_or_comp'(T, _, _) :-
    compound(T), functor(T, F, 2), F == '$set_constr', !.
'$function_call_or_comp'(T, _, _) :-
    compound(T), functor(T, F, 1), F == '$set_enum', !.
'$function_call_or_comp'(T, _, _) :-
    '$get_ultimate_functor'(T, F),
    atom(F), !,
    '$type_info'(F, fun, _, _, _).
'$function_call_or_comp'(T, VTB, Unif) :-
    '$get_ultimate_functor'(T, F),
    var(F), in_unif__((F = UF), Unif), !,
    '$replace_ultimate_functor'(T, F, UF, UT),
    '$function_call_or_comp'(UT, VTB, Unif).
'$function_call_or_comp'(T, VTB, _Unif) :-
    '$get_ultimate_functor'(T, F),
    type_member_of__(F:'!'(FT), VTB), !,
    compound(FT), functor(FT, '->', 2).

%% forget can take a dyn_term with an atom functor but non-ground args.
%% This is the only such situation where we need to have a mode between
%% ! and ?/??. So both !? and !?? modes (used only internally) mean just this. 
'$dyn_term_mode'('!').
%'$dyn_term_mode'('!?').
'$dyn_term_mode'('!??').

%% Convert from ?/?? to !?/!?? when a nonvar call is made on something of
%% type dyn_term and used in reverse when type checking to move from
%% !?/!?? to the less restrictive ?/??
'$dyn_term_mode_modify'('?', '!??').   
'$dyn_term_mode_modify'('??', '!??').   

%% see comments in type_check_call_aux__ where we have multiple
%% declarations. The declarations with inner ! modes are processed first.
split_on_inner_bangs__([], _, [], []).
split_on_inner_bangs__([T|Types], N, [t(N, T)|Bangs], NonBangs) :-
    has_inner_bang__(T), !,
    N1 is N+1,
    split_on_inner_bangs__(Types, N1, Bangs, NonBangs).
split_on_inner_bangs__([T|Types], N, Bangs, [t(N, T)|NonBangs]) :-    
    N1 is N+1,
    split_on_inner_bangs__(Types, N1, Bangs, NonBangs).

has_inner_bang__([Type |_]) :-
    has_inner_bang_aux__(Type), !.
has_inner_bang__([_|Types]) :-
    has_inner_bang__(Types).

has_inner_bang_aux__('!'(T)) :- var(T), !.
has_inner_bang_aux__('!'(T)) :- \+ constructor_type__(T, _, _), !.
has_inner_bang_aux__('!'(T)) :-
    constructor_type__(T, _, SubT),
    has_inner_bang__(SubT).

%% is_compatible__(T1, T2) is true iff either T1 =< T2 or T2 =< T1
%% (without generating any le constraints
%%is_compatible__(A, B) :- errornl(is_compatible__(A, B)), fail.
is_compatible__(T1, '$union_type'(Types)) :-
    member(T2, Types), is_compatible__(T1, T2),!.
is_compatible__('$union_type'(Types), T1) :-
    member(T2, Types), is_compatible__(T1, T2),!.
is_compatible__(T1, T2) :-
    constructor_type__(T1, CT, T1L), !,
    constructor_type__(T2, CT, T2L),
    pairwise_is_compatible__(T1L, T2L).
is_compatible__(T1, T2) :-
    type_le_aux__(T1, T2, [], []), !.
is_compatible__(T1, T2) :-
    type_le_aux__(T2, T1, [], []), !.
is_compatible__(T1, T2) :-
    '$type_info'(_, macro_type, T1, DefT1, _), !,
    is_compatible__(DefT1, T2).
is_compatible__(T2, T1) :-
    '$type_info'(_, macro_type, T1, DefT1, _), !,
    is_compatible__(DefT1, T2).


pairwise_is_compatible__([],[]).
pairwise_is_compatible__([T1|R1], [T2|R2]) :-
    is_compatible__(T1, T2),
    pairwise_is_compatible__(R1,R2).

%% same as above but allowing le constraints to be generated
compatible__(T1, T2) :-
    type_le_aux__(T1, T2, [], _).
compatible__(T1, T2) :-
    type_le_aux__(T2, T1, [], _).


%% For compatible ground types get their intersection
%%intersection_of__(A,B,C) :- errornl(intersection_of__(A,B)),fail.
intersection_of__('$union_type'(T1L), '$union_type'(T2L), Int) :-
    findall(TI, (member(T, T1L), compatible__(T, '$union_type'(T2L)),
                    intersection_of__(T, '$union_type'(T2L), TI)), Int1),
    (
      Int1 = [Int]
    ->
      true
    ;
      Int = '$union_type'(Int1)
    ).
intersection_of__('$union_type'(T1L), T2, Int) :- !,
    intersection_of__(T2, '$union_type'(T1L), Int).
intersection_of__(T1, '$union_type'(T2L), Int) :- !,
    once((
          member(T2, T2L), is_compatible__(T1, T2),
          intersection_of__(T1, T2, Int)
         )).
intersection_of__(T1, T2, Int) :-
    constructor_type__(T1, CT, T1L),
    constructor_type__(T2, CT, T2L), !,
    map(intersection_of__, [T1L,T2L,IntL]),
    Int =.. [CT|IntL].

intersection_of__(T1, T2, Int) :-
    type_le_aux__(T1, T2, [], []), !, Int = T1.
intersection_of__(T2, T1, Int) :-
    type_le_aux__(T1, T2, [], []), !, Int = T1.
intersection_of__(T1, T2, Int) :-
    '$type_info'(_, macro_type, T1, DefT1, _), !,
    intersection_of__(DefT1, T2, Int).
intersection_of__(T2, T1, Int) :-
    '$type_info'(_, macro_type, T1, DefT1, _), !,
    intersection_of__(DefT1, T2, Int).



%% For the calls not and forall we need to be able to determine what variables
%% will be non ground at certain points.
%%non_ground__(A,B,C,D) :- errornl(non_ground__(A,B,C,D)),fail.
non_ground__([], _, _, []).
non_ground__([V|Vars], VTB, Unif, NG) :-
    type_member_of__((V:MT), VTB),
    push_modes_in__(MT, PMT),
    fully_ground__(PMT), !,
    non_ground__(Vars, VTB, Unif, NG).
non_ground__([V|Vars], VTB, Unif, NG) :-
    in_unif__((V = VT), Unif),
    collect_vars(VT, VTVars),
    non_ground__(VTVars, VTB, Unif, []), !,
    non_ground__(Vars, VTB, Unif, NG).
non_ground__([V|Vars], VTB, Unif, [V|NG]) :-
    non_ground__(Vars, VTB, Unif, NG).

fully_ground__('!'(T)) :-
    atomic(T), !.
fully_ground__('!'(T)) :-
    var(T), !.
fully_ground__('!'(T)) :-
    compound(T), functor(T, F, _), F == set, !.
fully_ground__('!'(T)) :-
    compound(T), functor(T, F, _), F == '$$var$$', !.
fully_ground__('!'(T)) :-
    compound(T), is_code_type_(T), !.
fully_ground__('!'(T)) :-
    !,
    constructor_type__(T, _, SubT),
    forall(member(ST, SubT), fully_ground__(ST)).

is_code_type_(rel(_)) :- !.
is_code_type_(dyn(_)) :- !.
is_code_type_(act(_)) :- !.
is_code_type_(tel(_)) :- !.
is_code_type_(fun(_)) :- !.
is_code_type_(T) :-
    '$type_info'(_, macro_type, T, DefT, _),
    is_code_type_(DefT).

%% For use in debugging system code
debug_call(Call) :-
    (
      errornl(debug_call______(Call)),
      call(Call),
      (
        errornl(exit_debug_call______(Call))
      ;
        errornl(fail_debug_call______(Call)), fail
      )
    ;
      errornl(fail_exit_debug_call______(Call)), fail
    ).


%%XXX
%%get_function_rule_head_types__(Head, FunType, Kind, Args, ATypes, ResultType,
%%                               Error)
%% Extracts all the args and types and the result type for a function call
%% to be used for type checking a function.
%% EG1: for
%% fun f(nat, nat) -> nat
%% For Head = f(2, X) we get
%% FunType = fun(nat, nat) -> nat
%% Args = [2,X]
%% ATypes = [!(nat), !(nat)]
%% ResultType = nat
%%
%% EG2:
%% For Head = curry(f)(2)  (f as above) we get
%% FunType = fun(fun(T1, T2) -> T3) -> fun(T1) -> fun(T2) -> T3
%% Args = [f, 2]
%% ATypes = [fun(T1, T2) -> T3), T1]
%% ResultType =  fun(T2) -> T3
%% Kind is used, for example, in
%% fun curryR(rel(T1,??T2)) -> fun(T1) -> rel(??T2)
%% curryR(Rel)(X)(Y) <= Rel(X,Y)
%% to check that curryR(Rel)(X)(Y) is a relation

%%get_function_rule_head_types__(Head, FunType, Kind, Args, ATypes, ResultType, Error) :- errornl(get_function_rule_head_types__(Head, FunType, Kind, Args, ATypes, ResultType, Error)),fail.
get_function_rule_head_types__(Head, _FunType, _Kind, _Args, _ArgTypes,
                               _ResultType, Error) :-
    '$get_ultimate_functor'(Head, F),
    \+'$type_info'(F, fun, _, _, _), !,
    Error = undeclared_function(Head).
get_function_rule_head_types__(Head, FunType, Kind, Args, ATypes, ResultType,
                               Error) :-
    '$get_ultimate_functor'(Head, F),
    '$type_info'(F, fun, FunType, _, VarNames),
    '$set_var_names'(FunType, VarNames, _),
    get_function_rule_head_types_aux__(Head, FunType, Kind, [], ArgsList,
                                       ArgTypesList, ResultType, Error),
    check_function_arities__(ArgsList, ArgTypesList, Head, Error),
    flatten_args_list__(ArgsList, ArgTypesList, Args, ArgTypes, Error),
   '$add_annotations'(ArgTypes, ATypes, '!') .


%%get_function_rule_head_types_aux__(A,B,C,D,E,F,G,H) :- errornl(get_function_rule_head_types_aux__(A,B,C,D,E,F,G,H)),fail.
get_function_rule_head_types_aux__(_Head, _FunType, _Kind, InArgs, OutArgs,
                                   _ArgsTypes, _ResultType, Error) :-
    nonvar(Error), !,
    OutArgs = InArgs.
get_function_rule_head_types_aux__(Head, FunType, Kind, InArgs, OutArgs,
                                   ArgsTypes, ResultType, Error) :-
    compound(Head),
    nonvar(FunType),
    '$type_info'(_,macro_type,FunType,FullFunType,_), !,
    get_function_rule_head_types_aux__(Head, FullFunType, Kind, InArgs, OutArgs,
                                       ArgsTypes, ResultType, Error).
get_function_rule_head_types_aux__(Head, FunType, Kind, InArgs, OutArgs,
                                   ArgsTypes, _ResultType, Error) :-
    compound(Head), FunType \= (_ -> _), FunType \= Kind(_), !,
    OutArgs = InArgs,
    ArgsTypes = [],
    Error = head_type_mismatch.
get_function_rule_head_types_aux__(Head, ('$tuple_type'(D) -> R), Kind, 
                                   InArgs, OutArgs,
                                   [D|ArgsTypes],
                                   ResultType, Error) :-
    compound(Head),
    !,
    '@..rev'(Head, F, Args),
    get_function_rule_head_types_aux__(F, R,  Kind, [Args|InArgs], OutArgs,
                                       ArgsTypes,
                                       ResultType, Error).
get_function_rule_head_types_aux__(Head, Kind('$tuple_type'(KType)), Kind, 
                                   InArgs, OutArgs,
                                   [KType],
                                   _ResultType, _Error) :-
    compound(Head),
    !,
    '@..rev'(Head, _F, Args),
    OutArgs = [Args|InArgs].
get_function_rule_head_types_aux__(_Head, ResultType,  _, InArgs, OutArgs,
                                   [],
                                   ResultType, _Error) :-
    OutArgs = InArgs.

%% used in type_check_call_ho_var_functor__ to extract the Args and ArgTypes
%% to be used in type checking
%%get_function_code_types__(A,B,C,D,E,F,G) :- errornl(get_function_code_types__(A,B,C,D,E,F,G)),fail.
get_function_code_types__(_Call, _FunType, _Kind, InArgs, OutArgs,
                          _ArgsTypes, Error) :-
    nonvar(Error), !,
    OutArgs = InArgs.
get_function_code_types__(Call, FunType, Kind, InArgs, OutArgs,
                          ArgsTypes, Error) :-
    compound(Call),
    nonvar(FunType),
    '$type_info'(_,macro_type,FunType,FullFunType,_), !,
    get_function_code_types__(Call, FullFunType, Kind, InArgs, OutArgs,
                              ArgsTypes, Error).
get_function_code_types__(Call, FunType, Kind, InArgs, OutArgs,
                          ArgsTypes, Error) :-
    compound(Call), FunType \= (_ -> _), FunType \= Kind(_), !,
    OutArgs = InArgs,
    ArgsTypes = [],
    (
      FunType = FKind(_)
    ->
      Error = kind_error(Call, Kind, FKind)
    ;
      Error = head_type_mismatch
    ).
get_function_code_types__(Call, ('$tuple_type'(D) -> R), Kind, 
                          InArgs, OutArgs, [D|ArgsTypes], Error) :-
    compound(Call),
    !,
    '@..rev'(Call, F, Args),
    get_function_code_types__(F, R,  Kind, [Args|InArgs], OutArgs,
                                       ArgsTypes, Error).
get_function_code_types__(Call, Kind('$tuple_type'(KType)), Kind, 
                          InArgs, OutArgs, [KType], _Error) :-
    compound(Call),
    !,
    '@..rev'(Call, _F, Args),
    OutArgs = [Args|InArgs].
get_function_code_types__(Call, FKind('$tuple_type'(KType)), Kind, 
                          InArgs, OutArgs, [KType], Error) :-
    Error = kind_error(Call, FKind, Kind),
    OutArgs = InArgs.

%% check_function_arities__(ArgsList, ArgsTypeList, Head, Error)
%% check that each list of args in ArgsList as the same length as the
%% corresponding list in ArgsTypeList
%%check_function_arities__(_ArgsList, _ArgTypesList, _Head, Error) :- errornl(check_function_arities__(_ArgsList, _ArgTypesList, _Head, Error)),fail.
check_function_arities__(_ArgsList, _ArgTypesList, _Head, Error) :-
    nonvar(Error), !.
check_function_arities__([], [], _, _).
check_function_arities__(['$tuple'(As)|_ArgsList],
                         ['$tuple_type'(Ts)|_ArgTypesList], Head, Error) :-
    length(As, N),
    length(Ts, TN),
    N \= TN, !,
    Error = arity_error(Head, Ts).
check_function_arities__([As|_ArgsList], [Ts|_ArgTypesList], Head, Error) :-
    list(As),
    length(As, N),
    length(Ts, TN),
    N \= TN, !,
    Error = arity_error(Head, Ts).
check_function_arities__([_As|ArgsList], [_Ts|ArgTypesList], Head, Error) :-
    '@..rev'(Head, F, _),    
    check_function_arities__(ArgsList, ArgTypesList, F, Error).

%% Flatten a list of list of args and the corresponding list of list of types
%% into  a list of args and a list of types
flatten_args_list__(_ArgsList, _ArgTypesList, Args, ArgTypes, Error) :-
    nonvar(Error), !,
    Args = [],
    ArgTypes = [].
flatten_args_list__([], [], [], [], _Error).
flatten_args_list__([As|ArgsList], [ATs|ArgTypesList], Args, ArgTypes, Error) :-
      append(As,  ArgsRest, Args),
      append(ATs,  ArgTypesRest, ArgTypes),
      flatten_args_list__(ArgsList, ArgTypesList, ArgsRest,
                          ArgTypesRest, Error).

%% Same as above but for a list of tuples and tuple types
%%flatten_tuple_args_list__(A,B,C,D,E) :- errornl(flatten_tuple_args_list__(A,B,C,D,E)),fail.
flatten_tuple_args_list__(_,_,[],[], Err) :- nonvar(Err), !.
flatten_tuple_args_list__([], [], [], [], _Error).
flatten_tuple_args_list__(['$tuple'(As)|ArgsList],
                          ['$tuple_type'(ATs)|ArgTypesList], Args, ArgTypes,
                          Error) :-
    append(As,  ArgsRest, Args),
    append(ATs,  ArgTypesRest, ArgTypes),
    flatten_tuple_args_list__(ArgsList, ArgTypesList, ArgsRest, ArgTypesRest,
                              Error).

%% for type checking unifications we do this using the same idea as type
%% checking in the head of a rule. Any type errors generated may refer to
%% the head and so need to be translated - so head_merge_conflict(MT2)
%% gets converted to merge_conflict(MT2)
head_error_to_unif_error__(HeadError, _Error) :- var(HeadError), !.
head_error_to_unif_error__(type_error(Index, PTerm, Term, MT1,
                                      head_merge_conflict(MT2), DecI), Error) :-
    !,
    %%strip_modes__(MT1, T1),
    %%strip_modes__(MT2, T2),
    Error = type_error(Index, PTerm, Term, MT1, merge_conflict(MT2), DecI).
head_error_to_unif_error__(E, Error) :-
    Error = E.


%% check_error__(Err, Where, Error) wraps Err with Where (head or body)
%% to produce the full error term
%%check_error__(Err, Where, Error) :- errornl(check_error__(Err, Where, Error)),fail.
check_error__(_Err, _Where, Error) :-
    nonvar(Error), !.
check_error__(Err, Where, Error) :-
    nonvar(Err), !, Error = Where(Err).
check_error__(_, _, _).

%% the error terms below are generated by a findall and so there is a disconnect
%% between the variables in the error term and those in the actual Head
%% and Body - this unifies those variables.
%%bind_error_vars__(A,B,C) :- errornl(bind_error_vars__(A,B,C)),fail.
bind_error_vars__([], _Head, _Body).
bind_error_vars__([err__(_, _, Head, Body)|Errors], Head, Body) :-
    bind_error_vars__(Errors, Head, Body).

%% Type checking rules is done inside a findall that trys alternative
%% type declarations.
%% process_errors__(Errors, RuleInfo) takes  a list of "error terms" of teh form
%% err__(Error, TypeInfo, Head, Body) and a term containing the rule info
%% for the rule under test.
%%process_errors__(Errors, RuleInfo) :- errornl(process_errors__(Errors, RuleInfo)),fail.
process_errors__(Errors, RuleInfo) :-
    length(Errors, TotalLen),
    filter('$process_real_err', Errors, RealErrors),
    length(RealErrors, RealLen),
    %% TotalLen is the total number of terms (some of which might not be errors
    %% RealLen is the length of the lsit of real error terms (RealErrors)
    process_errors_aux__(RealLen, TotalLen, RealErrors, RuleInfo).

'$process_real_err'(err__(Error, _, _, _)) :- nonvar(Error).

%%process_errors_aux__(N, M, Errors, RuleInfo) :- errornl(process_errors_aux__(N, M, Errors, RuleInfo)),fail.
%% No errors - nothing to do
process_errors_aux__(0, _, _, _) :- !.
%% ignore false_always_fails errors
process_errors_aux__(N, M, Errors, RuleInfo) :-
    Errors = [err__(body(body_call_error(raise( _),
                                         raise_always_exits(_))),
                    _, _, _)|Rest],
    !,
    N1 is N-1,
    process_errors_aux__(N1, M, Rest, RuleInfo).
process_errors_aux__(N, M, Errors, RuleInfo) :-
    Errors = [err__(body(body_call_error(false( _),
                                         false_always_fails(_))),
                    _, _, _)|Rest],
    !,
    N1 is N-1,
    process_errors_aux__(N1, M, Rest, RuleInfo).
%% Only one declaration and it is an error
process_errors_aux__(1, 1, [err__(Error, Decl, _, _)], RuleInfo) :-
    !,
    %% report as a single error message
    do_error_with_decl__(Error, RuleInfo, Decl, single).
%% all errors are the same - report that error
process_errors_aux__(N, N, Errors, RuleInfo) :-
    freeze_term(Errors, Vars),
    Errors = [err__(ET, Decl, _, _)|Rest],
    forall(member(E, Rest),
           E = err__(ET, _, _, _)), !,
    thaw_term(Vars),
    do_error_with_decl__(ET , RuleInfo, Decl, single).
%% ignore not_le_type_test errors
process_errors_aux__(N, M, Errors, RuleInfo) :-
    Errors = [err__(body(body_call_error(type(V, _),
                                         not_le_type_test(type(V, _), _))),
                    _, _, _)|Rest],
    !,
    N1 is N-1,
    process_errors_aux__(N1, M, Rest, RuleInfo).
%% report first error (but note that there are multiple (different) errors
process_errors_aux__(_, _, Errors, RuleInfo) :-
    Errors = [err__(E, Decl, _, _)|_],
    do_error_with_decl__(E, RuleInfo, Decl, multiple).


%% For the different kinds of rule shapes extract the rule head.
rule_head__((Head :: _ <= _), Head) :- !.
rule_head__((Head <= _), Head) :- !.
rule_head__((Head :: _ ~> _), Head) :- !.
rule_head__((Head ~> _), Head) :- !.
rule_head__((Head :: _ -> _), Head) :- !.
rule_head__((Head -> _), Head) :- !.
rule_head__(Head, Head).

%%XXX

inner_term_type__(term).
inner_term_type__(Type) :-
    compound(Type),
    Type =.. [_|Args],
    member(A, Args), inner_term_type__(A).