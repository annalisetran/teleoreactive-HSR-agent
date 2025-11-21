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
	  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Writer (pretty printer)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Determine if brackets are needed after '::' - really a precedence
%% issue
'$not_colon_term_needs_brackets'(T) :-
    freeze_term(T),
    '$colon_term_needs_brackets'(T),
    !,
    fail.
'$not_colon_term_needs_brackets'(_).

'$colon_term_needs_brackets'('=?'(_, _)).
'$colon_term_needs_brackets'('=@'(_, _)).
'$colon_term_needs_brackets'('@='(_, _)).
'$colon_term_needs_brackets'('='(_, _)).
'$colon_term_needs_brackets'('\\='(_, _)).
'$colon_term_needs_brackets'('@>'(_, _)).
'$colon_term_needs_brackets'('@<'(_, _)).
'$colon_term_needs_brackets'('@>='(_, _)).
'$colon_term_needs_brackets'('@=<'(_, _)).
'$colon_term_needs_brackets'(in(_, _)).


%% Pretty printing term T
writeTerm_(T) :-
    writeTerm_(stdout, T).

writeTerm_(Stream, T) :-
    var(T), !, writeR(Stream, T).
%% when printing a watched rule we want to strip the '_watched' from the name
writeTerm_(Stream, T) :-
    atom(T),
    atom_concat(T1, '_watched', T), !,
    write(Stream, T1).
%% The TR nil action () is represented internally as '$nil'
writeTerm_(Stream, T) :-
    atom(T), T = '$nil', !,
    write(Stream, '()').
%% The null action {} is represented internally as $null_action'
writeTerm_(Stream, T) :-
    atom(T), T = '$null_action', !,
    write(Stream, '{}').
%% For writing temp decl terms
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 3), F == '$code_decl', !,
    '$write_code_decl'(Stream, T).
%% For writing declarations for rel,act,fun, ..
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 2), F == 'def_doc', !,
    T = def_doc(T1, _),
    writeTerm_(Stream, T1).
%% For terms that are actually code we pretty print as code
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 1), F == receive, !,
    show_body_(T, 0, Stream).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 2), F == try_except, !,
    show_body_(T, 0, Stream).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 2), F == repeat_until, !,
    show_body_(T, 0, Stream).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 1), F == atomic_action, !,
    show_body_(T, 0, Stream).
%% ?(Condition)
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 1), F == '$rel_escape', !,
    show_body_(T, 0, Stream).
%% The remote query term (code)
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 2), F == '$remote_query', !,
    show_body_(T, 0, Stream).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 3), F == '$remote_query', !,
    show_body_(T, 0, Stream).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 1), F == 'case', !,
    show_body_(T, 0, Stream).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 1), F == 'wait', !,
    show_body_(T, 0, Stream).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 1), F == 'wait_case', !,
    show_body_(T, 0, Stream).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 3), F == from,
    T = from(Msg, Addr, Test), !,
    writeTerm_(Msg),
    write(Stream, ' from '),
    writeTerm_(Addr),
    (
     Test = '$true'(_)
    ->
     true
    ;
     write(Stream, ' :: '),
     writeTerm_(Stream, Test)
    ).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 3), F == from_thread,
    T = from(Msg, Addr, Test), !,
    writeTerm_(Msg),
    write(Stream, ' from_thread '),
    writeTerm_(Addr),
    (
     Test = '$true'(_)
    ->
     true
    ;
     write(Stream, ' :: '),
     writeTerm_(Stream, Test)
    ).
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 1), F == not,
    T = not(Call), Call = '&'(_, _),
    !,
    write(Stream, 'not ('),
    writeTerm_(Stream, Call),
    write(Stream, ')').
writeTerm_(Stream, T) :-
    compound(T), functor(T, F, 1), F == not,
    !,
    T = not(Call),
    write(Stream, 'not '),
    writeTerm_(Stream, Call).

%% regular expressions as in String =? T/RE ++ ...
writeTerm_(Stream, '$re'(T, RE)) :-
    !,
    writeTerm_(Stream, T),
    write(Stream, '/'),
    writeTerm_(Stream, RE).
%% type declarations with doc
writeTerm_(Stream, '$doc'(Type, Doc)) :-
    !,
    writeTerm_(Stream, Type),
    nl(Stream),
    writeTerm_(Stream, Doc).
writeTerm_(Stream, ':'(V, T)) :-
    var(T),
    !,
    writeTerm_(Stream, V),
    write(Stream, ':'),
    writeTerm_(Stream, T).
writeTerm_(Stream, ':'(V, '^'(T))) :-
    !,
    write(Stream, '^'),
    writeTerm_(Stream, V),
    write(Stream, ':'),
    writeTerm_(Stream, T).
%% non-code terms
%% quote atoms are required
writeTerm_(Stream, T) :-
    atom(T), !,
    writeq(Stream, T).
writeTerm_(Stream, T) :-
    integer(T), !, write(Stream, T).
writeTerm_(Stream, T) :-
    number(T), !, write(Stream, T).
%% double quote strings
writeTerm_(Stream, T) :-
    string(T), !, write(Stream, T).

%% The empty set
writeTerm_(Stream, '$set'([])) :-
    !,
    write(Stream, '{}').
%% Non-empty set
writeTerm_(Stream, '$set'(T)) :-
    !,
    write(Stream, '{'),
    writeT_list_elems_(Stream, T),
    write(Stream, '}').
%% same again with set_enum
writeTerm_(Stream, '$set_enum'([])) :-
    !,
    write(Stream, '{}').
writeTerm_(Stream, '$set_enum'(T)) :-
    !,
    write(Stream, '{'),
    writeT_list_elems_(Stream, T),
    write(Stream, '}').
%% write '$same_handle_'(H1, H2) as same_handle(H1, H2) - needed
%% because there is a same_handle in QP with a different semantics
writeTerm_(Stream, '$same_handle_'(H1, H2)) :-
    !,
    writeT_compound(Stream, same_handle(H1, H2)).
writeTerm_(Stream, T) :-
    list(T), !,
    writeT_list_(Stream, T).
%% write compound terms - we freeze the variables so we can deal with
%% variable functors more simply
writeTerm_(Stream, T) :-
    compound(T),
    freeze_term(T),
    writeT_compound_(Stream, T),
    fail.
writeTerm_(_Stream, _).

    
writeT_list_(Stream, T) :-
    write(Stream, '['),
    freeze_term(T, V),
    writeT_list_elems_(Stream, T),
    thaw_term(V),
    write(Stream, ']'), !.
writeT_list_elems_(Stream, [X]) :-
    !,
    writeTerm_(Stream, X).
writeT_list_elems_(Stream, [X1, X2|T]) :-
    !,
    writeTerm_(Stream, X1),
    write(Stream, ', '),
    writeT_list_elems_(Stream, [X2|T]).
writeT_list_elems_(Stream, [X|T]) :-
    writeTerm_(Stream, X),
    write(Stream, ',..'),
    writeTerm_(Stream, T).

%% pretty print compound terms - deals with all prefix/infix ops
%% including precedence and associativity
%%writeT_compound_(Stream, Term) :- errornl(write___(Term)),fail.

%% Print rules as clauses
writeT_compound_(Stream, Term) :-
    '$is_clause_defn'(Term), !,
    show_clause_(Term, Stream).
writeT_compound_(Stream, Term) :-
    functor(Term, F, _),
    var(F),
    Term = F('$none_'),
    !,
    writeTerm_(Stream, F),
    write(Stream, '()').
%% Functor is a variable - use default write
writeT_compound_(Stream, Term) :-
    functor(Term, F, _),
    var(F),
    !,
    Term =.. [_|Args],
    writeTerm_(Stream, F),
    write(Stream, '('),
    writeT_args_(Stream, Args, ', '),
    write(Stream, ')').
writeT_compound_(Stream, F(X)) :-
    X == '$none_',
    !,
    writeTerm_(Stream, F),
    write(Stream, '()').

writeT_compound_(Stream, percept(Ps)) :-
    !,
    write(Stream, 'percept '),
    writeT_args_(Stream, Ps, ', ').
%% Next 2 are used in writing terms where variables are replaced
%% by their names
writeT_compound_(Stream, '$$var$$'(T)) :-
    atom(T),
    !,
    write(Stream, T).
writeT_compound_(Stream, '$$var$$'(T)) :-
    !,
    writeTerm_(Stream, T).
writeT_compound_(Stream, to(T1, T2)) :-
    !,
    writeTerm_(Stream, T1),
    write(Stream, ' to '),
    writeTerm_(Stream, T2).
writeT_compound_(Stream, from(T1, T2)) :-
    !,
    writeTerm_(Stream, T1),
    write(Stream, ' from '),
    writeTerm_(Stream, T2).
% writeT_compound_(Stream, fork_as(T1, T2)) :-
%     !,
%     write(Stream, 'fork '),
%     writeTerm_(Stream, T1),
%     write(Stream, ' as '),
%     writeTerm_(Stream, T2).
writeT_compound_(Stream, (type T)) :-
    !,
    write(Stream, 'type '),
    writeTerm_(Stream, T).
writeT_compound_(Stream, (T1 & T2)) :-
    !,
    writeTerm_(Stream, T1),
    write(Stream, ' & '),
    writeTerm_(Stream, T2).
writeT_compound_(Stream, (T1 ; T2)) :-
    !,
    writeTerm_(Stream, T1),
    write(Stream, ' ; '),
    writeTerm_(Stream, T2).
writeT_compound_(Stream, (T1 , T2)) :-
    !,
    writeTerm_(Stream, T1),
    write(Stream, ', '),
    writeTerm_(Stream, T2).
writeT_compound_(Stream, (T1 .. T2)) :-
    !,
    writeTerm_(Stream, T1),
    write(Stream, ' .. '),
    writeTerm_(Stream, T2).
writeT_compound_(Stream, ('$tuple_type'(Tuple) -> T2)) :-
    !,
    write(Stream, fun),
    
    writeTerm_(Stream, '$tuple_type'(Tuple)),
    write(Stream, ' -> '),
    writeTerm_(Stream, T2).
writeT_compound_(Stream, (T1 -> T2)) :-
    !,
    writeTerm_(Stream, T1),
    write(Stream, ' -> '),
    writeTerm_(Stream, T2).
writeT_compound_(Stream, (T1 | T2)) :-
    !,
    writeTerm_(Stream, T1),
    write(Stream, ' | '),
    writeTerm_(Stream, T2).
writeT_compound_(Stream, (A == B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' == '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, (tel_action_term ::= Types)) :-
    !,
    '$write_tel_action'(Stream, Types).
writeT_compound_(Stream, (A ::= B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' ::= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, global_int_decl(A,B)) :-
    !,
    write(Stream, 'int '),
    writeTerm_(Stream, A),
    write(Stream, ' := '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, global_num_decl(A,B)) :-
    !,
    write(Stream, 'num '),
    writeTerm_(Stream, A),
    write(Stream, ' := '),
    writeTerm_(Stream, B).    
writeT_compound_(Stream, A > B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' > '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A >= B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' >= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A < B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' < '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A =< B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' =< '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A = B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' = '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A \= B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' \\= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A @> B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' @> '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A @>= B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, '@ >= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A @< B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' @< '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A @=< B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' @=< '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A @= B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' @= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A '=?' B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' =? '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, '=@'(A,B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' =@ '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A $= B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' $= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, A in B) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' in '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, '$list_constr_goal'(_, A, B)) :-
    !,
    writeT_compound_(Stream, '$list_constr'(A, B)).
writeT_compound_(Stream, '$list_constr'(A, B)) :-
    !,
    write(Stream, '['),
    writeTerm_(Stream, A),
    write(Stream, ' :: '),
    writeTerm_(Stream, B),
    write(Stream, ']').
writeT_compound_(Stream, '$set_constr'(A, B)) :-
    !,
    write(Stream, '{'),
    writeTerm_(Stream, A),
    write(Stream, ' :: '),
    writeTerm_(Stream, B),
    write(Stream, '}').
writeT_compound_(Stream, default(A, B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' default '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, start_drive(A,B)) :-
    !,
    write(Stream, start_drive),
    write(Stream, ' '),
    writeTerm_(Stream, A),
    write(Stream, ' '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, '$pfindall'(A,B,C)) :-
    !,
    write(Stream, ' listof('),
    writeTerm_(Stream, A),
    write(Stream, ', '),
    writeTerm_(Stream, B),
    write(Stream, ' :: '),
    writeTerm_(Stream, C),
    write(Stream, ')').
writeT_compound_(Stream, '$exists'(V, G)) :-
    G = '&'(_, _),
    !,
    write(Stream, 'exists '),
    writeT_varlist_(Stream, V),
    write(Stream, ' ('),
    writeTerm_(Stream, G),
    write(Stream, ')').
writeT_compound_(Stream, '$exists'(V, G)) :-
    !,
    write(Stream, 'exists '),
    writeT_varlist_(Stream, V),
    write(Stream, ' '),
    writeTerm_(Stream, G).
writeT_compound_(Stream, '$forall'(V, G1, G2)) :-
    !,
    write(Stream, 'forall '),
    writeT_varlist_(Stream, V),
    write(Stream, ' ('),
    writeTerm_(Stream, G1),
    write(Stream, ' => '),
    writeTerm_(Stream, G2),
    write(Stream, ')').
writeT_compound_(Stream, '$forall_actions'(V, G1, G2)) :-
    !,
    write(Stream, 'forall '),
    writeT_varlist_(Stream, V),
    write(Stream, ' {'),
    writeTerm_(Stream, G1),
    write(Stream, ' ~> '),
    writeTerm_(Stream, G2),
    write(Stream, '}').

writeT_compound_(Stream, '$type_macro'(Type)) :-
    !,
    writeTerm_(Stream, Type).
writeT_compound_(Stream, '$union_type'(Args)) :-
    !,
    writeT_args_(Stream, Args, ' || ').
writeT_compound_(Stream, '$enum_type'([A])) :-
    !,
    writeTerm_(Stream, A).
writeT_compound_(Stream, '$enum_type'(Args)) :-
    !,
    writeT_args_(Stream, Args, ' | ').
writeT_compound_(Stream, '$constr_enum_type'([A])) :-
    !,
    writeTerm_(Stream, A).
writeT_compound_(Stream, '$enum_type'(Args)) :-
    !,
    writeT_args_(Stream, Args, ' | ').
writeT_compound_(Stream, '$constr_enum_type'(Args)) :-
    !,
    writeT_args_(Stream, Args, ' | ').
writeT_compound_(Stream, '@'(A, B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' @ '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, ':='(A, B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' := '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, '+:='(A, B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' +:= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, '-:='(A, B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' -:= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, '$'(A)) :-
    !,
    write(Stream, '$'),
    writeTerm_(Stream, A).
writeT_compound_(Stream, (A <= B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' <= '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, dyn(A)) :-
    A = '$tuple_type'(_),
    !,
    write(Stream, 'dyn'),
    writeTerm_(Stream, A).
writeT_compound_(Stream, for(A)) :-
    A = '$tuple_type'(_),
    !,
    write(Stream, 'for'),
    writeTerm_(Stream, A).
writeT_compound_(Stream, rel(A)) :-
    A = '$tuple_type'(_),
    !,
    write(Stream, 'rel'),
    writeTerm_(Stream, A).
writeT_compound_(Stream, act(A)) :-
    A = '$tuple_type'(_),
    !,
    write(Stream, 'act'),
    writeTerm_(Stream, A).
writeT_compound_(Stream, tel(A)) :-
    A = '$tuple_type'(_),
    !,
    write(Stream, 'tel'),
    writeTerm_(Stream, A).
writeT_compound_(Stream, (A :: B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' :: '),
    (
      '$not_colon_term_needs_brackets'(B)
    ->
      writeTerm_(Stream, B)
    ;
      write(Stream, '('),
      writeTerm_(Stream, B),
      write(Stream, ')')
    ).
writeT_compound_(Stream, (::(A))) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' ::').
writeT_compound_(Stream, '$set'(A, B)) :-
    !,
    write(Stream, '{'),
    writeTerm_(Stream, A),
    write(Stream, ' :: '),
    writeTerm_(Stream, B),
    write(Stream, '}').
writeT_compound_(Stream, (A .. B)) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, A),
    write(Stream, ' .. '),
    writeTerm_(Stream, B),
    write(Stream, ')').
writeT_compound_(Stream, set(A)) :-
    !,
    write(Stream, 'set('),
    writeTerm_(Stream, A),
    write(Stream, ')').
writeT_compound_(Stream, list(A)) :-
    !,
    write(Stream, 'list('),
    writeTerm_(Stream, A),
    write(Stream, ')').
writeT_compound_(Stream, (# A)) :-
    !,
    write(Stream, '#'),
    writeTerm_(Stream, A).
writeT_compound_(Stream, in(A,B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' in '),
    writeTerm_(Stream, B).

writeT_compound_(Stream, (A <> B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' <> '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, (A ++ B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' ++ '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, (A <>? B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' <>? '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, (A ++? B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' ++? '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, -A) :-
    !,
    write(Stream, '-'),
    writeT_wrap_arith_(Stream, A).
writeT_compound_(Stream, '^'(A)) :-
    !,
    write(Stream, '^'),
    writeAnnotatedTerm_(Stream, A).
writeT_compound_(Stream, ?A) :-
    !,
    write(Stream, '?'),
    writeAnnotatedTerm_(Stream, A).
writeT_compound_(Stream, '!?'(A)) :-
    !,
    write(Stream, '!?'),
    writeAnnotatedTerm_(Stream, A).
writeT_compound_(Stream, '@'(T)) :-
    !,
    write(Stream, '@'),
    writeAnnotatedTerm_(Stream, T).
writeT_compound_(Stream, '!'(A)) :-
    !,
    write(Stream, '!'),
    writeAnnotatedTerm_(Stream, A).
writeT_compound_(Stream, '??'(A)) :-
    !,
    write(Stream, '??'),
    writeTerm_(Stream, A).
writeT_compound_(Stream, '!??'(A)) :-
    !,
    write(Stream, '!??'),
    writeAnnotatedTerm_(Stream, A).
writeT_compound_(Stream, '+?'(A)) :-
    !,
    writeAnnotatedTerm_(Stream, A),
    write(Stream, '?').
writeT_compound_(Stream, +A) :-
    !,
    write(Stream, '+'),
    writeT_wrap_arith_(Stream, A).
writeT_compound_(Stream, (A <=)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' <=').

writeT_compound_(Stream, inter(A, B)) :-
    !,
    writeT_wrap_sets_(Stream, A),
    write(Stream, ' inter '),
    writeT_wrap_sets_(Stream, B).
writeT_compound_(Stream, union(A, B)) :-
    !,
    writeT_union_diff1_(Stream, A),
    write(Stream, ' union '),
    writeT_union_diff2_(Stream, B).
writeT_compound_(Stream, diff(A, B)) :-
    !,
    writeT_union_diff1_(Stream, A),
    write(Stream, ' diff '),
    writeT_union_diff2_(Stream, B).
    
writeT_compound_(Stream, (A ** B)) :-
    !,
    writeT_wrap_arith_(Stream, A),
    write(Stream, ' ** '),
    writeT_wrap_arith_(Stream, B).
writeT_compound_(Stream, (A :: B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' :: '),
    (
      '$has_operator'(B)
    ->
      write(Stream, '('),
      writeTerm_(Stream, B),
      write(Stream, ')')
    ;
      writeTerm_(Stream, B)
    ).
writeT_compound_(Stream, (A : B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' : '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, (A * B)) :-
    !,
    writeT_mult_div1_(Stream, A),
    write(Stream, ' * '),
    writeT_mult_div2_(Stream, B).
writeT_compound_(Stream, (A / B)) :-
    !,
    writeT_mult_div1_(Stream, A),
    write(Stream, ' / '),
    writeT_mult_div2_(Stream, B).
writeT_compound_(Stream, (A + B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' + '),
    writeTerm_(Stream, B).
writeT_compound_(Stream, (A - B)) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, ' - '),
    writeT_minus2_(Stream, B).
writeT_compound_(Stream, A) :-
    A = '$tuple'([]),
    !,
    write(Stream, '()').
writeT_compound_(Stream, A) :-
    A = '$tuple'(Args),
    !,
    write(Stream, '('),
    writeT_args_(Stream, Args, ', '),
    write(Stream, ')').
writeT_compound_(Stream, A) :-
    A = '$tuple_type'([]),
    !,
    write(Stream, '()').
writeT_compound_(Stream, A) :-
    A = '$tuple_type'(Args),
    !,
    write(Stream, '('),
    writeT_args_(Stream, Args, ', '),
    write(Stream, ')').
writeT_compound_(Stream, A) :-
    A = atom_naming(T), T = [T1],
    !,
    write(Stream, 'atom_naming('),
    writeTerm_(T1),
    write(Stream, ')').
writeT_compound_(Stream, A) :-
    A = atom_naming(T), T = alt_types([T1]),
    !,
    write(Stream, 'atom_naming('),
    writeTerm_(T1),
    write(Stream, ')').
writeT_compound_(Stream, A) :-
    A = atom_naming(alt_types(T)),
    !,
    write(Stream, 'atom_naming('),
    writeT_alt_types_(Stream, T),
    write(Stream, ')').
    %writeTerm_(Stream, T).
writeT_compound_(Stream, A) :-
    A = term_naming(T), T = [T1],
    !,
    write(Stream, 'term_naming('),
    writeTerm_(T1),
    write(Stream, ')').
writeT_compound_(Stream, A) :-
    A = term_naming(T), T = alt_types([T1]),
    !,
    write(Stream, 'term_naming('),
    writeTerm_(T1),
    write(Stream, ')').
writeT_compound_(Stream, A) :-
    A = term_naming(alt_types(T)),
    !,
    write(Stream, 'term_naming('),
    writeT_alt_types_(Stream, T),
    write(Stream, ')').
    %writeTerm_(Stream, T).

writeT_compound_(Stream, A) :-
    A = alt_types(Args),
    !,
    writeT_args_(Stream, Args, ', ').
writeT_compound_(Stream, '$tel'(Head, Rules)) :-
    !,
    writeTerm_(Stream, Head),
    write(Stream, ' {'), nl(Stream),
    write_tr_rules_(Stream, Rules),
    write(Stream, '    }'), nl(Stream).
writeT_compound_(Stream, Rule) :-
    Rule = 'tr~>'(_, _), !,
    write_a_tr_rule_(Stream, Rule).
writeT_compound_(Stream, robotic_action(D)) :-
    !,
    write(Stream, 'robotic_action '),
    (
     list(D)
    ->
     writeT_args_(Stream, D, ', ')
    ;
     writeTerm_(Stream, D)
    ).
writeT_compound_(Stream, tel_start(D)) :-
    !,
    write(Stream, 'tel_start '),
    (
     list(D)
    ->
     writeT_args_(Stream, D, ', ')
    ;
     writeTerm_(Stream, D)
    ).
writeT_compound_(Stream, tel_atomic(D)) :-
    !,
    writeT_args_(Stream, D, ', ').


writeT_compound_(Stream, A) :-
    write_a_tr_action_(Stream, A).


writeT_compound_(Stream, A) :-
    A =.. [F|Args],
    writeTerm_(Stream, F),
    write(Stream, '('),
    writeT_args_(Stream, Args, ', '),
    write(Stream, ')').

write_tr_rules_(_Stream, []).
write_tr_rules_(Stream, [R|Rs]) :-
    write_a_tr_rule_(Stream, R),
    nl(Stream),
    write_tr_rules_(Stream, Rs).

write_a_tr_rule_(Stream, 'tr~>'(LHS, '++'(['$nil'], B))) :-
    !,
    tab(Stream, 4),
    write_a_tr_LHS_(Stream, LHS),
    write(Stream, ' ~+> '),
    write_agent_action_(Stream, B).
write_a_tr_rule_(Stream, 'tr~>'(LHS, RHS)) :-
    tab(Stream, 4),
    write_a_tr_LHS_(Stream, LHS),
    write(Stream, ' ~> '),
    nl(Stream),
    tab(Stream, 12),
    write_a_tr_RHS_(Stream, RHS).


write_a_tr_LHS_(Stream, '$while_inhibit'(OP, G, '$time'(_OPT, G1,T))) :-
    !,
    writeTerm_(Stream, G),
    write_term_list(Stream, [' ', OP, ' ']),
    writeTerm_(Stream, G1),
    write_term_list(Stream, [' min_time ']),
    writeTerm_(Stream, T).
% write_a_tr_LHS_(Stream, '$while_inhibit'(min_time, G, '$time'(_OPT, T))) :-
%     !,
%     writeTerm_(Stream, G),
%     write_term_list(Stream, [' min_time ']),
%     writeTerm_(Stream, T).
write_a_tr_LHS_(Stream, '$while_inhibit'(OP, G, '$time'(_OPT, T))) :-
    !,
    writeTerm_(Stream, G),
    write_term_list(Stream, [' ', OP, ' min_time ']),
    writeTerm_(Stream, T).
write_a_tr_LHS_(Stream, '$while_inhibit'(OP, G, G1)) :-
    !,
    writeTerm_(Stream, G),
    write_term_list(Stream, [' ', OP, ' ']),
    writeTerm_(Stream, G1).
write_a_tr_LHS_(Stream, '$time'(_OP, G, T)) :-
    !,
    writeTerm_(Stream, G),
    write_term_list(Stream, [' min_time ']),
    writeTerm_(Stream, T).
write_a_tr_LHS_(Stream, '$while'(_OP, G, '$time'(_OPT, W, T))) :-
    !,
    writeTerm_(Stream, G),
    write_term_list(Stream, [' or_while ']),
    writeTerm_(Stream, W),
    write_term_list(Stream, [' min_time ']),
    writeTerm_(Stream, T).
write_a_tr_LHS_(Stream, '$while'(_OP, G, '$time'(_OPT, T))) :-
    !,
    writeTerm_(Stream, G),
    write_term_list(Stream, [' or_while min_time ']),
    writeTerm_(Stream, T).
write_a_tr_LHS_(Stream, '$while'(_OP, G, A)) :-
    !,
    writeTerm_(Stream, G),
    write_term_list(Stream, [' or_while ']),
    writeTerm_(Stream, A).
write_a_tr_LHS_(Stream, '$inhibit'(_OP, G, A)) :-
    !,
    write_a_tr_LHS_(Stream, G),
    write_term_list(Stream, [' commit_while ']),
    write_a_tr_inhibit_(Stream, A).
write_a_tr_LHS_(Stream, G) :-
    !,
    writeTerm_(Stream, G).


write_a_tr_inhibit_(Stream, '$time'(_OP, U, T)) :-
    !,
    writeTerm_(Stream, U), 
    write_term_list(Stream, [' min_time ']),
    writeTerm_(Stream, T).
write_a_tr_inhibit_(Stream, '$time'(_OP, T)) :-
    !,
    write_term_list(Stream, ['min_time ']),
    writeTerm_(Stream, T).
write_a_tr_inhibit_(Stream, U) :-
    writeTerm_(Stream, U).

write_a_tr_RHS_(Stream, '++'(A,B)) :-
    !,
    write_a_tr_action_(Stream, A),
    write(Stream, ' ++ '),
    write_agent_action_(Stream, B).

write_a_tr_RHS_(Stream, A) :-
    write_a_tr_action_(Stream, A).


write_agent_action_(Stream, As) :-
    As = (_ ; _), !,
    %writeL_(Stream, [nl_]),
    %tab(Stream, 16),
    writeL_(Stream, ["{", nl_]),
    show_body_(As, 16, Stream),
    nl(Stream),
    tab(Stream, 16),
    write(Stream, '}').
write_agent_action_(Stream, A) :-
    !,
    writeL_(Stream, [nl_]),
    tab(Stream, 12),
    writeTerm_(Stream, A).
    
write_a_tr_action_(Stream, timed_seq(TS)) :-
    !,
    write(Stream, '['),
    write_a_tr_timed_seq_(Stream, TS),
    write(Stream, ']').
write_a_tr_action_(_Stream, []).
write_a_tr_action_(Stream, [A|As]) :-
    writeTerm_(Stream, A),
    (
      As = []
    ->
      true
    ;
      write(Stream, ', ')
    ),
    write_a_tr_action_(Stream, As).

write_a_tr_timed_seq_(_Stream, []).
write_a_tr_timed_seq_(Stream, [A|As]) :-
    (
      A = '$time'(_OP, A1, T)
    ->
      write_a_tr_action_(Stream, A1),
      write(Stream, ':'),
      writeTerm_(Stream, T)
    ;
      write_a_tr_action_(Stream, A)
    ),
    (
      As = []
    ->
      true
    ;
      write(Stream, ', ')
    ),
    write_a_tr_timed_seq_(Stream, As).
    
writeT_args_(Stream, [A], _Op) :-
    !,
    writeTerm_(Stream, A).
writeT_args_(Stream, [A|T], Op) :-
    !,
    writeTerm_(Stream, A),
    write(Stream, Op),
    writeT_args_(Stream, T, Op).
    
writeAnnotatedTerm_(Stream, A) :-
    var(A), !,
    writeTerm_(Stream, A).
writeAnnotatedTerm_(Stream, A) :-
    compound(A),
    ( A = '<='(_) ; A = (_ -> _)), !,
    write(Stream, '('),
    writeTerm_(Stream, A),
    write(Stream, ')').
writeAnnotatedTerm_(Stream, A) :-
    writeTerm_(Stream, A).

writeT_mult_div1_(Stream, (A+B)) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, (A+B)),
    write(Stream, ')').
writeT_mult_div1_(Stream, (A-B)) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, (A-B)),
    write(Stream, ')').
writeT_mult_div1_(Stream, A) :-
    writeTerm_(Stream, A).

writeT_mult_div2_(Stream, (A+B)) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, (A+B)),
    write(Stream, ')').
writeT_mult_div2_(Stream, (A-B)) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, (A-B)),
    write(Stream, ')').
writeT_mult_div2_(Stream, (A*B)) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, (A*B)),
    write(Stream, ')').
writeT_mult_div2_(Stream, (A/B)) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, (A/B)),
    write(Stream, ')').
writeT_mult_div2_(Stream, A) :-
    writeTerm_(Stream, A).
    
writeT_minus2_(Stream, A+B) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, (A+B)),
    write(Stream, ')').
writeT_minus2_(Stream, A-B) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, (A-B)),
    write(Stream, ')').
writeT_minus2_(Stream, A) :-
    writeTerm_(Stream, A).

writeT_wrap_arith_(Stream, A) :-
    arith_(A),
    !,
    write(Stream, '('),
    writeTerm_(Stream, A),
    write(Stream, ')').
writeT_wrap_arith_(Stream, A) :-
    writeTerm_(Stream, A).

arith_((_ + _)).
arith_((_ - _)).
arith_((_ * _)).
arith_((_ / _)).
arith_((_ ** _)).

writeT_wrap_sets_(Stream, A) :-
    setop_(A), !,
    write(Stream, '('),
    writeTerm_(Stream, A),
    write(Stream, ')').
writeT_wrap_sets_(Stream, A) :-
    writeTerm_(Stream, A).
    
%setop_(inter(_, _)).
setop_(union(_, _)).
setop_(diff(_, _)).


writeT_union_diff1_(Stream, A) :-
    writeTerm_(Stream, A).
writeT_union_diff2_(Stream, diff(A,B)) :-
    !,
    write(Stream, '('),
    writeTerm_(Stream, A),
    write(Stream, ' diff '),
    writeTerm_(Stream, B),
    write(Stream, ')').
writeT_union_diff2_(Stream, A) :-
    writeTerm_(Stream, A).

'$write_code_decl'(Stream, '$code_decl'(Kind, Decl, Doc)) :-
    writeTerm_(Stream, Kind),
    write(Stream, ' '),
    '$write_decls'(Stream, Decl),
    (
      Doc = ""
    ->
      true
    ;
      nl(Stream),
      write(Stream, Doc)
    ).

'$write_decls'(Stream, ['$decl'(F, '$tuple_type'(Types))]) :-
    !,
    Term '@=..' [F|Types],
    writeTerm_(Stream, Term).
'$write_decls'(Stream, ['$decl'(F, ('$tuple_type'(Types) -> RT))]) :-
    !,
    Term '@=..' [F|Types],
    writeTerm_(Stream, (Term -> RT)).
'$write_decls'(Stream, ['$decl'(F, '$tuple_type'(Types))|Decl]) :-
    !,
    Term '@=..' [F|Types],
    writeTerm_(Stream, Term),
    write(Stream, ', '),
    '$write_decls'(Stream, Decl).
'$write_decls'(Stream, ['$decl'(F, ('$tuple_type'(Types) -> RT))|Decl]) :-
    !,
    Term '@=..' [F|Types],
    writeTerm_(Stream, (Term -> RT)),
    write(Stream, ', '),
    '$write_decls'(Stream, Decl).

%%writeT_alt_types_(S, T) :- errornl(writeT_alt_types_(S, T)),fail.
writeT_alt_types_(Stream, [H1, H2|T]) :-
    !,
    writeTerm_(Stream, H1),
    write(Stream, ' && '),
    writeT_alt_types_(Stream, [H2|T]).
writeT_alt_types_(Stream, [H1]) :-
    writeTerm_(Stream, H1).


writeT_varlist_(Stream, [H1, H2|T]) :-
    !,
    writeTerm_(Stream, H1),
    write(Stream, ', '),
    writeT_varlist_(Stream, [H2|T]).
writeT_varlist_(Stream, [H1]) :-
    writeTerm_(Stream, H1).

writeL_(List) :-
    writeL_(stdout, List).

%%writeL_(_Stream, L) :- errornl(w_________(L)), fail.
writeL_(_Stream, []) :- !.
%% Write a variable
writeL_(Stream, [T|Rest]) :-
    var(T), !,
    write(Stream, T),
    writeL_(Stream, Rest).
%% Write newline
writeL_(Stream, [nl_|Rest]) :-
    !,
    nl(Stream),
    writeL_(Stream, Rest).
%% Write N spaces
writeL_(Stream, [sp_(N)|Rest]) :-
    !,
    tab(Stream, N),
    writeL_(Stream, Rest).
%% Write atom with quotes if required
writeL_(Stream, [T|Rest]) :-
    atom(T), !,
    writeTerm_(Stream, T),
    writeL_(Stream, Rest).
%% Write string witout double quotes
writeL_(Stream, [T|Rest]) :-
    string(T), !,
    write_string(Stream, T),
    writeL_(Stream, Rest).
%% Quote string
writeL_(Stream, [q_(T)|Rest]) :-
    !,
    writeq(Stream, T),
    writeL_(Stream, Rest).
%% Don't quote atom
writeL_(Stream, [uq_(T)|Rest]) :-
    !,
    write(Stream, T),
    writeL_(Stream, Rest).
%% Name all the variables in T
writeL_(Stream, [wr_(T)|Rest]) :-
    !,
    name_vars(T),
    writeTerm_(Stream, T),
    writeL_(Stream, Rest).
%% pretty print T as a rule
writeL_(Stream, [show_(T)|Rest]) :-
    !,
    show_clause_(T, Stream),
    writeL_(Stream, Rest).
%% default print
writeL_(Stream, [X|Rest]) :-
    writeTerm_(Stream, X),
    writeL_(Stream, Rest).

