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

%% For stypes, types and show

?- compile_time_only(consult(op_decls)).

%% Display all user types

types :-
    '$user_type_info'(F, defined_type, T, DT, Vars),
    F \= tel_action_term,
     Defn = '::='(T,DT),
    '$vars2names'(Defn, Vars),
    write('def '),
    writeTerm_(Defn), nl, fail.
types :-
    '$user_type_info'(_, macro_type, T, DT, Vars),
     Defn = '=='(T,DT),
    '$vars2names'(Defn, Vars),
    write('def '),
    writeTerm_(Defn), nl, fail.
types :-
    findall((F:K), ('$user_type_info'(F, K, _, _, _),
                       K \= defined_type,
                       K \= macro_type,
                       F \= tel_action_term,
                       \+'$should_suppress_show_type'(F)), Fs),
    sort(Fs, SFs),
    member((A:_Kind), SFs),
    display_type_desc_(stdout, A), nl,fail.
types :- nl.

%% Display all system types
stypes :-
    findall(F,  ('$builtin_type_info'(F, defined_type, T, _DT, _Vars),
                    \+'$supress_defined_type'(T)
                ), Fs),
    sort(Fs, SFs),
    member(F, SFs),
    '$builtin_type_info'(F, defined_type, T, DT, Vars),
     Defn = '::='(T,DT),
     nl,
    '$vars2names'(Defn, Vars),
    write('def '),
    writeTerm_(Defn), 
     '$display_type_comment'(stdout, T), 
     nl,
     fail.
stypes :-
    findall(F,  ('$builtin_type_info'(F, macro_type, T, _DT, _Vars),
                    \+'$supress_defined_type'(T)
                ), Fs),
    sort(Fs, SFs),
    member(F, SFs),
    '$builtin_type_info'(F, macro_type, T, DT, Vars),
    \+'$supress_defined_type'(T) ,
     Defn = '=='(T,DT),
    nl,
    '$vars2names'(Defn, Vars),
    write('def '),
    writeTerm_(Defn), 
     '$display_type_comment'(stdout, T), 
     nl,
     fail.
stypes :-
    findall((F:K), ('$builtin_type_info'(F, K, _, _, _),
                       K \= defined_type,
                       K \= macro_type,
                       \+'$supress_builtin_type'(F)
                   ), Fs),
    sort(Fs, SFs),
    member((A:_Kind), SFs),
    nl,
    display_type_desc_(stdout, A), nl,fail.
stypes :- nl.


%% Display all types for given names
types(PList) :-
    list(PList), !,
    forall(member(P, PList), types(P)).

%% Display all types for all names that start with the given sub-name
types(PP) :-
    findall(P, ('$user_type_info'(P, defined_type, _, _, _),
                      (P==PP; once(sub_atom(P,_,_,PP)))), AllP),
    sort(AllP, SAllP),
    member(Nm, SAllP),
    '$user_type_info'(Nm, defined_type, T, DT, Vars),
    Defn = '::='(T,DT),
    '$vars2names'(Defn, Vars),
    write('def '),
    writeTerm_(Defn), 
    '$display_type_comment'(stdout, T), nl, fail.
types(PP) :-
    findall(P, ('$user_type_info'(P, macro_type, _, _, _),
                      (P==PP; once(sub_atom(P,_,_,PP)))), AllP),
    sort(AllP, SAllP),
    member(Nm, SAllP),
    '$user_type_info'(Nm, macro_type, T, DT, Vars),
    Defn = '=='(T,DT),
    '$vars2names'(Defn, Vars),
    write('def '),
    writeTerm_(Defn), 
    '$display_type_comment'(stdout, T), nl, fail.
types(PP) :-
    findall(P, ('$user_type_info'(P, K, _, _, _),
                   K \= defined_type,
                   K \= macro_type,
                   P \= tel_action_term,
                   (P==PP; once(sub_atom(P,_,_,PP)))), AllP),
    sort(AllP, SAllP),
    member(Nm, SAllP),
    display_type_desc_(stdout, Nm),
    nl,
    fail.
types(_) :- nl.

stypes(PList) :-
    list(PList), !,
    forall(member(P, PList), stypes(P)).

stypes(PP) :-
    findall(P, ('$builtin_type_info'(P, defined_type, _, _, _),
                      (P==PP; once(sub_atom(P,_,_,PP)))), AllP),
    sort(AllP, SAllP),
    member(Nm, SAllP),
    '$builtin_type_info'(Nm, defined_type, T, DT, Vars),
    \+'$supress_defined_type'(T) ,
     Defn = '::='(T,DT),
    '$vars2names'(Defn, Vars),
    nl,
    write('def '),
    writeTerm_(Defn), 
     '$display_type_comment'(stdout, T), 
     nl,
     fail.
stypes(PP) :-
    findall(P, ('$builtin_type_info'(P, macro_type, _, _, _),
                      (P==PP; once(sub_atom(P,_,_,PP)))), AllP),
    sort(AllP, SAllP),
    member(Nm, SAllP),
    '$builtin_type_info'(Nm, macro_type, T, DT, Vars),
    \+'$supress_defined_type'(T) ,
     Defn = '=='(T,DT),
    '$vars2names'(Defn, Vars),
    nl,
    write('def '),
    writeTerm_(Defn), 
     '$display_type_comment'(stdout, T), 
     nl,
     fail.
stypes(PP) :-
    findall(P, ('$builtin_type_info'(P, K, _, _, _),
                   K \= defined_type,
                   K \= macro_type,
                   P \= tel_action_term,
                   (P==PP; once(sub_atom(P,_,_,PP)))), AllP),
    sort(AllP, SAllP),
    member(Nm, SAllP),
    nl,
    display_type_desc_(stdout, Nm),
    nl,
    fail.
stypes(_).

%% Extract and display the type description for given type
%%display_type_desc_(Stream, A) :- errornl(display_type_desc_(Stream, A)),fail.
display_type_desc_(Stream, A) :-
    '$state_belief'(A), !,
    '$user_type_info'(A, rel, ['?'(Type)], _, _),
    writeL_(Stream, [Type, " ", A]).
display_type_desc_(Stream, P) :-
    once('$type_descr'(P, _, Kind, Doc)),
    Kind \= defined_type,
    findall(Type,
            ('$type_descr'(P, Type, _, _)), AllT),
    (
      AllT = [T]
    ->
      writeL_(Stream, [Kind, " ", T])
    ;
      AllT = [T|Rest],
      writeL_(Stream, [Kind, " ", T, ",",nl_]),
      '$display_alt_types'(Stream, Rest)
    ),
    (
      Doc = ""
    ->
      true
    ;
      writeL_(Stream, [nl_, uq_('"'), Doc, uq_('"')])
    ).

%% When there are multiple declarations display the types (does not include
%% the first) on a new line tabbed in 4 spaces
'$display_alt_types'(Stream, [T]) :- !,
    tab(Stream, 4),
    writeL_(Stream, [T]).
'$display_alt_types'(Stream, [T|Rest]) :- !,
    tab(Stream, 4),
    writeL_(Stream, [T, ",", nl_]),
    '$display_alt_types'(Stream, Rest).


%% display type info for use in errors.ql
display_type_(Stream, P) :-
    once('$type_info'(P, K, _, _, _)),
    display_type_(K, Stream, P).

display_type_(rel, Stream, P) :-
    (
     '$belief_type_info'(P, _, _)
    ->
     Kind = dyn
    ;
     Kind = rel
    ),
    findall(Kind('$tuple_type'(T1)),
            ('$type_info'(P, _, T1, _, DV),'$vars2names'(T1, DV)), AllT),
    (
      AllT = [T]
    ->
      true
    ;
      T = alt_types(AllT)
    ),
    writeL_(Stream, [(P : T)]).
display_type_(act, Stream, P) :-
    findall(act('$tuple_type'(T1)),
            ('$type_info'(P, _, T1, _, DV),'$vars2names'(T1, DV)), AllT),
    (
      AllT = [T]
    ->
      true
    ;
      T = alt_types(AllT)
    ),
    writeL_(Stream, [(P : T)]).
display_type_(fun, Stream, P) :-
    findall(T1,
            ('$type_info'(P, _, T1, _, DV),'$vars2names'(T1, DV)), AllT),
    (
      AllT = [T]
    ->
      true
    ;
      T = alt_types(AllT)
    ),
    writeL_(Stream, [(P : T)]).
display_type_(tel, Stream, P) :-  
    '$type_info'(P, tel, T, _, DVars),
    '$vars2names'(T, DVars),
    strip_modes__(T, ST),
    writeL_(Stream, [(P : tel('$tuple_type'(ST)))]).

%% display the comment for a type if it has one
'$display_type_comment'(Stream, P) :-
    '$type_descr'(P, _Args, _, Comment), Comment \= "", !,
    writeL_(Stream, [nl_,uq_('"'), Comment, uq_('"')]).
'$display_type_comment'(_, _).


%% display the rules of a relation P
show_relation_clause_(P) :-
    findall(C, get_a_relation_clause_(P, C), Cs),
    (
      Cs  = []
    ->
      !
    ;
      display_type_desc_(stdout, P),
      %'$display_type_comment'(stdout, P) ,
      writeL_([nl_]),
      member(c(Rule, Vars), Cs),
      '$vars2names'(Rule, Vars),
      show_clause_(Rule),
      writeL_([nl_]),
      fail
    ).
show_relation_clause_(P) :-
    '$type_info'(P, rel, _, T, _),
    cache_info__(T, _, Rem, _),
    findall(T, call(Rem), Ps),
    (
      Ps = []
    ->
      writeL_(["% There are no current remembered instances."])
    ;
      writeL_(["% With current remembered instances:"]),
      forall(member(C, Ps), (writeL_([nl_]),show_clause_(C)))
    ),
    writeL_([nl_]),
    fail.      
show_relation_clause_(_P) :-
    writeL_([nl_]).

%% display the percept info for P
show_percept_clause_(P) :-
    findall(C, get_a_percept_clause_(P, C), Cs),
    (
      Cs = []
    ->
      !
    ;
     (
      '$type_descr'(P, Args, _), Args \= [_]
     ->
      FullP =.. Args
     ;
      FullP = P
     ),
     % '$percept_type_info'(P, Types, _),
     % writeL_(["percept ", FullP:'$tuple_type'(Types), nl_]),
     %'$display_type_comment'(stdout, P),
     %nl,
      member(c(Rule, Vars), Cs),
      '$vars2names'(Rule, Vars),
      show_clause_(Rule),
      writeL_([nl_]),
      fail
    ).
show_percept_clause_(_P) :-
    writeL_([nl_]).

%% display the beliefs for P
show_belief_clause_(P) :-
    '$state_belief'(P), !,
    '$user_type_info'(P, rel, ['?'(Type)], _, _),
    call(P(X)),
    writeL_([Type, " ", P, " := ", X, nl_]).
show_belief_clause_(P) :-
    findall(C, get_a_belief_clause_(P, C), Cs),
    display_type_desc_(stdout, P),
    writeL_([nl_]),
    (
      Cs = []
    ->
      !
    ;
     member(c(Rule, Vars), Cs),
      '$vars2names'(Rule, Vars),
      show_clause_(Rule),
      writeL_([nl_]),
      fail
    ).
show_belief_clause_(_P) :-
    writeL_([nl_]).

%% display the action rules for P
show_action_clause_(P) :-
    findall(C, get_a_action_clause_(P, C), Cs),
    (
      Cs = []
    ->
      !
    ;
      display_type_desc_(stdout, P),
      writeL_([nl_]),
      member(c(Rule, Vars), Cs),
      '$vars2names'(Rule, Vars),
      show_clause_(Rule),
      writeL_([nl_]),
      fail
    ).
show_action_clause_(_P) :-
    writeL_([nl_]).

%% display the function rules for P
show_function_clause_(P) :-
    findall(C, get_a_function_clause_(P, C), Cs),
    (
      Cs = []
    ->
      !
    ;
      display_type_desc_(stdout, P),
      writeL_([nl_]),
      member(c(Rule, Vars), Cs),
      '$vars2names'(Rule, Vars),
      show_clause_(Rule),
      writeL_([nl_]),
      fail
    ).
show_function_clause_(P) :-
    '$type_info'(P, _, ('$tuple_type'(Type) -> _), _, _),
    length(Type, N),
    N1 is N+1,
    functor(T, P, N1),
    cache_info__(T, _, Rem, _),
    T =.. [P|Args],
    append(FArgs, [Result], Args),
    Head '@=..' [P|FArgs],
    Rule = (Head -> Result),
    findall(Rule, call(Rem), Ps),
    (
      Ps = []
    ->
      writeL_(["% There are no current remembered instances."])
    ;
      writeL_(["% With current remembered instances:"]),
      forall(member(C, Ps), (writeL_([nl_]),show_clause_(C)))
    ),
    writeL_([nl_]),
    fail.      
show_function_clause_(_P) :-
    writeL_([nl_]).

%% display the TR rules for P
show_tel(P) :-
    '$type_info'(P, tel, _T, H, _DVars), !,
    ( TR = '$tel'(H, _R) ; TR = '$tel_cleanup'(H, _R, _) ),
    '$saved_input_with_vars'(TR, Vars, _),
    display_type_desc_(stdout, P),
    writeL_([nl_]),
    '$vars2names'(TR, Vars),
    writeTerm_(TR),
    writeL_([nl_]).

%% Display the percept declaration
show_percept :-
    findall(D, ('$belief_type_info'(_, _, D), '$percept'(D)), AllTypes),
    AllTypes = [E1|ETypes],
    !,
    write(tel_percept), write(' '), writeTerm_(E1),
    forall(member(E, ETypes), (write(', '), writeTerm_(E))),
    nl.
show_percept.
%% Display the tel_action declaration
show_tel_action :-
    '$user_type_info'(tel_action_term, _, _, Types, _),
    !,
    '$write_tel_action'(stdout, Types), nl.
show_tel_action.

'$write_tel_action'(Stream, Types) :-
    Types = '$constr_enum_type'([E1|ETypes]),
    write(Stream, tel_action), write(Stream, ' '),
    writeTerm_(Stream, E1),
    forall(member(E, ETypes), (write(Stream, ', '), writeTerm_(Stream, E))).
%% Show all the user declarations and definitions
show :-
    show_percept,fail.
show :-
    show_tel_action,fail.

show :-
    findall(PX, '$user_relation_type_info'(PX, _, _), PS),
    sort(PS, PSS),
    member(P, PSS),
    show_relation_clause_(P),
    fail.
show :-
    findall(PX, ('$belief_type_info'(PX, _, D), \+'$percept'(D)), PS),
    sort(PS, PSS),
    member(P, PSS),
    show_belief_clause_(P),
    fail.

show :-
    show_action_clause_(main),
    fail.
show :-
    findall(PX, '$user_action_type_info'(PX, _, _), PS),
    sort(PS, PSS),
    member(P, PSS),
    show_action_clause_(P),
    fail.
show :-
    findall(PX,
            ('$user_type_info'(PX, fun, _, T, _), T \= '$function'), PS),
    sort(PS, PSS),
    member(P, PSS),
    show_relation_clause_(P),
    fail.
show :-
    findall(PX, '$user_type_info'(PX, fun, _, '$function', _), PS),
    sort(PS, PSS),
    member(P, PSS),
    show_function_clause_(P),
    fail.
show :-
    findall(PX, '$user_type_info'(PX, tel, _, _, _), PS),
    sort(PS, PSS),
    member(P, PSS),
    show_tel(P),
    fail.

show.

%% show the declaration and definition for P (and only P)
%%show_exact_(P) :- errornl(show_exact_(P)), fail.
show_exact_(P) :-
    once('$user_relation_type_info'(P, _, _)),
    \+ '$belief_type_info'(P, _, _),
    show_relation_clause_(P),
    fail.
show_exact_(P) :-
    once('$belief_type_info'(P, _, _)),
    show_belief_clause_(P),
    fail.
show_exact_(P) :-
    once('$user_action_type_info'(P, _, _)),
    show_action_clause_(P),
    fail.
show_exact_(P) :-
    once('$user_type_info'(P, fun, _, T, _)), T \= '$function',
    show_relation_clause_(P),
    fail.

show_exact_(P) :-
    once('$user_type_info'(P, fun, _, '$function', _)),
    show_function_clause_(P),
    fail.
show_exact_(_P).

%% Show decls and defs for all in PList
show(PList) :-
    list(PList), !,
    forall(member(P, PList), show(P)).

%% Show decls and defs for all whose names start with PP
show(PP) :-
    findall(P, ('$user_relation_type_info'(P, _, _),
                   ( P==PP; once(sub_atom(P,1,_,PP) ))), Ps),
    sort(Ps, SPs),
    member(P, SPs),
    show_exact_(P),
    %nl,
    fail.

show(PP) :-
    findall(P, ('$belief_type_info'(P, _, _),
                   ( P==PP; once(sub_atom(P,1,_,PP) ))), Ps),
    sort(Ps, SPs),
    member(P, SPs),
    show_exact_(P),
    fail.

show(PP) :-
    once(( PP == main ; once(sub_atom(main, 1, _, PP)) )),
    show_action_clause_(main),
    nl,
    fail.
show(PP) :-
    findall(P, ('$user_action_type_info'(P, _, _),
                   ( P==PP; once(sub_atom(P,1,_,PP) ))), Ps),
    sort(Ps, SPs),
    member(P, SPs),
    show_exact_(P),
    nl,
    fail.
show(PP) :-
    findall(P, ('$user_type_info'(P, fun, _, _, _),
                   ( P==PP; once(sub_atom(P,1,_,PP) ))), Ps),
    sort(Ps, SPs),
    member(P, SPs),
    show_exact_(P),
    nl,
    fail.
show(PP) :-
    '$user_type_info'(P, tel, _, _, _),
    (
      (P==PP; once(sub_atom(P,1,_,PP)))
    ->
      show_tel(P),
      nl,
      fail
    ;
      fail
    ). 
show(_).

%% Pretty print rule C - should use same indentation as if done in emacs
show_clause_(C) :-
    show_clause_(C, stdout).

%%show_clause_(A, B) :- errornl(show_clause_(A, B)),fail.
show_clause_((Head :: Test <= Body), Stream) :-
    !,
    writeTerm_(Stream, Head),
    writeL_(Stream, [" ::", nl_]),
    show_body_(Test,4, Stream),
    writeL_(Stream, [" <=", nl_]),
    show_body_(Body,8, Stream).
    
show_clause_((Head <= Body), Stream) :-
    !,
    writeTerm_(Stream, Head),
    writeL_(Stream, [" <=", nl_]),
    show_body_(Body,4, Stream).

show_clause_((Head :: Test ~> Body), Stream) :-
    !,
    writeTerm_(Stream, Head),
    writeL_(Stream, [" ::", nl_]),
    show_body_(Test,4, Stream),
    writeL_(Stream, [" ~>", nl_]),
    show_body_(Body,8, Stream).
show_clause_('$tr'(Head, Body), Stream) :-
    !,
    show_clause_(Body, Stream),
    writeL_(Stream, [nl_, "of ", Head]).
show_clause_((Head ~> Body), Stream) :-
    !,
    writeTerm_(Stream, Head),
    writeL_(Stream, [" ~>", nl_]),
    show_body_(Body,4, Stream).
show_clause_('~>'(Head), Stream) :-
    !,
    writeTerm_(Stream, Head),
    writeL_([" ~>", nl_]).
show_clause_((Head :: Test), Stream) :-
     !,
    writeTerm_(Stream, Head),
    writeL_(Stream, [" ::", nl_]),
    show_body_(Test,4, Stream).

show_clause_((Head :: Test -> Body), Stream) :-
    !,
    writeTerm_(Stream, Head),
    writeL_(Stream, [" ::", nl_]),
    show_body_(Test,4, Stream), 
    writeL_(Stream, [" -> "]),
    show_cond_(Body,8, Stream).

show_clause_((Head -> Body), Stream) :-
    !,
    writeTerm_(Stream, Head),
    writeL_(Stream, [" -> "]),
    show_cond_(Body,8, Stream).

%% the IP '$show_cond' is set so that write TODO
show_clause_(Head, Stream) :-
    writeTerm_(Stream, Head).

show_clause_("->", SPHead,SPBody, Stream) :- !,
    writeL_(Stream, [(SPHead -> SPBody)]).

'$is_clause_defn'((_A :: _B <= _C)).
'$is_clause_defn'((_A :: _B ~> _C)).
%'$is_clause_defn'((_A :: _B)).
'$is_clause_defn'((_A <= _C)).
'$is_clause_defn'((_A  ~> _C)).

%% Pretty print body Cond - should use same indentation as if done in emacs
%%show_body_( Cond,Indent, Stream) :- errornl(show_body_( Cond,Indent, Stream)),fail.
show_body_( Cond,Indent, Stream) :-
    var(Cond), !,
    tab(Stream, Indent),
    writeL_([Cond]).
show_body_( Cond,Indent, Stream) :-
    Cond\= (_ & _),
    Cond\= (_ ; _),
    Cond\=(_,_),
    !,
    tab(Stream, Indent),
    show_cond_(Cond, Indent, Stream).
show_body_(And(Cond,Conds), Indent, Stream):-
    tab(Stream, Indent),
    show_cond_(Cond, Indent, Stream),
    (And='&' -> write(Stream, ' &\n')
    ;
     And=';' -> write(Stream, ' ;\n')
    ;
     write(Stream, ' ,\n')
    ),
    show_body_(Conds, Indent, Stream).

%% Pretty print except part of try-except
%%'$show_except_alt'(A,B,C,D) :- errornl('$show_except_alt'(A,B,C,D)),fail.
'$show_except_alt'('$catch'(Patt, '$empty', Act), Indent1, Indent2, Stream) :-
    !,
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, [Patt, " ~> "]),
    Indent3 is Indent2+4,
    (
     Act = (_;_)
    ->
     tab(Stream, Indent2),
     writeL_(Stream, ["{", nl_]),
     show_body_(Act, Indent3, Stream),
     nl(Stream),
     tab(Stream, Indent2),
     write(Stream, '}')
    ;
     show_cond_(Act, Indent2, Stream)
    ),
    nl(Stream).
'$show_except_alt'('$catch'(Patt, Test, Act), Indent1, Indent2, Stream) :-
    !,
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, [Patt, " :: "]),
    tab(Stream, Indent2),
    show_cond_(Test, Indent2, Stream),
    writeL_(Stream, [" ~> ", nl_]),
    Indent3 is Indent2+4,
    (
     Act = (_;_)
    ->
     tab(Stream, Indent2),
     writeL_(Stream, ["{", nl_]),
     show_body_(Act, Indent3, Stream),
     nl(Stream),
     tab(Stream, Indent2),
     write(Stream, '}')
    ;
     show_body_(Act, Indent2, Stream)
    ),
    nl(Stream).


'$show_case_alt'('$case_alt'(Test, Act), Indent1, Indent2, Stream) :-
    %%tab(Stream, Indent1),
    show_body_(Test, Indent1, Stream),
    writeL_(Stream, [" ~>", nl_]),
    show_body_(Act, Indent2, Stream),
    nl(Stream).
                   
%%'$show_receive_alt'(A, I1, I2, _) :- errornl('$show_receive_alt'(A,I1, I2)), fail.
'$show_receive_alt'('$normal'('$receive_from'(MP, from(AP), '$empty', Act)),
		    Indent1, Indent2, Stream) :-
    !,
    tab(Stream, Indent1),
    writeL_(Stream, [MP, " from ", AP, " ~>", nl_]),
    %Indent3 is Indent2+3,
     show_body_(Act, Indent2, Stream),
    nl(Stream).
'$show_receive_alt'('$normal'('$receive_from'(MP, from_thread(AP),
					      '$empty', Act)),
		    Indent1, Indent2, Stream) :-
    !,
    tab(Stream, Indent1),
    writeL_(Stream, [MP, " from_thread ", AP, " ~>", nl_]),
    %Indent3 is Indent2+3,
     show_body_(Act, Indent2, Stream),
    nl(Stream).
% '$show_receive_alt'('$normal'('$receive_from'(remote_query_(MP),
%                                               from_thread(AP),
% 					      type(MP, '!'(string)), Act)),
% 		    Indent1, Indent2, Stream) :-
%     !,
%     tab(Stream, Indent1),
%     writeL_(Stream, [MP, " query_from ", AP, " ~>", nl_]),
%     %Indent3 is Indent2+3,
%      show_body_(Act, Indent2, Stream),
%     nl(Stream).
% '$show_receive_alt'('$normal'('$receive_from'(remote_query_(MP),
%                                               from_thread(AP),
%                                               (type(MP, '!'(string)) & Test),
%                                               Act)),
% 		    Indent1, Indent2, Stream) :-
%     !,
%     tab(Stream, Indent1),
%     writeL_(Stream, [MP, " query_from ", AP, " :: ", nl_]),
%     tab(Stream, Indent2),
%     show_cond_(Test, Indent2, Stream),
%     writeL_(Stream, [" ~>", nl_]),
%     Indent3 is Indent2+4,
%     show_body_(Act, Indent3, Stream),
%     nl(Stream).

'$show_receive_alt'('$normal'('$receive_from'(MP, from(AP), Test, Act)),
		    Indent1, Indent2, Stream) :-
    !,
    tab(Stream, Indent1),
    writeL_(Stream, [MP, " from ", AP, " :: ", nl_]),
    tab(Stream, Indent2),
    show_cond_(Test, Indent2, Stream),
    writeL_(Stream, [" ~>", nl_]),
    Indent3 is Indent2+4,
    show_body_(Act, Indent3, Stream),
    nl(Stream).
'$show_receive_alt'('$normal'('$receive_from'(MP, from_thread(AP), Test, Act)),
		    Indent1, Indent2, Stream) :-
    !,
    tab(Stream, Indent1),
    writeL_(Stream, [MP, " from_thread ", AP, " :: ", nl_]),
    tab(Stream, Indent2),
    show_cond_(Test, Indent2, Stream),
    writeL_(Stream, [" ~>", nl_]),
    Indent3 is Indent2+4,
     show_body_(Act, Indent3, Stream),
    nl(Stream).

'$show_receive_alt'(timeout(T, Act),
		    Indent1, Indent2, Stream) :-
    !,
    tab(Stream, Indent1),
    writeL_(Stream, ["timeout ",T, " ~>", nl_]),
    %Indent3 is Indent2+3,
     show_body_(Act, Indent2, Stream),

     nl(Stream).

%%show_cond_(A,B,C) :- errornl(show_cond_(A,B,C)),fail.
show_cond_(atomic_action(Acts),Indent, Stream) :-
    compound(Acts), Acts = (_;_),
    !,
    %tab(Stream, Indent),
    writeL_(Stream, ["atomic_action {",nl_]),
    Indent1 is Indent+4,
    show_body_(Acts, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, ["}"]).
show_cond_(atomic_action(A),Indent, Stream) :-
    !,
    Indent1 is Indent+4,
    writeL_(Stream, ["atomic_action "]),
    show_cond_(A, Indent1, Stream).

show_cond_(repeat_until(Acts, '$fail'),Indent, Stream) :-
    compound(Acts), Acts = (_;_),
    !,
    Indent1 is Indent+4,
    writeL_(Stream, ["repeat {", nl_]),
    show_body_(Acts, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, ["}", nl_]).
show_cond_(repeat_until(Act, '$fail'),Indent, Stream) :-
    !,
    Indent1 is Indent+4,
    writeL_(Stream, ["repeat", nl_]),
    show_body_(Act, Indent1, Stream),
    nl(Stream).
show_cond_(repeat_until(Acts, Conds),Indent, Stream) :-
    compound(Acts), Acts = (_;_),
    compound(Conds), Conds = (_&_),
    !,
    Indent1 is Indent+4,
    writeL_(Stream, ["repeat {", nl_]),
    show_body_(Acts, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, ["}", nl_]),
    tab(Stream, Indent),
    writeL_(Stream, ["until (", nl_]),
    show_body_(Conds, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, [")", nl_]).
show_cond_(repeat_until(Acts, Conds),Indent, Stream) :-
    compound(Acts), Acts = (_;_),
    !,
    Indent1 is Indent+4,
    writeL_(Stream, ["repeat {", nl_]),
    show_body_(Acts, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, ["}", nl_]),
    tab(Stream, Indent),
    writeL_(Stream, ["until", nl_]),
    show_body_(Conds, Indent1, Stream),
    nl(Stream).
show_cond_(repeat_until(Acts, Conds),Indent, Stream) :-
    compound(Conds), Conds = (_&_),
    !,
    Indent1 is Indent+4,
    writeL_(Stream, ["repeat", nl_]),
    show_body_(Acts, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent),
    writeL_(Stream, ["until (", nl_]),
    show_body_(Conds, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, [")", nl_]).
show_cond_(repeat_until(Acts, Conds),Indent, Stream) :-
    !,
    Indent1 is Indent+4,
    writeL_(Stream, ["repeat", nl_]),
    show_body_(Acts, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent),
    writeL_(Stream, ["until", nl_]),
    show_body_(Conds, Indent1, Stream),
    nl(Stream).

     
show_cond_( try_except(Try, Excepts),Indent, Stream) :-
    !,
    Indent1 is Indent+4,
    Indent2 is Indent+8,
    writeL_(Stream, ["try {", nl_]),
    show_body_(Try, Indent1, Stream),
    nl(Stream),
    tab(Stream, Indent1),
    writeL_(Stream, ["}", nl_]),
    tab(Stream, Indent),
    writeL_(Stream, ["except {"]),
    forall(member(E, Excepts),
	   '$show_except_alt'(E, Indent1, Indent2, Stream)),
    tab(Stream, Indent1),
    writeL_(Stream, ["}"]).
show_cond_(wait(Goal),Indent, Stream) :-
    !,
    writeL_(Stream, ["wait("]),
    show_cond_(Goal, Indent, Stream),
    writeL_(Stream, [")"]).
show_cond_( wait_case(AllAlts),Indent, Stream) :-
    list(AllAlts), append(Alts, [timeout(T, A)], AllAlts), !,
    writeL_(Stream, ["wait_case {", nl_]),
    Indent1 is Indent+4,
    Indent2 is Indent+8,
    forall(member(Alt, Alts),
	   '$show_case_alt'(Alt, Indent1, Indent2, Stream)),
    tab(Stream, Indent1),
    writeL_(Stream, ["timeout ", T, " ~> ", nl_]),
    show_body_(A, Indent2, Stream),
    writeL_(Stream, [nl_]),
    tab(Stream, Indent1),
    writeL_(Stream, ["}"]).
show_cond_( wait_case(Alts),Indent, Stream) :-
    !,
    writeL_(Stream, ["wait_case {", nl_]),
    Indent1 is Indent+4,
    Indent2 is Indent+8,
    forall(member(Alt, Alts),
	   '$show_case_alt'(Alt, Indent1, Indent2, Stream)),
    tab(Stream, Indent1),
    writeL_(Stream, ["}"]).   
show_cond_( case(Alts),Indent, Stream) :-
    !,
    %tab(Stream, Indent),
    writeL_(Stream, ["case {", nl_]),
    Indent1 is Indent+4,
    Indent2 is Indent+8,
    forall(member(Alt, Alts),
	   '$show_case_alt'(Alt, Indent1, Indent2, Stream)),
    tab(Stream, Indent1),
    writeL_(Stream, ["}"]).
show_cond_( receive(Alts),Indent, Stream) :-
    !,
    %tab(Stream, Indent),
    writeL_(Stream, ["receive {", nl_]),
    Indent1 is Indent+4,
    Indent2 is Indent+8,
    forall(member(Alt, Alts),
	   '$show_receive_alt'(Alt, Indent1, Indent2, Stream)),
    tab(Stream, Indent1),
    writeL_(Stream, ["}"]).
show_cond_('$rel_escape'(A),_Indent, Stream) :-
    !,
    writeL_(Stream, ["?("]),
    writeTerm_(Stream, A),
    writeL_(Stream, [")"]).
show_cond_('$act_escape'(A),_Indent, Stream) :-
    !,
    writeL_(Stream, ["{"]),
    writeTerm_(Stream, A),
    writeL_(Stream, ["}"]).
show_cond_(from(M, A, '$true'(_)),_Indent, Stream) :-
    !,
    %tab(Stream, Indent),
    writeL_(Stream, [M, " from ", A]).
show_cond_(from(M, A, T),_Indent, Stream) :-
    !,
    %tab(Stream, Indent),
    writeL_(Stream, [M, " from ", A, " :: ", T]).
show_cond_(to(M, A),_Indent, Stream) :-
    !,
    %tab(Stream, Indent),
    writeL_(Stream, [M, " to ", A]).
% show_cond_(from_thread(remote_query_(M), A, type(M, '!'(string))),
%           _Indent, Stream) :-
%     !,
%     writeL_(Stream, [M, " query_from ", A]).
% show_cond_(from_thread(remote_query_(M), A, (type(M, '!'(string)) & Test)),
%                       _Indent, Stream) :-
%     !,
%     writeL_(Stream, [M, " query_from ", A, " :: ", Test]).
show_cond_(from_thread(M, A, '$true'(_)),_Indent, Stream) :-
    !,
    writeL_(Stream, [M, " from_thread ", A]).
show_cond_(from_thread(M, A, T),_Indent, Stream) :-
    !,
    writeL_(Stream, [M, " from_thread ", A, " :: ", T]).
show_cond_(to_thread(M, A),_Indent, Stream) :-
    !,
    %tab(Stream, Indent),
    writeL_(Stream, [M, " to_thread ", A]).
show_cond_('$forall'(V, G1, G2),Indent, Stream) :-
    !,
    write(Stream, 'forall '),
    writeT_varlist_(Stream, V),
    write(Stream, ' ('),
    nl(Stream),
    Indent1 is Indent+4,
    Indent2 is Indent+8,
    show_body_(G1, Indent1, Stream),
    write(Stream, ' => '),
    nl(Stream),
    show_body_(G2, Indent2, Stream),
    write(Stream, ')').
show_cond_('$forall_actions'(V, G1, G2),Indent, Stream) :-
    !,
    write(Stream, 'forall '),
    writeT_varlist_(Stream, V),
    write(Stream, ' {'),
    nl(Stream),
    Indent1 is Indent+4,
    Indent2 is Indent+8,
    show_body_(G1, Indent1, Stream),
    write(Stream, ' ~> '),
    nl(Stream),
    show_body_(G2, Indent2, Stream),
    write(Stream, '}').
% show_cond_('$remote_query'('$num_query'(N, Query), Addr), Indent, Stream) :-
%     !,
%     writeL_(Stream, [N]), write(Stream,  ' of '),
%     show_cond_(Query, Indent, Stream),
%     write(Stream, ' query_to '),
%     writeL_(Stream, [Addr]).
% show_cond_('$remote_query'(Query, Addr), Indent, Stream) :-
%     !,
%     show_cond_(Query, Indent, Stream),
%     write(Stream, ' query_at '),
%     writeL_(Stream, [Addr]).

show_cond_('$remote_query'('$num_vars_query'(N, Vars, Query), Addr),
           Indent, Stream) :-
    !,
    write(Stream, '('),
    writeL_(Stream, [N]), write(Stream,  ' of '),
    '$write_remote_vars'(Vars, Stream),
    write(Stream, ' :: '),
    show_cond_(Query, Indent, Stream),
    write(Stream, ') query_at '),
    writeL_(Stream, [Addr]).
show_cond_('$remote_query'('$num_query'(N, Query), Addr),
           Indent, Stream) :-
    !,
    write(Stream, '('),
    writeL_(Stream, [N]), write(Stream,  ' of '),
    show_cond_(Query, Indent, Stream),
    write(Stream, ') query_at '),
    writeL_(Stream, [Addr]).
show_cond_('$remote_query'('$vars_query'(Vars, Query), Addr),
           Indent, Stream) :-
    !,
    write(Stream, '('),
    '$write_remote_vars'(Vars, Stream),
    write(Stream, ' :: '),
    show_cond_(Query, Indent, Stream),
    write(Stream, ') query_at '),
    writeL_(Stream, [Addr]).
show_cond_('$remote_query'(Query, Addr),
           Indent, Stream) :-
    Query = (_ & _),
    !,
    write(Stream, '('),
    show_cond_(Query, Indent, Stream),
    write(Stream, ') query_at '),
    writeL_(Stream, [Addr]).
show_cond_('$remote_query'(Query, Addr),
           Indent, Stream) :-
    !,
    show_cond_(Query, Indent, Stream),
    write(Stream, ' query_at '),
    writeL_(Stream, [Addr]).


show_cond_('$num_query'(N, Query),
          Indent, Stream) :-
    !,
    writeL_(Stream, [N]), write(Stream,  ' of '),
    show_cond_(Query, Indent, Stream).
show_cond_(Cond, _, Stream) :-
    ip_set('$show_cond', true),
    writeTerm_(Stream, Cond),
    ip_set('$show_cond', _).


%% extract a percept fact
get_a_percept_clause_(P, c(H,[])) :-
    '$percept_type_info'(P, _, H),
    call(H).

%% extract a belief fact
get_a_belief_clause_(P, c(H,[])) :-
    '$belief_type_info'(P, _, H),
    call(H).

%% extract a belief fact as a relation clause
get_a_relation_clause_(P, c(H,[])) :-
    '$belief_type_info'(P,_,H), !,
    call(H).

%% extract a relation clause
get_a_relation_clause_(P, c(Rule,Vars)) :-
    '$user_relation_type_info'(P, _, H), !,
    '$saved_input_with_vars'(Rule, Vars, _),
    (
      Rule = (H :: _ <= _)
    ->
     true
    ;
      Rule = (H :: _ )
    ->
     true
    ;
      Rule = (H <= _)
    ->
     true
    ;
      Rule = H
    ).

%% extract a relation clause for a higher order relation
get_a_relation_clause_(P, c(Rule,Vars)) :-
    '$type_info'(P, fun, _, T, _), T \= '$function', !,
    '$saved_input_with_vars'(Rule, Vars, _),
    (
      Rule = (H :: _ <= _)
    ;
      Rule = (H :: _ )
    ;
      Rule = (H <= _)
    ;
      Rule = H
    ),
    '$get_ultimate_functor'(H, P).

%% extract an action clause
get_a_action_clause_(P, c(Rule,Vars)) :-
    '$user_action_type_info'(P, _, H),!,
    '$saved_input_with_vars'(Rule, Vars, _),
    (
      Rule = (H :: _ ~> _)
    ;
    %  Rule = '~>'((H :: _ ))
    %;
      Rule = (H ~> _)
    ;
      Rule = '~>'(H)
    ).

%% extract a function clause
get_a_function_clause_(P, c(Rule,Vars)) :-
    '$type_info'(P, fun, _, '$function', _), !,
    '$saved_input_with_vars'(Rule, Vars, _),
    (
      Rule = (H :: _ -> _)
    ;
      Rule = (H -> _)
    ),
    '$get_ultimate_functor'(H, P).


'$user_relation_type_info'(A, C, D) :-
    '$user_type_info'(A, rel, C, D, _),
    \+'$belief'(D), \+'$percept'(D).
'$user_relation_type_info'(A, C, D) :-
    '$builtin_type_info'(A, rel, C, D, _),
    '$user_defined'(A).

'$user_action_type_info'(A, C, D) :-
    '$user_type_info'(A, act, C, D, _).
'$user_action_type_info'(A, C, D) :-
    '$builtin_type_info'(A, act, C, D, _),
    '$user_defined'(A).

'$write_remote_vars'([V], Stream) :-
    !,
    writeL_(Stream, [V]).
'$write_remote_vars'([(V)|Rest], Stream) :-
    !,
    writeL_(Stream, [V]),
    write(Stream, ', '),
    '$write_remote_vars'(Rest, Stream).

