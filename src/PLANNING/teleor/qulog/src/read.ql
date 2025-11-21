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
%% See the License for the specific language governing permisions and
%% limitations under the License.

%% Code for the parsing qulog/teleor terms/code

?- op(990,xfx,'<=').
?-op(200,xf,'<=').
?-op(200,xf,'~>').
?-op(200,fx,'?').
?-op(920,xfx,'=>').
?-op(920,xfx,'~>').
?- op(600,xfx,'..').
?-op(1010,xfx,'::').
?-op(906,fx,'@').
?-op(906,fx,'forall').
?-op(906,fx,'exists').
?- op(910, xfy,  '&' ). 
?-op(700,xfx,'=?').   
?-op(700,xfx,'=@').   
?-op(500,xfy,'<>').
?-op(500,xfy,'++').   
?-op(1120,xfy,'<>?').
?-op(1120,xfy,'++?').   
?-op(600,xfx,'in').
?- op(200, fy, 'not').
?- op(190,fx, '#').
?- op(1101, xfx, '::=').
?- op(50, xfx, '?').
?- dynamic('$parser_info'/4).

%% parse QString into the qulog term T
'$read_qulog_term_from_string'(QString, T, VarList) :-
    (
      string_length(QString, 0)
    ->
      T = end_of_file
    ;
      retractall('$parser_info'(_, _, _, _)),
      %% information used to display syntax error messages
      assert('$parser_info'(QString, 1, 1, stdin)),
      %% tokenize QString
      string2tokens__(QString, Tokens, VarList, _TokenPositions),
      Tokens \= [],
      %% if the parser produces a syntax error (inside phrase) then
      %% it will display an error message and throw a syntax error
      catch(
            (
              phrase(term__(T1), Tokens, [])
            ->
              true
            ;
              report_error__('Can\'t parse', Tokens)
            ),
            %% catch syntax_error
            syntax_error,
            %% simply fail if syntax_error caught
            fail
           )
    ),
    %% T may be given and so this unification might fail if
    %% the parsed term is not the same as the supplied term
    T = T1.

%% read qulog term T from stream Stream
'$read_qulog_term'(T, Stream, VarList) :-
    stream_property(0, line_number(StartLine)),
    %% read QString from Stream - stop reading at end of term
    read_qulog_string__(QString,  Stream),
    stream_property(0, line_number(EndLine)),
    (
      string_length(QString, 0)
    ->
      T = end_of_file
    ;
      %% same as for read_qulog_term_from_string
      retractall('$parser_info'(_, _, _, _)),
      assert('$parser_info'(QString, StartLine, EndLine, Stream)),
      string2tokens__(QString, Tokens, VarList, _TokenPositions),
      Tokens \= [],
      catch(
            (
              phrase(term__(T1), Tokens, [])
            ->
              true
            ;
              report_error__('Can\'t parse', Tokens)
              ),
            syntax_error,
            fail
           )
    ),
    T = T1.

'$name_all_vars'([]).
'$name_all_vars'([var(Var,Value)|Rest]) :-
    '$readR_var'(Var,Value),
    '$name_all_vars'(Rest).

%% read a query from stdin, VarNamesOut gives the mapping between
%% variables and their names
interpreter_read_query__(Query, VarNamesOut) :-
    stream_property(0, line_number(StartLine)),
    read_qulog_string__(QString),
    stream_property(0, line_number(EndLine)),
    (
      string_length(QString, 0)
    ->
      Query = end_of_file,
      VarNames = []
    ;
      retractall('$parser_info'(_, _, _, _)),
      assert('$parser_info'(QString, StartLine, EndLine, stdin)),
      string2tokens__(QString, Tokens, VarList, TokenPositions),
      ip_set('$token_positions', t(Tokens, TokenPositions)),
      Tokens \= [],
      '$collect_var_tables'(VarList, _Variables, VarNames, _Singles, []),
      '$name_all_vars'(VarList),
      remove_duplicates(VarNames, VarNamesOut),
      catch(
            (
              phrase(query__(Query1), Tokens, [])
            ->
              true
            ;
              report_error__('Can\'t parse', Tokens)
              ),
            syntax_error,
            fail
           ),
      Query = Query1
    ).

    
'$unify_on_names'([]).
'$unify_on_names'([A|B]) :-
    member(A, B), !,
    '$unify_on_names'(B).
'$unify_on_names'([_A|B]) :-
    '$unify_on_names'(B).

%% Take InString and strip any %-to-EOL comments then
%% strip trailing whitespaces and check and remove "."
%% to get OutString
%% Fails if not of this form
strip_terminator_whitespace__(InString, OutString) :-
    (
      string_concat(A1, A2, InString),
      string_concat("%", _, A2)
    ->
      S0 = A1
    ;
      S0 = InString
    ),
    string_concat(S1, S2, S0),
    whitespaces__(S2),
    string_concat(OutString, ".", S1),
    !.

%% All chars in S are whitespaces
whitespaces__(S) :- string_length(S, 0), !.
whitespaces__(S) :-
    string_concat(S1, " ", S), !,
    whitespaces__(S1).
whitespaces__(S) :-
    string_concat(S1, "\t", S), !,
    whitespaces__(S1).
whitespaces__(S) :-
    string_concat(S1, "\n", S), !,
    whitespaces__(S1).

%% Read a qulog string QString - to be parsed in to term/code
read_qulog_string__(QString) :-
    read_qulog_string_aux__(not_at_end, "", QString, stdin).
read_qulog_string__(QString, Stream) :-
    read_qulog_string_aux__(not_at_end, "", QString, Stream).

/* enable if NEWLINE NEWLINE terminates the query */
% read_qulog_string_aux__('\n', Line, QString, Stream) :-
%     !,
%     get_line(Stream, _),
                                %     QString = Line.

%% EOF - return empty string
read_qulog_string_aux__('', _, "", _) :-
    !.
read_qulog_string_aux__(_, Line, QString, Stream) :-
    get_line(Stream, Line1),
    (
      Line1 = ""
    ->
      %% 
      QString = Line
    ;
      %% placeholder for future GUI interface that treats the ASCII char
      %% \U0004 as a Ctrl_D
      string_to_list(Line1, [4|_]) %Line = "\\U0004"
    ->
      QString = Line
    ;
     string_concat(Line, Line1, NewLine),
     (
      '$string_has_error_token'(NewLine, _)
     ->
       %% We have an incomplete token - e.g. inside a multi-line
       %% comment or string so we need to read another line
       peek(Stream, Char),
       read_qulog_string_aux__(Char, NewLine, QString, Stream)
     ;      
       strip_terminator_whitespace__(Line1, Line0)
     ->
       %% have a terminating .\nl
       string_concat(Line, Line0, QString)
     ;
       %% read more
       peek(Stream, Char),
       read_qulog_string_aux__(Char, NewLine, QString,  Stream)
     )
    ).
%% Read qulog term string String from Stream and return the start
%% and end lines - used for reporting syntax errors from qulog files.
%% NOTE that a term terminates just before a
%% newline, non-whitespace
read_term_string__(Stream, String, StartLine, EndLine) :-
    stream_property(Stream, line_number(StartLine)),
    get_line(Stream, Line),
    (
     \+string_concat(_, "\n", Line)
    ->
      String = Line
    ;
      string_to_list(Line, [4|_]) %Line = "\\U0004"
    ->
      String = Line
    ;
      peek(Stream, Char),
      read_term_string_aux__(Char, Stream, Line, String)
    ),
    stream_property(Stream, line_number(EndLine)).

read_term_string_aux__(_Char0, Stream, Line, String) :-
    '$string_has_error_token'(Line, Value), !,
    %% inside a token or comment - keep reading
    read_to_tokens_fixed__(Stream, Line, Value, String).
read_term_string_aux__(Char0, Stream, Line, String) :-
    '$read_term_string_whitespace'(Char0),
    !,
    %% newline followed by space - keep reading
    get_line(Stream, Line1),
    (
     \+string_concat(_, "\n", Line1)
    ->
     %% EOF - concat on new line
     string_concat(Line, Line1, String)
    ;
      string_to_list(Line1, [4|_]) %Line = "\\U0004"
    ->
      String = Line
    ;
     string_concat(Line, Line1, NewLine),
     peek(Stream, Char),
     read_term_string_aux__(Char, Stream, NewLine, String)
    ).
read_term_string_aux__('/', Stream, Line, String) :-
    !,
    %% line starts with / - probably / followed by * so read line
    %% OK as no defn/declaration starts with /       
    get_line(Stream, Line1),
    (
     \+string_concat(_, "\n", Line1)
    ->
     string_concat(Line, Line1, String)
    ;
      string_to_list(Line1, [4]) %Line = "\\U0004"
    ->
      String = Line
    ;    
     string_concat(Line, Line1, NewLine),
     peek(Stream, Char),
     read_term_string_aux__(Char, Stream, NewLine, String)
    ).
read_term_string_aux__('"', Stream, Line, String) :-
    !,
    %% either start or end of a string at beginning of line
    %% in either case keep reading
    get_line(Stream, Line1),
    (
     \+string_concat(_, "\n", Line1)
    ->
     string_concat(Line, Line1, String)
    ;
      string_to_list(Line1, [4|_]) %Line = "\\U0004"
    ->
      String = Line
    ;    
     string_concat(Line, Line1, NewLine),
     peek(Stream, Char),
     read_term_string_aux__(Char, Stream, NewLine, String)
    ).
read_term_string_aux__('%', Stream, Line, String) :-
    !,
    %% list starting with % comment - keep reading
    get_line(Stream, Line1),
    (
     \+string_concat(_, "\n", Line1)
    ->
     String = Line
    ;
      string_to_list(Line1, [4|_]) %Line = "\\U0004"
    ->
      String = Line
    ;    
     peek(Stream, Char),
     read_term_string_aux__(Char, Stream, Line, String)
    ).
read_term_string_aux__('}', Stream, Line, String) :-
    !,
    %% line starts with } - keep reading
    get_line(Stream, Line1),
    string_concat(Line, Line1, String).
    
read_term_string_aux__(_, _Stream, Line, String) :-
    %% We have reached the end of the decl/defn string
    String = Line.

'$read_term_string_whitespace'(' ').
'$read_term_string_whitespace'('\t').
'$read_term_string_whitespace'('\r').
'$read_term_string_whitespace'('\n').

%% Convert String into a list of Qulog tokens
%% VarList is the var - name mapping and
%% TokenPositions gives the position in the stream of the token - used
%% for parse error messages
string2tokens__(String, Tokens, VarList, TokenPositions) :-
    open_string(read(String), Stream),
    get_tokens__(Stream, Tokens1, TokenPositions1),
    close(Stream),
    '$post_process_tokens'(Tokens1, Tokens, VarList, TokenPositions1,
                           TokenPositions).
%% When string_has_error_token succeeds we keep reading until
%% we get a valid token - e.g. read until the end of a string/comment
read_to_tokens_fixed__(Stream, InString, Value, OutString) :-
    get_line(Stream, Line1),
    (
     Line1 = -1
    ->
     '$token_error_msg'(Value, InString),
     throw(syntax_error)
    ;
      string_to_list(Line1, [4]) %Line1 = "\\U0004"
    ->
     '$token_error_msg'(Value, InString),
     throw(syntax_error)
    ;
     %string_concat(Line1, "\n", Line1NL),
     string_concat(InString, Line1, NewLine),
     (
      '$string_has_error_token'(NewLine, V1)
     ->
      read_to_tokens_fixed__(Stream, NewLine, V1, OutString)
     ;
      peek(Stream, Char),
      read_term_string_aux__(Char, Stream, NewLine, OutString)
     )
    ).

'$token_error_msg'(6, String) :- !,
    writeL_(stderr,
	   [nl_, "Error: EOF reached inside multi-line comment while reading",
	    nl_, String, nl_]).
'$token_error_msg'(7, String) :- !,
    writeL_(stderr,
	   ["Error: EOF reached inside quoted term while reading",
	    nl_, String, nl_]).
'$token_error_msg'(_, String) :- !,
    writeL_(stderr,
	   ["Error: Invalid token while reading",
	    nl_, String, nl_]).

%% We use the QuProlog tokens but some of the Qulog tokens are different
%% This postprocesses the QP tokens into Qulog tokens
%%'$post_process_tokens'(A,B,C,D,E) :- errornl('$post_process_tokens'(A,B,C,D,E)),fail.
'$post_process_tokens'([], [], [], _, []).
%% Drop a . token at EOF
'$post_process_tokens'([t(5, '.')], [], [], _, [])  :- !.
%% Drop a . newline
'$post_process_tokens'([t(5, '.'), t(13,_)|Rest], Output, VarList,
                       [_,_|Pos], TPos)  :- !,
    '$post_process_tokens'(Rest, Output, VarList, Pos, TPos).
%% '=#' is not a Qulog token so split
'$post_process_tokens'([t(4, '=#')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '='), t(4, '#')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for :?
'$post_process_tokens'([t(4, ':?')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, ':'), t(4, '?')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for :??
'$post_process_tokens'([t(4, ':??')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, ':'), t(4, '??')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for +#
'$post_process_tokens'([t(4, '+#')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '+'), t(4, '#')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for -#
'$post_process_tokens'([t(4, '-#')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '-'), t(4, '#')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for *#
'$post_process_tokens'([t(4, '*#')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '*'), t(4, '#')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for ..-
'$post_process_tokens'([t(4, '..-')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '..'), t(4, '-')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for ..+
'$post_process_tokens'([t(4, '..+')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '..'), t(4, '+')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for *- 
'$post_process_tokens'([t(4, '*-')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '*'), t(4, '-')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for **-
'$post_process_tokens'([t(4, '**-')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+2,
    '$post_process_tokens'([t(4, '**'), t(4, '-')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for +-
'$post_process_tokens'([t(4, '+-')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '+'), t(4, '-')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for --
'$post_process_tokens'([t(4, '--')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '-'), t(4, '-')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for =-
'$post_process_tokens'([t(4, '=-')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '='), t(4, '-')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).
%% ditto for =+
'$post_process_tokens'([t(4, '=+')|Rest], Output, VarList,
                       [P|Pos], TPos) :-
    !,
    P1 is P+1,
    '$post_process_tokens'([t(4, '='), t(4, '+')|Rest], Output, VarList,
                           [P,P1|Pos], TPos).



%% change '!?' into a syntax token
% '$post_process_tokens'([t(7, '!'), t(4, '?')|Rest], Output,
%                        Varlist, [P|Pos], TPos) :-
%     !,
%     P1 is P+1,
%     '$post_process_tokens'([t(4,'!?')|Rest], Output, Varlist, [P1|Pos], TPos).
% %% change '!??' into a syntax token
% '$post_process_tokens'([t(7, '!'), t(4, '??')|Rest], Output,
%                        Varlist, [P|Pos], TPos) :-
%     !,
%     P1 is P+1,
%     '$post_process_tokens'([t(4,'!??')|Rest], Output, Varlist, [P1|Pos], TPos).
%% the token is an atom or quoted atom that is reserved syntax -
%% cange to a syntax token
'$post_process_tokens'([t(Kind, A)|Rest], [syntax(A)|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    ( Kind = 4 ; Kind = 7 ),
    '$parser_reserved_syntax'(A),
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% the token is an atom or quoted atom that is an operator in qulog
%% cange to op token
'$post_process_tokens'([t(Kind, A)|Rest], [op(A)|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    ( Kind = 4 ; Kind = 7 ),
    '$parser_reserved_operator'(A),
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).

%% change '!' into a syntax token
'$post_process_tokens'([t(11, 0)|Rest], [syntax('!')|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% change the [ token followed by the ] token into the atom([]) token
'$post_process_tokens'([t(1, '['), t(1, ']')|Rest],
		       [atom([])|PRest], Varlist, [P,_|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% change various open brackets into ob tokens
'$post_process_tokens'([t(1, '(')|Rest], [ob('(')|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% note space before (
'$post_process_tokens'([t(1, ' (')|Rest], [ob(' (')|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(1, '[')|Rest], [ob('[')|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(1, '{')|Rest], [ob('{')|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% change various close brackets into cb tokens
'$post_process_tokens'([t(1, ')')|Rest], [cb(')')|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(1, ']')|Rest], [cb(']')|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(1, '}')|Rest], [cb('}')|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% change punctuation tokens into punct tokens
'$post_process_tokens'([t(1, P)|Rest], [punct(P)|PRest], Varlist,
                       [P1|Pos], [P1|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% add appropriate wrappers to all other kinds of tokens
'$post_process_tokens'([t(4, A)|Rest], [atom(A)|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(6, A)|Rest], [var(_, A)|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(7, A)|Rest], [atom(A)|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(9, S)|Rest], [string(S)|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(3, N)|Rest], [int(N)|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
'$post_process_tokens'([t(14, N)|Rest], [num(N)|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% ignore newline
'$post_process_tokens'([t(13, _S)|Rest], PRest, Varlist,
                       [_P|Pos], TPos) :-
    !,
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).
%% a variable - convert to var token
'$post_process_tokens'([t(2, Value)|Rest], [Variable|PRest],
		       [Variable|VarList], [P|Pos], [P|TPos]) :-
    !,
    Variable = var(_Var,Value),
    %%'$readR_var'(Var,Value), %% if remember name
    '$post_process_tokens'(Rest, PRest, VarList, Pos, TPos),
    once((
      member(Variable, VarList)
    ;
      true
    )).
      
%% leave any other tokens as is
'$post_process_tokens'([X|Rest], [X|PRest], Varlist,
                       [P|Pos], [P|TPos]) :-
    '$post_process_tokens'(Rest, PRest, Varlist, Pos, TPos).


%% table of operators
'$parser_reserved_operator'('+').
'$parser_reserved_operator'('-').
'$parser_reserved_operator'('*').
'$parser_reserved_operator'('/').
'$parser_reserved_operator'('\\').
'$parser_reserved_operator'('//').
'$parser_reserved_operator'('/\\').
'$parser_reserved_operator'('\\/').
'$parser_reserved_operator'('>>').
'$parser_reserved_operator'('<<').
'$parser_reserved_operator'('**').
'$parser_reserved_operator'('?').
'$parser_reserved_operator'('==').
'$parser_reserved_operator'('<=').
'$parser_reserved_operator'(':>>').
'$parser_reserved_operator'('+>').
'$parser_reserved_operator'('->').
'$parser_reserved_operator'('=>').
'$parser_reserved_operator'(';').
'$parser_reserved_operator'('..').
'$parser_reserved_operator'('|').
'$parser_reserved_operator'('!').
'$parser_reserved_operator'('=').
'$parser_reserved_operator'('\\=').
'$parser_reserved_operator'('=?').
'$parser_reserved_operator'('=@').
'$parser_reserved_operator'('@').
'$parser_reserved_operator'('<>').
'$parser_reserved_operator'('<>?').
'$parser_reserved_operator'('?-').
'$parser_reserved_operator'('++').
'$parser_reserved_operator'('++?').
%'$parser_reserved_operator'('@..').
'$parser_reserved_operator'('#').
'$parser_reserved_operator'('in').
'$parser_reserved_operator'('>').
'$parser_reserved_operator'('>=').
'$parser_reserved_operator'('=<').
'$parser_reserved_operator'('<').
'$parser_reserved_operator'('>').
'$parser_reserved_operator'('<=').
'$parser_reserved_operator'('@>').
'$parser_reserved_operator'('@>=').
'$parser_reserved_operator'('@<').
'$parser_reserved_operator'('@<=').
'$parser_reserved_operator'('@=').
'$parser_reserved_operator'('$').
'$parser_reserved_operator'(':=').
'$parser_reserved_operator'('+:=').
'$parser_reserved_operator'('-:=').
'$parser_reserved_operator'('mod').
'$parser_reserved_operator'('union').
'$parser_reserved_operator'('inter').
'$parser_reserved_operator'('diff').
'$parser_reserved_operator'('to').
'$parser_reserved_operator'('to_thread').
'$parser_reserved_operator'('from').
'$parser_reserved_operator'('from_thread').
%%'$parser_reserved_operator'('at').

%% table of reserved syntax
'$parser_reserved_syntax'(':').
'$parser_reserved_syntax'('::').
'$parser_reserved_syntax'('::=').
'$parser_reserved_syntax'('|').
'$parser_reserved_syntax'('||').
'$parser_reserved_syntax'('..').
'$parser_reserved_syntax'('..,').
'$parser_reserved_syntax'('->').
'$parser_reserved_syntax'('~>').
'$parser_reserved_syntax'('~+>').
'$parser_reserved_syntax'('<=').
'$parser_reserved_syntax'('=>').
'$parser_reserved_syntax'('query_at').
%'$parser_reserved_syntax'('query_from').
'$parser_reserved_syntax'('forall').
'$parser_reserved_syntax'('exists').
'$parser_reserved_syntax'('rel').
'$parser_reserved_syntax'('act').
'$parser_reserved_syntax'('dyn').
'$parser_reserved_syntax'('mrel').
'$parser_reserved_syntax'('mfun').
'$parser_reserved_syntax'('tel_percept').
'$parser_reserved_syntax'('tel_action').
'$parser_reserved_syntax'('tel').
'$parser_reserved_syntax'('tel_start').
'$parser_reserved_syntax'('tel_atomic').
'$parser_reserved_syntax'('fun').
'$parser_reserved_syntax'('def').
'$parser_reserved_syntax'('?-').
'$parser_reserved_syntax'('!').
'$parser_reserved_syntax'('?').
%'$parser_reserved_syntax'('!?').
'$parser_reserved_syntax'('??').
%'$parser_reserved_syntax'('!??').
'$parser_reserved_syntax'('@').
'$parser_reserved_syntax'('&').
'$parser_reserved_syntax'(';').
'$parser_reserved_syntax'('=?').
'$parser_reserved_syntax'('repeat').
'$parser_reserved_syntax'('until').
'$parser_reserved_syntax'('or_while').
'$parser_reserved_syntax'('commit_while').
'$parser_reserved_syntax'('min_time').
'$parser_reserved_syntax'('atomic_action').
'$parser_reserved_syntax'('receive').
'$parser_reserved_syntax'('try').
'$parser_reserved_syntax'('timeout').
'$parser_reserved_syntax'('except').
'$parser_reserved_syntax'('wait_case').
'$parser_reserved_syntax'('wait').
'$parser_reserved_syntax'('case').
'$parser_reserved_syntax'('retry').
'$parser_reserved_syntax'('default').

%% String has error token of kind Kind - only a problem if
%% Kind is a nonvar
%% we keep reading tokens until:
%% EOF reached - OK
%% we get an error token 
'$string_has_error_token'(String, Kind) :-
    open_string(read(String), Stream),
    '$string_has_error_token_aux'(Stream, Kind),
    close(Stream),
    nonvar(Kind).

'$string_has_error_token_aux'(Stream, Kind) :-
    '$read_next_token'(Stream, T1, V1),
    '$string_has_error_token_aux2'(Stream, Kind, T1, V1).

'$string_has_error_token_aux2'(_Stream, _Kind, T1, _V1) :-
    %% EOF - OK
    T1 = 8, !.
'$string_has_error_token_aux2'(_Stream, Kind, T1, V1) :-
    %% error token V1 is 6 for string, 7 for quoted atom 
    T1 = 0, !, Kind = V1.
'$string_has_error_token_aux2'(Stream, Kind, _T1, _V1) :-
    %% keep reading tokens
    '$read_next_token'(Stream, T2, V2),
    '$string_has_error_token_aux2'(Stream, Kind, T2, V2).

%% Tokenize Stream to get the list of tokens T and theire positions
%% in the stream - used in string2tokens
get_tokens__(Stream, T, [Pos|TPos]) :-
    stream_property(Stream, position(Pos)),
    '$read_next_token'(Stream, T1, V1),
    get_tokens_aux__(Stream, T1, V1, T, TPos).

%%get_tokens_aux__(_Stream, T1, V1, T, TPos) :- errornl(get_tokens_aux(T1,V1)),fail.
%% EOF - stop reading
get_tokens_aux__(_Stream, T1, _V1, T, []) :- T1 = 8, !, T = [].
%% keep reading tokens
get_tokens_aux__(Stream, T1, V1, T, [Pos|TPos]) :-
    T = [t(T1,V1)|Rest],
    stream_property(Stream, position(Pos)),
    '$read_next_token'(Stream, T2, V2),
    get_tokens_aux__(Stream, T2, V2, Rest, TPos).

%% Used in consult and tr_checks to give warnings about singleton variables
%% in definitions
'$warn_of_single_vars'(T, Singles, VarNames) :-
    '$vars2names'(T, VarNames),
    '$warn_of_single_vars1'(T, Singles),
    fail.
'$warn_of_single_vars'(_, _, _).

%%'$warn_of_single_vars1'(X, Y) :- errornl('$warn_of_single_vars1'(X, Y)),fail.
'$warn_of_single_vars1'(_T, []) :- !.
'$warn_of_single_vars1'('$code_decl'(_, _, _), _) :- !.
'$warn_of_single_vars1'((A:_), _) :-
    compound(A), !.
'$warn_of_single_vars1'(T, Singles) :-
    '$warn_of_single_vars_aux'(Singles, Vars),
    write('Singleton variables warning: '),
    write(Vars),
    write(' in'),
    nl,
    writeTerm_(T),
    nl,nl,
    fail.

'$warn_of_single_vars_aux'([], []).
'$warn_of_single_vars_aux'([(_V = VN)|Singles], [VN|Vars]) :-
    '$warn_of_single_vars_aux'(Singles, Vars).

%% Report a syntax error - Msg is the report and Rest is remaining tokens
%% This is used to add " ###here### " at the approriate position
report_error__(Msg, Rest) :-
    list(Msg), !,
    '$parser_info'(String, StartLine, EndLine, FileName),
    nl,
    add_here__(String, Rest, HereString),
    (
     FileName = stdin
    ->
      write_term_list(stderr, [nl, 'Syntax Error: lines ', StartLine - EndLine,
			      nl]),
      write_term_list(stderr, Msg),
      nl(stderr)
                     
    ;
      write_term_list(stderr, [nl, 'Syntax Error: file ', FileName,
			      ' : lines ', StartLine - EndLine,
			      nl]),
      write_term_list(stderr, Msg),
      nl(stderr)
    ),
    write_string(stderr, HereString),
                                %nl,nl,
    throw(syntax_error).

report_error__(Msg, Rest) :-
    '$parser_info'(String, StartLine, EndLine, FileName),
    nl,
    add_here__(String, Rest, HereString),
    (
     FileName = stdin
    ->
     write_term_list(stderr, [nl, 'Syntax Error: lines ', StartLine - EndLine,
			      nl, Msg, nl])
    ;
     write_term_list(stderr, [nl, 'Syntax Error: file ', FileName,
			      ' : lines ', StartLine - EndLine,
			      nl, Msg, nl])
    ),
    write_string(stderr, HereString),
    %nl,nl,
    throw(syntax_error).

%% add_here__(String, Rest, HereString) :-errornl(add_here__(String, Rest, HereString)),fail.
add_here__(String, Rest, HereString) :-
    ip_lookup('$token_positions', t(Tokens, TokenPositions)),
    '$position_of_rest_start'(Tokens, TokenPositions, Rest, Pos),
    (
      Pos = -1
    ->
      (
        string_concat(String1, "\n", String)
      ->
        string_concat(String1, " ###here### ", HereString)
      ;
        string_concat(String, " ###here### ", HereString)
      )
    ;
      string_concat(S1, S2, String),
      string_length(S1, Pos),
      !,
      string_concat(S1, " ###here### ", S3),
      (
        string_concat(S4, "\n", S2)
      ->
        true
      ;
        S4 = S2
      ),
      string_concat(S3, S4, HereString)
    ).
add_here__(String, _Rest, String).
	    
'$position_of_rest_start'(Rest, [], Rest, -1) :- !.
'$position_of_rest_start'(Rest, [Pos|_], Rest, Pos) :- !.
'$position_of_rest_start'([_|Tokens], [_|Positions], Rest, Pos) :-
    '$position_of_rest_start'(Tokens, Positions, Rest, Pos).

%% Used to build arithemtic expression from subterms
'$process_arith_tuples'(F, A1, B, Term) :-
    compound(A1), A1 = '$tuple'([A]),
    !,
    '$process_arith_tuples'(F, A, B, Term).
'$process_arith_tuples'(F, A, B1, Term) :-
    compound(B1), B1 = '$tuple'([B]),
    !,
    '$process_arith_tuples'(F, A, B, Term).
'$process_arith_tuples'(F, A, B, Term) :-
    Term = F(A,B).

%% similar to above but for set operators: union, inter, diff
'$process_set_tuples'(F, A1, B, Term) :-
    compound(A1), A1 = '$tuple'([A]),
    !,
    '$process_set_tuples'(F, A, B, Term).
'$process_set_tuples'(F, A, B1, Term) :-
    compound(B1), B1 = '$tuple'([B]),
    !,
    '$process_set_tuples'(F, A, B, Term).
'$process_set_tuples'(F, A, B, Term) :-
    Term = F(A,B).



%%%%%%%%%%%%%%%%%%%%%%
%% The Parser DCG
%%%%%%%%%%%%%%%%%%%%%%

%% rules for handling syntax errors
error_handler__(Msg) --> Rest, {report_error__(Msg, Rest)}, consume_rest.
error_handler__(Msg, Rest) --> {report_error__(Msg, Rest)}, consume_rest.

%% rules for not getting closing brackets
cb_error_handler__(cb(']')) -->
    Rest, {report_error__('Expecting ]', Rest)}, consume_rest.
cb_error_handler__(cb(')')) -->
    Rest, {report_error__('Expecting )', Rest)}, consume_rest.
cb_error_handler__(cb('}')) -->
    Rest, {report_error__('Expecting }', Rest)}, consume_rest.

%% consume remaining tokens
consume_rest --> [_], consume_rest.
consume_rest --> [].

%% look ahead to see if T is in the remaining tokens
has_token__(T) --> Tokens, {member(T, Tokens)}.

%% see if the next token is T
next_token__(T) --> Tokens, {Tokens = [T|_]}.
%% see if the next tokens (in order) are Ts
next_tokens__(Ts) --> Tokens, {append(Ts, _, Tokens)}.

%% see if token T occurs before token Before
has_token_before__(T, Before) -->
    Tokens,
    {
     member(X, Tokens),
     (
       X = T
     ->
       !,true
     ;
       member(X, Before)
     ->
       !, fail
    ;
       fail
     )
    }.


%% check that there are no leftover tokens - else display error
no_leftover__ -->
    Tokens,
    ( {Tokens \= []} -> error_handler__('Unprocessed tokens', Tokens) ; [] ).

%% check that we are at the end of the tokens -i.e. no more tokens to process
at_end__ -->
    Tokens,
    { Tokens = [] }, !.
at_end__ -->
    Tokens,
    { ip_lookup('$parsing_query_at',P), P == true,
      (
        Tokens = [cb(')')|_]
      ;
        Tokens = [syntax__('query_at')|_]
      )
    }, !.


%% Try calling GrammarRuleCall and if that fails
%% create an error using the error message
parse_with_error_handling__(GrammarRuleCall, _ErrorMessage, In, Out) :-
    GrammarRuleCall(In, Out),!.
parse_with_error_handling__(_GrammarRuleCall, ErrorMessage) -->
    error_handler__(ErrorMessage).



ob_with_check__ -->
    (
      [ob('(')]
    ->
      []
    ;
      error_handler__('Expecting (')
    ).
comma_with_check__ -->
    (
      [punct(',')]
    ->
      []
    ;
      error_handler__('Expecting ,')
    ).

atom_with_check__(G) -->
    (
      atom__(G)
    ->
      []
    ;
      next_token__(syntax(G))
    ->
      error_handler__(['The reserved symbol \'', G, '\' can\'t be used as an atom'])
    ;
      error_handler__('Expecting atom')
    ).
op_with_check__(G) -->
    (
      [op(G)]
    ->
      []
    ;
      {atom_concat('Expecting ', G, Err)},
      error_handler__(Err)
    ).
syntax_with_check__(G) -->
    (
      [syntax(G)]
    ->
      []
    ;
      {atom_concat('Expecting ', G, Err)},
      error_handler__(Err)
    ).


open_paren_with_check__ -->
     (
      [ob('{')]
    ->
      []
    ;
      error_handler__('Expecting {')
    ).

%% consume the closing bracket else produce error
matching_bracket__(CB) -->
    (
      [CB]
    ->
      []
    ;
      cb_error_handler__(CB)
    ).


%% Parsing a sequence:
%% Kind is the kind of term forming the elements of the non-empty sequence
%% Sep is the separator of the sequence
%% Lst is the returned list of Kind terms
%% Err is the error message to report if required
'$seq'(Kind, Sep, [First|Lst], Err) -->
    '$seq_elem_with_check'(Kind, First, Err),
    '$seq_rest'(Kind, Sep, Lst, Err).

%% Sep is next token so continue
'$seq_rest'(Kind, Sep, [Next|Lst], Err) -->
    [Sep], !,
    '$seq_elem_with_check'(Kind, Next, Err),
    '$seq_rest'(Kind, Sep, Lst, Err).
%% Sep is not next token so finish
'$seq_rest'(_Kind, _Sep, [], _Err) --> [].

%% '$seq_item'(Kind, Elem, Err) - consume Elem if of kind Kind or generate
%% an Err error
%%'$seq_elem_with_check'(Kind, Elem, Err) --> TK, {errornl(seq_elem__(Kind, Elem, Err, TK)), fail}.
'$seq_elem_with_check'(Kind, Elem, _Err) -->
    '$seq_item'(Kind, Elem), !.
'$seq_elem_with_check'(_Kind, _Elem, Err) -->  
    error_handler__(Err).
      
'$seq_item'(atom, A) --> !,
    (
      next_token__(syntax(A))
    ->
      error_handler__(['The reserved symbol \'', A, '\' can\'t be used as an atom'])
    ;
      atom__(A)
    ).
'$seq_item'(string__, S) --> !, string__(S).
'$seq_item'(anynum, N) --> !, anynum__(N).
'$seq_item'(simple_compound, SC) --> !, simple_compound_(SC).
'$seq_item'(atom_or_simple_compound, SC) --> !, atom_or_simple_compound_(SC).
'$seq_item'(var_or_simple_compound, SC) --> !, var_or_simple_compound_(SC).
'$seq_item'(term__, V) --> !, term__(V).
'$seq_item'(var, V) --> !, var_(V).
'$seq_item'(type_expression__, TE) --> !, type_expression__(TE).
'$seq_item'(annotated_type_expression__, TE) --> !, annotated_type_expression__(TE).
'$seq_item'(action, A) --> !, action__(A).
'$seq_item'(annotated_declaration_arg__, AD) --> !,
    annotated_declaration_arg__(AD).
'$seq_item'(unannotated_declaration_arg__, AD) --> !,
    unannotated_declaration_arg__(AD).
'$seq_item'(fun_declaration_, F) --> !, fun_declaration_(F).
'$seq_item'(percept_declaration_, AD) --> !,
    percept_declaration_(AD).
'$seq_item'(tel_action_declaration_, AD) --> !,
    tel_action_declaration_(AD).
'$seq_item'(annotated_declaration_, AD) --> !,
    annotated_declaration_(AD).
'$seq_item'(unannotated_declaration_, AD) --> !,
    unannotated_declaration_(AD).
'$seq_item'(simple_tr_action, AD) --> !,
    simple_tr_action__(AD).
'$seq_item'(simple_term__, AD) --> !,
    simple_term__(AD).
'$seq_item'(term__, AD) --> !,
    term__(AD).
'$seq_item'(atom__, AD) --> !,
    atom__(AD).
'$seq_item'(var_type__, (V:T)) --> !,
    var_(V),
    syntax__(':'),
    type_expression__(T).

%% Used in parsing declaration arguments that may or may not contain
%% var :  (no modes on types)
unannotated_declaration_arg__(Decl) -->
    has_token_before__(syntax(':'), [punct(','), cb(')')]), !,
    parse_with_error_handling__(var_(V), 'Expecting variable'),
    syntax_with_check__(':'),
    parse_with_error_handling__(type_expression__(T),
                                'Expecting unannotated type expression'),
    (
      syntax__(default)
    ->
      term__(D),
      { Decl = default(V:T, D) }
    ;
      { Decl = V:T}
    ).    
unannotated_declaration_arg__(Decl) -->
    parse_with_error_handling__(type_expression__(T),
                                'Expecting unannotated type expression'),
    (
      syntax__(default)
    ->
      term__(D),
      { Decl = default(T, D) }
    ;
      { Decl = T}
    ).

%% possible modes on types
annotated_declaration_arg__(Decl) -->
    has_token_before__(syntax(':'), [punct(','), cb(')')]), !,
    parse_with_error_handling__(var_(V), 'Expecting variable'),
    syntax_with_check__(':'),
    parse_with_error_handling__(annotated_type_expression__(T),
                                'Expecting annotated type expression'),
    (
      syntax__(default)
    ->
      term__(D),
      { Decl = default(V:T, D) }
    ;
      { Decl = V:T}
    ).      
annotated_declaration_arg__(Decl) -->
    parse_with_error_handling__(annotated_type_expression__(T),
                                'Expecting annotated type expression'),
    (
      syntax__(default)
    ->
      term__(D),
      { Decl = default(T, D) }
    ;
      { Decl = T}
    ).
    

%% Parsing a bracketed sequence:
%% Kind is the kind of term forming the elements of the sequence
%% Sep is the separator of the sequence
%% Lst is the returned list of Kind terms
%% Err is the error message to report if required
'$bracketed_seq'(_Kind, _Sep, [], _Err) -->
    any_open_bracket__,
    [cb(')')], !.
'$bracketed_seq'(Kind, Sep, Lst, Err) -->
    any_open_bracket__,
    '$seq'(Kind, Sep, Lst, Err),
     matching_bracket__(cb(')')).
    
%% Parsing a {  } sequence:
%% Kind is the kind of term forming the elements of the sequence
%% Sep is the separator of the sequence
%% Lst is the returned list of Kind terms
%% Err is the error message to report if required
'$paren_seq'(_Kind, _Sep, [], _Err) -->
    [ob('{')],
    [cb('}')], !.

'$paren_seq'(Kind, Sep, Lst, Err) -->
    [ob('{')],
    '$seq'(Kind, Sep, Lst, Err),
     matching_bracket__(cb('}')).

%% Parsing a [  ] sequence:
%% Kind is the kind of term forming the elements of the sequence
%% Sep is the separator of the sequence
%% Lst is the returned list of Kind terms
%% Err is the error message to report if required
'$square_seq'(_Kind, _Sep, [], _Err) -->
    [atom__('[]')], !.

'$square_seq'(Kind, Sep, Lst, Err) -->
    [ob('[')],
    '$seq'(Kind, Sep, Lst, Err),
     matching_bracket__(cb(']')).

%% ----------------------------------------------------
%% atom_seq = atom, {",", !, atom};
%% ----------------------------------------------------
atom_seq__(As) -->
    '$seq'(atom__, punct(','), As, 'Expecting atom').

atom_or_syntax_seq__(As) -->
    atom_or_syntax__(A),
    (
      [punct(',')]
    ->
      atom_or_syntax_seq__(Rest),
      {As = [A|Rest]}
    ;
      {As = [A]}
    ).

%% ----------------------------------------------------
%% var_seq = ( var, {",", !, var} ) | ( "(", var, {",", !, var}, ")" );
%% ----------------------------------------------------
var_seq__(Vars) -->
    (
      any_open_bracket__
    ->
      '$seq'(var, punct(','), Vars, 'Expecting variable'),
      matching_bracket__(cb(')'))
    ;
      '$seq'(var, punct(','), Vars, 'Expecting variable')
    ).

%% ----------------------------------------------------
%% var_type_seq = ( var, ":", type_expression,
%%                    {",", !, var,":", type_expression} ) |
%%                ( "(", var, ":", type_expression,
%%                    {",", !, var,":", type_expression}, ")" );
%% ----------------------------------------------------
var_type_seq__(Vars) -->
    (
      any_open_bracket__
    ->
      '$seq'(var_type__, punct(','), Vars, 'Expecting variable:type'),
      matching_bracket__(cb(')'))
    ;
      '$seq'(var_type__, punct(','), Vars, 'Expecting variable:type')
    ).

%% ----------------------------------------------------
%% bracketed_var_seq  = ("(", ")", !) | ("(", var_seq, ")", !);
%% ----------------------------------------------------
bracketed_var_seq__(Vars) -->
    '$bracketed_seq'(var, punct(','), Vars, 'Expecting variable').

%% ----------------------------------------------------
%% bracketed_arg_seq  = ("(", { term, "," }, ")", !) | ("(", arg_seq, ")", !);
%% ----------------------------------------------------
bracketed_arg_seq__(Args) -->
    '$bracketed_seq'(term__, punct(','), Args, 'Expecting term').

%% ----------------------------------------------------
%% bracketed_type_arg_seq  = ("(", type_expression, {",", type_expression}, ")", !) ;
%% ----------------------------------------------------
bracketed_type_arg_seq__(Args) -->
    '$bracketed_seq'(type_expression__, punct(','), Args, 'Expecting type expression').


%% ----------------------------------------------------
%% bracketed_type_seq =
%%     ( "(", ")" ) |
%%     ( "(", type_expression, { ",", type_expression}, ")" );
%% ----------------------------------------------------
bracketed_type_seq__([]) -->
    any_open_bracket__,
    [cb(')')], !.
bracketed_type_seq__(Types) -->
    any_open_bracket__,
    '$bracketed_type_seq_aux'(Types), !.

'$bracketed_type_seq_aux'([T|Ts]) -->
    \+ next_token__(punct(',')),
    parse_with_error_handling__(type_expression__(T), 'Expecting type expression'),
    (
      [cb(')')]
    ->
      {Ts = [] }
    ;
      comma_with_check__,
      '$bracketed_type_seq_aux'(Ts)
    ).
        
%% ----------------------------------------------------
%% bracketed_annotated_type_seq =
%%     ( "(", ")" ) |
%%     ( "(", annotated_type_expression, { ",", annotated_type_expression}, ")" );
%% ----------------------------------------------------
bracketed_annotated_type_seq__(Types) -->
    '$bracketed_seq'(annotated_type_expression__, punct(','), Types,
                     'Expecting annotated type expression').

%% ----------------------------------------------------
%% query =
%%     (
%%        ( int, "of", var_seq, "::", conditions ) |
%%        ( int, "of", conditions ) |
%%        ( int, "of", "exists", var_seq, conditions ) |
%%        ( var_seq, "::", conditions ) |
%%        ( "exists", var_seq, conditions ) |
%%        basic_query 
%%      ), !;
%% ----------------------------------------------------

query__(Query) -->
    int__(N),
    atom__(of), !,
    (
      [syntax('exists')],
      (
        var_seq__(Vars)
      ->
        %% ( int, "of", "exists", var_seq, conditions )
        parse_with_error_handling__(conditions__(T),
                                    'Can\'t parse exists body'),
        {Query = '$num_query'(N, '$exists'(Vars, T))}
      ;
        error_handler__('Expecting bound variables')
      )
    ;
      next_token__(var(_,_)),var_seq__(Vars),
      [syntax('::')],
      %% ( int, "of", var_seq, "::", conditions )
      parse_with_error_handling__(conditions__(T),
                                  'Can\'t parse :: body'),
      {Query = '$num_vars_query'(N, Vars, T)}
    ;
      %% ( int, "of", conditions )
      parse_with_error_handling__(conditions__(T),
                                  'Can\'t parse after \`of\`'),
      {Query = '$num_query'(N, T)}
    ).
    
query__(Query) -->
    next_token__(var(_,_)), 
    var_seq__(Vars), 
    %% ( var_seq, "::", conditions )
    [syntax('::')],
    parse_with_error_handling__(conditions__(T),
                                'Can\'t parse :: body'),
    {Query = '$vars_query'(Vars, T)}.
query__(Query) -->
    [syntax('exists')], !,
    %% ( "exists", var_seq, conditions )
    (
      var_seq__(Vars)
    ->
      parse_with_error_handling__(conditions__(T),
                                  'Can\'t parse exists body'),
      {Query = '$exists'(Vars, T)}
    ;
      error_handler__('Expecting bound variables')
    ).
query__(Query) -->
    %% basic_query
    basic_query__(Query),
    at_end__, !.

%% ----------------------------------------------------
% basic_query =
%%     ( "prolog" ) |
%%     ( "bs" ) |
%%     ( "logging", atom ) |  
%%     ( "logging", "(", atom , ")" ) |  
%%     ( "logging", atom, "@", atom ) |  
%%     ( "logging", "(", atom, "@", atom , ")" ) |  
%%     ( "unlog" ) |
%%     ( "types", {atom_or_syntax_seq} ) |
%%     ( "stypes", {atom_or_syntax_seq} ) |
%%     ( "show", {atom_or_syntax_seq} ) |
%%     ( "watch", {atom_seq} ) |
%%     ( "watch", "(", {atom_seq}, ")" ) |
%%     ( "unwatch", {atom_seq} ) |
%%     ( "unwatch", "(", {atom_seq}, ")" ) |
%%     ( "watched" ) |
%%     ( "answers", int ) |
%%     ( "consult", atom ) |
%%     ( "consult", "(", atom, ")" ) |
%%     ( "pconsult", atom ) |
%%     ( "pconsult", "(", atom, ")" ) |
%%     ( "[", atom, "]" ) |
%%     conditions |
%%     action_seq;
%% ----------------------------------------------------

basic_query__(prolog) -->
    %% ( "prolog" )
    atom__(prolog), !, no_leftover__.
basic_query__(bs) -->
    %% ( "bs" )
    atom__(bs), !, no_leftover__.
basic_query__(logging(Addr)) -->
    atom__(logging), any_open_bracket__, !,
    %%( "logging", "(", atom , ")" ) | ( "logging", "(", atom, "@", atom , ")" )
    atom_with_check__(Process),
    (
      syntax__('@')
    ->
      atom_with_check__(Machine),
      { Addr = '@'(Process, Machine) }
    ;
      { Addr = '@'(Process, localhost) }
    ),
    matching_bracket__(cb(')')), no_leftover__.
basic_query__(logging(Addr)) -->
    atom__(logging), !,
    %% ( "logging", "(", atom , ")" ) | ( "logging", atom, "@", atom )
    atom_with_check__(Process),
    (
      syntax__('@')
    ->
      atom_with_check__(Machine),
      { Addr = '@'(Process, Machine) }
    ;
      { Addr = '@'(Process, localhost) }
    ),
    no_leftover__.
basic_query__(unlog) -->
    atom__(unlog), !.
basic_query__(Types) -->
    %% ( "types", {atom_seq} )
    atom__(types), !,
    (
      at_end__
    ->
      {Types = types}
    ;
      atom_or_syntax_seq__(T),
      {Types = types(T)}
    ).
basic_query__(STypes) -->
    atom__(stypes), !,
    (
      at_end__
    ->
      {STypes = stypes}
    ;
      atom_or_syntax_seq__(T),
      {STypes = stypes(T)}
    ).
basic_query__(Show) -->
    %% ( "stypes", {atom_seq} )
    atom__(show), !,
    (
      at_end__
    ->
      {Show = show}
    ;
      atom_or_syntax_seq__(T),
      {Show = show(T)}
    ).
basic_query__(watch(T)) -->
    atom__(watch), any_open_bracket__, !,
    atom_seq__(T),
    matching_bracket__(cb(')')).
basic_query__(watch(T)) -->
    atom__(watch), !,
    atom_seq__(T).
basic_query__(unwatch) -->
    atom__(unwatch), at_end__, !.
basic_query__(unwatch(T)) -->
    atom__(unwatch), any_open_bracket__, !,
    atom_seq__(T),
    matching_bracket__(cb(')')).
basic_query__(unwatch(T)) -->
    atom__(unwatch), !,
    atom_seq__(T).
basic_query__(watched) -->
    %% ( "watched" )
    atom__(watched), !, no_leftover__.
basic_query__(answers(N)) -->
    %% ( "answers", int )
    atom__(answers), !,
    (
      int__(N)
    ->
      []
    ;
      error_handler__('Expecting integer')
    ).
basic_query__(consult(File)) -->
    %% ( "consult", atom ) 
    %% ( "consult", "(", atom, ")" ) 
    atom__(consult), !,
    (
      any_open_bracket__
    ->
      atom_with_check__(File),
      matching_bracket__(cb(')'))
    ;
      atom_with_check__(File)
    ).
basic_query__(pconsult(File)) -->
    %% ( "pconsult", atom ) 
    %% ( "pconsult", "(", atom, ")" ) 
    atom__(pconsult), !,
    (
      any_open_bracket__
    ->
      atom_with_check__(File),
      matching_bracket__(cb(')'))
    ;
      atom_with_check__(File)
    ).

basic_query__(Conditions) -->
    \+ has_token__(syntax(';')),
    %% conditions
    conditions__(Conditions), at_end__, !.
basic_query__(Actions) -->
    %% actions
    action_seq__(Actions), at_end__, !.
basic_query__(consult(File)) -->
    %% ( "[", atom_seq, "]" )
    [ob('[')],
    atom__(File),
    matching_bracket__(cb(']')),
    at_end__, !.



%% ----------------------------------------------------
%% program_item =
%%     ( "?-", term ) |
%%     enum_definition |
%%     macro_definition |
%%     declaration |
%%     ( compound_term, rel_rule_body ) |
%%     ( compound_term, act_rule_body ) |
%%     ( compound_term, fun_rule_body ) |
%%     ( simple_compound, tel_procedure_body );

%% enum_definition = "def", definition_head, "::=", enum_type {string};
%% macro_definition = "def", definition_head, "==", macro_type {string};
%% ----------------------------------------------------

program_item__('?-'(G)) -->
    %% ( "?-", term )
    [syntax('?-')], term__(G), no_leftover__.
program_item__(PI) --> %% type or macro defn
    %% enum_definition |
    %% macro_definition     
    syntax__(def), !,
    definition_head_(DefHead),
    (
      [syntax('::=')]
    ->
      enum_type_(Type),
      (
        at_end__
      ->
        {PI = (DefHead ::= Type)}
      ;
        parse_with_error_handling__(string__(Doc), 'Expecting string'),
        {PI = def_doc((DefHead ::= Type), Doc)}
      )
    ;
      [op('==')]
    ->
      macro_type_(Type),
      (
        at_end__
      ->
        {PI = (DefHead == Type)}
      ;
        parse_with_error_handling__(string__(Doc), 'Expecting string'),
        {PI = def_doc((DefHead == Type), Doc)}
      )
    ;
      error_handler__('Expecting ::= or ==')
    ).
    
program_item__(PI) -->
    %% declaration
    declaration_(PI), !.
program_item__(PI) -->  %%rule defs
    parse_with_error_handling__(compound_term__(Head),
                                'Expecting compound term'),
    (
      \+ [ob('{')]
    ->
      []
    ;
      {functor(Head, F, _), atom(F)}
    ->
      []
    ;
      error_handler__('Teleo procedure head is not a simple compound')
    ),
    (
      %% tel_procedure_definition
      [ob('{')]
    ->
      tel_procedure_body_(Rules),
      { PI = '$tel'(Head, Rules) }
    ;
      (
        [syntax('::')]
      ->
        conditions__(Guard)
      ;
        {Guard = '$$none$$'}
      ),
 
      (
        [syntax('~>')]
      ->
        %% act_rule_definition
        act_rule_body_(Head, Guard, PI)
      ;
        [syntax('->')]
      ->
        %% fun_rule_definition
        fun_rule_body_(Head, Guard, PI)
      ;
        %% rel_rule_definition
        rel_rule_body_(Head, Guard, PI)
      )
    ).

%% ----------------------------------------------------
%% definition_head =  atom |  ( atom, bracketed_var_seq ); 
%% ----------------------------------------------------
definition_head_(_) -->
    next_token__(syntax(A)), !,
    error_handler__(['The reserved symbol \'', A, '\' can\'t be used in a rule head']).
definition_head_(Head) -->
    atom_with_check__(F),
    (
      \+ any_open_bracket__
    ->
      {Head = F}
    ;
      bracketed_var_seq__(Args),
      {Head =.. [F|Args]}
    ).

%% ----------------------------------------------------
%% macro_type =
%%      ( atom_or_simple_compound,  "||", 
%%          atom_or_simple_compound,  {"||", atom_or_simple_compound} ) |
%%      macro_type_expression;
%% ----------------------------------------------------
macro_type_('$union_type'([T|Rest])) -->
    has_token__(syntax('||')), !,
    %% ( atom_or_simple_compound,  "||", 
    %%     atom_or_simple_compound,  {"||", atom_or_simple_compound} )
    ( 
      atom_or_simple_compound_(T)
    ->
      '$seq_rest'(atom_or_simple_compound, syntax('||'), Rest,
             'Expecting an atom or a simple compound')
    ;
      error_handler__('Expecting atom or simple compound')
    ).
macro_type_(T) -->
    %% macro_type_expression
    macro_type_expression__(T), !.
macro_type_(_) -->
    error_handler__('Expecting macro type').

%% ----------------------------------------------------
%% enum_type =
%%      ( int, "..", int ) | 
%%      ( atom, "|", atom, {"|", atom} ) |
%%      ( string, "|", string, {"|", string} ) |
%%      ( anynum, "|", anynum, {"|", anynum} ) |
%%      ( simple_compound, {"|", simple_compound} ) |
%%      ( "(", enum_type, ")" ); 
%% ----------------------------------------------------
enum_type_(Type) -->
    %% ( "(", enum_type, ")" )
    any_open_bracket__, !,
    enum_type_(Type),
    matching_bracket__(cb(')')).
enum_type_((T1 .. T2)) -->
    %% ( int, "..", int )
    int__(T1),
    [syntax('..')], !,
    parse_with_error_handling__(int__(T2), 'Expecting integer').
enum_type_(_) -->
    next_token__(syntax(A)), !,
    error_handler__(['The reserved symbol \'', A, '\' can\'t be used as an atom']).
enum_type_('$enum_type'([A|Atoms])) -->
    atom__(A), 
    next_token__(syntax('|')), !,
    '$seq_rest'(atom, syntax('|'), Atoms, 'Expecting atom').
enum_type_('$enum_type'([A])) -->
    atom__(A), \+any_open_bracket__, !.
enum_type_('$enum_type'([S|Strings])) -->
    %% ( string, {"|", string} )
    string__(S), 
    next_token__(syntax('|')), !,
    '$seq_rest'(string__, syntax('|'), Strings, 'Expecting string').
enum_type_('$enum_type'([A])) -->
    string__(A), \+any_open_bracket__, !.
enum_type_('$enum_type'([N|Numbers])) -->
    %% ( anynum, {"|", anynum} )
    anynum__(N), 
    next_token__(syntax('|')), !,
    '$seq_rest'(anynum, syntax('|'), Numbers, 'Expecting number').
enum_type_('$constr_enum_type'(Comp)) -->
    %% ( simple_compound, {"|", simple_compound} )
    '$seq'(simple_compound, syntax('|'), Comp,
           'Expecting a simple compound').
enum_type_(_) -->
    error_handler__('Expecting enum type').


%% ----------------------------------------------------
%% macro_type_expression =
%%     simple_type_compound |
%%     atom |
%%      ( "(", type_expression, ",", type_expression, 
%%              {",", type_expression}, ")" ) |  
%%      code_type_expression;
%% ----------------------------------------------------
macro_type_expression__(TE) -->
    code_type_expression__(TE), !.
macro_type_expression__(SC) -->
    %% simple compound
    simple_type_compound_(SC), !.
macro_type_expression__(SC) -->
    %% atom
    atom__(SC), !.
macro_type_expression__(TE) -->
    %% ( "(", type_expression, ",", type_expression, 
    %%         {",", type_expression}, ")" )
    any_open_bracket__, !,
    (
      type_expression__(TE1)
    ->
      '$tuple_type_expression_aux'(TE2),
      {TE = '$tuple_type'([TE1|TE2])}
    ;
      error_handler__('Expecting type expression')
    ).

%% ----------------------------------------------------
%% type_expression =
%%      code_type_expression |
%%      var |
%%      simple_type_compound |
%%      atom |
%%      ( "(", type_expression, ")" ) |
%%      ( "(", type_expression, ",", type_expression, 
%%              {",", type_expression}, ")" ) ;
%% ----------------------------------------------------
type_expression__(CodeType) -->
    code_type_expression__(CodeType),!.
type_expression__(_) -->
    next_token__(syntax('!')),!,
    error_handler__(['Mode ! not allowed in an unmoded type expression']).
type_expression__(_) -->
    next_token__(syntax('?')),!,
    error_handler__(['Mode ? not allowed in an unmoded type expression']).
type_expression__(_) -->
    next_token__(syntax('??')),!,
    error_handler__(['Mode ?? not allowed in an unmoded type expression']).
type_expression__(_) -->
     next_token__(syntax(A)), !,
    error_handler__(['The reserved symbol \'', A, '\' can\'t be used in a type expression']).
type_expression__(V) -->
    %% var
    var_(V), !.
type_expression__(SC) -->
    %% simple compound
    simple_type_compound_(SC), !.
type_expression__(A) -->
    %% atom
    atom__(A), !.
type_expression__(TE) -->
    %% ( "(", type_expression, ")" )
    %% ( "(", type_expression, ",", type_expression, 
    %%         {",", type_expression}, ")" )
   any_open_bracket__, !,
    (
      type_expression__(TE1)
    ->
      (
        [cb(')')]
      ->
        {TE = TE1}
      ;
        '$tuple_type_expression_aux'(TE2),
        {TE = '$tuple_type'([TE1|TE2])}
      )
    ;
      error_handler__('Expecting type expression')
    ).

'$tuple_type_expression_aux'([]) -->
    [cb(')')], !.
'$tuple_type_expression_aux'([T1|Ts]) -->
    [punct(',')],
    type_expression__(T1),
    '$tuple_type_expression_aux'(Ts).
'$tuple_type_expression_aux'(_Ts) -->
    error_handler__('Expecting , or ) ').


%% ----------------------------------------------------
%% code_type_expression =
%%      fun_type_expression |
%%      dyn_type_expression |
%%      rel_type_expression |
%%      act_type_expression |
%%      tel_type_expression;
%% ----------------------------------------------------
code_type_expression__(CodeType) -->
    %%\+ \+ any_open_bracket__,
    fun_type_expression__(CodeType), !.
code_type_expression__(CodeType) -->
    dyn_type_expression__(CodeType), !.
code_type_expression__(CodeType) -->
    rel_type_expression__(CodeType), !.
code_type_expression__(CodeType) -->
    act_type_expression__(CodeType), !.
code_type_expression__(CodeType) -->
    tel_type_expression__(CodeType), !.


%% fun_type_expression = 
%%     ( "fun", bracketed_type_seq, "->",  type_expression ) ;
fun_type_expression__(FunType) -->
    syntax__(fun), !,
    parse_with_error_handling__(bracketed_type_seq__(Dom),
                                'Expecting bracketed type sequence'),
    (
      [syntax('->')]
    ->
      []
    ;
      error_handler__('Expecting ->')
    ),
    (
      type_expression__(Range)
    ->
      {FunType = ('$tuple_type'(Dom) -> Range)}
    ;
      error_handler__('Expecting type expression')
    ).

%% dyn_type_expression = 
%%     ( "dyn", bracketed_type_seq);
dyn_type_expression__(DynType) -->
    syntax__(dyn), !,
    parse_with_error_handling__(bracketed_type_seq__(Types),
                                'Expecting bracketed type sequence'),
    {DynType = dyn('$tuple_type'(Types))}.
       
       
%% rel_type_expression = 
%%     ( "rel", bracketed_annotated_type_seq);
rel_type_expression__(RelType) -->
    syntax__(rel), !,
    parse_with_error_handling__(bracketed_annotated_type_seq__(Types),
                                 'Expecting bracketed annotated type sequence'),
    {RelType = rel('$tuple_type'(Types))}.
       
%% act_type_expression = 
%%     ( "act", bracketed_annotated_type_seq);
act_type_expression__(ActType) -->
    syntax__(act), !,
    parse_with_error_handling__(bracketed_annotated_type_seq__(Types),
                                 'Expecting bracketed annotated type sequence'),
    {ActType = act('$tuple_type'(Types))}.

%% tel_type_expression = 
%%     ( "tel", bracketed_type_seq);   
tel_type_expression__(TelType) -->
    syntax__(tel), !,
    parse_with_error_handling__(bracketed_type_seq__(Types),
                                'Expecting bracketed type sequence'),
    {TelType = tel('$tuple_type'(Types))}.
       

%% ----------------------------------------------------
%% annotated_type_expression =
%%     mode_annotation |
%%     ( mode_annotation , inner_annotated_type_expression ) |
%%     inner_annotated_type_expression;
%% ----------------------------------------------------

    
annotated_type_expression__(AT(term)) -->
    mode_annotation_(AT),
    \+ \+ ( [punct(',')] ; [cb(')')] ), !.
annotated_type_expression__(Mode(Type)) -->
    mode_annotation_(Mode),
    inner_annotated_type_expression__(Type), !.
annotated_type_expression__(AT) -->
    inner_annotated_type_expression__(AT), !.
annotated_type_expression__(_) -->
    error_handler__('Expecting annotated type expression').

%% ----------------------------------------------------
%% mode_annotation = "!" | "?" | "??" ; 
%% ----------------------------------------------------
mode_annotation_('!') --> [syntax('!')], !.
mode_annotation_('?') --> [syntax('?')], !.
mode_annotation_('??') --> [syntax('??')], !.


%% ----------------------------------------------------
%% inner_annotated_type_expression =
%%    type_expression |
%%    ( atom, bracketed_annotated_type_seq);
%% ----------------------------------------------------
inner_annotated_type_expression__(Type) -->
    atom__(F), next_token__(ob('(')), !,
    bracketed_annotated_type_seq__(Types),
    { Type =.. [F|Types] }.
inner_annotated_type_expression__(Type) -->
    type_expression__(Type), !.
inner_annotated_type_expression__(_) -->
    error_handler__('Expecting type expression or type constructor').


%% ----------------------------------------------------
%% declaration =
%%  ( "fun", fun_declaration, {",", fun_declaration}, [ string ] ) |
%%  ( "act", annotated_declaration, {",", annotated_declaration}, [ string ] ) |
%%  ( "rel", annotated_declaration, {",", annotated_declaration}, [ string ] ) |
%%  ( "dyn", unannotated_declaration,
%%           {",", unannotated_declaration}, [ string ] ) |
%%  ( "mrel", annotated_declaration,
%%           {",", annotated_declaration}, [ string ] ) |
%%  ( "mfun", fun_declaration, {",", fun_declaration}, [ string ] ) |
%%  ( ( "tel" | "tel_start" | "tel_atomic" ), unannotated_declaration,
%%           {",", unannotated_declaration}, [ string ] ) |
%%  ( "tel_percept", unannotated_declaration,
%%           {",", unannotated_declaration} ) |
%%  ( "tel_action", ( unannotated_declaration,
%%           {",", unannotated_declaration} ) |
%%  global_num_declaration;
%% ----------------------------------------------------
      

declaration_('$code_decl'(fun, Decls, Doc)) -->
    [syntax(fun)], !,
    %% "fun", fun_declaration, {",", fun_declaration}, [ string ]  
    '$seq'(fun_declaration_, punct(','), Decls,
           'Expecting function declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(mfun, Decls, Doc)) -->
    [syntax(mfun)], !,
    %% "mfun", fun_declaration, {",", fun_declaration}, [ string ]  
    '$seq'(fun_declaration_, punct(','), Decls,
           'Expecting function declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(act, Decls, Doc)) -->
    [syntax(act)], !,
    %% "act", annotated_declaration, {",", annotated_declaration}, [ string ]
    '$seq'(annotated_declaration_, punct(','), Decls,
           'Expecting annotated declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(rel, Decls, Doc)) -->
    [syntax(rel)], !,
    %% "rel", annotated_declaration, {",", annotated_declaration}, [ string ]
    '$seq'(annotated_declaration_, punct(','), Decls,
           'Expecting annotated declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(dyn, Decls, Doc)) -->
    [syntax(dyn)], !,
    %% "dyn", unannotated_declaration,
    %%     {",", unannotated_declaration}, [ string ]
    '$seq'(unannotated_declaration_, punct(','), Decls,
           'Expecting unannotated declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(mrel, Decls, Doc)) -->
    [syntax(mrel)], !,
    %% "mrel", unannotated_declaration,
    %%     {",", unannotated_declaration}, [ string ] 
    '$seq'(annotated_declaration_, punct(','), Decls,
           'Expecting annotated declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(tel, Decls, Doc)) -->
    [syntax(tel)], !,
    %% "tel" , unannotated_declaration,
    %%           {",", unannotated_declaration}, [ string ] 
    '$seq'(unannotated_declaration_, punct(','), Decls,
           'Expecting unannotated declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(tel_start, Decls, Doc)) -->
    [syntax(tel_start)], !,
    %%  "tel_start" , unannotated_declaration,
    %%           {",", unannotated_declaration}, [ string ] 
    '$seq'(unannotated_declaration_, punct(','), Decls,
           'Expecting unannotated declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(tel_atomic, Decls, Doc)) -->
    [syntax(tel_atomic)], !,
    %% "tel_atomic", unannotated_declaration,
    %%           {",", unannotated_declaration}, [ string ]
    '$seq'(unannotated_declaration_, punct(','), Decls,
           'Expecting unannotated declaration'),
    (
      at_end__
    ->
      {Doc = ""}
    ;
      parse_with_error_handling__(string__(Doc), 'Expecting string')
    ).
declaration_('$code_decl'(percept, Decls, "")) -->
    [syntax(tel_percept)], !,
    {ip_set('$in_teleor_pa', true)},
    '$seq'(percept_declaration_, punct(','), Decls,
           'Expecting percept declaration'),
    {ip_set('$in_teleor_pa', _)}.
declaration_(def_doc((tel_action_term ::= '$constr_enum_type'(Type)), "")) -->
    [syntax(tel_action)], !,
    {ip_set('$in_teleor_pa', true)},
    '$seq'(simple_compound, punct(','), Type, 'Expecting a simple compound'),
    {ip_set('$in_teleor_pa', _)}.

declaration_(global_int_decl(A, N)) -->
    %% ("int", atom, ":=", int)
    atom__(int), !,
    atom_with_check__(A),
    op_with_check__(':='),
    parse_with_error_handling__(int__(N), 'Expecting integer').
declaration_(global_num_decl(A, N)) -->
    %% ("num", atom, ":=", num)
    atom__(num), !,
    atom_with_check__(A),
    op_with_check__(':='),
    parse_with_error_handling__(anynum__(N), 'Expecting integer').

    
%% ----------------------------------------------------
%% fun_declaration =
%%    unannotated_declaration, "->", type_expression;
%% ----------------------------------------------------
fun_declaration_('$decl'(Fun, (DT -> RT))) -->
    unannotated_declaration_('$decl'(Fun,DT)),
    syntax_with_check__('->'),
    parse_with_error_handling__(type_expression__(RT),
                                'Expecting type expression').

%% ----------------------------------------------------
%% percept_declaration =
%%     atom, (
%%         ( "(", ")" ) |
%%         ( "(", [var, ":"], type_expression,
%%            { ",",  [var, ":"], type_expression}, ")" )
%%         );
%% ----------------------------------------------------
percept_declaration_(Decl) -->
    atom_with_check__(Name),
    '$bracketed_seq'(unannotated_declaration_arg__,  punct(','), Types,
                     'Expecting type expression'),
    { Decl = '$decl'(Name, '$tuple_type'(Types)) }.
%% ----------------------------------------------------
%% tel_action_declaration =
%%     atom, (
%%         ( "(", ")" ) |
%%         ( "(", [var, ":"], type_expression,
%%            { ",",  [var, ":"], type_expression}, ")" )
%%         );
%% ----------------------------------------------------
tel_action_declaration_(Decl) -->
    atom_with_check__(Name),
    '$bracketed_seq'(unannotated_declaration_arg__,  punct(','), Types,
                     'Expecting type expression'),
    { Decl = Name('$tuple_type'(Types)) }.

%% %% ----------------------------------------------------
%% unannotated_declaration =
%%     atom, (
%%         ( "(", ")" ) |
%%         ( "(", [var, ":"], type_expression, ["default", term],
%%            { ",",  [var, ":"], type_expression}, ")" )
%%         );
%% ----------------------------------------------------
unannotated_declaration_(Decl) -->
    atom_with_check__(Name),
    '$bracketed_seq'(unannotated_declaration_arg__,  punct(','), Types,
                     'Expecting type expression'),
    { Decl = '$decl'(Name, '$tuple_type'(Types)) }.

%% annotated_declaration =
%%     atom, (
%%         ( "(", ")" ) |
%%         ( "(", [var, ":"], annotated_type_expression, ["default", term],
%%            { ",",  [var, ":"], annotated_type_expression}, ")" )
%%         );
annotated_declaration_(Decl) -->
    atom_with_check__(Name),
    '$bracketed_seq'(annotated_declaration_arg__,  punct(','), Types,
           'Expecting annotated type expression'),
    { Decl = '$decl'(Name, '$tuple_type'(Types)) }.

%% ----------------------------------------------------
%% rel_rule_body =
%%     ["::", conditions], ["<=", conditions];
%% ----------------------------------------------------
rel_rule_body_(Head, Guard, Rule) -->
    (
      [syntax('<=')]
    ->
      parse_with_error_handling__(conditions__(Body), "Expecting conditions")
    ;
      {Body = '$$none$$'}
    ),
    no_leftover__,
    {
     (
       Guard = '$$none$$', Body = '$$none$$'
     ->
       Rule = Head
     ;
       Guard = '$$none$$'
     ->
       Rule = (Head <= Body)
     ;
       Body = '$$none$$'
     ->
       Rule = (Head :: Guard)
     ;
       Rule = (Head :: Guard <= Body)
     )
    }.
%% ----------------------------------------------------
%% act_rule_body =
%%     ["::", conditions], "~>", action_seq;
%% ----------------------------------------------------
act_rule_body_(Head, Guard, Rule) -->
    action_seq__(Body),
    no_leftover__,
    {
     (
       Guard = '$$none$$'
     ->
       Rule = (Head ~> Body)
     ;
       Rule = (Head :: Guard ~> Body)
     )
    }.
%% ----------------------------------------------------
%% fun_rule_body =
%%     ["::", conditions], "->", action_seq;
%% ----------------------------------------------------
fun_rule_body_(Head, Guard, Rule) -->
    term__(Body),
    no_leftover__,
    {
     (
       Guard = '$$none$$'
     ->
       Rule = (Head -> Body)
     ;
       Rule = (Head :: Guard -> Body)
     )
    }.

%% ----------------------------------------------------
%% conditions = a_condition, { "&", a_condition };
%% ----------------------------------------------------
conditions__(Conds) -->
    a_condition__(C), !,
    (
      [syntax('&')],
      parse_with_error_handling__(conditions__(Conds1), 'Expecting conditions'),
      { Conds = (C & Conds1) }
    ->
      []
    ;
      next_token__(syntax('&'))
    ->
      error_handler__('Unexpected &')
    ;
      next_token__(syntax(';'))
    ->
      error_handler__('Unexpected ;')
    ;
      { Conds = C }
    ).

%% ----------------------------------------------------
%% a_condition =
%%     ( "forall", var_seq,
%%        "(", !, exists_conditions, "=>", exists_conditions, ")" ) |
%%     ( "not", exists_a_condition ) |
%%     ( "not", "(", exists_conditions, ")" ) |
%%     ( "once", a_condition ) |
%%     ( "once", "(", conditions, ")" ) |
%%     ( "(", int, "of", var_type_seq, "::",
%%                 exists_conditions__, ")" , "query_at", pedro_handle ) |
%%     ("(", var_type_seq, "::",
%%                 exists_conditions__, ")", "query_at", pedro_handle ) |
%%     ( simple_condition, "query_at", pedro_handle ) |

%%     ("(", query, ")", "query_at", pedro_handle ) |
%%     ( simple_condition, "query_at", pedro_handle ) |

%%     simple_condition ;
%% ----------------------------------------------------

a_condition__(RemoteQuery) -->
    any_open_bracket__,
    {ip_set('$parsing_query_at', true)},
    query__(Query),
    matching_bracket__(cb(')')),
    {ip_set('$parsing_query_at', _)},
    syntax__('query_at'),!,
   (
      pedro_handle_(Addr)
    ->
      []
    ;
      error_handler__('Expecting a pedro handle')
    ),
    {  RemoteQuery = '$remote_query'(Query, Addr) }.
a_condition__(C) -->
    [syntax(forall)], 
    var_seq__(Bound),
    \+next_token__(ob('{')),
    !,
    (
      any_open_bracket__
    ->
      (
        exists_conditions__(Anti),
        syntax_with_check__('=>')
      ->
        (
          exists_conditions__(Conc),
          matching_bracket__(cb(')'))
        ->
          []
        ;
          error_handler__('Expecting exists_conditions')
        )
      ;
        error_handler__('Expecting exists_conditions')
      )
    ;
      error_handler__('Expecting (')
    ),
    { C = '$forall'(Bound, Anti, Conc) }.
a_condition__(C) -->
    atom__(not), !,
    %%  ( "not", exists_a_condition ) |
    %%  ( "not", "(", exists_conditions, ")" )
    (
      any_open_bracket__
    ->
      parse_with_error_handling__(exists_conditions__(Conds),
                                  'Expecting exists_conditions'),
      matching_bracket__(cb(')')),
      { C = not(Conds) }
    ;
      exists_a_condition__(Cond),
      { C = not(Cond) }
    ).
a_condition__(C) -->
    atom__(once), !,
    %%  ( "once", a_condition ) |
    %%  ( "once", "(", conditions, ")" )
    (
      any_open_bracket__
    ->
      parse_with_error_handling__(conditions__(Conds),
                                  'Expecting conditions'),
      matching_bracket__(cb(')')),
      { C = once(Conds) }
    ;
      parse_with_error_handling__(a_condition__(Cond),
                                  'Expecting a_condition'),
      { C = once(Cond) }
    ).

% a_condition__(RemoteQuery) -->
%     any_open_bracket__,
%     next_tokens__([var(_,_)]),
%     var_seq__(VarTypes),
%     syntax__('::'), !,
%     exists_conditions__(Q),
%     matching_bracket__(cb(')')),
%     syntax_with_check__('query_at'),
%     pedro_handle_(Addr),
%     { RemoteQuery = '$remote_query'(VarTypes, Q, Addr) }.
a_condition__(RemoteQuery) -->
    simple_condition_(Q),
    syntax__('query_at'), !,
    parse_with_error_handling__(pedro_handle_(Addr), 'Expecting pedro_handle'),
    { RemoteQuery = '$remote_query'(Q, Addr) }.
a_condition__(Q) -->
     simple_condition_(Q).
%% ----------------------------------------------------
%% exists_a_condition =
%%     ( "exists", var_seq, simple_condition ) |
%%     a_condition;
%% ----------------------------------------------------

exists_a_condition__('$exists'(Vars, G)) -->
    syntax__(exists), !,
    var_seq__(Vars),
    parse_with_error_handling__(simple_condition_(G),
                                'Expecting simple_condition').

exists_a_condition__(G) -->
    a_condition__(G).



%% ----------------------------------------------------
%% exists_conditions =
%%     ( "exists", var_seq, a_condition ) |  conditions;
%% ----------------------------------------------------

exists_conditions__('$exists'(Vars, G)) -->
    syntax__(exists), !,
    var_seq__(Vars),
    parse_with_error_handling__(a_condition__(G), 'Expecting a_condition').
exists_conditions__(G) -->
    conditions__(G).

%% ----------------------------------------------------
%% simple_condition =
%%     ( "(", conditions, ")" ) |
%%     "true" | "false" |
%%     ( "type", "(", term, ",", annotated_type_expression, ")" ) |
%%     ( "listof", "(", term, ",", term, "::", exists_conditions, ")" ) |
%%     compound_term |
%%     ( term, "=?", qeqrhs) |
%%     ( term, test_op, term );
%% ----------------------------------------------------
simple_condition_(_G) -->
    next_token__(syntax(exists)), !,
    %syntax__(exists), !,
    error_handler__('Unexpected exists').
simple_condition_(G) -->
    any_open_bracket__, 
    conditions__(G), !,
    matching_bracket__(cb(')')).
simple_condition_(true('$none_')) -->
    atom__(true), !.
simple_condition_(false('$none_')) -->
    atom__(false), !.
simple_condition_(type(T, Type)) -->
    atom__(type), !,
    ob_with_check__,
    term__(T),
    comma_with_check__,
    parse_with_error_handling__(annotated_type_expression__(Type),
                                'Expecting annotated_type_expression'),
    matching_bracket__(cb(')')).
simple_condition_('$pfindall'(Result, Pattern, Goal)) -->
    atom__(listof), !,
    ob_with_check__,
    term__(Result),
    comma_with_check__,
    term__(Pattern),
    syntax_with_check__('::'),
    parse_with_error_handling__(exists_conditions__(Goal),
                                'Expecting exists_conditions'),
    matching_bracket__(cb(')')).
simple_condition_(Cond) -->
    term__(T),
    (
      next_token__(syntax('=?'))
    ->
      !,
      {ip_lookup(in_eqq__, EQQ)},
      (
        {EQQ == true}
      ->
        error_handler__('Error: \'=?\' appears inside an outer \'=?\' - please use an auxilary relation.')
      ;
        {ip_set(in_eqq__, true)},
        [syntax('=?')],
        parse_with_error_handling__(qeqrhs_(QEQ),'Can\' parse after \'=?\''),
        {Cond = '=?'(T, QEQ), ip_set(in_eqq__, _)}
      )
    ;
      test_op_(Op)
    ->
      !, term__(RHS),
      { Cond = Op(T, RHS) }
    ;
      { fail }
    ).
simple_condition_(Cond) -->
    compound_term__(Cond).   

%% ----------------------------------------------------
%% qeqrhs =
%%    ( qeqrhs_string_term, "++", qeqrhs_string_term,
 %%               {"++", qeqrhs_string_term} ) |
 %%   ( qeqrhs_list_term, "<>", qeqrhs_list_term, {"<>", qeqrhs_list_term} );
%% ----------------------------------------------------
qeqrhs_((ST ++? Rest)) -->
    qeqrhs_string_term__(ST, Slash),
    (
      {Slash == true}
    ->
      !, op_with_check__('++?')
    ;
      [op('++?')], !
    ),
    '$qeqrhs_string_aux'(Rest).
qeqrhs_((ST <>? Rest)) -->
    qeqrhs_list_term__(ST),
    [op('<>?')], !,
    '$qeqrhs_list_aux'(Rest).

'$qeqrhs_string_aux'(StrPatt) -->
    (
      qeqrhs_string_term__(ST, _Slash)
    ->
      []
    ;
      error_handler__('Expecting string term')
    ),
    (
      [op('++?')]
    ->
      !,
      '$qeqrhs_string_aux'(StrPatt1),
      { StrPatt = (ST ++? StrPatt1) }
    ;
      { StrPatt = ST }
    ).

'$qeqrhs_list_aux'(LstPatt) -->
    (
      qeqrhs_list_term__(LT)
    ->
      []
    ;
      error_handler__('Expecting list term')
    ),
    (
      [op('<>?')]
    ->
      '$qeqrhs_list_aux'(LstPatt1),
      { LstPatt = (LT <>? LstPatt1) }
    ;
      { LstPatt = LT }
    ).

%% ----------------------------------------------------
%% qeqrhs_string_term =
%%     ( simple_string_term, "::", conditions ) |
%%     simple_string_term;
%% ----------------------------------------------------
qeqrhs_string_term__(ST, Slash) -->
    simple_string_term__(ST1, Slash),
    (
      syntax__('::')
    ->
      parse_with_error_handling__(conditions__(Cond), 'Expecting conditions'), 
      { ST = '::'(ST1, Cond) }
    ;
      { ST = ST1 }
    ).
%% ----------------------------------------------------
%% qeqrhs_list_term =
%%     ( simple_list_term, "::", conditions ) |
%%     simple_list_term;
%% ----------------------------------------------------
qeqrhs_list_term__(LT) -->
    simple_list_term__(LT1),
    (
      syntax__('::')
    ->
      parse_with_error_handling__(conditions__(Cond), 'Expecting conditions'),
      { LT = '::'(LT1, Cond) }
    ;
      { LT = LT1 }
    ).

%% ----------------------------------------------------
% simple_string_term =
%     string | var | ( var, "/", ( string | var ) ) | compound_term;
%% ----------------------------------------------------
simple_string_term__(ST, _) -->
    string__(ST), !.
simple_string_term__(ST, Slash) -->
    var_(V), \+ next_token__(ob('(')), !,
    (
      [op('/')]
    ->
      ( string__(RE) ; var_(RE) ),
      { ST = '$re'(V, RE), Slash = true }
    ;
      {ST = V}
    ).
simple_string_term__(ST) -->
    compound_term__(ST).

%% ----------------------------------------------------
% simple_list_term =
%     var | compound_term | list_term | list_comprehension ;
%% ----------------------------------------------------
simple_list_term__(V) -->
    var_(V), \+ next_token__(ob('(')), !.
simple_list_term__(LT) -->
    list_comprehension_(LT), !.
simple_list_term__(LT) -->
    list_term__(LT), !.
simple_list_term__(CT) -->
    compound_term__(CT).

%% ----------------------------------------------------
%% list_term =
%%    ( "[", "]" ) |
%%    "[", term, list_tail;
%%
%% list_tail =
%%    "]" |
%%    ( "|", term, "]" ) |
%%    ( ",..", "]" ) |
%%    ( ",..", term, "]" ) |
%%    ( term, ",", list_tail );
%% ----------------------------------------------------
list_term__([]) -->
    [ob('['), cb(']')], !.
list_term__([H|T]) -->
    [ob('[')], term__(H), list_tail(T).

list_tail([]) -->
    [cb(']')], !.
list_tail(T) -->
    [syntax('|')], !, term__(T), matching_bracket__(cb(']')).
list_tail(T) -->
    [punct(','), syntax('..')], !,
    (
      [cb(']')]
    ->
      {T = _}
    ;
      term__(T),
      matching_bracket__(cb(']'))
    ).
list_tail([H|T]) --> [punct(',')], !,
    (
      term__(H)
    ->
      list_tail(T)
    ;
      error_handler__('Expecting term')
    ).
list_tail(_) -->
    error_handler__('Expecting ] , ,.. or |').


%% ----------------------------------------------------
%% list_comprehension = 
%%     ( "[", simple_term, "::", exists_conditions, "]" );
%% ----------------------------------------------------
list_comprehension_('$list_constr'(Patt, Goal)) -->
    [ob('[')],
    \+ [cb(']')],
    term__(Patt),
    syntax__('::'),
    parse_with_error_handling__(exists_conditions__(Goal),
                                'Expecting exists_conditions'),
    matching_bracket__(cb(']')).

%% ----------------------------------------------------
%%action_seq = action, {";", action};
%% ----------------------------------------------------
action_seq__(Actions) -->
    action__(A),
    (
      [syntax(';')],
      action_seq__(Actions1),
      { Actions = (A ; Actions1) }
    ->
        []
    ;
      next_token__(syntax(';'))
    ->
      error_handler__('Unexpected ;')
    ;
      next_token__(syntax('&'))
    ->
      error_handler__('Unexpected &')
    ;
      { Actions = A }
    ).

%% ----------------------------------------------------
%% action =
%%     ( "forall", var_seq,
%%        "{", !, exists_conditions, "~>", action_seq, "}" ) |
%%     ( "{", "}" ) |
%%     ( "{", action_seq, "}" ) |
%%     ( atom, global_num_op, term) |
%%     ( "atomic_action", action ) |
%%     ( "case", "{", case_alt, { case_alt }, "}" ) |
%%     ( "wait", "(", conditions, ")" ) |
%%     ( "wait_case", "{", case_alt, { case_alt },
%%                       [ "timeout", term, "~>", action_seq ], "}" ) |
%%     ( "receive", "{", receive_alt, { receive_alt },
%%                       [ "timeout", term, "~>", action_seq ], "}" ) |
%%     ( "try", action, "except",
%%                       "{", except_alt, { except_alt }, "}" ) |
%%     ( "repeat", action, [ "until", a_condition ] ) |
%%     ( "?", "(", exists_conditions, ")" ) |
%%     ( term, "from", agent_handle, ["::", conditions ] ) |
%%     ( term, "from_thread", pedro_handle, ["::", conditions ] ) |
%%     simple_tr_pp_action;
%% ----------------------------------------------------
action__('$null_action') --> 
    [ob('{'), cb('}')], !.
action__(AS) -->
    [ob('{')], !,
    action_seq__(AS),
    matching_bracket__(cb('}')).
action__('$forall_actions'(Bound, LHS, RHS)) -->
    syntax__(forall), !,
    var_seq__(Bound),
    (
      [ob('{')]
    ->
      parse_with_error_handling__(exists_conditions__(LHS),
                                  'Expecting exists_conditions'),
      syntax_with_check__('~>'),
      action_seq__(RHS),
      matching_bracket__(cb('}'))
    ;
      error_handler__('Expecting {')
    ).
action__(atomic_action(Act)) -->
    syntax__(atomic_action), !,
    (
      action__(Act)
    ->
      []
    ;
      error_handler__('Expecting simple action')
    ).
action__(case(Alts)) -->
    syntax__(case), !,
    open_paren_with_check__,
    '$case_alts'(Alts).
action__(wait_case(Alts)) -->
    syntax__(wait_case), !,
    open_paren_with_check__,
    '$wait_alts'(Alts).
action__(wait(Cond)) -->
    syntax__(wait), !,
    ob_with_check__,
    (
      conditions__(Cond)
    ->
      []
    ;
      error_handler__('Expecting conditions')
    ), 
    matching_bracket__(cb(')')).
action__(receive(Alts)) -->
    syntax__(receive), !,
    open_paren_with_check__,
    '$receive_alts'(Alts).
action__(try_except(Try, Except)) -->
    syntax__(try), !,
    action__(Try),
    %open_paren_with_check__,
    %action_seq__(Try),
    %matching_bracket__(cb('}')),
    syntax_with_check__(except),
    open_paren_with_check__,
    '$except_alts'(Except).
action__(repeat_until(Act, Cond)) -->
    syntax__(repeat), !,
    action__(Act),
    %open_paren_with_check__,
    %(
    %  action_seq__(Act)
    %->
    %  []
    %;
    %  error_handler__('Expecting simple action')
    %),
    %matching_bracket__(cb('}')),
    (
      syntax__(until)
    ->
      a_condition__(Cond)
    ;
      {Cond = '$fail'}
    ).
action__('$rel_escape'(Goal)) -->
    syntax__('?'), !,
    (
      any_open_bracket__
    ->
      []
    ;
      error_handler__('Expecting (')
    ),
    (
      exists_conditions__(Goal), matching_bracket__(cb(')'))
    ->
      []
    ;
      error_handler__('Expecting a condition')
    ).
action__(from(T, Addr, Test)) -->
    has_token_before__(op(from), [syntax(';'), cb(['}'])]),
    term__(T),
    [op(from)],
    !,
    agent_handle_(Addr),
    (
     syntax__('::')
    ->
     a_condition__(Test)
    ;
     {Test = '$true'('$none_')}
    ).
action__(from_thread(T, Addr, Test)) -->
    has_token_before__(op(from_thread), [syntax(';'), cb(['}'])]),
    term__(T),
    [op(from_thread)],
    !,
    pedro_handle_(Addr),
    (
     syntax__('::')
    ->
     a_condition__(Test)
    ;
     {Test = '$true'('$none_')}
    ).
action__(A) -->
    simple_tr_pp_action__(A),!.

%% ----------------------------------------------------
%% tr_pp_action =
%%     ( simple_tr_pp_action { ";" simple_tr_pp_action }  )
%% simple_tr_pp_action =
%%     ( term, "to", agent_handle ) |
%%     ( term, "to_thread", pedro_handle ) |
%%     ( atom, global_num_op, term) |
%%     compound_term;
%% ----------------------------------------------------
% tr_pp_action__(AS) -->
%     [ob('{')], !,
%     simple_tr_pp_action_seq__(AS),
%     matching_bracket__(cb('}')).
tr_pp_action__(AS) -->
    simple_tr_pp_action_seq__(AS), !. %simple_tr_pp_action__(A), !.  
tr_pp_action__(_) -->
    error_handler__('Invalid TR rule RHS').

simple_tr_pp_action__(to(T, Addr)) -->
    has_token_before__(op(to), [syntax(';'), cb(['}'])]),
    term__(T),
    [op(to)],!,
    agent_handle_(Addr).
simple_tr_pp_action__(to_thread(T, Addr)) -->
    has_token_before__(op(to_thread), [syntax(';'), cb(['}'])]),
    term__(T),
    [op(to_thread)],!,
    pedro_handle_(Addr).
simple_tr_pp_action__(Op(A, T)) -->
    atom__(A),
    global_num_op_(Op), !,
    term__(T).
simple_tr_pp_action__(CT) -->
    compound_term__(CT), !.

simple_tr_pp_action_seq__(Actions) -->
    simple_tr_pp_action__(A),
    (
      [syntax(';')],
      simple_tr_pp_action_seq__(Actions1),
      { Actions = (A ; Actions1) }
    ->
        []
    ;
      next_token__(syntax(';'))
    ->
      error_handler__('Unexpected ;')
    ;
      next_token__(syntax('&'))
    ->
      error_handler__('Unexpected &')
    ;
      next_token__(op('++'))
    ->
      error_handler__('Unexpected ++')
    ;
      next_token__(syntax('~>'))
    ->
      error_handler__('Unexpected ~>')
    ;
      { Actions = A }
    ).


%% ----------------------------------------------------
%% case_alt =
%%    conditions, "~>", action_seq;
%% ----------------------------------------------------

case_alt__(Alt) -->
    conditions__(Test),
    (
      syntax__('~>')
    ->
      []
    ;
      error_handler__('Expecting ~>')
    ),
    action_seq__(Act),
    { Alt = '$case_alt'(Test, Act) }.

'$case_alts'(Out) -->
    [cb('}')], !,
    {Out = []}.
'$case_alts'(Out) -->
    case_alt__(Alt),
    '$case_alts'(Alts),
    { Out = [Alt|Alts] }.

'$wait_alts'(Out) -->
    [cb('}')], !,
    {Out = []}.
'$wait_alts'(Out) -->
    syntax__(timeout), !,
    term__(T),
    (
      syntax__('~>')
    ->
      []
    ;
      error_handler__('Expecting ~>')
    ),
    action_seq__(Act),
    (
      \+syntax__(';')
    ->
      []
    ;
      error_handler__('Unexpected ;')
    ),
    (
      [cb('}')]
    ->
      {Out = [timeout(T, Act)]}
    ;
      error_handler__('The timeout is not the last alternative')
    ).
'$wait_alts'(Out) -->
    case_alt__(Alt), !,
    '$wait_alts'(Alts),
    { Out = [Alt|Alts] }.

%% ----------------------------------------------------
%% receive_alt =
%%     ( term, "from", agent_handle, [ "::", conditions ], "~>", action_seq ) |
%%     ( term, "from_thread", pedro_handle, [ "::", conditions ],
%%            "~>", action_seq );
%%     ( "query", term, "from_thread", pedro_handle, [ "::", conditions ],
%%            "~>", action_seq );
%% ----------------------------------------------------
% receive_alt_(Alt) -->
%     term__(Patt),
%     syntax__('query_from'), !,
%     pedro_handle_(Addr),
%     (
%      syntax__('::')
%     ->
%      conditions__(Test),
%      {HasTest = true}
%     ;
%      {HasTest = false}
%     ),
%     (
%      syntax__('~>')
%     ->
%      []
%     ;
%      error_handler__('Expecting ~>')
%     ),
%     action_seq__(Act),
%     {
%      (
%        HasTest = true
%      ->
%        Alt = '$receive_from'(remote_query_(Patt), from_thread(Addr),
%                              (type(Patt, '!'(string)) & Test), Act)
%      ;
%        Alt = '$receive_from'(remote_query_(Patt), from_thread(Addr),
%                              type(Patt, '!'(string)), Act)
%      )
%     }.
receive_alt_(Alt) -->
    term__(Patt),
    (
      [op(from)]
    ->
      parse_with_error_handling__(agent_handle_(Addr1),
                                  'Expecting agent_handle'),
      {Addr = from(Addr1)}
    ;
      [op(from_thread)]
    ->
      parse_with_error_handling__(pedro_handle_(Addr1),
                                  'Expecting pedro_handle'),
      {Addr = from_thread(Addr1)}
    ;
     error_handler__('Expecting from or from_thread')
    ),
    (
     syntax__('::')
    ->
     parse_with_error_handling__(conditions__(Test), 'Expecting conditions'),
     {HasTest = true}
    ;
     {HasTest = false}
    ),
    (
     syntax__('~>')
    ->
     []
    ;
     error_handler__('Expecting ~>')
    ),
    action_seq__(Act),
    {
     (
       HasTest = true
     ->
       Alt = '$receive_from'(Patt, Addr, Test, Act)
     ;
       Alt = '$receive_from'(Patt, Addr, '$empty', Act)
     )
    }.


'$receive_alts'(Out) -->
    [cb('}')], !,
    {Out = []}.
'$receive_alts'(Out) -->
    syntax__(timeout), !,
    term__(T),
    (
      syntax__('~>')
    ->
      []
    ;
      error_handler__('Expecting ~>')
    ),
    action_seq__(Act),
    (
      \+syntax__(';')
    ->
      []
    ;
      error_handler__('Unexpected ;')
    ),
    (
      [cb('}')]
    ->
      {Out = [timeout(T, Act)]}
    ;
      error_handler__('The timeout is not the last alternative')
    ).
'$receive_alts'(Out) -->
    receive_alt_(Alt), !,
    '$receive_alts'(In),
    {Out = ['$normal'(Alt)|In]}.
'$receive_alts'(_Out) -->
    error_handler__('Expecting } or receive alt').


%% ----------------------------------------------------
%% except_alt =
%%     term, [ "::", conditions ], "~>", action_seq;
%% ----------------------------------------------------
except_alt_('$catch'(Pattern, Test, Act)) -->
    term__(Pattern),
    (
      syntax__('::')
    ->
      parse_with_error_handling__(conditions__(Test), 'Expecting conditions')
    ;
      {Test = '$empty'}
    ),
    (
     syntax__('~>')
    ->
     []
    ;
     error_handler__('Expecting ~>')
    ),
    action_seq__(Act),
    (
     \+[op(';')]
    ->
     []
    ;
     error_handler__('Unexpected ;')
    ).
    

'$except_alts'(Out) -->
    [cb('}')], !,
    {Out = []}.
'$except_alts'(Out) -->
    except_alt_(Alt), !,
    '$except_alts'(In),
    {Out = [Alt|In]}.
'$except_alts'(_Out) -->
    error_handler__('Expecting } or except alt').



%% ----------------------------------------------------
%% global_num_op =
%%     ":=" | "+:=" | "-:=";
%% ----------------------------------------------------
global_num_op_(':=') --> [op(':=')], !.
global_num_op_('+:=') --> [op('+:=')], !.
global_num_op_('-:=') --> [op('-:=')], !.

%% ----------------------------------------------------
%% test_op =
%%     "=" | "==" | "\=" | "=@" | ">" | "<" | ">=" | "=<" |
%%     "@>" | "@<" | "@>=" | "@=<" | "in";
%% ----------------------------------------------------
test_op_('=') --> [op('=')], !.
test_op_('==') --> [op('==')], !.
test_op_('\\=') --> [op('\\=')], !.
test_op_('=@') --> [op('=@')], !.
test_op_('>') --> [op('>')], !.
test_op_('<') --> [op('<')], !.
test_op_('>=') --> [op('>=')], !.
test_op_('=<') --> [op('=<')], !.
test_op_('@>') --> [op('@>')], !.
test_op_('@<') --> [op('@<')], !.
test_op_('@>=') --> [op('@>=')], !.
test_op_('@=<') --> [op('@=<')], !.
test_op_('in') --> [op('in')], !.

%% ----------------------------------------------------
%% atom_or_simple_compound = atom | simple_compound;
%% ----------------------------------------------------
atom_or_simple_compound_(C) --> simple_compound_(C), !.
atom_or_simple_compound_(A) --> atom__(A).

var_or_simple_compound_(C) --> simple_compound_(C), !.
var_or_simple_compound_(A) --> var_(A).

%% ----------------------------------------------------
%% simple_compound = atom, bracketed_arg_seq;
%% ----------------------------------------------------
simple_compound_(_) -->
    next_token__(syntax(A)), !,
    error_handler__(['The reserved symbol \'', A, '\' can\'t be used as an constructor']).
simple_compound_(C) -->
    atom__(F),
    bracketed_arg_seq__(Args),
    { (
        Args = []
      ->
        C = F('$none_')
      ;
        C =.. [F|Args]
      )
    }.

%% ----------------------------------------------------
%% simple_type_compound = atom, bracketed_type_arg_seq;
%% ----------------------------------------------------
simple_type_compound_(_) -->
    next_token__(syntax(A)), !,
    error_handler__(['The reserved symbol \'', A, '\' can\'t be used as an constructor']).
simple_type_compound_(C) -->
    atom__(F),
    bracketed_type_arg_seq__(Args),
    { (
        Args = []
      ->
        C = F('$none_')
      ;
        C =.. [F|Args]
      )
    }.


%% ----------------------------------------------------
%% compound_term = ( atom | var ), bracketed_arg_seq, { bracketed_arg_seq };
%% ----------------------------------------------------
compound_term__(_T) -->
    next_token__([syntax(A),ob('(')]), !,
    error_handler__(['The reserved symbol \'', A, '\' can\'t be used as an constructor']).
    
compound_term__(T) -->
    atom__(F),
    \+ \+  [ob('(')],
    '$compound_term_args'(F, T).
compound_term__(T) -->
    var_(F),
    \+ \+  [ob('(')],
    '$compound_term_args'(F, T).

'$compound_term_args'(F, T) -->
    bracketed_arg_seq__(Args), !, 
    { (
        Args = []
      ->
        C = F('$none_')
      ;
        C =.. [F|Args]
      )
    },
    '$compound_term_args'(C, T).
'$compound_term_args'(T, T) --> [].
    

%% ----------------------------------------------------
%% pedro_handle = (atom | var), [":", (atom | var)], ["@", (atom | var)];
%% ----------------------------------------------------
pedro_handle_(Addr) -->
    '$atom_or_var'(T),
    (
      syntax__(':')
    ->
      '$atom_or_var'(P),
      {Seen = true}
    ;
      []
    ),
    (
      syntax__('@')
    ->
      '$atom_or_var'(M),
      { var(Seen)
      ->
        Addr = '@'(T, M) 
      ;
        Addr = ':'(T, '@'(P, M))
      }
    ;
      { var(Seen)
      ->
        Addr = T
      ;
        Addr = ':'(T, P)
      }
      
    ).
    
%% ----------------------------------------------------
%% agent_handle = (atom | var), ["@", (atom | var)];
%% ----------------------------------------------------
agent_handle_(Addr) -->
    '$atom_or_var'(T),
    (
      syntax__('@')
    ->
      '$atom_or_var'(M),
      { Addr = '@'(T, M) }
    ;
      next_token__(syntax(':'))
    ->
      {fail}
    ;
      { Addr = T }
    ).

'$atom_or_var'(AV) --> atom__(AV), !.
'$atom_or_var'(AV) --> var_(AV).

%% ----------------------------------------------------
%% tel_procedure_body =
%%    "{", tel_rule, { tel_rule }, "}";
%% ----------------------------------------------------

tel_procedure_body_(Rules) -->
    tel_rule_seq__(Rules),
    matching_bracket__(cb('}')).


tel_rule_seq__([]) -->
    next_token__(cb('}')), !.
tel_rule_seq__([R|Rules]) -->
    tel_rule_(R), !,
    tel_rule_seq__(Rules).

%% ----------------------------------------------------
%% tel_rule =
%%     ( tr_rule_LHS, "~+>",  tr_pp_action) |
%%     ( tr_rule_LHS, "~>", tr_rule_RHS );
%% ----------------------------------------------------
tel_rule_('tr~>'(LHS, '++'(['$nil'], QA))) -->
    tr_rule_LHS_(LHS),
    syntax__('~+>'), !,
    tr_pp_action__(QA).
tel_rule_('tr~>'(LHS, RHS)) -->
    tr_rule_LHS_(LHS),
    syntax_with_check__('~>'),
    tr_rule_RHS_(RHS).

%% ----------------------------------------------------
%% tr_rule_LHS =
%%     conditions,
%%     [ ( "commit_while", while_commit  ) | ( "or_while", while_commit)) ];
%% ----------------------------------------------------
tr_rule_LHS_(LHS) -->
    conditions__(Guard),
    '$tr_rule_LHS_aux'(Guard, LHS), !.
tr_rule_LHS_(_) -->
    error_handler__('Invalid TR rule LHS').


'$tr_rule_LHS_aux'(Guard, LHS) -->
    syntax__(or_while), !,
    '$while_part'(Guard, LHS),
    (
      \+syntax__(commit_while)
    ->
      []
    ;
      error_handler__('Unexpected commit_while')
    ).
'$tr_rule_LHS_aux'(Guard, LHS) -->
    syntax__(commit_while), !,
    '$inhibit_part'(Guard, LHS),
    (
      \+syntax__(or_while)
    ->
      []
    ;
      error_handler__('Unexpected or_while')
    ).
'$tr_rule_LHS_aux'(Guard, Guard) --> [].

'$while_part'(Guard, LHS) -->
    while_commit_(WG),
    {LHS = '$while'(while, Guard, WG)}.
'$inhibit_part'(Guard, LHS) -->
    while_commit_(WG),
    {LHS = '$inhibit'(inhibit, Guard, WG)}.


%% ----------------------------------------------------
%% while_commit =
%%     conditions |
%%     ( "min_time", term ) |
%%     (conditions, "min_time", term );
%% ----------------------------------------------------
while_commit_(WU) -->
    syntax__(min_time), !,
    term__(T),
    {WU = '$time'(min, T)}.
while_commit_(WU) -->
    conditions__(C),
    (
      syntax__(min_time)
    ->
      term__(T),
      {WU = '$time'(min, C, T)}
    ;
      {WU = C}
    ).
      
%% ----------------------------------------------------
%% tr_rule_RHS = tr_action, [ "++", tr_pp_action ];
%% ----------------------------------------------------
tr_rule_RHS_(Action) -->
    tr_action__(A),
    (
      [op('++')]
    ->
      tr_pp_action__(QA),
      { Action = '++'(A, QA) }
    ;
      { Action = A }
    ).

%% ----------------------------------------------------
%% tr_action =
%%     ( "(", ")" ) |
%%     ( simple_tr_action, {",", simple_tr_action} ) |
%%     ( "(", simple_tr_action, {",", simple_tr_action}, ")" ) |
%%     ( "[", tr_timed_seq, "]" );
%% ----------------------------------------------------
tr_action__(Action) -->
    any_open_bracket__, [cb(')')], !,
    {Action = ['$nil']}.
tr_action__(Action) -->
    any_open_bracket__, !,
    '$seq'(simple_tr_action, punct(','), Action,
           'Expecting TR action'),
    (
     [cb(')')]
    ->
     []
    ;
     error_handler__('Expecting )')
    ).
tr_action__(_Action) -->
    [ob('{')], !,
    error_handler__('Unexpecting {').
tr_action__(Action) -->
    [ob('[')], !,
    tr_timed_seq__(Action1),
    (
     [cb(']')]
    ->
     []
    ;
     error_handler__('Expecting ]')
    ),
    {Action = timed_seq(Action1)}.
tr_action__(Action) -->
    '$seq'(simple_tr_action, punct(','), Action,
           'Expecting TR action').



%% ----------------------------------------------------
%% tr_timed_seq =
%%     simple_tr_action, ":", term, {",",  simple_tr_action, ":", term },
%%         [",", simple_tr_action];
%% ----------------------------------------------------
tr_timed_seq__(Action) -->
    simple_tr_action__(A1),
    (
      syntax__(':')
    ->
      term__(T),  
     (
        [punct(',')]
      ->
        tr_timed_seq__(Action2),
        {Action = ['$time'(for, [A1], T)|Action2]}
      ;
        {Action = ['$time'(for, [A1], T)]}
      )
    ;
      {Action = [[A1]]}
    ).
%% ----------------------------------------------------
%% simple_tr_action = var | compound_term;
%% ----------------------------------------------------

simple_tr_action__('$nil') -->
    any_open_bracket__, [cb(')')], !.
simple_tr_action__(A) -->
    compound_term__(A), !.
simple_tr_action__(A) -->
    var_(A).


%% ----------------------------------------------------
%% term =
%% simple_term | 
%% ( simple_term, "<>", simple_term, { "<>", simple_term } ) |
%% ( simple_term, "++", simple_term, { "++", simple_term } ) |
%% ( simple_term, set_op, simple_term, { set_op, simple_term } ) |
%% ( simple_term, arith_op, simple_term, { arith_op, simple_term } );
%% ----------------------------------------------------

term__(T) -->
    simple_term__(T1),
    (
      [op('<>')]
    ->
      '$term_<>'(Rest),
      { T = '<>'(T1, Rest) }
    ;
      [op('++')]
    ->
      '$term_++'(Rest),
      { T = '++'(T1, Rest) }
    ;
      ( next_token__(op(union)) ; next_token__(op(diff)) ; next_token__(op(inter)) )
    ->
      set_expression__(T1, T)
    ;
      '$next_token_arith_op'
    ->
      arith_expression__(T1, T), !
     ;
      {T = T1}
     ).
      
'$next_token_arith_op' -->
    next_token__(op(Tok)),
    {'$next_token_arith_op_aux'(Tok)}.

'$next_token_arith_op_aux'('**').
'$next_token_arith_op_aux'('*').
'$next_token_arith_op_aux'('/').
'$next_token_arith_op_aux'('//').
'$next_token_arith_op_aux'('+').
'$next_token_arith_op_aux'('-').
'$next_token_arith_op_aux'('>>').
'$next_token_arith_op_aux'('<<').
'$next_token_arith_op_aux'('mod').
'$next_token_arith_op_aux'('rem').
'$next_token_arith_op_aux'('\\/').
'$next_token_arith_op_aux'('/\\').

%% ----------------------------------------------------
% simple_term =
%     var |
%     atom |
%     anynum |
%     string |
%     compound_term |
%     agent_handle | pedro_handle |
%     ( "(", term, ")" ) |  
%     tuple |      
%     ( "$", atom ) |
%     set_term |
%     set_comprehension |
%     list_term |
%     list_comprehension |
%     ( "-", simple_term ) |
%     ( "#", simple_term );
%% ----------------------------------------------------

simple_term__(T) -->
    any_open_bracket__, !,
    term__(T1),
    (
      [cb(')')]
    ->
      {T = T1}
    ;
      '$rest_tuple_seq'(Ts),
      { T = '$tuple'([T1|Ts]) }
    ).
simple_term__('$'(T)) -->
    [op('$')], 
    atom__(T), !.
simple_term__(T) -->
    anynum__(T), !.
simple_term__('-'(T)) -->
    [op('-')], \+ [ob('(')], !,
    simple_term__(T).
simple_term__('#'(T)) -->
    [op('#')], \+ [ob('(')], !,
    simple_term__(T).
simple_term__(T) -->
     next_token__(ob('{')), !, '$set_term_or_comprehension'(T).
simple_term__(T) -->
    next_token__(ob('[')), !,'$list_term_or_comprehension'(T).
simple_term__(T) -->
    compound_term__(T), !.
simple_term__(T) -->
    agent_handle_(T), !.
simple_term__(T) -->
    pedro_handle_(T), !.
simple_term__(T) -->
    var_(T), !.
simple_term__(T) -->
    string__(T), !.
simple_term__(T) -->
    atom__(T), !.


%% ----------------------------------------------------
%% set_term =
%%     ( "{", "}" ) |
%%     ( "{", arg_seq, "}" );
%%
%% set_comprehension =
%%    "{", simple_term, "::", exists_conditions, "}";
%% ----------------------------------------------------
    

'$set_term_or_comprehension'(T)  -->
     set_comprehension_(T), !.
'$set_term_or_comprehension'(T)  -->
    set_term__(T).

set_comprehension_('$set_constr'(Patt, Cond)) -->
    [ob('{')],
    term__(Patt),
    syntax__('::'), !,
    exists_conditions__(Cond),
    matching_bracket__(cb('}')).

set_term__('$set_enum'(Lst)) -->
    '$paren_seq'(term__, punct(','), Lst, 'Expecting term').

'$list_term_or_comprehension'(T)  -->
    list_comprehension_(T), !.
'$list_term_or_comprehension'(T)  -->
    list_term__(T).





'$rest_tuple_seq'([]) -->
    [cb(')')],!.
'$rest_tuple_seq'([H|T]) -->
    [punct(',')], !,
    term__(H),
    '$rest_tuple_seq'(T).
'$rest_tuple_seq'(_) -->
    error_handler__('Expecting \')\' or \',\'').

'$term_<>'(App) -->
    (
      simple_term__(A1)
    ->
      (
        [op('<>')], !,
        '$term_<>'(B),
        { App = (A1 <> B) }
      ;
        {App = A1}
      )
    ;
      error_handler__('Expecting term')
    ).
'$term_++'(App) -->
    (
      simple_term__(A1)
    ->
      (
        [op('++')], !,
        '$term_++'(B),
        { App = (A1 ++ B) }
      ;
        {App = A1}
      )
    ;
      error_handler__('Expecting term')
    ).
    
arith_expression__(A, Result) -->
    power_term__(A, Power),
    mult_div_term__(Power, MultDiv),
    add_minus_term__(MultDiv, Result).

set_expression__(A, Result) -->
    inter_term__(A, Inter),
    union_diff_term__(Inter, Result).

power_term__(A, Result) -->
    [op('**')], !,
    simple_term__(A1),
    power_term__(A1, R1),
    { '$process_arith_tuples'('**',  A, R1, Result) }.
power_term__(A, A) --> [].

mult_div_term__(A, Result) -->
    [op('*')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    { '$process_arith_tuples'('*',  A, A2, A3) }, 
    mult_div_term__(A3, Result).
mult_div_term__(A, Result) -->
    [op('/')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    { '$process_arith_tuples'('/',  A, A2, A3) }, 
    mult_div_term__(A3, Result).
mult_div_term__(A, Result) -->
    [op('//')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    { '$process_arith_tuples'('//',  A, A2, A3) }, 
    mult_div_term__(A3, Result).
mult_div_term__(A, Result) -->
    [op('>>')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    { '$process_arith_tuples'('>>',  A, A2, A3) }, 
    mult_div_term__(A3, Result).
mult_div_term__(A, Result) -->
    [op('<<')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    { '$process_arith_tuples'('<<',  A, A2, A3) }, 
    mult_div_term__(A3, Result).
mult_div_term__(A, Result) -->
    [op('mod')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    { '$process_arith_tuples'('mod',  A, A2, A3) }, 
    mult_div_term__(A3, Result).
mult_div_term__(A, A) --> [].

add_minus_term__(A, Result) -->
    [op('+')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    mult_div_term__(A2, A3),
    { '$process_arith_tuples'('+',  A, A3, A4) }, 
    add_minus_term__(A4, Result).
add_minus_term__(A, Result) -->
    [op('-')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    mult_div_term__(A2, A3),
    { '$process_arith_tuples'('-',  A, A3, A4) }, 
    add_minus_term__(A4, Result).
add_minus_term__(A, Result) -->
    [op('\\/')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    mult_div_term__(A2, A3),
    { '$process_arith_tuples'('\\/',  A, A3, A4) }, 
    add_minus_term__(A4, Result).
add_minus_term__(A, Result) -->
    [op('/\\')], !,
    simple_term__(A1),
    power_term__(A1, A2),
    mult_div_term__(A2, A3),
    { '$process_arith_tuples'('/\\',  A, A3, A4) }, 
    add_minus_term__(A4, Result).
add_minus_term__(A,A) --> [].


inter_term__(A, Result) -->
    [op('inter')], !,
    simple_term__(A1),
    inter_term__(A1, A2),
    { '$process_set_tuples'('inter',  A, A2, A3) }, 
    union_diff_term__(A3, Result).
inter_term__(A, A) --> [].

union_diff_term__(A, Result) -->
    [op('union')], !,
    simple_term__(A1),
    inter_term__(A1, A2),
    { '$process_set_tuples'('union',  A, A2, A3)}, 
    union_diff_term__(A3, Result).
union_diff_term__(A, Result) -->
    [op('diff')], !,
    simple_term__(A1),
    inter_term__(A1, A2),
    { '$process_set_tuples'('diff',  A, A2, A3) }, 
    union_diff_term__(A3, Result).
union_diff_term__(A, A) --> [].





syntax__(T) --> [syntax(T)].
atom__(T) --> [atom(T)], !.
atom__(T) --> [op(T)].
atom_or_syntax__(T) --> syntax__(T), !.
atom_or_syntax__(T) --> atom__(T).
int__(T) --> [op('-')], [int(T1)], !, {T is -T1}.
int__(T) --> [int(T)].
num__(T) --> [op('-')], [num(T1)], !, {T is -T1}.
num__(T) --> [num(T)].
anynum__(N) --> int__(N).
anynum__(N) --> num__(N).
string__(T) --> [string(T)].
var_(V) -->
    TK, 
    [var(V, _)],
    {
     ip_lookup('$in_teleor_pa', NoVar),
     (
       NoVar == true
     ->
       report_error__('Unexpected variable', TK)
     ;
       true
     )
    }.



any_open_bracket__ --> [ob('(')].
any_open_bracket__ --> [ob(' (')].

						  
% code_token__(dyn_term) --> [atom(dyn_term)].
% code_token__(rel_term) --> [atom(rel_term)].
% code_token__(act_term) --> [atom(act_term)].


% code_annotation__('!?') --> [syntax('!?')].
% code_annotation__('!??') --> [syntax('!??')].

