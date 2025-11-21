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

%%
%% Take a program file *.qlg and :
%% 1. Parse it
%% 2. Do some consistency checks and other "sanity checks"
%% 3. Type check
%% 4. Translate into QuProlg code.


%% Some operator declarations to make code easier read
?- compile_time_only(consult(op_decls)).

%% for storing away errors for later reporting
%% used when the same error message would be generated multiple times
%% by the same analysis - this suppresses repeats
?- dynamic(type_error_remember_/1).
%% Set when we are in the teleor interpreter (and consulting teleor files)
%% This enables more types etc than available in the qulog interpreter
?- dynamic('$running_teleo'/0).

%% '$pconsulted_preds'(F,N)  F/N is a predicated defined in a pconsulted file
?- dynamic('$pconsulted_preds'/2).

%% temporary storage while processing program
?- dynamic('$tel'/2).       %% remember TR programs
?- dynamic('$runtime_goal'/1).  %% remember goals to exec at runtime
?- dynamic('$goal'/1). %% remember goals to run at consult time
?- dynamic('$delay_pconsult'/1).

%% assertions for TR programs
?- dynamic('$tel_atomic'/1).
?- dynamic('$tel_start'/1).
?- dynamic('$resource_info'/3).
?- dynamic('$primitive_resource_info'/3).
?- dynamic('$tel_action'/1).

%% Use for reconsult - the late-time-of-update for the file is stored
%% so when we do a reconsult we can determine which consulted files
%% have been changed - similar to a make
%% Together with '$saved_consulted' and '$consulted' is used to decide
%% what files need reconsulting - either because that file has changed
%% or a file it consults has changed. NOTE: even if this file has not changed
%% it needs to be reconsulted if it consults a file that has changed because,
%% for example, the sub-file might have changed some types definitions
%% or declarations which might now cause this file to produce a type error
?- dynamic('$consulted_timestamp'/3).
?- dynamic('$saved_consulted'/1).
?- dynamic('$consulted'/1).
%% Link the stream with each open file being processed 
?- dynamic('$file_stream'/2).
%% Used to check all the ?- commands are at the beginning of the file
?- dynamic('$head_order_check'/1).
%% For keeping track of undeclared code (and producing an error)
?- dynamic('$undeclared_defn'/1).


?- dynamic('$tmp_clause'/2). % temp storage of definitions
?- dynamic('$current_working_dir'/1). % for consults within consults

%% for listing, clause etc so that the same var names are used
%% the last arg is the name of the file the defs/decls came from
%% essentially keeping an un-translated copy of definitions for show
%% and for error messages
?- dynamic('$saved_input_with_vars'/3).

%% Internal representations of some declarations
%% user type info:
%% '$user_type_info'(Functor, Kind, Types, Term, VarNames)
?- dynamic('$user_type_info'/5).
%% user type decription:
%% '$user_type_descr'(Functor, Types, Kind, Doc)
%% where variables are replaced by their names so that
%% showing types is easier
?- dynamic('$user_type_descr'/4).
%% store percept names
?- dynamic('$percept'/1).
%% store dyn info
?- dynamic('$belief'/1).
%% store global value info : eg int a:=0
?- dynamic('$state_belief'/1).

?- dynamic('$tmp_percepts'/1).

?- multifile(qulog2qp_map/2).
?- multifile(qulog2qp_wrap/1).

%% The type checker determines the types of the "returned values" for
%% the query and this is used by the translator to inject appropriate
%% type check code into the remote_query call so that the actual
%% return value share the correct type.
?- dynamic('$remote_query_info'/2).

%% When declared percepts have a relation definition then
%% should_infer_remember_(HeadPattern) is asserted and
%% can_infer_now_(Head) :- Body
%% is also asserted
%%?- dynamic(should_infer_remember_/1).
%%?- dynamic(can_infer_now_/1).


%% Both below used so that reconsulting a file will remove old defs
%% the file currently being consulted - used to remember what
%% was loaded by this consult
?- dynamic('$current_consulted_file'/1).
%% the info about what was consulted from this file
?- dynamic('$asserted_file_info'/2).

%% used by rem caching to determine if caching has already been done
?- dynamic('$is_cached'/1, 0).
?- dynamic(cache_info__/4).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Temp - for debugging
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dump_state_ :-
    errornl('%%%%%%%%%%%% dumping state %%%%%%%%%%%%%%%%'),
    fail.
dump_state_ :-
    '$tel_action'(X),
    errornl('$tel_action'(X)),
    fail.
dump_state_ :-
    '$tel_start'(TR),
    errornl(tel_start(TR)),
    fail.
dump_state_ :-
    '$tel_atomic'(Term),
    errornl('$tel_atomic'(Term)),
    fail.
dump_state_ :-
    '$resource_info'(Term, A2, P),
    errornl('$resource_info'(Term, A2, P)),
    fail.
dump_state_ :-
    '$primitive_resource_info'(Term, A2, P),
    errornl('$primitive_resource_info'(Term, A2, P)),
    fail.

dump_state_ :-
    '$user_type_info'(A,B,C,D,E),
    errornl('$user_type_info'(A,B,C,D,E)),
    fail.

dump_state_ :-
    errornl('%%%%%%%%%%%%%%%%%%%%%%%%%%%%').




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Top-level of consult
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Asserted when parser detects a syntax error
?- dynamic('$syntax_errors'/0).

%% Add .qlg extension and open
open_qulog_file_(File, Mode, Stream) :-
    concat_atom([File, '.qlg'], QlgFile),
    '$accessible'(QlgFile), !,
    open(QlgFile, Mode, Stream).

open_qulog_file_(File, _Mode, _Stream) :-
    exception(permission_error(unrecoverable, qconsult_(File), default)).


%% Consult a .qlg file. Remove decls and code from an earlier consult
%% of this file (a reconsult) 

top_level_qconsult_(File) :-
    %% For reconsulting we need to know what is the top-level file
    assert('$top_level_consulted_file'(File)),
    writeL_(["Consulting ", File, "...",nl_]),
    absolute_file_name(File, FilePath),
    file_directory_name(File, Path),
    getcwd(CurrentDir),
    chdir(Path),
    assert('$current_working_dir'(Path)),
    catch(
          catch('$qconsult_file'(FilePath),
                consult_error(_E),
                (chdir(CurrentDir),'$failure_cleanup_consult')),
          Ball,
          (
            chdir(CurrentDir),
            %% translate QP exceptions and display error message
            '$qulog_interpreter_catch_msg'(Ball),
              '$failure_cleanup_consult'
          )
         ),
    chdir(CurrentDir).


qconsult_(File) :-
    writeL_(["Consulting ", File, "...",nl_]),
    absolute_file_name(File, FilePath),
    file_directory_name(File, Path),
    getcwd(CurrentDir),
    chdir(Path),
    asserta('$current_working_dir'(Path)),
    '$qconsult_file'(FilePath),
    retract('$current_working_dir'(Path)),
    chdir(CurrentDir).

%% All the work done here
'$qconsult_file'(File) :-
    catch(open_qulog_file_(File, read, Stream), exception(E),
          '$handle_consult_exception'(E)),
    assert('$file_stream'(File, Stream)),
    forall(qulog2qp_map(_, C),  retractall(C)),
    %% for reconsulting - remove all data from previous (attempted) consult
    '$delete_all_code'(File),
    %% asserta is used so the first fact found will be for the
    %% "inner consult" - for reconsult
    asserta('$current_consulted_file'(File)),
    %% retract any leftover tmp info
    retractall('$syntax_errors'),
    retractall('$head_order_check'(_)),
    retractall('$tmp_percepts'(_)),
    retractall('$remote_query_info'(_,_)),
    retractall('$is_cached'(_)),
    retractall(qulog2qp_map(_, _)),
    retractall(qulog2qp_wrap(_)),
    retractall( '$pconsulted_preds'(_, _)),
    %% set the error count to 0 - this is updated each time an error is found
    global_state_set(num_errors, 0),
    %% attempt to read in and process terms of the file
    qconsult_terms_(Stream, File),
    %% add any extra system declarations  after user declarations loaded
    '$add_extra_decls',
    '$add_resource_decl',
    %% If any errors are detected at this point then show the number
    %% of errors and then throw an exception (exit consult and cleanup)
    '$report_errors'(File),
    (
      retract('$syntax_errors')
    ->
      writeL_(stderr, [nl_, "Syntax errors found - file not consulted",
                      nl_, nl_]),
      throw(consult_error(syntax))
    ;
      retract('$file_stream'(File, Stream1)),
      close(Stream1)
    ),
    %% Check macro type definitions
    '$check_macros',
    %% check that in t1(X) ::= t2, the X occurs on RHS
    '$check_defined_type_vars',
    %% Check if all rules have an allowed type declaration for the head
    '$check_if_clauses_have_types',
    %% check if rel/act have appropriate mode declarations
    '$check_declaration_modes',
    %% add '$percept' and '$belief' facts
    '$declare_percepts_top',
    %% check the types on RHS of type definition or types used
    %% in declarations are valid
    '$check_valid_types',
    %% add '$tel_action' facts
    '$check_tel_action_interface',
    '$report_errors'(File),
    %% check that the definitions of relations/functions/actions/TR programs
    %% are valid defns with correct types and modes.
    '$check_definitions',
    %% if tel_start is defined then there should be percepts
    '$check_if_percepts_declared',
    %% check that the same TR program has not beed defined twice
    '$check_no_repeated_trs_interface',
    %% check for contiguous rules for definition
    '$check_contiguous_defs',
    %% Are there any errors now?
    '$report_errors'(File),
     %% trans_fun looks to see if a variable functor comes from a TR
    %% program and if so translates the function differently
    %% this needs to be set to empty before non-TR defns are translated
    ip_set('$tel_vars', []),
    %% add qulog2qp_map  and wrap facts
    forall('$delay_pconsult'(PFile), '$extract_qulog2qp_facts'(PFile)),
    '$add_qulog2qp_map_for_wrap',
    '$check_and_compile_qulog2qp',
    %% translate all function/relation/action clauses to prolog code
    '$translate_all_clauses',
   '$add_cache_dependencies',
    %% the remainder is about TR programs
    %% assert the resource info for each TR program
    '$add_tr_resource_info_interface',
    '$do_run_tr_checks',
    %% Are there any errors?
    '$report_errors'(File),
    %% check that if resources are required then they are correctly defined
    %% and used
    '$check_resource_uses_interface',
    '$report_errors'(File),
    '$check_is_defined',
    %% if a TR program is defined then we need to know how TR guards
    %% depend on dynamic facts. This generates the dependency info
    %% for all relations in the program
    %%%%%%%%%'$tr_build_clause_info_interface',
    %% translate the TR programs to prolog clauses
    '$translate_tel_interface',
    %% consult QP level files at the end so that type info can be accessed
    forall(retract('$delay_pconsult'(PFile)), '$do_pconsult'(PFile)),
    retractall(qulog2qp_map('$dummy', '$dummy')),
    retractall(qulog2qp_wrap('$dummy')),
    '$report_errors'(File),
    %% pop consulted file info
    retract('$current_consulted_file'(File)),
    writeL_(["... ", File, " consulted ", nl_]), !.
'$qconsult_file'(File) :-
    '$report_errors'(File).

%% the exception type is the union of builtin and user exception types
%% and so a defn of user_exception is required. If the user doesn't define it
%% then a default defn is given
'$add_extra_decls' :-
    \+'$user_type_info'(user_exception, _, _, _, _), 
    assert('$user_type_info'(user_exception, defined_type, user_exception,
                           '$constr_enum_type'([default_exception_('$none_')]),
                             [])), fail.

'$add_extra_decls'.


'$extract_qulog2qp_facts'(File) :-
    concat_atom([File, '.ql'], QlFile),
    open(QlFile, read, S),
    read(S, Term),
    '$extract_qulog2qp_facts_aux'(Term, S),
    close(S).

'$extract_qulog2qp_facts_aux'(end_of_file, _) :- !.
'$extract_qulog2qp_facts_aux'(qulog2qp_map(A, B), S) :- !,
    assert(qulog2qp_map(A, B)),
    read(S, Term),
    '$extract_qulog2qp_facts_aux'(Term, S).
'$extract_qulog2qp_facts_aux'(qulog2qp_wrap(A), S) :- !,
    assert(qulog2qp_wrap(A)),
    read(S, Term),
    '$extract_qulog2qp_facts_aux'(Term, S).
'$extract_qulog2qp_facts_aux'(_, S) :- 
    read(S, Term),
    '$extract_qulog2qp_facts_aux'(Term, S).

%% For QP consulted files we wrap calls to the Qulog level with type checks
%% and expand function calls using term expansion within consult
'$do_pconsult'(File) :-
    add_expansion('$pconsult_expand'),
    consult(File),
    del_expansion('$pconsult_expand').

%% translate the body of each rule
'$pconsult_expand'('?-'(G), '?-'(G)) :- !,
    assert('$goal'(G)).
'$pconsult_expand'((Head :- Body), (Head :- TransBody)) :- !,
    '$remember_pconsult_pred'(Head),
    transform_subterms('$pconsult_expand_body', Body, TransBody).
'$pconsult_expand'(qulog2qp_map(_, _), qulog2qp_map('$dummy', '$dummy')).
'$pconsult_expand'(qulog2qp_wrap(_), qulog2qp_wrap('$dummy')).

'$remember_pconsult_pred'(Head) :-
    functor(Head, F, N),
    (
      '$pconsulted_preds'(F, N)
    ->
      true
    ;
      assert('$pconsulted_preds'(F, N))
    ).

%% If the call is defined as a rel or act at the Qulog level wrap the call
'$pconsult_expand_body'(Call, ('$check_rel'(FullCall), TransCall)) :-
    freeze_term(Call),
    add_default_args__(Call, FullCall),
    \+'$pconsult_exclude'(FullCall), '$type_info'(_, rel, _, FullCall, _),
    \+ qulog2qp_map(FullCall, _), !,
    '$translate_goal'(FullCall, TransCall),
    thaw_term(Call).
'$pconsult_expand_body'(Call, ('$check_act'(FullCall), TransCall)) :-
    freeze_term(Call),
    add_default_args__(Call, FullCall),
    \+'$pconsult_exclude'(FullCall), '$type_info'(_, act, _, FullCall, _),
    \+ qulog2qp_map(FullCall, _), !,
    '$translate_goal'(FullCall, TransCall),
    thaw_term(Call).
'$pconsult_expand_body'(Call, Call).

%% Check that Call is a valid rel call
'$check_rel'(Call) :-
    \+ allowed_rel_call(Call), !,
    term2string(Call, CallString),
    throw(prolog_call_type_error(CallString)).
'$check_rel'(_Call).

%% Ditto for act calls
'$check_act'(Call) :-
    \+ allowed_act_call(Call), !,
    term2string(Call, CallString),
    throw(prolog_call_type_error(CallString)).
'$check_act'(_Call).

%% Exclude the following with declarations at the Qulog level as they
%% shouldn't need to be wrapped.
'$pconsult_exclude'((_ = _)).
'$pconsult_exclude'((_ \= _)).
'$pconsult_exclude'((_ @< _)).
'$pconsult_exclude'((_ @=< _)).
'$pconsult_exclude'((_ @> _)).
'$pconsult_exclude'((_ @>= _)).
'$pconsult_exclude'(call(_)).
'$pconsult_exclude'(var(_)).


'$add_resource_decl' :-
    \+'$user_type_info'(resource, _, _, _, _),
    assert('$user_type_info'(resource, defined_type, resource,
                           '$enum_type'([all__]), [])), fail.
'$add_resource_decl'.


'$check_is_defined' :-
    '$get_decls'(rel, Decls),
    member(F, Decls),
    once('$user_type_info'(F, _, _, Call, _)),
    \+'$belief_type_info'(_,_,Call),
    \+'$percept_type_info'(_,_,Call),
    functor(Call, F, _N),
    (
      clause(Call, _)
    ->
      true
    ;
      qulog2qp_map(Call, _)
    ->
      true
    ;
      writeL_(stderr, ["Warning: the relation ", F, " has a declaration but no definition", nl_])
    ),
    fail.
'$check_is_defined' :-
    '$get_decls'(act, Decls),
    member(F, Decls),
    once('$user_type_info'(F, _, _, Call, _)),
    functor(Call, F, _N),
    (
      clause(Call, _)
    ->
      true
    ;
      qulog2qp_map(Call, _)
    ->
      true
    ;
      writeL_(stderr, ["Warning: the action ", F, " has a declaration but no definition", nl_])
    ),
    fail.
'$check_is_defined' :-
    '$get_decls'(fun, Decls),
    member(F, Decls),
    '$generate_apply_name'(F, FApplyName),
    (
      clause(FApplyName(_, _, _), _)
    ->
      true
    ;
      writeL_(stderr, ["Warning: the function ", F, " has a declaration but no definition", nl_])
    ),
    fail.
'$check_is_defined'.

'$get_decls'(Kind, Decls) :-
    findall(F, '$user_type_info'(F, Kind, _, _, _), AllDecls),
    sort(AllDecls, Decls).

%% interface to Teleor specific stuff
'$check_tel_action_interface' :-
    '$running_teleo', !,
    '$check_tel_action'.
'$check_tel_action_interface'.

'$check_no_repeated_trs_interface' :-
    '$running_teleo', !,
    '$check_no_repeated_trs'.
'$check_no_repeated_trs_interface'.


'$add_tr_resource_info_interface' :-
    '$running_teleo', !,    
    '$add_tr_resource_info'.
'$add_tr_resource_info_interface'.

'$check_resource_uses_interface' :-
    '$running_teleo', !,    
    '$check_resource_uses'.
'$check_resource_uses_interface'.

% '$tr_build_clause_info_interface' :-
%     '$running_teleo', !,    
%     '$tr_build_clause_info'.
% '$tr_build_clause_info_interface'.


'$translate_tel_interface' :-
    '$running_teleo', !,
     forall(retract('$tel'(Head, Body)),
            ('$expand_a_TR'(Head, Body))).       
'$translate_tel_interface'.
   
%% If the consult fails because of a syntax or other error then
%% all the code related to the file needs to be cleaned up
%% Given that a file can be consulted within another file
%% the '$current_consulted_file'/1 implements a stack of
%% files currently being consulted.
%% If a consult fails inside a top-level consult then we 
%% throw an exception to be picked up at the interpreter
%% level consult which should respond by removing all
%% consulted info and fail.
%% If the consult at the top-level fails because of, for example,
%% a type error in the top-level file then only the data
%% for that file should be cleaned up.

'$failure_cleanup_consult' :-
    once('$current_consulted_file'(File)),
    '$top_level_consulted_file'(File), !,
    %% The failure is in the top-level file
     (
       retract('$file_stream'(File, Stream1))
     ->
       close(Stream1)
     ;
       true
     ),
     forall(qulog2qp_map(_, C), retractall(C)),
    '$delete_all_code'(File),
    retractall(type_error_remember_(_)),
    retractall('$tmp_clause'(_, _)),
    %%retractall('$tr_clause'(_, _)),
    %%retractall('$seen_clause'(_, _, _)),
    %%retractall('$saved_input_with_vars'(_, _, _)),
    retract('$current_consulted_file'(File)),
    retractall('$asserted_file_info'(File, _)),
    retractall('$consulted_timestamp'(File, File, _)),
    retractall('$saved_consulted'(_)),
    retractall('$remote_query_info'(_,_)),
    retractall('$goal'(_)),
    retractall('$pconsulted_preds'(_, _)),
    retractall('$delay_pconsult'(_)),
    writeL_(["Error consulting ", File, nl_]),
    retractall('$top_level_consulted_file'(_)),
    !,
    fail.
'$failure_cleanup_consult' :-
    once('$current_consulted_file'(ErrorFile)),
    '$top_level_consulted_file'(File),
    '$delete_code_from_all',
    writeL_(["Error in file ",  ErrorFile, " while consulting ", File, nl_,
            "All user definitions and declarations removed.", nl_, nl_]), !,
    fail.


'$delete_code_from_all' :-
     forall(qulog2qp_map(_, C), retractall(C)),
    forall(retract('$file_stream'(_, Stream)), close(Stream)),
    findall(F, '$asserted_file_info'(F, _), Fs),
    sort(Fs, SFs),
    retractall('$current_consulted_file'(_)),
    %% remove all consulted code
    forall(member(File, SFs), '$delete_all_code'(File)),
    %% remove all tmp stuff
    retractall('$tmp_clause'(_, _)),
    %%retractall('$tr_clause'(_, _)),
    %%retractall('$seen_clause'(_, _, _)),
    %%retractall('$saved_input_with_vars'(_, _, _)),
    retractall('$asserted_file_info'(_, _)),
    retractall('$consulted_timestamp'(_, _, _)),
    retractall('$consulted'(_)),
    retractall('$saved_consulted'(_)),
    retractall('$remote_query_info'(_,_)),
    retractall('$goal'(_)),
    retractall('$pconsulted_preds'(_, _)),
    retract('$top_level_consulted_file'(_)).


'$handle_consult_exception'(permission_error(_, qconsult_(File), _)) :-
   writeL_(["Cannot read ", File, nl_]),
   writeL_(["Error consulting ", File, nl_]),
   throw(consult_error(read_error)).

'$check_if_percepts_declared' :-
    '$percept'(_), !.
'$check_if_percepts_declared' :-
    '$tel_start'(_), !,
    writeL_(stderr, [nl_, "Warning: no percepts declared", nl_, nl_]).
'$check_if_percepts_declared'.

'$report_errors'(File) :-
    global_state_lookup(num_errors, N),
    '$report_errors'(N, File).
'$report_errors'(0, _) :-
    !.
'$report_errors'(1, File) :-
    !,
    writeL_(stderr, [nl_, nl_,"1 error detected - ", File, " not consulted",
		    nl_,nl_]),
    throw(consult_error(type)).
'$report_errors'(N, File) :-
    writeL_(stderr, [nl_, nl_, N,
		    " errors detected - ", File, " not consulted",
		    nl_,nl_]),
    throw(consult_error(type)).

%% read the terms of the files
qconsult_terms_(Stream, File) :-
    repeat,
    %% read the string representing the next term to be processed
    catch(
	  read_term_string__(Stream, String, StartLine, EndLine),
	  syntax_error,
	  (assert('$syntax_errors'),fail)
	 ),
    (
      string_length(String, 0)
    ->
      %% EOF
      !
    ;
      retractall('$parser_info'(_, _, _, _)),
      stream_property(Stream, file_name(FileName)),
      %% store information about string for later
      %% processing of syntax errors
      assert('$parser_info'(String, StartLine, EndLine, FileName)),
      %% Get the tokens and process the variables in the term
      string2tokens__(String, Tokens, VarList, TokenPositions),
      ip_set('$token_positions', t(Tokens, TokenPositions)),
      '$collect_var_tables'(VarList, _Variables, VarNames, Singles, []),
      '$unify_on_names'(VarList),
      Tokens \= [],
      %% At this point Tokens is the list of tokens for the current term
      %% being processed
      %% The following parses the tokens
      catch(
            (
              phrase(program_item__(T), Tokens, [])
            ->
              '$warn_of_single_vars'(T, Singles, VarNames),
              %% save away for displaying or error messages
              %% using the original term and var names
              assert('$saved_input_with_vars'(T, VarNames, File)),
              %% process the term
              '$process_input_top'(T, VarNames, _)
            ;
              %% Unable to parse
              report_error__('Can\'t parse', Tokens)
            ),
            %% syntax errors are caught here
            syntax_error,
            assert('$syntax_errors')
           ),
      fail
    ).

%% Whenever something from the file is added to the system
%% we keep track of it so that if there is an error or
%% the file is reconsulted we can remove all the relevant information
%% Info should be a template
'$add_file_info'(Info) :-
    once('$current_consulted_file'(File)),
    (
     '$asserted_file_info'(File, Info)
    ->
     true
    ;
     assert('$asserted_file_info'(File, Info))
    ).
%% Delete all declarations/definitions/code that have been added
%% by consult(File)
'$delete_all_code'(File) :-
    %% remove all watched rules
   '$qulog_interpreter_call'(unwatch),
   retract('$asserted_file_info'(File, Info)),
   retractall(Info),
   fail.
'$delete_all_code'(File) :-
    %%retractall('$tr_clause'(_, _)),
    %%retractall('$seen_clause'(_, _, _)),
    retractall('$saved_input_with_vars'(_, _, File)).


%% '$process_input'(Term) takes a parsed Term
%% and decides what to do with it.
%% The last rule (non-command processing) binds the Flag variable and so
%% if a command is processed where Flag is not a variable then we produce
%% an error message
%%'$process_input_top'(G, Vars, Flag) :- errornl('$process_input_top'(G, Vars, Flag)),fail.

'$process_input_top'(G, _Vars, Flag) :-
    G = (?- _),
    nonvar(Flag),
    assert('$head_order_check'('$ok')),
    !,
    writeL_(stderr, ["Error: ", G, " occurs after a non-command", nl_]),
    throw(consult_error(consult)).
'$process_input_top'((?- G), _, _) :-
    list(G),
    assert('$head_order_check'('$ok')),
    !,
    forall(member(C, G), '$process_input_top'('?-'(consult(C)), _, _)).
'$process_input_top'((?- runtime(Pred)), _, _) :-
    !,
    assert('$head_order_check'('$ok')),
    '$add_file_info'('$runtime_goal'(_)),
    assert('$runtime_goal'(Pred)).
'$process_input_top'('?-'(consult(G)), _, _) :- !,
    assert('$head_order_check'('$ok')),
    '$process_input_top'('?-'(qconsult_(G)), _, _).
'$process_input_top'('?-'(qconsult_(G0)), _, _) :- !,
    assert('$head_order_check'('$ok')),
     absolute_file_name(G0, G),
    concat_atom([G, '.qlg'], Gqlg),
    stat(Gqlg, stat(Time, _)),
    %% The first asserted file is the current file
    once('$current_consulted_file'(File)),
    (
      '$top_level_consulted_file'(File)
    ->
      assert('$consulted'(G))
    ;
      true
    ),
    (
      '$consulted_timestamp'(G, _, Time)
    ->
      %% already consulted
      true
    ;
      %% used to determine if this file had been updated for later use
      %% when re-consulting
      assert('$consulted_timestamp'(G, File, Time)),
      '$add_file_info'('$goal'(_)),
      assert('$goal'(qconsult_(G))),
      (qconsult_(G)-> true ; throw(consult_error(consult)))
    ).
'$process_input_top'('?-'(pconsult(G)), _, _) :- !,
    assert('$head_order_check'('$ok')),
    '$add_file_info'('$goal'(_)),
    assert('$goal'(pconsult(G))),
    assert('$delay_pconsult'(G)).
'$process_input_top'('?-'(G), _, _) :- !,
    assert('$head_order_check'('$ok')),
    '$add_file_info'('$goal'(_)),
    assert('$goal'(G)),
    call(G).
'$process_input_top'(T, VarNames, Flag) :-
    %% non-goal defn/declaration
    (
      var(Flag)
    ->
      %% first defn/declaration
      (
        once('$current_consulted_file'(File)), '$top_level_consulted_file'(File)
      ->
        %% check consulted files and restart consult if necessary
        once('$check_asserted_files')
      ;
        true
      ),
      %% we have seen a defn/decl
      Flag = set
    ;
      true
    ),
    '$process_input'(T, VarNames).

%% '$process_input'(Term, Vars) processes the input Term (defn/decl)
%% Vars is the var-name-map from the parser - used for show and error messages
%%'$process_input'(A, B) :- errornl('$process_input'(A, B)), fail.

'$process_input'((A ::= AT), VarNames) :-
    !,
    '$add_decl_doc'(A, AT, defined_type, "", VarNames),
    '$process_input_term'(defined_type(A, AT, VarNames)).
'$process_input'((A == AT), VarNames) :-
    !,
    '$add_decl_doc'(A, AT, macro_type, "", VarNames),
    '$process_input_term'(macro_type(A, AT, VarNames)).
'$process_input'(def_doc((A ::= AT), Doc), VarNames) :-
    !,
    '$add_decl_doc'(A, AT, defined_type, Doc, VarNames),
    '$process_input_term'(defined_type(A, AT, VarNames)).
'$process_input'(def_doc((A == AT), Doc), VarNames) :-
    !,
    '$add_decl_doc'(A, AT, macro_type, Doc, VarNames),
    '$process_input_term'(macro_type(A, AT, VarNames)).


'$process_input'(global_int_decl(A,B), _) :-
    !,
    '$process_input_term'(global_int_decl(A,B)).
'$process_input'(global_num_decl(A,B), _) :-
    !,
    '$process_input_term'(global_num_decl(A,B)).

'$process_input'('$code_decl'(percept, Types, Doc), VarNames) :-
    !,
    ip_set('$code_decl_term', '$code_decl'(percept, Types, Doc)),
    '$check_decl'('$code_decl'(percept, Types, Doc), VarNames),
    forall(member('$decl'(A,B), Types),
           '$add_decl_doc'(A, B, percept, Doc, VarNames)),
    assert('$head_order_check'('$ok')),
    forall(member('$decl'(A1, B1), Types),
           '$add_user_type_info'(A1, B1, percept, VarNames)).
'$process_input'('$code_decl'(mrel, Types, Doc), VarNames) :-
    !,
    ip_set('$code_decl_term', '$code_decl'(mrel, Types, Doc)),
    '$check_decl'('$code_decl'(rel, Types, Doc), VarNames),
    '$check_mrel_modes'('$code_decl'(rel, Types, Doc), VarNames),
    forall(member('$decl'(A,B), Types),
           '$add_decl_doc'(A, B, mrel, Doc, VarNames)),
    assert('$head_order_check'('$ok')),
    %%'$gen_should_infer_remember_fact'(Types),
    forall(member('$decl'(A1,B1), Types),
           '$add_cache_info'(A1, B1)),
    forall(member('$decl'(A1, B1), Types),
           '$add_user_type_info'(A1, B1, mrel, VarNames)).
'$process_input'('$code_decl'(mfun, Types, Doc), VarNames) :-
    !,
    ip_set('$code_decl_term', '$code_decl'(mfun, Types, Doc)),
    '$check_decl'('$code_decl'(fun, Types, Doc), VarNames),
    forall(member('$decl'(A,B), Types),
           '$add_decl_doc'(A, B, mfun, Doc, VarNames)),
    assert('$head_order_check'('$ok')),
    %%'$gen_should_infer_remember_fact'(Types),
    forall(member('$decl'(A1,B1), Types),
           '$add_mfun_info'(A1, B1)),
    forall(member('$decl'(A1, B1), Types),
           '$add_user_type_info'(A1, B1, mrel, VarNames)).
'$process_input'('$code_decl'(Kind, Types, Doc), VarNames) :-
    !,
    ip_set('$code_decl_term', '$code_decl'(Kind, Types, Doc)),
    '$check_decl'('$code_decl'(Kind, Types, Doc), VarNames),
    forall(member('$decl'(A,B), Types),
           '$add_decl_doc'(A, B, Kind, Doc, VarNames)),
    assert('$head_order_check'('$ok')),
    forall(member('$decl'(A1, B1), Types),
           '$add_user_type_info'(A1, B1, Kind, VarNames)).

%% Save TR programs 
'$process_input'(('$tel'(TRHead, TRBody)), _) :-
    '$running_teleo',
    !,
    assert('$head_order_check'('$ok')),
    '$add_file_info'('$tel'(_, _)),
    assert('$tel'(TRHead, TRBody)).
'$process_input'(('$tel'(TRHead, TRBody)), Vars) :-
    !,
    '$display_error'(only_teleor('$tel'(TRHead, TRBody), Vars)).


%% clause for definitions for functions/relations/actions
'$process_input'((Head -> Body), _) :-
    !,
    '$process_input_term'(clause((Head -> Body))).
'$process_input'((Head ~> Body), _) :-
    !,
    '$process_input_term'(clause((Head ~> Body))).
'$process_input'(act(Head), _) :-
    !,
    '$process_input_term'(clause(act(Head))).
'$process_input'(Clause, _) :-
    !,
    '$process_input_term'(clause(Clause)).


%% need failure to remove binding of vars to names
'$add_decl_doc'(A,B,C,D,E) :-
    '$add_decl_doc_aux'(A,B,C,D,E),fail.
'$add_decl_doc'(_, _, _, _, _).

'$add_decl_doc_aux'(A, AT, defined_type, Doc, Vars) :-
    !,
    '$vars2names'((A, AT), Vars),
    assert('$user_type_descr'(A, AT, defined_type, Doc)).
'$add_decl_doc_aux'(A, AT, macro_type, Doc, Vars) :-
    !,
    '$vars2names'((A, AT), Vars),
    assert('$user_type_descr'(A, AT, macro_type, Doc)).
'$add_decl_doc_aux'(A, '$tuple_type'(B), Kind, Doc, Vars) :-
    '$add_file_info'('$user_type_descr'(A, _, _, _)),
    Term '@=..' [A|B],
    '$vars2names'(Term, Vars),
    assert('$user_type_descr'(A, Term, Kind, Doc)).
'$add_decl_doc_aux'(A, ('$tuple_type'(B) -> RT), Kind, Doc, Vars) :-
    '$add_file_info'('$user_type_descr'(A, _, _, _)),
    Term '@=..' [A|B],
    '$vars2names'(Term, Vars),
    assert('$user_type_descr'(A, (Term -> RT), Kind, Doc)).

'$add_cache_info'(A, '$tuple_type'(B)) :-
    '$strip_decl_vars'(B, StripB),
    length(StripB, N),
    length(Args, N),
    length(CopyArgs, N),
    atom_concat(A, '_cache_call$$', RemCallF),
    atom_concat(A, '_cache_$$', RemF),
    '@..'(A, Args, Call),
    '@..'(RemCallF, CopyArgs, RemCall),
    '@..'(RemF, Args, Rem),
    '@..'(RemF, CopyArgs, RemCopy),
    '$link_!_args'(StripB, Args, CopyArgs),
    %dynamic( RemF/N, 0),
    '$add_file_info'(cache_info__(_, _, _, _)),
    '$add_file_info'(RemCopy),
    assert(cache_info__(Call,  RemCall, Rem, RemCopy)),
    '$add_file_info'(Call),
    assert((Call :- cache_call__(RemCall, Rem, RemCopy))).

'$add_mfun_info'(A, ('$tuple_type'(B) -> _R)) :-
    '$strip_decl_vars'(B, StripB),
    length(StripB, N),
    N1 is N+1,
    length(Args, N1),
    length(CopyArgs, N1),
    atom_concat(A, '_cache_call$$', RemCallF),
    atom_concat(A, '_cache_$$', RemF),
    '@..'(A, Args, Call),
    '@..'(RemCallF, Args, RemCall),
    '@..'(RemF, Args, Rem),
    '@..'(RemF, CopyArgs, RemCopy),
    '$link_fun_args'(Args, CopyArgs),
    %dynamic( RemF/N1, 0),
    '$add_file_info'(cache_info__(_, _, _, _)),
    '$add_file_info'(RemCopy),
    assert(cache_info__(Call,  RemCall, Rem, RemCopy)),
    '$add_file_info'(Call),
    assert((Call :- cache_call__(RemCall, Rem, RemCopy))).
           
'$link_!_args'([], [], []).
'$link_!_args'(['!'(_)|Rest], [X|Args], [X|CopyArgs]) :-
    !,
    '$link_!_args'(Rest, Args, CopyArgs).
'$link_!_args'([_|Rest], [_|Args], [_|CopyArgs]) :-
    !,
    '$link_!_args'(Rest, Args, CopyArgs).

'$link_fun_args'([_], [_]).
'$link_fun_args'([X|Args], [X|CopyArgs]) :-
    !,
    '$link_fun_args'(Args, CopyArgs).

%% Declarations can have variables - used for documentation - that need
%% to be removed. For some declarations default mode declaraions are
%% added where required so that when looking up type info all the required
%% modes are present
%%'$add_user_type_info'(A, B, Kind, VarNames) :- errornl('$add_user_type_info'(A, B, Kind, VarNames)),fail.
'$add_user_type_info'(A1, '$tuple_type'(B1), percept, VarNames) :-
    !,
    '$strip_decl_vars'(B1, StripB),
    length(StripB, N),
    length(Args, N),
    '@..'(A1, Args, RC),
    '$add_file_info'('$user_type_info'(A1, _, _, _, _)),
    '$add_annotations'(StripB, Annotated, '?'),
    %% percepts are treated as relations with default mode ?
    assert('$user_type_info'(A1, rel, Annotated, RC, VarNames)),
    %% remember for later processing
    assert('$tmp_percepts'(A1)).
'$add_user_type_info'(A1, '$tuple_type'(B1), dyn, VarNames) :-
    !,
    '$strip_decl_vars'(B1, StripB),
    length(StripB, N),
    length(Args, N),
    '@..'(A1, Args, RC),
    functor(RC, _, M),
    '$add_file_info'('$user_type_info'(A1, _, _, _, _)),
    '$add_annotations'(StripB, Annotated, '?'),
    %% dynamics are treated as relations with default mode ?
    assert('$user_type_info'(A1, rel, Annotated, RC, VarNames)),
    '$add_file_info'('$belief'(RC)),
    dynamic(A1/M, 0), 
    assert('$belief'(RC)).
'$add_user_type_info'(A1, '$tuple_type'(B1), mrel, VarNames) :-
    !,
    '$strip_decl_vars'(B1, StripB),
    length(StripB, N),
    length(Args, N),
    '@..'(A1, Args, RC),
    '$add_file_info'('$user_type_info'(A1, _, _, _, _)),
    '$add_annotations'(StripB, Annotated, '?'),
    %% infer-remembers are treated as relations with default mode ?
    assert('$user_type_info'(A1, rel, Annotated, RC, VarNames)).



'$add_user_type_info'(A1, '$tuple_type'(B1), tel_atomic, VarNames) :-
    !,
    '$add_file_info'('$tel_atomic'(_)),
    assert('$tel_atomic'(A1)),
    '$add_user_type_info'(A1, '$tuple_type'(B1), tel, VarNames).
'$add_user_type_info'(A1, '$tuple_type'(B1), tel_start, VarNames) :-
    !,
    '$add_file_info'('$tel_start'(_)),
    assert('$tel_start'(A1)),
    '$add_user_type_info'(A1, '$tuple_type'(B1), tel, VarNames).
    
'$add_user_type_info'(A1, '$tuple_type'(B1), Kind, VarNames) :-
    !,
    freeze_term(B1, FVars),
    once('$extract_default_args'(B1, B2, _SeenDefault, Args1, Args2, DTypes)),
    thaw_term(FVars),
    (
      Args2 = []
    ->
      true
    ;
      once('$add_annotations'(DTypes, ADTypes, '!')),
      '$check_types_default_args'(Args2, ADTypes),
      '$assert_default_args'(A1, Args1, Args2)
    ),
    '$strip_decl_vars'(B2, StripB),
    length(StripB, N),
    length(Args, N),
    '@..'(A1, Args, RC),
    '$add_file_info'('$user_type_info'(A1, _, _, _, _)),
    '$kind_mapping'(Kind, FullKind),
    '$add_annotations'(StripB, Annotated, '!'),
    map(push_modes_in__, Annotated, PushedAnnotated),
    assert('$user_type_info'(A1, FullKind, PushedAnnotated, RC, VarNames)).

'$add_user_type_info'(A1, ('$tuple_type'(B1) -> RT), _, VarNames) :-
    !,
    freeze_term(B1, FVars),
    once('$extract_default_args'(B1, B2, _SeenDefault, Args1, Args2, DTypes)),
    thaw_term(FVars),
    (
      Args2 \= [], compound(RT), RT = (_ -> _)
    ->
      %% We don't allow default args in HO functions - for the minimal
      %% uses of HO functions it's not worth the effort
      '$display_error'(ho_default_args(A1))
    ;
      true
    ),
    (
      Args2 = []
    ->
      true
    ;
      once('$add_annotations'(DTypes, ADTypes, '!')),
      '$check_types_default_args'(Args2, ADTypes),
      '$assert_default_args'(A1, Args1, Args2)
    ),
 
    '$strip_decl_vars'(B2, StripB),
    '$get_full_goal_term'(A1, ('$tuple_type'(StripB) -> RT), GT),
    '$add_file_info'('$user_type_info'(A1, _, _, _, _)),
    assert('$user_type_info'(A1, fun, ('$tuple_type'(StripB) -> RT),
                             GT, VarNames)).




%% Extract out the default args (and their types) so we can type check
%% and assert default_arg info - used above.

%%'$extract_default_args'(A,B,C,D,E,F) :- errornl('$extract_default_args'(A,B,C,D,E,F)), fail.
'$extract_default_args'([], [], _SeenDefault, [], [], []).
'$extract_default_args'([default((_:T), Def)|B1], [T|B2], SeenDefault,
                        A1, [Def|A2], [T|Ts]) :-
    !,
    SeenDefault = true,
    '$extract_default_args'(B1, B2, SeenDefault, A1, A2, Ts).
'$extract_default_args'([default(T, Def)|B1], [T|B2], SeenDefault,
                        A1, [Def|A2], [T|Ts]) :-
    !,
    SeenDefault = true,
    '$extract_default_args'(B1, B2, SeenDefault, A1, A2, Ts).
'$extract_default_args'(_, _, SeenDefault, _, _, _) :-
    atom(SeenDefault), !,
    ip_lookup('$code_decl_term', Decl),
    '$display_error'(default_args_order(Decl)).
'$extract_default_args'([T|B1], [T|B2], SeenDefault,
                        [_|A1], A2, Ts) :-
    !,
    '$extract_default_args'(B1, B2, SeenDefault, A1, A2, Ts).

%% EG in the builtins we have
%% act connect_to_pedro(Host : !atom default localhost,
%%                      Port : !int default 4550)
%% We generate
%% '$builtin_default_args'(connect_to_pedro('$none_'), connect_to_pedro(localhost, 4550)).
%% '$builtin_default_args'(connect_to_pedro(_10624), connect_to_pedro(_10624, 4550)).
%% (except we use user_default_arg)
%%'$assert_default_args'(A1, Args1, Args2) :- errornl('$assert_default_args'(A1, Args1, Args2)),fail.
'$assert_default_args'(F, Args1, Args2) :-
    '$add_file_info'('$user_default_args'(_, _)),
    append(D1, D2, Args2),
    D2 \= [],
    length(D1, N),
    length(A1, N),
    append(Args1, A1, Args),
    append(Args, D2, DArgs),
    '@..'(F, Args, FA),
    '@..'(F, DArgs, DFA),
    assert('$user_default_args'(FA, DFA)),
    fail.
'$assert_default_args'(_, _, _).

%% check the type of the default args. NOTE - the default args must be ground
%% and the mode must be fully ground
'$check_types_default_args'([], []).
'$check_types_default_args'([A|Args2], [T|DTypes]) :-
    (
      ground((A, T))
    ->
      push_modes_in__(T, MT),
      (
        fully_ground__(MT)
      ->
        has_type__(A, MT, [], [], _, [], LE, [], [], 0, A, Error),
        bind_type_constraints__(LE, Error),
        (
          var(Error)
        ->
          true
        ;
          ip_lookup('$code_decl_term', Decl),
          '$display_error'(default_arg_type(Decl, A, T))
        )
      ;
        ip_lookup('$code_decl_term', Decl),
        '$display_error'(default_arg_mode(Decl, A, T))
      )
    ;
      ip_lookup('$code_decl_term', Decl),
      '$display_error'(default_arg_not_ground(Decl, A, T))
    ),
    '$check_types_default_args'(Args2, DTypes).
    


%% When we do a consult from the interpreter then:
%% 1. It is the first consult in the interpreter session - there are
%%     no saved_consulted facts and so nothing is done
%% 2. There was a previous consult - all the facts about consulted files
%%    ( '$consulted' facts) are removed and replaced by '$saved_consulted'
%%    facts and the files are reconsulted. This compares the previous list
%%    of consulted files (in order) with the current list of consulted files
%%    (in order) and if there are any differences then we need to reconsult
'$check_asserted_files' :-
    '$saved_consulted'(_), !,
    '$top_level_consulted_file'(File),
    findall(F, ('$saved_consulted'(F), F \= File), OldConsulted),
    %once('$current_consulted_file'(File)),
    findall(File1, '$consulted'(File1), NewConsulted),
    retractall('$saved_consulted'(_)),
    (
      OldConsulted = NewConsulted
    ->
      true
    ;
      writeL_(stderr, [nl_,
                       "The consulted files have changed - restarting consult",
                       nl_]),
      throw(consult_error(consults_diff))
    ), !.
'$check_asserted_files'.

%% Sometimes a definition will have a variable representing a function
%% applied to arguments ( like using call in Prolog) - e.g.
%% mapF(F, [H|T]) -> [F(H)|mapF(F, T)]
%% This is translated to use apply_(F, H, Result)
%% and to see determine what apply does to a given function we generate
%% auxilary clauses - e.g. for the defn of factorial we would generate
%% 'factorial$apply'(fact, 0, A) :- factorial(0, A).
%% 'factorial$apply'(fact, A, B) :- factorial(A, B).
%% where factorial/2 is the translation of the factorial function
%% The definition of apply constructs the appropriate call on the
%% above auxilary predicate.
%% For higer-order predicates like curry we have definitions for
%% when all the possible number of args are already applied :
%% 'curry$apply'(curry(A)(B), C, D) :- %do the evaluation - all args given
%% 'curry$apply'(curry(A), B, curry(A)(B)). % build a structure
%% 'curry$apply'(curry, A, curry(A)).
%% This technique is also used for higher-order relations.
'$process_apply_clause'(Head, Body) :-
    functor(Head, F, N),
    functor(Template, F, N),
    '$add_file_info'(Template),
    Head = Apply(Fun, _Args, _Ret),
    (
      current_predicate(Apply/3), clause(Apply(_, _, _), _)
    ->
      true
    ;
      '$process_apply_clause_aux'(Apply, Fun, _)
    ),
    assert((Head :- Body)).

'$process_apply_clause_aux'(_, Fun, Fun) :-
    atom(Fun), !.
'$process_apply_clause_aux'(Apply, Fun, TemplateFun) :-
    Fun =.. [F|Args],
    '$process_apply_clause_aux'(Apply, F, TemplateF),
    length(Args, N),
    length(TempArgs, N),
    %'$list2tuple'(TempArgs, Tuple),
    TemplateFun =.. [TemplateF|TempArgs],
    assert((Apply(TemplateF, TempArgs, TemplateFun) :- !)).


%% Check if all rules have an allowed type declaration for the head
'$check_if_clauses_have_types' :-
    '$tmp_clause'(_, clause(Clause)),
    freeze_term(Clause),
    '$check_clause_head_for_type'(Clause),
    fail.
'$check_if_clauses_have_types' :-
    '$tel'(TRHead, TRBody),
    freeze_term('$tel'(TRHead, TRBody)),
    '$check_clause_head_for_type'('$tel'(TRHead, TRBody)),
    fail.
'$check_if_clauses_have_types'.

'$check_declaration_modes' :-
    '$saved_input_with_vars'(Decl,_,_),
    Decl = '$code_decl'(Kind, Decls, _),
    Kind \= fun,
    Kind \= tel,
    member('$decl'(Name,'$tuple_type'(Types)), Decls),
    member(T, Types),
    '$check_declaration_modes_aux'('!', T, T, Kind(Name(Types))),
    fail.
'$check_declaration_modes'.

%%'$check_declaration_modes_aux'(M0, MT, TopMT, Name) :- errornl('$check_declaration_modes_aux'(M0, MT, TopMT, Name)),fail.
'$check_declaration_modes_aux'(M0, MT, TopMT, Name) :-
    moded__(MT), !,
    MT = M(T),
    (
      min_mode__(M0, M, M0)
    ->
      '$check_declaration_modes_aux1'(M, T, TopMT, Name)
    ;
      '$display_error'(declaration_mode_error(TopMT, Name))
    ).
'$check_declaration_modes_aux'(M0, T, TopMT, Name) :-
    '$check_declaration_modes_aux1'(M0, T, TopMT, Name).

'$check_declaration_modes_aux1'(M, T, TopMType, Name) :-
    constructor_type__(T, _, Types), !,
    '$check_declaration_modes_aux2'(Types, M, TopMType, Name).
'$check_declaration_modes_aux1'(_, _, _, _).

%%'$check_declaration_modes_aux2'(A,B,C,D) :- errornl('$check_declaration_modes_aux2'(A,B,C,D)),fail.
'$check_declaration_modes_aux2'([], _, _, _).
'$check_declaration_modes_aux2'([T|Types], M, TopMType, Name) :-
    '$check_declaration_modes_aux'(M, T, TopMType, Name),
    '$check_declaration_modes_aux2'(Types, M, TopMType, Name).
 
    
    
    
    
    

 %% check that in t1(X) ::= t2, the X occurs on RHS
'$check_defined_type_vars' :-
    once('$current_consulted_file'(File)),
    '$user_type_info'(_P, defined_type, TName, T, VarNames),
    '$saved_input_with_vars'((TName ::= T), _, File),
    collect_vars(TName, HeadVars),
    collect_vars(T, BodyVars),
    (
      diff_list(HeadVars, BodyVars, [])
    ->
      true
    ;
      '$display_error'(invalid_type_unmatched_vars((TName ::= T), VarNames))
    ),
    fail.
'$check_defined_type_vars'.

'$check_tel_action' :-
    '$user_type_info'(tel_action_term, defined_type, tel_action_term, Defn, _),
    Defn = '$constr_enum_type'(Alts),
    member(D, Alts),
    '@..rev'(D, A, Types),
    length(Types, N),
    length(Vars, N),
    '@..'(A, Vars, Patt),
    '$add_file_info'('$tel_action'(_)),
    assert('$tel_action'(Patt)),
    fail.
'$check_tel_action'.


%% Look to see if a relation/action/function/TR has an associated
%% consistent type - if not add a general type involving term as the type

'$check_clause_head_for_type'('?-'(_)) :- !.
'$check_clause_head_for_type'((A :: _B)) :-
    !,
    '$check_clause_head_for_type_aux'(A, rel).
'$check_clause_head_for_type'((A :: _B -> _C)) :-
    !,
    '$check_clause_head_for_type_aux'(A, fun).
'$check_clause_head_for_type'((A -> _C)) :-
    !,
    '$check_clause_head_for_type_aux'(A, fun).
'$check_clause_head_for_type'((A :: _B <= _C)) :-
    !,
    '$check_clause_head_for_type_aux'(A, rel).
'$check_clause_head_for_type'((A <= _C)) :-
    !,
    '$check_clause_head_for_type_aux'(A, rel).
'$check_clause_head_for_type'((A :: _B ~> _C)) :-
    !,
    '$check_clause_head_for_type_aux'(A, act).
'$check_clause_head_for_type'((A ~> _C)) :-
    !,
    '$check_clause_head_for_type_aux'(A, act).
'$check_clause_head_for_type'('$tel'(A, _)) :-
    !,
    '$check_clause_head_for_type_aux'(A, tel).
'$check_clause_head_for_type'(A) :-
    !,
    '$check_clause_head_for_type_aux'(A, rel).

%% We check that the head of each rule has a type declaration and
%%'$check_clause_head_for_type_aux'(A, K) :- errornl('$check_clause_head_for_type_aux'(A, K)),fail.
%% The user as made a type declaration - nothing to do
'$check_clause_head_for_type_aux'(A, _) :-
    '$get_ultimate_functor'(A, F),
    %functor(A, F, _),
    '$user_type_info'(F, _, _, _, _), !.
%% The user is defining something that has a builtin type
%% but is intended for the user to define  - nothing to do
'$check_clause_head_for_type_aux'(A, _) :-
    '$get_ultimate_functor'(A, F),
    '$dynamic_builtin_type_info'(F, _, _, _, _), !.
%% The user is defining something that has a builtin type
%% that is not supposed to be defined by the user - error
'$check_clause_head_for_type_aux'(A, _K) :-
    '$get_ultimate_functor'(A, F),
    '$builtin_type_info'(F, Kind, _, _, _), !,
    (
      '$system_user_type'(F)
    ->
      true
    ;
    '$display_error'(builtin_redefinition(F, Kind))
    ).
%% Otherwise we produce an undeclared error - already processed
'$check_clause_head_for_type_aux'(A, _K) :-
    '$get_ultimate_functor'(A, F),
    '$undeclared_defn'(F), !.
%% Otherwise we produce an undeclared error
'$check_clause_head_for_type_aux'(A, Kind) :-
    '$get_ultimate_functor'(A, F),
    '$add_file_info'('$undeclared_defn'(_)),
    assert('$undeclared_defn'(F)),
    '$display_error'(undeclared_code(F, Kind)).


%% process all the terms that come from the parser (through process_input)
%'$process_input_term'(A) :- errornl('$process_input_term'(A)),fail.

%% For a global state var like
%% int count := 0
%% we add
%% '$user_type_info'(count, rel['?'(int)], count(_), []))
%% '$belief'(count(_))
%% '$state_belief'(count)
%% and eventually the fact count(0) will be added
%% The state variable is updated by rectract/assert
'$process_input_term'(global_int_decl(A,B)) :-
     !,
    assert('$head_order_check'('$ok')),
    (
      '$type_info'(A, _ , _, _, _)
    ->
      '$real_kind'(A, Kind),
      '$display_error'(re_declaration_kind(A, global_int_decl(A, B), Kind))
    ;
     dynamic(A/1, 0),
     '$add_file_info'('$user_type_info'(A, _, _, _, _)),
     assert('$user_type_info'(A, rel, ['?'(int)], A(_), [])),
     '$add_file_info'('$belief'(A(_))),
     '$add_file_info'('$state_belief'(A)),
     assert('$belief'(A(_))),
     assert('$state_belief'(A)),
     assert('$tmp_clause'(A, clause(A(B))))
    ).
'$process_input_term'(global_num_decl(A,B)) :-
     !,
    assert('$head_order_check'('$ok')),
    (
      '$type_info'(A, _ , _, _, _)
    ->
      '$real_kind'(A, Kind),
      '$display_error'(re_declaration_kind(A, global_num_decl(A, B), Kind))
    ;
      dynamic(A/1, 0),
     '$add_file_info'('$user_type_info'(A, _, _, _, _)),
      assert('$user_type_info'(A, rel, ['?'(num)], A(_), [])),
     '$add_file_info'('$belief'(A(_))),
     '$add_file_info'('$state_belief'(A)),
      assert('$belief'(A(_))),
      assert('$state_belief'(A)),
      assert('$tmp_clause'(A, clause(A(B))))
    ).

%% Anything that is a clause defn is saved away
'$process_input_term'(clause(A)) :-
    !,
    '$get_clause_head_functor'(A, HeadF),
    assert('$head_order_check'(HeadF)),
    assert('$tmp_clause'(HeadF, clause(A))).


%% process defined types
'$process_input_term'(defined_type(user_exception, AT, VarNames)) :-
    \+ '$user_type_info'(user_exception, _, _, _, _), !,
    '$add_file_info'('$user_type_info'(user_exception, _, _, _, _)),
    assert('$user_type_info'(user_exception, defined_type, user_exception,
                             AT, VarNames)).

    
'$process_input_term'(defined_type(A, AT, VarNames)) :-
    A \= user_exception,
    functor(A, P, _N),
    '$type_info'(A, Kind, _, _, _), !,
    assert('$head_order_check'('$ok')),
    %% A already has type declaration
    '$vars2names'((A, AT), VarNames),
    '$display_error'(re_declaration_kind(P, (A ::= AT), Kind)).
 '$process_input_term'(defined_type(A, AT, VarNames)) :-
    !,
    assert('$head_order_check'('$ok')),
    functor(A, P, _N),
    '$add_file_info'('$user_type_info'(P, _, _, _, _)),
    assert('$user_type_info'(P, defined_type, A, AT, VarNames)).
%% process macro types
'$process_input_term'(macro_type(A, AT, VarNames)) :-
    functor(A, P, _N),
    '$type_info'(A, Kind, _, _, _), !,
    assert('$head_order_check'('$ok')),
    %% A already has type declaration
    '$vars2names'((A, AT), VarNames),
    '$display_error'(re_declaration_kind(P, (A == AT), Kind)).
 '$process_input_term'(macro_type(A, AT, VarNames)) :-
    !,
    assert('$head_order_check'('$ok')),
    functor(A, P, _N),
    '$add_file_info'('$user_type_info'(P, _, _, _, _)),
    assert('$user_type_info'(P, macro_type, A, AT, VarNames)).


%% Should never get here
'$process_input_term'(D) :-
    writeL_(stderr, ["Error: unprocessed term ", D, nl_]).



%% Translate clauses (rules) into Prolog clauses
%%'$translate_clause'(X) :- errornl('$translate_clause'(X)),fail.
'$translate_clause'((Head :: Test -> Body)) :-
    !,
    '$translate_function'(Head, Test, Body).
'$translate_clause'((Head -> Body)) :-
    !,
    '$translate_function'(Head, true, Body).
'$translate_clause'((Head :: Test <= Body)) :-
    !,
    '$translate_body'(((Test & !) & Body), TransBody),
    '$assert_the_relation_clause'(Head, TransBody).
'$translate_clause'((Head <= Body)) :-
    !,
    '$translate_body'((Body), TransBody),
    '$assert_the_relation_clause'(Head, TransBody).
'$translate_clause'((Head :: Test ~> Body)) :-
    !,
    '$translate_body'((Test & ! & Body), TransBody),
    '$assert_the_relation_clause'(Head, TransBody).
%% Action rules without a commit have an implicit ::true  commit
'$translate_clause'((Head ~> Body)) :-
    !,
    '$translate_body'((!, Body), TransBody),
    '$assert_the_relation_clause'(Head, TransBody).
'$translate_clause'((Head :: Test)) :-
    !,
    '$translate_body'((Test & !), TransBody),
    '$assert_the_relation_clause'(Head, TransBody).
'$translate_clause'(Head) :-
    !,
    (
     '$type_info'(_, act, _, Head, _)
    ->
      %% implicit :: true commit
     '$assert_the_relation_clause'(Head, !)
    ;
     '$assert_the_relation_clause'(Head, true)
    ).

%% Add the translated rule to the DB
%%'$assert_the_relation_clause'(Head, TransBody) :- errornl('$assert_the_relation_clause'(Head, TransBody)),fail.
%% Special case when we have a higher order relation
%% These are functions that, given all but the last collection
%% of args returns a relation
'$assert_the_relation_clause'(Head, true) :-
    cache_info__(Head, RemHead, R, P), !,
    R = P,
    '@functor'(RemHead, F, N),
    '@functor'(Template, F, N),
    '$add_file_info'(Template),
    assert(RemHead).
'$assert_the_relation_clause'(Head, TransBody) :-
    cache_info__(Head, RemHead, R, P), !,
    R = P,
    '@functor'(RemHead, F, N),
    '@functor'(Template, F, N),
    '$add_file_info'(Template),
    assert((RemHead :- TransBody)).

'$assert_the_relation_clause'(Head, TransBody) :-
    compound(Head), functor(Head, F, _), compound(F), !,
    Head =.. [F|Args],
    '$generate_apply_name'(F, ApplyName),
    A1 = ApplyName(F, Args, true),
    '$process_apply_clause'(A1, TransBody).
'$assert_the_relation_clause'(Head, true) :-
    !,
    '@functor'(Head, F, N),
    '@functor'(Template, F, N),
    '$add_file_info'(Template),
    assert(Head).
'$assert_the_relation_clause'(Head, TransBody) :-
    '@functor'(Head, F, N),
    '@functor'(Template, F, N),
    '$add_file_info'(Template),
    assert((Head :- TransBody)).

%% For translating a function
%% f(A,B) -> Term
%% we translate first to
%% f(A, B, X) as the head and Term = X as the body
%% and then translate the body.
%% This converts function clauses into relation clauses where
%% the last argument is the result
%%'$translate_function'(Head, Test, Body) :- errornl('$translate_function'(Head, Test, Body)),fail.
%% Special case for higher-order functions
'$translate_function'(Head, Test, Body) :-
     '$get_function_type_info'(Head,HeadT, _, ResultType),
    length(HeadT, Len),
    Len \= 1, !,
    '$add_fun_args'(ResultType, Head, FullHead, Body, FullBody),
    freeze_term(FullBody),
    FBody = (FullBody = X),
    FullHead =.. [F|Args],
    %'$list2tuple'(Args, ArgsTuple),
    '$translate_body'(((Test, !), FBody), Unfolded),
    '$generate_apply_name'(F, ApplyName),
    A1 = ApplyName(F, Args, X),
    '$process_apply_clause'(A1, Unfolded).
'$translate_function'(Head, Test, Body) :-
     FBody = (Body = X),
    freeze_term(X),
    '@=..'(Head, P),
    append(P, [X], P1),
    A1 =.. P1,
    P = [F|Args],
    '$generate_apply_name'(F, ApplyName),
    '$add_file_info'(ApplyName(_, _, _)),
    (
      current_predicate(ApplyName/3), clause(ApplyName(_, _, _), _)
    ->
      true
    ;
      length(Args, ArgN),
      length(TempArgs, ArgN),
      %'$list2tuple'(TempArgs, TempArgTuple),
      append(TempArgs, [TempX], AllArgs),
      TempA1 =.. [F|AllArgs],
      assert((ApplyName(F, TempArgs, TempX) :- TempA1))
    ),
    '$translate_body'(((Test, !), FBody), Unfolded),
    '@functor'(A1, F1, N1),
    '@functor'(Template, F1, N1),
    '$add_file_info'(Template),
    assert_the_function_call__(A1, Unfolded).

assert_the_function_call__(Head, TransBody) :-
    cache_info__(Head, RemHead, _, _), !,
    '@functor'(RemHead, F, N),
    '@functor'(Template, F, N),
    '$add_file_info'(Template),
    assert((RemHead :- TransBody)).

assert_the_function_call__(A1, Unfolded) :-
    assert((A1 :- Unfolded)).

%% for translating the body we freeze the vars in goals so
%% they don't get matched against patterns
'$translate_body'(Goals, TransGoals) :-
    freeze_term(Goals),
    '$translate_goal'(Goals, TransGoals),
    thaw_term(Goals).

%% For higher order functions it's possible to write down a rule
%% where the LHS doesn't include all args.
%% EG
%% f : t1 -> (t2, t3) -> t4 -> t5
%% f(X) -> g(h(X))
%% Given this rule is correctly typed then we could rewrite as
%% f(X)(Y,Z)(W) -> g(h(X))(Y,Z)(W)
%% This predicate adds any "missing" arguments
'$add_fun_args'(Type, Term, OutTerm, Body, OutBody) :-
    compound(Type), Type = (T1 -> T2), !,
    '$tuple2list'(T1, T1L),
    length(T1L, N),
    length(Args, N),
    Term1 =.. [Term|Args],
    Body1 =.. [Body|Args],
    '$add_fun_args'(T2, Term1, OutTerm, Body1, OutBody).
'$add_fun_args'(_Type, Term, Term, Body, Body).




%% translate qulog goals to prolog goals
%% Note that arguments in goals might contain expressions that
%% need to be evaluated. These evaluations will be unfolded to prolog
%% goals that are called before the goal is called
%%'$translate_goal'(X,_) :- errornl('$translate_goal'(X)),fail.

%%'$translate_goal'(G, TG) :- write('$translate_goal'(G, TG)),nl,fail.
%% For goals whose functors are not atoms we need to go through apply
'$translate_goal'(G, TG) :-
    '$get_ultimate_functor'(G, UF),
    '$type_info'(UF, fun, (_ -> _), '$function', _), !,
    G '@=..' [F|Args],
    append(FArgs, [R], Args),
    Term '@=..' [F|FArgs],
    '$translate_goal'((Term = R), TG).
'$translate_goal'(G, TG) :-
    '$get_ultimate_functor'(G, F),
    var(F), !,
    '$expand_funs'(G, GT, GF),
    append(GF, [call(GT)], All),
    '$list2tuple'(All, TG).
%% The following are built-in rel/acts
'$translate_goal'(sort(A,B,C), TG) :-
    '$expand_funs'(A, AT, AF),
    '$expand_funs'(B, BT, BF),
    append(BF, ['$sort__'(AT,BT,C)], G1),
    append(AF, G1, G2),
    '$list2tuple'(G2, TG).
'$translate_goal'(re_match(A,B,C), TG) :-
    '$expand_funs'(A, AT, AF),
    '$expand_funs'(B, BT, BF),
    append(BF, ['$re_match__'(AT,BT,C)], G1),
    append(AF, G1, G2),
    '$list2tuple'(G2, TG).
'$translate_goal'((B1 & B2), (TB1, TB2)) :-
    !,
    '$translate_goal'(B1, TB1),
    '$translate_goal'(B2, TB2).
'$translate_goal'((B1 ; B2), (TB1, TB2)) :-
    !,
    '$translate_goal'(B1, TB1),
    '$translate_goal'(B2, TB2).
'$translate_goal'((B1 , B2), (TB1, TB2)) :-
    !,
    '$translate_goal'(B1, TB1),
    '$translate_goal'(B2, TB2).

'$translate_goal'(G, RG) :-
    G == true('$none_'), !,
    RG = true.
'$translate_goal'(G, RG) :-
    G == false('$none_'), !,
    RG = false.
'$translate_goal'(G, RG) :-
    G == fail('$none_'), !,
    RG = fail.
'$translate_goal'(same_thread_handle(H1, H2), RG) :-
    !,
    '$expand_funs'(H1, H1T, GH1),
    '$expand_funs'(H2, H2T, GH2),
    append(GH1, GH2, GA),
    append(GA, ['$same_thread_handle_'(H1T, H2T)], Goals),
    '$list2tuple'(Goals, RG).
'$translate_goal'(same_agent_handle(H1, H2), RG) :-
    !,
    '$expand_funs'(H1, H1T, GH1),
    '$expand_funs'(H2, H2T, GH2),
    append(GH1, GH2, GA),
    append(GA, ['$same_agent_handle_'(H1T, H2T)], Goals),
    '$list2tuple'(Goals, RG).

'$translate_goal'(raise(P), raise(P)) :- !.
'$translate_goal'(remember(G), RG) :-
    !,
    '$expand_funs'(G, GT, GA),
    append(GA, [remember(GT)], Goals),
    '$list2tuple'(Goals, RG).
'$translate_goal'(remember_for(G,T), RG) :-
    !,
    '$expand_funs'(G, GT, GA),
    '$expand_funs'(T, TT, TG),
    append(GA, TG, GG),
    append(GG, [remember_for(GT, TT)], Goals),
    '$list2tuple'(Goals, RG).
'$translate_goal'(forget(G), FG) :-
    '$expand_funs'(G, GT, GA),
    append(GA, [forget(GT)], Goals),
    '$list2tuple'(Goals, FG),
    !.
'$translate_goal'(forget_after(G,T), RG) :-
    !,
    '$expand_funs'(G, GT, GA),
    '$expand_funs'(T, TT, TG),
    append(GA, TG, GG),
    append(GG, [forget_after(GT, TT)], Goals),
    '$list2tuple'(Goals, RG).
'$translate_goal'(repeat_until(Act, '$fail'), FG) :-
    !,
    '$translate_goal'(Act, AGoal),
    '$tuple2list'(AGoal, AG),
    append([repeat|AG], [fail], Goals),
    '$list2tuple'(Goals, FG).
'$translate_goal'(repeat_until(Act, Cond), FG) :-
    !,
    '$translate_goal'(Act, AGoal),
    '$tuple2list'(AGoal, AG),
    '$translate_goal'(Cond, CGoal),
    '$tuple2list'(CGoal, CG),
    append(CG, ['!'], CGC),
    append([repeat|AG], CGC, Goals),
    '$list2tuple'(Goals, FG).
'$translate_goal'(try_except(Try, Excepts), FG) :-
    !,
    reverse(Excepts, RevExcepts),
    '$translate_try_except'(RevExcepts, Try, FG).

'$translate_goal'(case(Alts), Trans) :-
    !,
    '$translate_case'(Alts, Trans, case(Alts)).
'$translate_goal'(wait_case(Alts), Trans) :-
    !,
    '$translate_wait_case'(Alts, Trans).
'$translate_goal'(receive(Alts), Trans) :-
    !,
    '$translate_receive'(Alts, Trans).
'$translate_goal'(atomic_action(Acts), FG) :-
    !,
    '$translate_goal'(Acts, TransActs),
    FG = thread_atomic_goal(TransActs).
'$translate_goal'(wait(G), FG) :-
    !,
    '$translate_goal'(G, TransG),
    FG = thread_wait_on_goal(TransG).
% '$translate_goal'(wait(G, Preds, '$wait_timeout'(Time, Act)), FG) :-
%     !,
%     '$translate_goal'(G, TransG),
%     '$translate_goal'(Act, TransAct),
%     FG = '$qulog_wait'(TransG, Preds, '$wait_timeout'(Time, TransAct)).
% '$translate_goal'(wait(G, Preds, '$empty'), FG) :-
%     !,
%     '$translate_goal'(G, TransG),
%     FG = '$qulog_wait'(TransG, Preds, '$empty').
'$translate_goal'('$rel_escape'(G), FG) :-
    !,
    '$translate_goal'(G, TransG),
    FG = '$rel_escape'(TransG).
'$translate_goal'('$remote_query'('$num_vars_query'(N, V, G), H), FG) :-
    !,
    '$expand_funs'(H, HT, HGoals),
    '$expand_funs'(N, NT, NGoals),
    append(NGoals, HGoals, NHGoals),
    retract('$remote_query_info'('$remote_query'('$num_vars_query'(N,V, G), H), VTB)),
    Remote = '$remote_query'(VTB, '$num_vars_query'(NT, V, G), HT),
    append(NHGoals, [Remote], FGList),
    '$list2tuple'(FGList, FG).
'$translate_goal'('$remote_query'('$vars_query'(V, G), H), FG) :-
    !,
    '$expand_funs'(H, HT, HGoals),
    retract('$remote_query_info'('$remote_query'('$vars_query'(V, G), H), VTB)),
    Remote = '$remote_query'(VTB, '$vars_query'(V, G), HT),
    append(HGoals, [Remote], FGList),
    '$list2tuple'(FGList, FG).
'$translate_goal'('$remote_query'('$num_query'(N, G), H), FG) :-
    !,
    '$expand_funs'(H, HT, HGoals),
    '$expand_funs'(N, NT, NGoals),
    append(NGoals, HGoals, NHGoals),
    retract('$remote_query_info'('$remote_query'('$num_query'(N, G), H), VTB)),
    Remote = '$remote_query'(VTB, '$num_query'(NT,G), HT),
    append(NHGoals, [Remote], FGList),
    '$list2tuple'(FGList, FG).
'$translate_goal'('$remote_query'(G, H), FG) :-
    !,
    '$expand_funs'(H, HT, HGoals),
    retract('$remote_query_info'('$remote_query'(G, H), VTB)),
    Remote = '$remote_query'(VTB, G, HT),
    append(HGoals, [Remote], FGList),
    '$list2tuple'(FGList, FG).
'$translate_goal'(fork_as(G, N), FG) :-
    !,
    '$translate_goal'(G, TransG),
    FG = fork_as(TransG, N, thread).
'$translate_goal'(fork_as(G, N, R), FG) :-
    !,
    '$translate_goal'(G, TransG),
    FG = fork_as(TransG, N, R).
'$translate_goal'(fork_light_as(G, N, R), FG) :-
    !,
    '$translate_goal'(G, TransG),
    FG = fork_light_as(TransG, N, R).
'$translate_goal'(fork_light_as(G, N), FG) :-
    !,
    '$translate_goal'(G, TransG),
    FG = fork_light_as(TransG, N, thread).
'$translate_goal'(fork_sizes_as(G, S, N), FG) :-
    !,
    '$translate_goal'(G, TransG),
    '$expand_funs'(S, TransS, SGoals),
    append(SGoals, [fork_sizes_as(TransG, TransS, N, thread)], FG1),
    '$list2tuple'(FG1, FG).
'$translate_goal'(fork_sizes_as(G, S, N, R), FG) :-
    !,
    '$translate_goal'(G, TransG),
    '$expand_funs'(S, TransS, SGoals),
    append(SGoals, [fork_sizes_as(TransG, TransS, N, R)], FG1),
    '$list2tuple'(FG1, FG).
'$translate_goal'(from(Msg, Addr, Test), FG) :-
    !,
    '$expand_funs'(Msg, TransMsg, MsgGoals),
    '$expand_funs'(Addr, TransAddr, AddrGoals),
    '$translate_goal'(Test, TransTest),
    append(MsgGoals, AddrGoals, Goals),
    append(Goals, [from(TransMsg, TransAddr, TransTest)], AllGoals),
    '$list2tuple'(AllGoals, FG).

'$translate_goal'(from_thread(Msg, Addr, Test), FG) :-
    !,
    '$expand_funs'(Msg, TransMsg, MsgGoals),
    '$expand_funs'(Addr, TransAddr, AddrGoals),
    '$translate_goal'(Test, TransTest),
    append(MsgGoals, AddrGoals, Goals),
    append(Goals, [from_thread(TransMsg, TransAddr, TransTest)], AllGoals),
    '$list2tuple'(AllGoals, FG).


'$translate_goal'(open(A,B,C), '$open_'(A,B,C)) :-
    !.
'$translate_goal'(close(A), '$close_'(A)) :-
    !.
'$translate_goal'(get_line(A, S), '$get_line_'(A, S)) :-
    !.
'$translate_goal'(read_term(A, S), '$read_term_'(A, S)) :-
    !.
'$translate_goal'(peek(A, S), '$peek_'(A, S)) :-
    !.
'$translate_goal'(put_line(A, S), G) :-
    !,
    '$expand_funs'(A, TransA, AGoals),
    append(AGoals, ['$put_line_'(TransA, S)], G1),
    '$list2tuple'(G1, G).


'$translate_goal'(once(B), once(G)) :-
    !,
    '$translate_goal'(B, G).
'$translate_goal'(not(B), not(G)) :-
    !,
    '$translate_goal'(B, G).
'$translate_goal'('$exists'(V, A), Result) :-
    %% for exists we replace the body by a copy that uses new variables
    %% for the bound vars but the same vars for the other vars - this
    %% means if we reuse the bound vars in other quantified positions
    %% then, in the translation, they will be different vars and so
    %% won't clash
    collect_vars(A, AVars),
    diff_list(AVars, V, AV),
    copy_term((AV, V, A), (AV, V1, A1)), 
    free_vars_(A1, FreeVars),
    diff_list(FreeVars, V1, Unbound),
    (
      Unbound = []
    ->
      Result = once(AT)
    ;
      Result = AT
    ),
    !,
    '$translate_goal'(A1, AT).

'$translate_goal'('$pfindall'(Res, Patt, Goal), TG) :-
    !,
    '$expand_funs'(Res, TRes, ResG),
    '$expand_funs'(Patt, TPatt, PattG),
    '$translate_goal'(Goal, TGoal),
    append(ResG, [(X = TRes)], UResG),
    (
      PattG = []
    ->
      PG = true
    ;
      '$list2tuple'(PattG, PG)
    ),
    '$list2tuple'(UResG, Unif),
    TG = ('$pfindall'(X, TPatt, (TGoal, PG)), Unif).
'$translate_goal'('$forall'(_V, A, B), forall(AT, BT)) :-
    !,
    '$translate_goal'(A, AT),
    '$translate_goal'(B, BT).
'$translate_goal'('$forall_actions'(_V, A, B), forall(AT, BT)) :-
    !,
    '$translate_goal'(A, AT),
    '$translate_goal'(B, BT).
%% Example:
%% r(X, Y, Z) <= X =? Y::t(Y) ++? Z
%% translates to
%% r(X, Y, Z) :- string_concat(Y, Z, X), t(Y)
'$translate_goal'(( SExp '=?' (L1 ++? L2)), G) :-
    !,
    '$expandS'(SExp,L-Concats1),
    '$expandSP'(L, (L1 ++? L2), Concats2),
    C = (Concats1,Concats2),
    freeze_term(C),
    '$translate_goal'(C, G).
%% Example:
%% p(X, Y, Z) <= X =? [Y]::q(Y) <>? Z
%% translates to
%% p(X, Y, Z) :- append([Y], Z, X), q(Y)
'$translate_goal'(( LExp '=?' (L1 <>? L2)), G) :-
    !,
    '$expandL'(LExp,L-Concats1),
    '$expandLP'(L, (L1 <>? L2), Concats2),
    C = (Concats1,Concats2),
    freeze_term(C),
    '$translate_goal'(C, G).
%% Note: expression evaluation happens before unification
%% EG
%% 2+X = Y
%% translates to
%% +(2, X, A), (A = Y)
'$translate_goal'((A = B), G) :-
    !,
    '$expand_funs'(A, AT, CA),
    '$expand_funs'(B, BT, CB),
    append(CA, [(AT = BT)], G1),
    append(CB, G1, G2),
    '$list2tuple'(G2, G).
'$translate_goal'('=@'(A, B), G) :-
    !,
    '$expand_funs'(A, AT, CA),
    '$expand_funs'(B, BT, CB),
    append(CA, ['=@'(AT, BT)], G1),
    append(CB, G1, G2),
    '$list2tuple'(G2, G).
'$translate_goal'((A \= B), G) :-
    !,
    '$expand_funs'(A, AT, CA),
    '$expand_funs'(B, BT, CB),
    append(CA, [(AT \= BT)], G1),
    append(CB, G1, G2),
    '$list2tuple'(G2, G).
'$translate_goal'(call(A), G) :-
    !,
    '$expand_funs'(A, AT, CA),
    append(CA, ['$call__'(AT)], G1),
    '$list2tuple'(G1, G).


%% for action bodies {} is parsed as '$null_action'
'$translate_goal'('$null_action', G) :-
    !,
    G = true.
%% The folllowing are normal calls
%% First add default args
'$translate_goal'(Call, G) :-
    '$default_args'(Call, FullCall), !,
    '$translate_goal'(FullCall, G).
%% Higher order relations - EG
%% curryR(A)(fact(B))(C)
%% translates to
%% fact(B, D), apply_(curryR(A), D, E), 'curryR$apply'(E, C, true)
'$translate_goal'(B, G) :-
    '$type_info'(_, fun, _, B, _), !,
    B =.. [F|Args],
    '$generate_apply_name'(B, ApplyName),
    %'$list2tuple'(Args, Tuple),
    '$expand_funs'(Args, BArgs, C),
    '$expand_funs'(F, EF, FC),
    append(FC, C, AllC),
    BCall = ApplyName(EF, BArgs, Val),
    (
      '$list2tuple'(AllC, CG)
    ->
      G = (CG, BCall, call(Val))
    ;
      G = (BCall, call(Val))
    ).
%% goals with var functors are unfolded in a similar way to above
'$translate_goal'(B, G) :-
    '$var_functor'(B),!,
    B =.. [F|Args],
    '$expand_funs'(Args, TArgs, C),
    BT =.. [F|TArgs],
    (
      '$list2tuple'(C, CG)
    ->
      G = (CG, BT)
    ;
      G = BT
    ).
%% default - call on predicate - args might need evaluating first
'$translate_goal'(B, G) :-
    var(B), !,
    G = B.
'$translate_goal'(B, G) :-
    '$get_ultimate_functor'(B, UF), var(UF), !,
    '$expand_funs'(B, BT, C),
    (
      '$list2tuple'(C, CG)
    ->
      G = (CG, BT)
    ;
      G = BT
    ).
'$translate_goal'(transform_subterms(Higher, From, To),
                  '$transform_subterms_'(Higher, From, To)) :-
    !.
'$translate_goal'(collect_simple_terms(Higher, From, Base, To),
                  '$collect_simple_terms_'(Higher, From, Base, To)) :-
    !.
'$translate_goal'(flush_output(A), '$flush_output__'(A)) :-
    !.
'$translate_goal'(B, G) :-
    atom(B), !,
    G = B.
'$translate_goal'(B, G) :-
    '@..rev'(B, F, Args),
    '$expand_funs'(Args, TArgs, C),
    '@..'(F, TArgs, BT),
    (
      qulog2qp_map(BT, TransBT)
    ->
      true
    ;
      TransBT = BT
    ),
    %'$add_allowed_remote_query_from_wrapper'(TransBT, WrappedBT),
    (
      '$list2tuple'(C, CG)
    ->
      G = (CG, TransBT)
    ;
      G = TransBT
    ).


% '$add_allowed_remote_query_from_wrapper'(Call, WrappedCall) :-
%     Call = '$remote_query'(_V, _Q, _ServerAddr),
%     '$has_allowed_remote_queries'(Addr), !,
%      WrappedCall = '$check_allowed_remote_query_from'(Call, Addr).
% '$add_allowed_remote_query_from_wrapper'(Call, WrappedCall) :-
%     '$has_allowed_remote_queries'(Addr), !,
%      WrappedCall = '$check_allowed_remote_query_from'(Call, Addr).
% '$add_allowed_remote_query_from_wrapper'(Call, Call).
    
%% translated to a cascading catches
%% EG
%% try { TryAct } except {
%%     Pat1 :: Test1 ~> Act1
%%     Pat2 ~> Act2
%%     }
%% is translated to
%% catch(
%%       catch(TryAct, Pat1,
%%            (Test1 -> Act1 ; throw(Pat1))),
%%       Pat2,
%%       Act2)
%% Note that if Test1 fails and Pat2 is the same as Pat1 then the internal
%% throw (beause of Test1 failing) will be caught by the outer catch
%%
'$translate_try_except'(['$catch'(Patt, '$empty', Act)], Try, G) :-
    !,
    '$translate_goal'(Try, TTry),
    '$translate_goal'(Act, TAct),
    G = catch(TTry, Patt, TAct).
'$translate_try_except'(['$catch'(Patt, Test, Act)], Try, G) :-
    !,
    '$translate_goal'(Try, TTry),
    '$translate_goal'(Test, TTest),
    '$translate_goal'(Act, TAct),
    G = catch(TTry, Patt, (TTest -> TAct ; throw(Patt))).
'$translate_try_except'(['$catch'(Patt, '$empty', Act)|Rest], Try, G) :-
    !,
    '$translate_try_except'(Rest, Try, G1),
    '$translate_goal'(Act, TAct),
    G = catch(G1, Patt, TAct).
'$translate_try_except'(['$catch'(Patt, Test, Act)|Rest], Try, G) :-
    !,
    '$translate_try_except'(Rest, Try, G1),
    '$translate_goal'(Test, TTest),
    '$translate_goal'(Act, TAct),
    G = catch(G1, Patt, (TTest -> TAct ; throw(Patt))).

%% case is just the same as a Prolog if-then-else except that
%% a catchall is added that throws a failure exception
%% if the earlier cases fail
'$translate_case'([], (term2string(action_failure(Case), AString),
                          throw(action_failure(AString))), Case).
'$translate_case'(['$case_alt'(Test, Act)|Alts],
                  ((TransTest -> TransAct); TransAlts), Case) :-
    '$translate_goal'(Test, TransTest),
    '$translate_goal'(Act, TransAct),
    '$translate_case'(Alts, TransAlts, Case).

%% The translation depends on a timeout being present - if none
%% then a non-timeout version of thread_wait_on_goal is used that
%% waits indefinitely. If a timeout is present then a wait_for arg is added
%% and the thread_wait_on_goal is the test of an if-then-else as
%% thread_wait_on_goal fails if the timeout expires before the goal becomes
%% true.
%% In both cases, because thread_wait_on_goal waits on a single goal then
%% we wait on the disjunction of the tests. However, in order to avoid
%% re-evaluating the tests we add a new variable that gets set to the "index"
%% of the first test that succeeds and that index is used to determine
%% which action to run.
%% Note that another reason to do this is that, is an edge case, the
%% belief store may change between when the disjunction succeeded
%% and when the action was chosen. By using the index we fix the choice
%% of action at the time the choice was made.
%% EG
%% wait_case {
%%    Test1 ~> Act1
%%    Test2 ~> Act2
%%    Test3 ~> Act3
%%    }
%% translates to
%% thread_wait_on_goal((Test1, I=1);(Test2, I=2);(Test3, I=3)),
%% (
%%   I = 1 -> Act1 ;
%%   I = 2 -> Act2 ;
%%   Act3
%% )
%%
'$translate_wait_case'(Alts, Trans) :-
    append(As, [timeout(T, Act)], Alts), !,
    '$translate_goal'(Act, TransAct),
    '$create_wait_disj'(As, 1, _X, Disj, IfElse),
    Trans = (thread_wait_on_goal(Disj, [wait_for(T)])
            -> IfElse ; TransAct).
'$translate_wait_case'(Alts, Trans) :-
    '$create_wait_disj'(Alts, 1, _X, Disj, IfElse),
    Trans = (thread_wait_on_goal(Disj), IfElse).

'$create_wait_disj'(['$case_alt'(G, A)], N, X, (TransG,X=N), TransA) :-
    !,
    '$translate_goal'(G, TransG),
    '$translate_goal'(A, TransA).
'$create_wait_disj'(['$case_alt'(G, A)|Alts], N, X,
                    ((TG,X=N);TransG), (X=N -> TA ;TransA)) :-
    !,
    '$translate_goal'(G, TG),
    '$translate_goal'(A, TA),
    N1 is N + 1,
    '$create_wait_disj'(Alts, N1, X, TransG, TransA).

%% receive is implemented by first peeking at the message buffer and
%% using if-then-else to match against the message/address patterns.
%% One-sided unification is used for message matching to prevent variables in
%% the message from being instantiated by matching against the message pattern.
%% If no matches are found the code blocks for more messages until the
%% optional timeout is reached.
%% Note the once at the top-level - this is to remove the choice point
%% created by peek.
'$translate_receive'(Alts, once(TransAlts)) :-
     '$translate_receive_aux'(Alts, Code, Msg, Ref, Addr, Timeout),
    ( var(Timeout)
    ->
        TransAlts = ( '$ipc_peek_and_check_term_or_throw'(Msg, Ref, Addr,
                                                          block, false),
                        Code )
    ;
        TransAlts = ( '$ipc_peek_and_check_term_or_throw'(Msg, Ref, Addr,
                                                          Timeout, false), 
                        Code )
    ).

'$translate_receive_aux'([timeout(Timeout, Act)], Code, Msg, _R, _A, Timeout) :-
    !,
    '$translate_goal'(Act, TransAct1),
    Code = ('$is_timeout'(Msg) -> TransAct1 ; fail).
'$translate_receive_aux'([Alt],  ( Code ; fail ), Msg, Ref, Addr, _Timeout) :-
    !,
    '$translate_receive_alt'(Alt, Code, Msg, Ref, Addr).
'$translate_receive_aux'([A|Alts], (C1;C2), Msg, Ref, Addr, Timeout) :-
    '$translate_receive_alt'(A, C1, Msg, Ref, Addr),
    '$translate_receive_aux'(Alts, C2, Msg, Ref, Addr, Timeout).


'$translate_receive_alt'('$normal'('$receive_from'(M, A, '$empty', Act)),
                         Code, Msg, Ref, Addr) :- !,
    '$translate_goal'(Act, TransAct1),
    Code = ('$receive_from_match_message'(M, A, Msg, Addr) ->
               '$ipc_commit'(Ref), TransAct1).

'$translate_receive_alt'('$normal'('$receive_from'(M, A, Test, Act)),
                         Code, Msg, Ref, Addr) :- !,
    '$translate_goal'(Act, TransAct1),
    '$translate_goal'(Test, TransTest),
    Code = ('$receive_from_match_message'(M, A, Msg, Addr), TransTest ->
               '$ipc_commit'(Ref), TransAct1).



'$receive_from_match_message'(M, from(A), Msg, Addr) :-
    !,
    '$same_agent_handle_'(Addr, A),
    freeze_term(Msg, Frozen),
    '=@'(M, Msg),
    thaw_term(Frozen).
'$receive_from_match_message'(M, from_thread(A), Msg, Addr) :-
    '$qp2qlg_handle'(Addr, QAddr),
    '$same_thread_handle_'(QAddr, A),
    freeze_term(Msg, Frozen),
    '=@'(M, Msg),
    thaw_term(Frozen).

'$qp2qlg_handle'(A, B) :-
    \+ ground(A), !,
    errornl(shouldnt_get_here('$qp2qlg_handle'(A, B))),
    fail.
'$qp2qlg_handle'('@'(':'(TID, PID), Host), ':'(TID, '@'(PID, Host))).



%% T is a term that may contain subterms that need expanding
%% T1 is the resulting term and C is the collection of auxilary
%% goals that evaluate the sub-expressions

%%'$expand_funs'(T, T1, C) :- errornl('$expand_funs'(T, T1, C)),fail.
'$expand_funs'(T, T1, C) :-
    compound(T), functor(T, F, _), '$type_info'(F, fun, _, _, _),
    '$default_args'(T, FullT), !,
    '$expand_funs'(FullT, T1, C).
'$expand_funs'(T, T1, C) :-
    atomic(T), !, T1 = T, C = [].
'$expand_funs'(T, T1, C) :-
    string(T), !, T1 = T, C = [].
'$expand_funs'(T, T1, C) :-
    var(T), !, T1 = T, C = [].
'$expand_funs'(T, X, C) :-
    T = '$list_constr'(V, G), !,
    ip_set('$had_eval', true),
    '$translate_goal'(G,B),
    '$expand_funs'(V, V1, VG),
    (
      VG = []
    ->
      C = [findall(V1, B, X)]
    ;
      '$list2tuple'(VG, VGT),
      C = [findall(V1, (B,VGT), X)]
    ).
'$expand_funs'(T, X, C) :-
    T = '$set_constr'(V, G), !,
    ip_set('$had_eval', true),
    '$translate_goal'(G,B),
    '$expand_funs'(V, V1, VG),
    (
      VG = []
    ->
      C = [findall(V1, B, X1), sort(X1, Y), X = '$set'(Y)]
    ;
      '$list2tuple'(VG, VGT),
      C = [findall(V1, (B,VGT), X1), sort(X1, Y), X = '$set'(Y)]
    ).
'$expand_funs'(T, X, C) :-
    T = '$set_enum'(B), !,
    '$expand_funs'(B, B1, BC),
    append(BC, [sort(B1, XL), X = '$set'(XL)], C).
'$expand_funs'(T, X, C) :-
    T = toset(B), !,
    ip_set('$had_eval', true),
    '$expand_funs'(B, B1, BC),
    append(BC, [toset(B1, X)], C).
'$expand_funs'(T, X, C) :-
    T = tolist(B), !,
    ip_set('$had_eval', true),
    '$expand_funs'(B, B1, BC),
    append(BC, [tolist(B1, X)], C).
'$expand_funs'(T, X, C) :-
    T = (L1 <> L2), !,
    ip_set('$had_eval', true),
    '$expand_funs'(L1, EL1, L1C),
    '$expand_funs'(L2, EL2, L2C),
    append(L1C, L2C, LC),
    append(LC, [append(EL1, EL2, X)], C).
'$expand_funs'(T, X, C) :-
    T = (S1 ++ S2), !,
    ip_set('$had_eval', true),
    '$expand_funs'(S1, ES1, S1C),
    '$expand_funs'(S2, ES2, S2C),
    append(S1C, S2C, SC),
    append(SC, [string_concat(ES1, ES2, X)], C).
'$expand_funs'(T, X, C) :-
    T = '$'(A), !,
    ip_set('$had_eval', true),
    C = [A(X)].
'$expand_funs'(T, X, C) :-
    T = map(A,B), !,
    ip_set('$had_eval', true),
    '$expand_funs'(A, A1, AF),
    '$expand_funs'(B, B1, BF),
    append(AF, BF, ABF),
    append(ABF, ['$map_'(A1,B1,X)], C).
'$expand_funs'(T, X, C) :-
    T = filter(A,B), !,
    ip_set('$had_eval', true),
    '$expand_funs'(A, A1, AF),
    '$expand_funs'(B, B1, BF),
    append(AF, BF, ABF),
    append(ABF, ['$filter_'(A1,B1,X)], C).
'$expand_funs'(T, X, D) :-
    T = filter_map(A,B, C), !,
    ip_set('$had_eval', true),
    '$expand_funs'(A, A1, AF),
    '$expand_funs'(B, B1, BF),
    '$expand_funs'(C, C1, CF),
    append(BF, CF, BCF),
    append(AF, BCF, ABCF),
    append(ABCF, ['$filter_map_'(A1,B1,C1,X)], D).

'$expand_funs'(T, X, C) :-
    ('$higher_order_term'(T) ;  '$var_functor'(T) ),
    '@..rev'(T, F, P),
    ip_lookup('$tel_vars', PV),
    \+member_eq(F, PV), !,
    ip_set('$had_eval', true),
    '$expand_funs'(F, FP, CF),
    '$expand_funs'(P, TP, CP),
    C1 = apply_(FP, TP, X),
    append(CF, CP, CFP),
    append(CFP, [C1], C).
'$expand_funs'(T, X, C) :-
     '@..rev'(T, F, P),
    '$type_info'(F, fun, ('$tuple_type'(DT) -> Types), '$function', _),
    T '@=..' [F|PN],
    length(DT, N),
    length(PN, N),
    '$higher_order_type'(Types),
    !,
    ip_set('$had_eval', true),
    '$expand_funs'(F, FP, CF),
    '$expand_funs'(P, TP, CP),
    C1 = apply_(FP, TP, X),
    append(CF, CP, CFP),
    append(CFP, [C1], C).
'$expand_funs'(T, X, C) :-
   '@..rev'(T, F, P),
    '$type_info'(F, fun, ('$tuple_type'(DT) -> _Types), '$function', _),
    T '@=..' [F|PN],
    length(DT, N),
    length(PN, N),
    !,
    ip_set('$had_eval', true),
    '$expand_funs'(F, FP, CF),
    '$expand_funs'(P, TP, CP),
    append(TP, [X], P1),
    C1 =.. [FP|P1],
    append(CF, CP, CFP),
    append(CFP, [C1], C).
'$expand_funs'(T, X, C) :-
    list(T), !,
    T = [Head|Tail],
    '$expand_funs'(Head, HeadX, HeadC),
    '$expand_funs'(Tail, TailX, TailC),
    X = [HeadX|TailX],
    append(HeadC, TailC, C).
'$expand_funs'(T, X, C) :-
    compound(T), !,
    T =.. [F|P],
    '$expand_funs'(F, FP, CF),
    '$expand_funs'(P, TP, CP),
    X =.. [FP|TP],
    append(CF, CP, C).
 '$expand_funs'(T, _X, _C) :-
     errornl(failed_expand_funs(T)).



%% T is a compound term whose functor is a variable
'$var_functor'(T) :-
    compound(T),
    functor(T, F, _),
    var(F).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Run checks on TR programs
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


'$do_run_tr_checks' :-
    '$running_teleo', !,
    '$run_tr_checks'.
'$do_run_tr_checks'.  



%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Check that the types used in declarations and type definitions are OK
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
'$check_valid_types' :-
    once('$current_consulted_file'(File)),
    '$defined_type'(TName, T),
    '$saved_input_with_vars'((TName ::=  T), VarNames, File),
    ip_set(valid_type_context, valid_type_check((TName ::=  T), VarNames)),
    '$is_a_valid_type'(T),
    fail.
'$check_valid_types' :-
    '$user_type_info'(F, _, Types, _, _),
    '$saved_input_with_vars'(Decl, VarNames, _),
    Decl = '$code_decl'(_, Decls, _),
    once(member('$decl'(F, _), Decls)),
    ip_set(valid_type_context, valid_type_check(Decl, VarNames)),
    %member(T, Types),
    check_constructor_mode_and_type__(Types, '!'),
    %%% XXX '$is_a_valid_type'(T),
    fail.
'$check_valid_types' :-
    '$belief_type_info'(_, Types, BelClause),
    '$saved_input_with_vars'( belief(BelClause), VarNames, _),
    member(T, Types),
    ip_set(valid_type_context, valid_type_check(belief(BelClause), VarNames)),
    '$is_a_valid_type'(T),
    fail.
'$check_valid_types' :-
    '$percept_type_info'(_, Types, BelClause),
    '$saved_input_with_vars'( percept(BelClause), VarNames, _),
    member(T, Types),
    ip_set(valid_type_context, valid_type_check(percept(BelClause), VarNames)),
    '$is_a_valid_type'(T),
    fail.

'$check_valid_types' :-
    '$user_type_info'(_, defined_type, TName, '$enum_type'(T), _),
    sort(T, TS),
    member(X, TS),
    once((delete(X, T, TX), member(X, TX))),
    '$display_error'(repeated_enum_type(TName, X)).
'$check_valid_types' :-
    '$user_type_info'(_, defined_type, TName, '$constr_enum_type'(T), _),
    findall(F, (member(E, T), functor(E, F, _)), Functors),
    sort(Functors, FunctorsS),
    member(X, FunctorsS),
    once((delete(X, Functors, FunctorsX), member(X, FunctorsX))),
    '$display_error'(repeated_constr_enum_type(TName, X)).
    
%% check for overlaps in defined enum types
'$check_valid_types' :-
    '$user_type_info'(_, defined_type, TName, '$enum_type'(T), _VarNames),
    '$check_for_type_enum_overlaps'(TName, T),
    fail.
'$check_valid_types' :-
    '$user_type_info'(_, defined_type, TName, '$constr_enum_type'(T), VarNames),
    '$check_for_type_constr_enum_overlaps'(TName, T, VarNames),
    fail.
%% See if atom in enum type is used as a type or as a constructor in enum type
'$check_valid_types' :-
    '$user_type_info'(_, defined_type, TName, '$enum_type'(Enum), VarNames),
    member(A, Enum),
    (
     '$user_type_info'(A, Kind, _, _, _)
    ->
     '$display_error'(enum_is_type(A, TName, Kind))
    ;
     true
    ),
    '$defined_type'(Constr, '$constr_enum_type'(ConstrList)),
    (
     member(C, ConstrList), functor(C, A, _)
    ->
      '$display_error'(enum_is_constr_enum(A, TName, Constr,
                                           (TName ::= '$enum_type'(Enum)),
                                           VarNames))
    ;
     true
    ),
    fail.
%% See if functor of enum constructor is a type
'$check_valid_types' :-
    '$user_type_info'(_, defined_type, TName,
                      '$constr_enum_type'(Enum), VarNames),
    member(A, Enum),
    '$real_functor'(A, F, _),
    (
     '$user_type_info'(F, Kind, _, _, _)
    ->
      '$display_error'(constr_enum_is_type(A, TName, Kind,
                                           (TName ::=
                                           '$constr_enum_type'(Enum)),
                                           VarNames))
    ;
     true
    ),
    '$defined_type'(Constr, '$constr_enum_type'(ConstrList)),
    TName \= Constr,
    member(C, ConstrList),
    '$real_functor'(C, F, _),
    %% another enum type consutructor has the same functor (F)
    (
     '$not_identical_mod_vars'(A, C),
     \+type_error_remember_(overlapping_constructor__(F))
    ->
      %% the constructors have different type args and we haven't seen
      %% this error before
      assert(type_error_remember_(overlapping_constructor__(F))),
      '$user_type_info'(_, defined_type, Constr, '$constr_enum_type'(Enum2),
                        VarNames2),
      '$display_error'(constr_enum_is_constr_enum(A, TName, Constr,
                                                  (TName ::=
                                                  '$constr_enum_type'(Enum)),
                                                  VarNames,
                                                  (Constr ::=
                                                  '$constr_enum_type'(Enum2)),
                                                  VarNames2))
    ;
      '$real_functor'(TName, _, N1),
      '$real_functor'(Constr, _, N2),
      N1 \= N2
    ->
      !,
      '$user_type_info'(_, defined_type, Constr, '$constr_enum_type'(Enum2),
                        VarNames2),
      '$display_error'(constr_different_arities(A, TName, Constr,
                                                  (TName ::=
                                                  '$constr_enum_type'(Enum)),
                                                  VarNames,
                                                  (Constr ::=
                                                  '$constr_enum_type'(Enum2)),
                                                VarNames2))
    ;

     true
    ),
    fail.
%%check for int range overlap
'$check_valid_types' :-
    '$defined_type'(TName, (A..B)),
    '$check_for_type_range_overlaps'(TName, A, B),
    fail.

'$check_valid_types'.


'$check_for_type_enum_overlaps'(TName, Enum) :-
    Enum = [N|_],
    number(N),
    '$defined_type'(RName, (A..B)),
    once((
	  member(M, Enum),
          integer(M),
	  between(A, B, M)
	 )),
    %% M is an integer in Enum that is in A..B so they overlap
    once((
	  member(M1, Enum), \+ (between(A, B, M1)),
          %% M1 is in Enum but not in A..B
	  between(A, B, M2), \+ member(M2, Enum)
         %% M2 is in (A..B) but not in Enum
	 )),
    %% Enum and A..B overlap but neither is a subset of the other
    '$process_overlap_error'(RName, TName).
'$check_for_type_enum_overlaps'(TName, Enum) :-
    '$defined_type'(RName, '$enum_type'(OtherEnum)),
    TName \= RName,
    once(( member(M, Enum), member(M, OtherEnum) )),
    once((
	  member(M1, Enum), \+ member(M1, OtherEnum),
	  member(M2, OtherEnum), \+ member(M2, Enum)
	 )),
    '$process_overlap_error'(RName, TName).

'$check_for_type_constr_enum_overlaps'(TName, Enum, _) :-
    \+compound(TName), !,
    '$defined_type'(RName, '$constr_enum_type'(OtherEnum)),
    TName \= RName,
    once(( member(M, Enum), member(M, OtherEnum) )),
    %% Enum and OtherEnum overlap
    once((
	  member(M1, Enum), \+ member(M1, OtherEnum),
	  member(M2, OtherEnum), \+ member(M2, Enum)
	 )),
    %% Neither is a subset of the other
    '$process_overlap_error'(RName, TName).

%% For contructor enums we don't allow any overlaps (even subsets)
'$check_for_type_constr_enum_overlaps'(TName, Enum, VarNames) :-
    compound(TName),
    '$user_type_info'(_, defined_type, RName,
                      '$constr_enum_type'(OtherEnum), RVarNames),
    TName \= RName,
    once(( member(M, Enum), member(M, OtherEnum) )),
    '$vars2names'(TName, VarNames),
    '$vars2names'(RName, RVarNames),
    '$process_poly_overlap_error'(RName, TName).
   

'$process_overlap_error'(RName, TName) :-
    type_error_remember_(overlapping_types(RName, TName)), !, fail.
'$process_overlap_error'(RName, TName) :-
    assert(type_error_remember_(overlapping_types(TName, RName))),
    '$display_error'(overlapping_types(TName, RName)).

'$process_poly_overlap_error'(RName, TName) :-
    type_error_remember_(overlapping_poly_types(RName, TName)), !, fail.
'$process_poly_overlap_error'(RName, TName) :-
    assert(type_error_remember_(overlapping_poly_types(TName, RName))),
    '$display_error'(overlapping_poly_types(TName, RName)).

%% don't allow  A ... A1 .... B .... B1 for types (A..B) and (A1..B1)
%% ie overlaps but neither a subset of the other
'$check_for_type_range_overlaps'(TName, A, B) :-
    '$defined_type'(RName, (A1..B1)),
    A1 > A, A1 =< B,
    B1 > B,
    '$process_overlap_error'(RName, TName).



'$check_macros' :-
    '$user_type_info'(_A, macro_type, B, T, VarNames),
    \+'$is_a_type'(T),
    '$display_error'(invalid_macro_defn(B, T, VarNames)),
    fail.
'$check_macros' :-
    '$user_type_info'(_A, macro_type, B, '$union_type'(T), VarNames),
    compound(B),
    '$display_error'(invalid_macro_union_type_defn(B, '$union_type'(T),
                                                   VarNames)),
    fail.
 '$check_macros'.   


%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Check definitions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% This tests the validity/types/modes of definitions of
%% relations, functions and TR programs.

'$check_definitions' :-
    once('$current_consulted_file'(File)),
    '$saved_input_with_vars'(Clause, Vars, File),
    '$check_definitions_aux'(Clause, Vars),
    fail.
'$check_definitions'.

%%'$check_definitions_aux'(A, B) :- errornl('$check_definitions_aux'(A, B)),fail.
%% declarations are ignored
'$check_definitions_aux'('$code_decl'(_,_,_), _) :- !.
'$check_definitions_aux'((_A ::= _AT), _Vars) :- !.
'$check_definitions_aux'((_A == _AT), _Vars) :- !.
'$check_definitions_aux'(def_doc((_A ::= _AT), _Doc), _Vars) :- !.
'$check_definitions_aux'(def_doc((_A == _AT), _Doc), _Vars) :- !.
'$check_definitions_aux'(belief(_), _) :- !.
'$check_definitions_aux'(percept(_), _) :- !.
'$check_definitions_aux'(tel_start(_), _) :- !.
'$check_definitions_aux'(tel_atomic(_), _) :- !.
'$check_definitions_aux'(global_int_decl(_,_), _) :- !.
'$check_definitions_aux'(global_num_decl(_,_), _) :- !.
%'$check_definitions_aux'((_:_), _Vars) :- !.
'$check_definitions_aux'((?- _), _Vars) :- !.
'$check_definitions_aux'(Clause, Vars) :-
    Clause = '$tel'(_A, _B), !,
    once('$check_tel'(Clause, Vars)).
'$check_definitions_aux'(Clause, Vars) :-
    Clause = (_Head :: _Test -> _Body), !,
    once('$check_function'(Clause, Vars)).
'$check_definitions_aux'(Clause, Vars) :-
    Clause = (_Head -> _Body), !,
    once('$check_function'(Clause, Vars)).
'$check_definitions_aux'(Clause, Vars) :-
    Clause = (_ :: _ ~> _), !,
    once('$check_action'(Clause, Vars)).
'$check_definitions_aux'(Clause, Vars) :-
    Clause = ( _ ~> _), !,
    once('$check_action'(Clause, Vars)).
%% Everything else must be a relation rule
'$check_definitions_aux'(Clause, Vars) :-
    once('$check_relation'(Clause, Vars)).


'$check_relation'(Clause, VarNames) :-
    Clause = (Head :: Test <= Body),
    !,
    '$check_relation_aux'(Head, Test, Body, Clause, VarNames).
'$check_relation'(Clause, VarNames) :-
    Clause = (Head <= Body),
    !,
    '$check_relation_aux'(Head, true, Body, Clause, VarNames).
'$check_relation'(Clause, VarNames) :-
    Clause = (Head :: Test),
    !,
    '$check_relation_aux'(Head, Test, true, Clause, VarNames).
'$check_relation'(Clause, VarNames) :-
    Clause = Head,
    !,
    '$check_relation_aux'(Head, true, true, Clause, VarNames).

'$check_ultimate_functor'(Head, Clause, VarNames) :-
    '$get_ultimate_functor'(Head, F),
    \+atom(F), !,
    '$display_error'(non_atom_functor(F, Head, Clause, VarNames)).
'$check_ultimate_functor'(_Head, _Clause, _VarNames).

'$check_relation_aux'(Head, _Test, _Body, Clause, VarNames) :-
    '$is_dyn_with_rule'(Head, Clause, VarNames), !,fail.
'$check_relation_aux'(Head, Test, Body, Clause, VarNames) :-
    %% prevent vars in rule from being instantiated while checking
    freeze_term(Clause, ClauseVars),
    %% The ultimate functor of the head must be an atom
    '$check_ultimate_functor'(Head, Clause, VarNames),
    %% No function evaluations are allowed in the head
    '$check_head_for_evaluation'(Head, Clause, VarNames),
    %% check that all calls in Test/Body are rel calls
    '$check_goal_expressions'(Test, Clause, VarNames, rel),
    '$check_goal_expressions'(Body, Clause, VarNames, rel),
    thaw_term(ClauseVars),
    '$get_underscores'(Clause, VarNames, Unders),
    type_check_rule__(Clause, VarNames, Unders).

'$is_dyn_with_rule'(Head, Clause, VarNames) :-
    '$is_rule'(Clause),
    functor(Head, F, _),
    '$user_type_info'(F, _, _, Pattern, _),
    '$belief'(Pattern),
    '$display_error'(dyn_has_rule(F, Clause, VarNames)).

'$is_rule'((_ :: _)).
'$is_rule'((_ <= _)).
'$is_rule'((_ -> _)).

%%'$check_action'(Clause, VarNames) :- errornl('$check_action'(Clause, VarNames)),fail.
'$check_action'(Clause, VarNames) :-
    Clause = (Head :: Test ~> Body),
    !,
    '$check_action_aux'(Head, Test, Body, Clause, VarNames).
'$check_action'(Clause, VarNames) :-
    Clause = (Head ~> Body),
    !,
    '$check_action_aux'(Head, true, Body, Clause, VarNames).
'$check_action'(Clause, VarNames) :-
    Clause = (Head :: Test),
    !,
    '$check_action_aux'(Head, Test, true, Clause, VarNames).
'$check_action'(Clause, VarNames) :-
    Clause = Head,
    !,
    '$check_action_aux'(Head, true, true, Clause, VarNames).

%% similar to check_relation except the body calls (not Test) are all
%% act calls
'$check_action_aux'(Head, _Test, _Body, Clause, VarNames) :-
    '$is_dyn_with_rule'(Head, Clause, VarNames), !,fail.
'$check_action_aux'(Head, Test, Body, Clause, VarNames) :-
    freeze_term(Clause, XFVars),
    '$check_head_for_evaluation'(Head, Clause, VarNames),
    '$check_goal_expressions'(Test, Clause, VarNames, rel),
    '$flatten_actions'(Body, FlatBody, Clause, VarNames),
    '$check_goal_expressions'(FlatBody, Clause, VarNames, act),
    thaw_term(XFVars),
    '$get_underscores'(Clause, VarNames, Unders),
    type_check_rule__(Clause, VarNames, Unders).




    

%%'$check_function'(Clause, VarNames) :- errornl('$check_function'(Clause, VarNames)),fail.
'$check_function'(Clause, VarNames) :-
    Clause = (Head :: Test -> Body), !,
    freeze_term(Clause, XFVars),
    %Head =.. [_|HeadArgs],
    %'$check_arg_expressions'(HeadArgs, Clause, VarNames),
    '$check_head_for_evaluation'(Head, Clause, VarNames),
    '$check_goal_expressions'(Test, Clause, VarNames, rel),
    %% Body is the result of the function - check it is a valid expression
    '$check_expressions'(Body, Clause, VarNames),
    thaw_term(XFVars),
    '$get_underscores'(Clause, VarNames, Unders),
    type_check_rule__(Clause, VarNames, Unders).


'$check_function'(Clause, VarNames) :-
    Clause = (Head  -> Body), !,
    freeze_term(Clause, XFVars),
    \+ '$is_dyn_with_rule'(Head, Clause, VarNames),
    %Head =.. [_|HeadArgs],
    %'$check_arg_expressions'(HeadArgs, Clause, VarNames),
    '$check_head_for_evaluation'(Head, Clause, VarNames),
    '$check_expressions'(Body, Clause, VarNames),
    thaw_term(XFVars),
    '$get_underscores'(Clause, VarNames, Unders),
    type_check_rule__(Clause, VarNames, Unders).

    
'$check_head_for_evaluation'(Head, Clause, VarNames) :-
    Head =.. [_P|Args],
    '$check_arg_expressions'(Args, Clause, VarNames),
    '$check_head_for_evaluation_aux'(Args, Clause, VarNames).

%%'$check_head_for_evaluation_aux'(Args, Clause, VarNames) :- errornl('$check_head_for_evaluation_aux'(Args, Clause, VarNames)),fail.
'$check_head_for_evaluation_aux'(Var, _, _) :- var(Var), !.
'$check_head_for_evaluation_aux'([], _, _).
'$check_head_for_evaluation_aux'([A|Args], Clause, VarNames) :-
    var(A), !,
    '$check_head_for_evaluation_aux'(Args, Clause, VarNames).
'$check_head_for_evaluation_aux'([A|Args], Clause, VarNames) :-
    list(A), !,
    '$check_head_for_evaluation_aux'(A, Clause, VarNames),
    '$check_head_for_evaluation_aux'(Args, Clause, VarNames).
'$check_head_for_evaluation_aux'([A|Args], Clause, VarNames) :-
    atomic(A), !,
    '$check_head_for_evaluation_aux'(Args, Clause, VarNames).
'$check_head_for_evaluation_aux'([A|Args], Clause, VarNames) :-
    string(A), !,
    '$check_head_for_evaluation_aux'(Args, Clause, VarNames).
'$check_head_for_evaluation_aux'([A|Args], Clause, VarNames) :-
     '@functor'(A, P, L),
    '$type_info'(P, fun, (Types -> _), _, _), !,
    (
     '$tuple_type_to_list'(Types, TL),
     length(TL, L),
     '$display_error'(eval_in_head(A, Clause, VarNames))
    ;
      true
    ),
    '$check_head_for_evaluation_aux'(Args, Clause, VarNames).
'$check_head_for_evaluation_aux'([A|Args], Clause, VarNames) :-
    '@..rev'(A, F, AArgs),
    '$check_head_for_evaluation_aux'([F|AArgs], Clause, VarNames),
    '$check_head_for_evaluation_aux'(Args, Clause, VarNames).

%%'$check_goal_expressions'(G, Clause, VarNames, RA) :-  errornl('$check_goal_expressions'(G, Clause, VarNames, RA)),fail.
%% '$check_goal_expressions'(Call, Clause, VarNames, Kind)
%% checks to see if Call is a Kind call (i.e. act or rel)
%%XXX
'$check_goal_expressions'(true, Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(true, Clause, VarNames)).
'$check_goal_expressions'(true, _, _, _) :-
    %% special case
    !.
'$check_goal_expressions'(G, _Clause, _VarNames, _) :-
    var(G),!.
%% The type checker will deal with this case as the type of
%% the ultimate functor is required in order to determine if OK
'$check_goal_expressions'(G, _Clause, _VarNames, _) :-
    '$get_ultimate_functor'(G, F),
    var(F), !.
%% show is only allowed at the top-level of the interpreter
'$check_goal_expressions'(show, Clause, VarNames, _) :-
    !,
    '$display_error'(bad_show(show, Clause, VarNames)).
'$check_goal_expressions'(show(A), Clause, VarNames, _) :-
    !,
    '$display_error'(bad_show(show(A), Clause, VarNames)).
%% add default args
'$check_goal_expressions'(G, Clause, VarNames, RA) :-
    '$default_args'(G, FullG), !,
    '$check_goal_expressions'(FullG, Clause, VarNames, RA).
'$check_goal_expressions'(G, Clause, VarNames, _) :-
    '$tel_action'(G), !,
    '$display_error'(bad_tel_action_goal(G, Clause, VarNames)).
%% Neither is tel
'$check_goal_expressions'(G, Clause, VarNames, _) :-
    '$type_info'(_, tel, _, G, _), !,
    '$display_error'(bad_tel_goal(G, Clause, VarNames)).
%% Parser should catch this problem
'$check_goal_expressions'((G1 & G2), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act((G1 & G2), Clause, VarNames)).
'$check_goal_expressions'((G1 & G2), Clause, VarNames, _RA) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, rel),
    '$check_goal_expressions'(G2, Clause, VarNames, rel).
%% Parser should catch this problem
'$check_goal_expressions'((G1 ; G2), Clause, VarNames, rel) :-
    !,
    '$display_error'(act_in_rel((G1 ; G2), Clause, VarNames)).
'$check_goal_expressions'((G1 ; G2), Clause, VarNames, _RA) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, act),
    '$check_goal_expressions'(G2, Clause, VarNames, act).
'$check_goal_expressions'(not(G1), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(not(G1), Clause, VarNames)).
'$check_goal_expressions'(not(G1), Clause, VarNames, _) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, rel).
'$check_goal_expressions'(once(G1), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(once(G1), Clause, VarNames)).
'$check_goal_expressions'(once(G1), Clause, VarNames, RA) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, RA).
'$check_goal_expressions'(fork(Act, N, S), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(fork(Act, N, S), Clause, VarNames)).
'$check_goal_expressions'(fork(Act, _N, _S), Clause, VarNames, _RA) :-
    !,
    '$check_goal_expressions'(Act, Clause, VarNames, act).
'$check_goal_expressions'(fork_light(Act, N), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(fork_light(Act, N), Clause, VarNames)).
'$check_goal_expressions'(fork_light(Act, _N), Clause, VarNames, _RA) :-
    !,
    '$check_goal_expressions'(Act, Clause, VarNames, act).
'$check_goal_expressions'('$null_action', _, _, _) :- !.
'$check_goal_expressions'('$exists'(_, G1), Clause, VarNames, RA) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, RA).
'$check_goal_expressions'('$at_forall'(V, G1, G2), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act('$at_forall'(V, G1, G2), Clause, VarNames)).
'$check_goal_expressions'('$at_forall'(_, G1, G2), Clause, VarNames, _RA) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, rel),
    '$check_goal_expressions'(G2, Clause, VarNames, rel).
'$check_goal_expressions'('$at_forall_actions'(V, G1, G2),
                          Clause, VarNames,rel) :-
    !,
    '$display_error'(action_in_rel('$at_forall_actions'(V, G1, G2),
				   Clause, VarNames)).
'$check_goal_expressions'('$at_forall_actions'(_V, G1, G2),
                          Clause, VarNames, RA) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, RA),
    '$check_goal_expressions'(G2, Clause, VarNames, RA).
'$check_goal_expressions'('$pfindall'(V, G1, G2), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act('$pfindall'(V, G1, G2), Clause, VarNames)).

'$check_goal_expressions'('$pfindall'(_, _, G2), Clause, VarNames, _RA) :-
    !,
    '$check_goal_expressions'(G2, Clause, VarNames, rel).
'$check_goal_expressions'('$forall'(V, G1, G2), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act('$forall'(V, G1, G2), Clause, VarNames)).
'$check_goal_expressions'('$forall'(_, G1, G2), Clause, VarNames, _RA) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, rel),
    '$check_goal_expressions'(G2, Clause, VarNames, rel).
'$check_goal_expressions'('$forall_actions'(V, G1, G2),
			  Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel('$forall_actions'(V, G1, G2),
				   Clause, VarNames)).
'$check_goal_expressions'('$forall_actions'(_, G1, G2), Clause, VarNames, _) :-
    !,
    '$check_goal_expressions'(G1, Clause, VarNames, rel),
    '$check_goal_expressions'(G2, Clause, VarNames, act).
'$check_goal_expressions'((G2 '=?' G1), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act((G2 '=?' G1), Clause, VarNames)).
'$check_goal_expressions'((G2 '=?' G1), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G1, Clause, VarNames),
    '$check_expressions'(G2, Clause, VarNames).
'$check_goal_expressions'('=@'(G2, G1), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act('=@'(G2,G1), Clause, VarNames)).
'$check_goal_expressions'('=@'(G2, G1), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G1, Clause, VarNames),
    '$check_expressions'(G2, Clause, VarNames).
'$check_goal_expressions'((G1 = G2), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act((G2 '=' G1), Clause, VarNames)).
'$check_goal_expressions'((G1 = G2), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G1, Clause, VarNames),
    '$check_expressions'(G2, Clause, VarNames).
'$check_goal_expressions'(wait(Goal), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(wait(Goal), Clause, VarNames)).
'$check_goal_expressions'(wait(Goal), Clause, VarNames, act) :-
    !,
    '$check_goal_expressions'(Goal, Clause, VarNames, rel).
'$check_goal_expressions'('$remote_query'(G, H), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act('$remote_query'(G, H), Clause, VarNames)).

'$check_goal_expressions'('$remote_query'('$num_vars_query'(N, _Vars, G), H),
                          Clause, VarNames, _) :-
    !,
    %% Information stored away in case an error is produced when testing
    %% is_a_valid_type in check_expressions
    ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
    '$check_expressions'(N, Clause, VarNames),
    '$check_expressions'(H, Clause, VarNames),
    '$check_goal_expressions'(G, Clause, VarNames, rel).
'$check_goal_expressions'('$remote_query'('$num_query'(N, G), H),
                          Clause, VarNames, _) :-
    !,
    %% Information stored away in case an error is produced when testing
    %% is_a_valid_type in check_expressions
    ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
    '$check_expressions'(N, Clause, VarNames),
    '$check_expressions'(H, Clause, VarNames),
    '$check_goal_expressions'(G, Clause, VarNames, rel).
'$check_goal_expressions'('$remote_query'('$vars_query'(_Vars, G), H),
                          Clause, VarNames, _) :-
    !,
    %% Information stored away in case an error is produced when testing
    %% is_a_valid_type in check_expressions
    ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
    '$check_expressions'(H, Clause, VarNames),
    '$check_goal_expressions'(G, Clause, VarNames, rel).
'$check_goal_expressions'('$remote_query'(G, H),
                          Clause, VarNames, _) :-
    !,
    ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
    '$check_expressions'(H, Clause, VarNames),
    '$check_goal_expressions'(G, Clause, VarNames, rel).
'$check_goal_expressions'('$remote_query'(G, H), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act('$remote_query'(G, H), Clause, VarNames)).
'$check_goal_expressions'(raise(Patt), Clause, VarNames, rel) :-
    !,
    '$display_error'(act_in_rel(raise(Patt), Clause, VarNames)).
'$check_goal_expressions'(raise(Patt), Clause, VarNames, _) :-
    !,
    '$check_expressions'(Patt, Clause, VarNames).
'$check_goal_expressions'(repeat_until(Act, '$fail'), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(repeat_until(Act, '$fail'), Clause,
                                   VarNames)).
'$check_goal_expressions'(repeat_until(Act, Cond), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(repeat_until(Act, Cond), Clause,
                                   VarNames)).
'$check_goal_expressions'(repeat_until(Act, '$fail'), Clause, VarNames, _) :-
    !,
    '$check_goal_expressions'(Act, Clause, VarNames, act).
'$check_goal_expressions'(repeat_until(Act, Cond), Clause, VarNames, _) :-
    !,
    '$check_goal_expressions'(Act, Clause, VarNames, act),
    '$check_goal_expressions'(Cond, Clause, VarNames, rel).
'$check_goal_expressions'(try_except(Try, Except), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(try_except(Try, Except), Clause, VarNames)).
'$check_goal_expressions'(try_except(Try, Except), Clause, VarNames, RA) :-
    !,
    '$check_goal_expressions'(Try, Clause, VarNames, RA),
    forall(member(E, Except),
	   '$check_try_except_expressions'(E, Clause, VarNames, RA)).
'$check_goal_expressions'(receive(Alts), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(receive(Alts), Clause, VarNames)).
'$check_goal_expressions'(receive(Alts), Clause, VarNames, RA) :-
    !,
    forall(member(Alt, Alts),
	   '$check_receive_expressions'(Alt, Clause, VarNames, RA)).
'$check_goal_expressions'(wait_case(AllAlts), Clause, VarNames, RA) :-
    append(Alts, [timeout(_, Act)], AllAlts),
    !,
    '$check_goal_expressions'(Act, Clause, VarNames, RA),
    forall(member(Alt, Alts),
	   '$check_case_expressions'(Alt, Clause, VarNames, RA)).
'$check_goal_expressions'(wait_case(Alts), Clause, VarNames, RA) :-
    !,
    forall(member(Alt, Alts),
	   '$check_case_expressions'(Alt, Clause, VarNames, RA)).
'$check_goal_expressions'(case(Alts), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(case(Alts), Clause, VarNames)).
'$check_goal_expressions'(case(Alts), Clause, VarNames, RA) :-
    !,
    forall(member(Alt, Alts),
	   '$check_case_expressions'(Alt, Clause, VarNames, RA)).
'$check_goal_expressions'(atomic_action(Acts), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(atomic_action(Acts), Clause, VarNames)).
'$check_goal_expressions'(atomic_action(Acts), Clause, VarNames, _) :-
    !,
    '$check_goal_expressions'(Acts, Clause, VarNames, act).
'$check_goal_expressions'('$rel_escape'(G), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel('$rel_escape'(G), Clause, VarNames)).
'$check_goal_expressions'('$rel_escape'(G), Clause, VarNames, _) :-
    !,
    '$check_goal_expressions'(G, Clause, VarNames, rel).
'$check_goal_expressions'('$act_escape'(G), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act('$act_escape'(G), Clause, VarNames)).
'$check_goal_expressions'('$act_escape'(G), Clause, VarNames, _) :-
    !,
    '$check_goal_expressions'(G, Clause, VarNames, act).
'$check_goal_expressions'(from(M,A,A1), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(from(M,A,A1), Clause, VarNames)).
'$check_goal_expressions'(from(Msg, _Addr, Test),
			  Clause, VarNames, _RA) :- !,
    '$check_expressions'(Msg, Clause, VarNames),
    '$check_goal_expressions'(Test, Clause, VarNames, rel).
'$check_goal_expressions'(from_thread(M,A,A1), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(from_thread(M,A,A1), Clause, VarNames)).
'$check_goal_expressions'(from_thread(Msg, _Addr, Test),
			  Clause, VarNames, _RA) :- !,
    '$check_expressions'(Msg, Clause, VarNames),
    '$check_goal_expressions'(Test, Clause, VarNames, rel).
  
'$check_goal_expressions'(remember(G), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(remember(G), Clause, VarNames)).
'$check_goal_expressions'(remember(Goals), Clause, VarNames, _RA) :-
    !,
    forall(member(G, Goals), '$check_belief_goal'(G, Clause, VarNames)).
'$check_goal_expressions'(remember_for(G,T), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(remember_for(G,T), Clause, VarNames)).
'$check_goal_expressions'(remember_for(Goals,T), Clause, VarNames, _RA) :-
    !,
    '$check_expressions'(T, Clause, VarNames),
    forall(member(G, Goals), '$check_belief_goal'(G, Clause, VarNames)).

'$check_goal_expressions'(forget(G), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(forget(G), Clause, VarNames)).
'$check_goal_expressions'(forget(Goals), Clause, VarNames, _RA) :-
    !,
    forall(member(G, Goals), '$check_belief_goal'(G, Clause, VarNames)).

'$check_goal_expressions'(forget_after(G,T), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(forget_after(G,T), Clause, VarNames)).
'$check_goal_expressions'(forget_after(Goals,T), Clause, VarNames, _RA) :-
    !,
    '$check_expressions'(T, Clause, VarNames),
    forall(member(G, Goals), '$check_belief_goal'(G, Clause, VarNames)).
'$check_goal_expressions'(forget_remember(GF,GR), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(forget_remember(GF,GR), Clause, VarNames)).
'$check_goal_expressions'(forget_remember(GF,GR), Clause, VarNames, _RA) :-
    !,
    forall(member(G, GF), '$check_belief_goal'(G, Clause, VarNames)),
    forall(member(G, GR), '$check_belief_goal'(G, Clause, VarNames)).


'$check_goal_expressions'(':='(A, B), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel(':='(A, B), Clause, VarNames)).
'$check_goal_expressions'(':='(A, B), Clause, VarNames, _RA) :-
    !,
    '$check_expressions'(B, Clause, VarNames),
    (
     '$is_state_belief'(A)
    ->
     true
    ;
     '$display_error'(not_a_global(A, ':='(A, B), Clause, VarNames))
    ).
'$check_goal_expressions'('+:='(A, B), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel('+:='(A, B), Clause, VarNames)).
'$check_goal_expressions'('+:='(A, B), Clause, VarNames, _RA) :-
    !,
    '$check_expressions'(B, Clause, VarNames),
    (
     '$is_state_belief'(A)
    ->
     true
    ;
     '$display_error'(not_a_global(A, '+:='(A,B), Clause, VarNames))
    ).
'$check_goal_expressions'('-:='(A, B), Clause, VarNames, rel) :-
    !,
    '$display_error'(action_in_rel('-:='(A, B), Clause, VarNames)).

'$check_goal_expressions'('-:='(A, B), Clause, VarNames, _) :-
    !,
    '$check_expressions'(B, Clause, VarNames),
    (
     '$is_state_belief'(A)
    ->
     true
    ;
     '$display_error'(not_a_global(A, '-:='(A,B), Clause, VarNames))
    ).
'$check_goal_expressions'(subscribe_as(Sub, ID), Clause, VarNames, rel) :-
    !,
    '$display_error'(subscribe_as(Sub, ID), Clause, VarNames).
 '$check_goal_expressions'(subscribe_as(Sub, _ID), Clause, VarNames, _) :-
    !,
    '$check_expressions'(Sub, Clause, VarNames).

'$check_goal_expressions'(ground(G), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(ground(G), Clause, VarNames)).
'$check_goal_expressions'(ground(G), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G, Clause, VarNames).
'$check_goal_expressions'(template(G), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(template(G), Clause, VarNames)).
'$check_goal_expressions'(template(G), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G, Clause, VarNames).
'$check_goal_expressions'(var(G), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(var(G), Clause, VarNames)).
'$check_goal_expressions'(var(G), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G, Clause, VarNames).
'$check_goal_expressions'(nonvar(G), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(var(G), Clause, VarNames)).
'$check_goal_expressions'(nonvar(G), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G, Clause, VarNames).
'$check_goal_expressions'(type(G1, G2), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(type(G1, G2), Clause, VarNames)).
'$check_goal_expressions'(type(_G1, G2), Clause, VarNames, _) :-
    !,
    ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
    '$is_a_valid_type'(G2).
'$check_goal_expressions'(isa(G1, G2), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(isa(G1, G2), Clause, VarNames)).
'$check_goal_expressions'(isa(G1, G2), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G1, Clause, VarNames),
    ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
    '$is_a_valid_type'(G2),
    (
     '$finite_type'(G2, [])
    ->
     true
    ;
     process_non_finite_isa(G2, isa(G1, G2))
    ).
'$check_goal_expressions'(type(G1, G2, V), Clause, VarNames, act) :-
    !,
    '$display_error'(rel_in_act(type(G1, G2, V), Clause, VarNames)).
'$check_goal_expressions'(type(G1, G2, _), Clause, VarNames, _) :-
    !,
    '$check_expressions'(G1, Clause, VarNames),
    ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
    '$is_a_valid_type'(G2).
'$check_goal_expressions'(G, Clause, VarNames, act) :-
    '$type_info'(_, rel, _, G, _), !,
    '$display_error'(rel_in_act(G, Clause, VarNames)).
'$check_goal_expressions'(G, Clause, VarNames, _) :-
    '$type_info'(_, rel, Types, G, _), !,
    (
     atom(G)
    ->
     true
    ;
      '@..rev'(G, _F, Args),
      length(Types, N),
      (
        length(Args, N)
      ->
        '$check_expressions'(Args, Clause, VarNames)
      ;
        thaw_term(Clause),
        '$vars2names'(Clause, VarNames),
        '$display_error'(arity_mismatch(G, Clause, N, relation))
      )
    ).
'$check_goal_expressions'(G, Clause, VarNames,rel) :-
    '$type_info'(_, act, _Types, G, _), !,
    '$display_error'(action_in_rel(G, Clause, VarNames)).
'$check_goal_expressions'(G, Clause, VarNames, _) :-
    '$type_info'(_, act, Types, G, _), !,    
    '@..rev'(G, _F, Args),
    length(Types, N),
    (
      length(Args, N)
    ->
      '$check_action_expressions'(Args, Types, Clause, VarNames)
    ;
      thaw_term(Clause),
      '$vars2names'(Clause, VarNames),
      '$display_error'(arity_mismatch(G, Clause, N, action))
    ).
%'$check_goal_expressions'(G, _Clause, _VarNames, _) :-
%    '$type_info'(_, fun, _, G, _), !.
'$check_goal_expressions'(G, Clause, VarNames, RelAct) :-
    %'@functor'(G, F, N),
    '$get_function_type_info'(G, Args, _Types, RelAct1), !,
    (
      RelAct = RelAct1
    ->
      map('$tuple2list', [Args, ArgsL]),
      '$check_expressions'(ArgsL, Clause, VarNames)
    ;
      RelAct = act, RelAct1 = rel
    ->
      '$display_error'(rel_in_act(G, Clause, VarNames))
    ;
      RelAct = rel, RelAct1 = act
    ->
      '$display_error'(action_in_rel(G, Clause, VarNames))
    ;
      '$display_error'(expression_in_relact(G, Clause, VarNames, RelAct))
    ).

'$check_goal_expressions'(G, Clause, VarNames, RelAct) :-
    compound(G), functor(G, F, _),
    '$type_info'(F, fun, _, _, _), !,
    '$display_error'(expression_in_relact(G, Clause, VarNames, RelAct)).
'$check_goal_expressions'(G, Clause, VarNames, RelAct) :-
    atomic(G), !,
    '$display_error'(expression_in_relact(G, Clause, VarNames, RelAct)).
'$check_goal_expressions'(G, Clause, VarNames, RelAct) :-
    string(G), !,
    '$display_error'(expression_in_relact(G, Clause, VarNames, RelAct)).
'$check_goal_expressions'(G, _Clause, _VarNames, _) :-
    var(G), !.
'$check_goal_expressions'(G, _Clause, _VarNames, _) :-
    '$get_ultimate_functor'(G, F),
    var(F), !.
'$check_goal_expressions'([_|_], _Clause, _VarNames, _) :- !.

'$check_goal_expressions'(Term, Clause, VarNames, _) :-
    '$display_error'(bad_goal(Term, Clause, VarNames)).

'$check_case_expressions'('$case_alt'(Test, Act), Clause, VarNames, RA) :-
    '$check_goal_expressions'(Test, Clause, VarNames, rel),
    '$check_goal_expressions'(Act, Clause, VarNames, RA).

%%'$check_receive_expressions'(A,Clause, VarNames, RA) :- errornl('$check_receive_expressions'(A)),fail.
'$check_receive_expressions'(_('$receive_from'(_MP, _AP, '$empty', Act)),
			     Clause, VarNames, RA) :-
    !,
    '$check_goal_expressions'(Act, Clause, VarNames, RA).
'$check_receive_expressions'(_('$receive_from'(_MessPatt, _ActPatt, Test, Act)),
			     Clause, VarNames, RA) :-
    !,
    '$check_goal_expressions'(Test, Clause, VarNames, rel),
    '$check_goal_expressions'(Act, Clause, VarNames, RA).
'$check_receive_expressions'(timeout(T, Act), Clause, VarNames, RA) :-
    !,
    '$check_expressions'(T, Clause, VarNames),
    '$check_goal_expressions'(Act, Clause, VarNames, RA).
    
'$check_try_except_expressions'('$catch'(Patt, '$empty', Act),
                                Clause, VarNames, RA) :-
    !,
    '$check_expressions'(Patt, Clause, VarNames),
    '$check_goal_expressions'(Act, Clause, VarNames, RA).
'$check_try_except_expressions'('$catch'(Patt, Test, Act),
                                Clause, VarNames, RA) :-
    '$check_expressions'(Patt, Clause, VarNames),
    '$check_goal_expressions'(Test, Clause, VarNames, rel),
    '$check_goal_expressions'(Act, Clause, VarNames, RA).

%%'$check_action_expressions'(A,B,C,D) :- errornl('$check_action_expressions'(A,B,C,D)),fail.
'$check_action_expressions'([], [], _Clause, _VarNames).
'$check_action_expressions'([A|Args], [_T|Types], Clause, VarNames) :-
    '$check_expressions'(A, Clause, VarNames),
    '$check_action_expressions'(Args, Types, Clause, VarNames).


'$check_arg_expressions'([],  _Clause, _VarNames).
'$check_arg_expressions'([A|Args], Clause, VarNames) :-
    '$check_expressions'(A, Clause, VarNames),
    '$check_arg_expressions'(Args, Clause, VarNames).

%%'$check_arities'(Args, Types,  Term, _) :- errornl('$check_arities'(Args, Types,  Term, _)),fail.
'$check_arities'([], [],  _Term, _).
'$check_arities'(_, _,  _, Error) :-
    nonvar(Error), !.
'$check_arities'([A|Args], [T|Types],  Term, Error) :-
    '$tuple_len'(A, ALen),
    '$tuple_type_len'(T, TLen),
    (
      ALen = TLen
    ->
      '$check_arities'(Args, Types,  Term, Error)
    ;
      Error = (arity_error(Term, function_arg))
    ).

    


%%'$check_expressions'(G, C, V) :- errornl('$check_expressions'(G, C, V)), fail.
%% '$check_expressions'(Expr, Clause, VarNames) checks Expr is a valid
%% expression
%% All non-compound expressions are valid
'$check_expressions'(G, _, _) :-
    \+compound(G), !.
'$check_expressions'('$'(A), Clause, VarNames) :-
    !,
    (
      '$is_state_belief'(A)
    ->
      true
    ;
      '$display_error'(not_a_global(A, '$'(A), Clause, VarNames))
    ).
% '$check_expressions'('@'(Type), Clause, VarNames) :- !,
%     '$check_expressions'(Type, Clause, VarNames).

'$check_expressions'(Expr, Clause, VarNames) :-
    Expr = F(_,_), F == '$list_constr',
    Expr = '$list_constr'(E,  G),
    !,
    '$check_expressions'(E, Clause, VarNames),
    '$check_goal_expressions'(G, Clause, VarNames, rel).
'$check_expressions'(Expr, Clause, VarNames) :-
    Expr = F(_,_), F == '$set_constr',
    Expr = '$set_constr'(E,  G),
    !,
    '$check_expressions'(E, Clause, VarNames),
    '$check_goal_expressions'(G, Clause, VarNames, rel).
%% used in =? (RHS)
'$check_expressions'(Expr, Clause, VarNames) :-
    Expr = F(_,_), F == '::',
    Expr = (_ :: G),
    !,
    '$check_goal_expressions'(G, Clause, VarNames, rel).
'$check_expressions'(Expr, Clause, VarNames) :-
    Expr = '$set'([H|T]),
    !,
    '$check_expressions'(H, Clause, VarNames),
    '$check_expressions'(T, Clause, VarNames).
'$check_expressions'(Expr, _Clause, _VarNames) :-
    Expr = '$set_enum'([]), !.
'$check_expressions'(Expr, Clause, VarNames) :-
    Expr = '$set_enum'([H|T]),
    !,
    '$check_expressions'(H, Clause, VarNames),
    '$check_expressions'(T, Clause, VarNames).
'$check_expressions'(Expr, Clause, VarNames) :-
    list(Expr), Expr = [H|T],
    !,
    '$check_expressions'(H, Clause, VarNames),
    '$check_expressions'(T, Clause, VarNames).
'$check_expressions'(Expr, Clause, VarNames) :-
    Expr = '$tuple'(Lst),
    !,
    '$check_expressions'(Lst, Clause, VarNames).

%%XXXX
'$check_expressions'(Term, Clause, VarNames) :-
    compound(Term), !,
    Term =.. [_|Args],
    '$check_expressions'(Args, Clause, VarNames).
% '$check_expressions'(Term, Clause, VarNames) :-
%     compound(Term),
%     '@..rev'(Term, P, Args),
%     '$type_info'(P, Kind, Types, _, _),
%     Kind \= rel, Kind \= act, Kind \= tel,  !,
%     %% type declaration 
%     length(Types, TLen),
%     process_definition_as_term_error(Term, P, TLen, Kind, Clause, VarNames),
%     '$check_expressions'(Args, Clause, VarNames).
%  '$check_expressions'(Term, Clause, VarNames) :-
%      compound(Term),
%      functor(Term, P, _),
%      P == list, !,
%      Term = list(Type),
%      ip_lookup(valid_type_context, Saved),
%      ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
%      '$is_a_valid_type'(Type),
%      ip_set(valid_type_context, Saved).
% '$check_expressions'(Term, Clause, VarNames) :-
%      compound(Term),
%      functor(Term, P, _),
%      P == set, !,
%      Term = set(Type),
%      ip_lookup(valid_type_context, Saved),
%      ip_set(valid_type_context, valid_type_check(Clause, VarNames)),
%      '$is_a_valid_type'(Type),
%      ip_set(valid_type_context, Saved).

% '$check_expressions'(Term, Clause, VarNames) :-
%     compound(Term), !,
%     '@..rev'(Term, P, Args),
%     '$check_expressions'([P|Args], Clause, VarNames).
'$check_expressions'(Term, Clause, VarNames) :-
    '$display_error'(bad_expression(Term, Clause, VarNames)).



'$check_belief_goal'(G, Clause, VarNames) :-
    (
     var(G)
    ->
     true
    ;
     ( compound(G), functor(G, F, _), var(F) )
    ->
     '$display_error'(bad_belief_goal(G, Clause, VarNames))
    ;
     G = A(_), '$is_state_belief'(A)
    ->
     '$display_error'(bad_belief_global_goal(A, Clause, VarNames))
    ;
     '$belief_type_info'(_, _, G)
    ->
     true
    ;
     '$display_error'(bad_belief_goal(G, Clause, VarNames))
    ).


'$get_underscores'(T, Vars, Unders) :-
    collect_vars(T, TVars),
    filter('$not_in_vars'(Vars), TVars, Unders).
    
'$not_in_vars'(Vars, X) :-
    member((A = _B), Vars),
    A == X,
    !,
    fail.
'$not_in_vars'(_Vars, _X).
   





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Support for term transformation
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

'$expandL'(L,AL-(AL=L)):-
   (var(L) ; list(L)),
   !.
'$expandL'('$set_constr'(E, Conds),L-(findall(E,Conds,L1), sort(L1, L))):-
   !.
'$expandL'('$list_constr'(E, Conds),L-(findall(E,Conds,L))):-
   !.
'$expandL'((L1 <>? L2), AL-(Expands1,Expands2,append(AL1,AL2,AL))):-
    !,
  '$expandL'(L1, AL1-Expands1),
  '$expandL'(L2,AL2-Expands2).
'$expandL'(L,AL-(AL=L)).


'$expandS'(S,AS-(AS=S)):-
   (var(S) ; string(S)),
   !.
'$expandS'((S1 ++? S2), AS-(Expands1,Expands2,string_concat(AS1,AS2,AS))):-
  '$expandS'(S1, AS1-Expands1),
  '$expandS'(S2,AS2-Expands2).


 '$expandLP'(L, (L1 <>? L2 <>? L3), Trans) :-
       !,
       (
	L1 = L11+ '::'([], Cond)
       ->
	Trans=( append(L11,RL,L),Cond,Apps)
       ;
        L1= '::'(L11, once(Cond))
       ->
	Trans = (append(L11,RL,L),once(Cond),Apps)
       ;
	Trans= ( append(L1,RL,L),Apps)
       ),
       '$expandLP'(RL, (L2 <>? L3) , Apps).  

 '$expandLP'(L, (L1 <>? L2), Trans) :-
       (
	(L1='::'(L11,Cond1), L2='::'(L22,Cond2))
       -> 
	Trans=(append(L11,L22,L),once(Cond1),once(Cond2))
       ;
	L1='::'(L11,Cond1)
       ->
	Trans=(append(L11,L2,L),once(Cond1))
       ;
        L2='::'(L22,Cond2)
       ->
	Trans=(append(L1,L22,L),once(Cond2))
       ;
        Trans=(append(L1,L2,L))
       ).
           


 '$expandSP'(L, (L1 ++? L2), Trans) :-
     !,
     (
       L1 = '::'('$re'(S1, RE),Cond)
     ->
       Trans = (re_match(RE, L, ['$tuple'([0,N])|_]),
                   sub_string(L, 0, N, Rest, S1),
                   sub_string(L, N, Rest, _, RL),
                   once(Cond),Concats)
     ;
       L1 = '$re'(S1, RE)
     ->
       Trans = (re_match(RE, L, ['$tuple'([0,N])|_]),
                   sub_string(L, 0, N, Rest, S1),
                   sub_string(L, N, Rest, _, RL),
                   Concats)
      ;
       L1= '::'(S1,Cond)
     ->
       Trans = (string_concat(S1,RL,L),once(Cond),Concats)
     ;
       Trans= ( string_concat(L1,RL,L),Concats)
     ),
     '$expandSP'(RL, L2, Concats).
'$expandSP'(L, L1, Trans) :-
    (
       L1 = '::'('$re'(S1, RE),Cond)
     ->
       Trans = (re_match(RE, L, ['$tuple'([0,N])|_]),
                   sub_string(L, 0, N, Rest, S1),
                   Rest = 0, once(Cond))
     ;
       L1 = '$re'(S1, RE)
     ->
       Trans = (re_match(RE, L, ['$tuple'([0,N])|_]),
                   sub_string(L, 0, N, Rest, S1),
                   Rest = 0)
      ;
       L1= '::'(S1,Cond)
     ->
       Trans = (L = S1,once(Cond))
     ;
       Trans= ( L = L1 )
     ).




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Auxliary predicates
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





'$list2tuple'([X], X) :- !.
'$list2tuple'([X|Y], (X, Ys)) :-
    '$list2tuple'(Y, Ys).

'$index_member'(X, I, L) :-
    L = [_|_],!,
    append(Y, [X|_], L),
    length(Y, N),
    I is N+1.
'$index_member'(X, I, L) :-
    '$tuple2list'(L, L1),
    append(Y, [X|_], L1),
    length(Y, N),
    I is N+1.


%%'$tuple_type_to_list'(TA, A) :-errornl('$tuple_type_to_list'(TA, A)),fail.
'$tuple_type_to_list'(TA, A) :-
    compound(TA), TA = '$tuple_type'(A), !.
%%'$tuple_type_to_list'(A, [A]).

%'$list_to_tuple_type'([A], A) :- !.
'$list_to_tuple_type'(As, '$tuple_type'(As)).


'$tuple2list'(A, B) :-
    var(A), !, B = [A].
'$tuple2list'(A, B) :-
    compound(A), functor(A, F, _), var(F), !,
    B = [A].
'$tuple2list'((A,B), Result) :-
     !,
     '$tuple2list'(A, AL),
     '$tuple2list'(B, BL),
     append(AL,BL,Result).
'$tuple2list'(A, [A]).

'$tuple_len'(TA, B) :-
    compound(TA), TA = '$tuple'(A), !,
    length(A, B).
'$tuple_len'(_, 1).

'$tuple_type_len'(TA, B) :-
    compound(TA), TA = '$tuple_type'(A), !,
    length(A, B).
'$tuple_type_len'(_, 1).

'$set_var_names'(Term, VarNames, Thawed) :-
    thaw_term(Term, Thawed),
    '$do_unifs'(VarNames),
    collect_vars(Term, UVars),
    '$unify_underscore'(UVars).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Checking against grammar but more exact checking
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

'$is_term'(T) :-
    atom(T).
'$is_term'(T) :-
    \+list(T),
    compound(T).




'$just_new_constructor'(C) :-
     '$is_term'(C),
    C =.. [P|Args],
    atom(P),
    \+is_declared_type_name_(P), !,
    forall(member(A, Args), (var(A) ; '$is_a_type'(A))).
'$just_new_constructor'(C) :-
    ip_lookup(defined_type_check, V), !, nonvar(V),
    V = dtc(A, AT, VarNames),
    '$display_error'(invalid_constructor_type((A ::= AT), VarNames, C)).


'$is_struct_type'(T) :-
    '$is_term'(T),
    '@..rev'(T, P, Args),
    %T =.. [P|Args],
    atom(P),
    forall(member(A, Args), '$is_a_type'(A)).



is_declared_type_name_(TN) :-
    '$defined_type'(A,_),
    functor(A,TN,_),
    !. 
   




is_primitive_or_defined_type_(T) :-
     member(T, [atom,nat,int,num,string,atomic,term, top, code,
                '$none_', dyn_term, act_term, rel_term, tel_term,
                tel_percept_term, tel_action_term]),
     !. 
is_primitive_or_defined_type_(T):-
    once('$defined_type'(T,_)). 


'$check_enum_atom_type'(T) :-
    is_declared_type_name_(T),
    ip_lookup(defined_type_check, V), !, nonvar(V),
    V = dtc(A, AT, VarNames),
    '$display_error'(invalid_atom_enum_type((A ::= AT), VarNames, T)).
'$check_enum_atom_type'(_).

%%'$is_a_type'(T) :-    errornl(is_a_type______(T)),fail.
'$is_a_type'(X) :- var(X), !.
'$is_a_type'('$enum_type'(TList)) :-
    !,
    (
     forall(member(T, TList), (atom(T), '$check_enum_atom_type'(T)))
    ;
     forall(member(T, TList), string(T))
    ;
     forall(member(T, TList), number(T))
    ).
'$is_a_type'('$constr_enum_type'(TList)) :-
    !,
    forall(member(T, TList), (compound(T), '$just_new_constructor'(T))).
'$is_a_type'( '$union_type'(Types)) :-
    !,
    forall(member(T, Types), '$is_a_type'(T)).
'$is_a_type'( list(T)) :-
    !,
    '$is_a_type'(T).
'$is_a_type'( set(T)) :-
    !,
    '$is_a_type'(T).
'$is_a_type'('$tuple_type'(T)) :-
    !,
    forall(member(T1, T), '$is_a_type'(T1)).
'$is_a_type'((T1 -> T2)) :-
    !,
    '$is_a_type'(T1),
    '$is_a_type'(T2).
'$is_a_type'(rel(Types)) :-
    !,
    forall(member(T1, Types), '$is_a_type'(T1)).
'$is_a_type'(tel(Types)) :-
    !,
    forall(member(T1, Types), '$is_a_type'(T1)).
'$is_a_type'(act(Types)) :-
    !,
    forall(member(T1, Types), '$is_a_type'(T1)).

'$is_a_type'(typeE(T)) :-
    !,
    '$is_a_type'(T). 
'$is_a_type'(T) :-
    '$is_a_stype'(T),
    !. 
'$is_a_type'(C1) :-
    atom(C1), !,
    (
     is_declared_type_name_(C1)
    ->
     true
    ;
     ip_lookup(defined_type_check, V), nonvar(V)
    ->
     V = dtc(A, AT, VarNames),
     '$display_error'(invalid_type((A ::= AT), VarNames, C1))
    ;
     fail
    ). 
'$is_a_type'(C1) :-
    !,
    (
      compound(C1),
      '$defined_type'(C1,_)
    ->
      true
    ;
      ip_lookup(defined_type_check, V), nonvar(V),
       V = dtc(A, AT, VarNames),
      '$display_error'(invalid_constructor_type((A ::= AT), VarNames, C1))
    ).
'$is_a_type'(T) :-
    errornl(failed_is_a_type______(T)),fail.

tuple_member_(T, (T,_Ts)).
tuple_member_(T, (_,Ts)) :- 
    !,
    tuple_member_(T,Ts).
tuple_member_(T, T). 



'$is_a_stype'(T) :-
    var(T), !.
'$is_a_stype'([]) :- !, fail.
'$is_a_stype'('$none_') :-
    !.
'$is_a_stype'(T) :-
    is_primitive_or_defined_type_(T),
    !,
    (
     compound(T)
    ->
     '$is_struct_type'(T) 
    ;
     true
    ).

% so a specialisation of a type does not need to be prefixed
'$is_a_stype'((N..M)) :-
    !,
    integer(N), integer(M),
    N<M.
'$is_a_stype'( '$enum_type'(Types)) :-
    closed_list(Types),
    !,
    forall(tuple_member_(T, Types), '$is_a_type'(T)).
'$is_a_stype'( '$constr_enum_type'(Types)) :-
    closed_list(Types),
    !,
    forall(tuple_member_(T, Types), '$is_a_type'(T)).


%%'$is_goal_term'(G) :- errornl('$is_goal_term'(G)),fail.
'$is_goal_term'(G) :-
    var(G), !, fail.

'$is_goal_term'(G) :-
    '$type_info'(_, act, _, G, _), !.
'$is_goal_term'(G) :-
    '$type_info'(_, rel, _, G, _), !.
'$is_goal_term'(G) :-
     '$get_function_type_info'(G, _Args, _Types, RT),
     RT == rel, !.
'$is_goal_term'(type(_, _)).
'$is_goal_term'(type(_, _, _)).
'$is_goal_term'(ground(_)).
'$is_goal_term'(once(_)).
'$is_goal_term'(remember(_)).
'$is_goal_term'(remember_for(_,_)).
'$is_goal_term'(forget(_)).
'$is_goal_term'(forget_remember(_,_)).
'$is_goal_term'('$at_forall'(_, _, _)).
'$is_goal_term'('$forall'(_, _, _)).
'$is_goal_term'('$pfindall'(_, _, _)).
'$is_goal_term'('$exists'(_, _)).
'$is_goal_term'('$at_forall_actions'(_, _, _)).
'$is_goal_term'('$forall_actions'(_, _, _)).
'$is_goal_term'('$remote_query'(_, _)).
'$is_goal_term'('=?'(_,_)).
%'$is_goal_term'(term).


%%'$is_a_valid_type'(T) :- errornl('$is_a_valid_type'(T)),fail.
'$is_a_valid_type'(T) :-
    '$is_a_valid_type_aux'(T), !.
'$is_a_valid_type'(T) :-
    ip_lookup(valid_type_context, valid_type_check(Clause, Vars)),
    '$display_error'(invalid_type(Clause, Vars, T)).

%%'$is_a_valid_type_aux'(T) :-errornl('$is_a_valid_type_aux'(T)),fail.
'$is_a_valid_type_aux'(T) :-
    var(T), !.
'$is_a_valid_type_aux'(T) :-
    T = typeE(T1),  moded__(T1), !,
    T1 = _M(T2),
    var(T2).
'$is_a_valid_type_aux'(T) :-
    T = typeE(T1), !,
    var(T1).
'$is_a_valid_type_aux'(T) :-
     is_primitive_or_defined_type_(T), !.
'$is_a_valid_type_aux'(T) :-
    T = '$enum_type'(Enum),!,
    once(
    (
     forall(member(X, Enum), atom(X))
    ;
     forall(member(X, Enum), string(X))
    ;
     forall(member(X, Enum), number(X))
    )).
'$is_a_valid_type_aux'(T) :-
    T = '$constr_enum_type'(Enum),!,
    forall(member(X, Enum), '$is_a_constructor_type'(X)).
'$is_a_valid_type_aux'(T) :-
    T = '$union_type'(Types), !,
    forall(member(ET, Types), '$is_a_valid_type_aux'(ET)).
'$is_a_valid_type_aux'(T) :-
    moded__(T),
    T = M(T1),   
    T1 = '$tuple_type'(Types), !,
    check_constructor_mode_and_type__(Types, M).
'$is_a_valid_type_aux'(T) :-
    T = '$tuple_type'(Types), !,
    forall(member(Ty, Types), '$is_a_valid_type_aux'(Ty)).
'$is_a_valid_type_aux'(T) :-
    moded__(T),
    T = M(T1),   
    T1 = set(Type), !,
    check_constructor_mode_and_type__([Type], M).
    %'$is_a_valid_type_aux'(Type).
'$is_a_valid_type_aux'(T) :-
    T = set(Type), !,
    '$is_a_valid_type_aux'(Type).
'$is_a_valid_type_aux'(T) :-
    T = list(Type), !,
    '$is_a_valid_type_aux'(Type).
'$is_a_valid_type_aux'(T) :-
    moded__(T),
    T = _(T1),
    T1 = list(Type), !,
    '$is_a_valid_type_aux'(Type).
'$is_a_valid_type_aux'(T) :-
    T = tuple(Type), !,
    '$is_a_valid_type_aux'(Type).
'$is_a_valid_type_aux'(T) :-
    moded__(T),
    T = _(T1),
    T1 = tuple(Type), !,
    '$is_a_valid_type_aux'(Type).
'$is_a_valid_type_aux'(T) :-
    T = (T1 -> T2), !,
    '$is_a_valid_type_aux'(T1),
    '$is_a_valid_type_aux'(T2).
'$is_a_valid_type_aux'(T) :-
    T = rel(T1), !,
    '$is_a_valid_type_aux'(T1).
'$is_a_valid_type_aux'(T) :-
    T = act(T1), !,
    '$is_a_valid_type_aux'(T1).
'$is_a_valid_type_aux'(T) :-
    T = dyn(T1), !,
    '$is_a_valid_type_aux'(T1).
'$is_a_valid_type_aux'(T) :-
    T = tel(T1), !,
    '$is_a_valid_type_aux'(T1).
'$is_a_valid_type_aux'('@'(_)) :- !.
'$is_a_valid_type_aux'(T) :-
    moded__(T), 
    T = M(T1), !,
    (
      constructor_type__(T1, _, IT1)
    ->
      check_constructor_mode_and_type__(IT1, M)
    ;
      '$is_a_valid_type_aux'(T1)
    ).  
'$is_a_valid_type_aux'(T) :-
    \+ moded__(T),
    compound(T),
    functor(T, P, _),
    '$defined_type'(P, _), !.
'$is_a_valid_type_aux'((N .. M)) :-
    integer(N), integer(M), N =< M, !.
'$is_a_valid_type_aux'(atom_naming(Type)) :- 
    is_code_type_(Type), '$is_a_valid_type_aux'(Type), !.
'$is_a_valid_type_aux'(term_naming(Type)) :- 
    is_code_type_(Type), '$is_a_valid_type_aux'(Type), !.
'$is_a_valid_type_aux'(T) :-
    ip_lookup(valid_type_context, valid_type_check(Clause, Vars)),
    '$display_error'(invalid_type(Clause, Vars, T)).

%%check_constructor_mode_and_type__(M, T) :- errornl(check_constructor_mode_and_type__(M, T)),fail.
check_constructor_mode_and_type__([], _).
check_constructor_mode_and_type__([T|Rest], M) :-
    moded__(T),
    T = M1(T1), !,
    (
      M = '?'
    ->
      ( M1 = '?' ; M1 = '??' )
    ;
      M = '??'
    ->
      M1 = '??'
    ;
      true
    ),
    (
      constructor_type__(T1, _, T2)
    ->
      check_constructor_mode_and_type__(T2, M1)
    ;
      '$is_a_valid_type'(T1)
    ),
    check_constructor_mode_and_type__(Rest, M).
check_constructor_mode_and_type__([T|Rest], M) :-
    '$is_a_valid_type'(T),
    check_constructor_mode_and_type__(Rest, M).

'$is_a_constructor_type'(T) :-
    compound(T),
    functor(T, F, N), N > 0,
    T =.. [F|Args],
    atom(F),
    \+ '$parser_reserved_operator'(F),
    forall(member(Ty, Args), '$is_a_valid_type_aux'(Ty)).

'$get_clause_head_functor'((Head :: _Test -> _Body), HeadF) :-
    !,
    '$get_ultimate_functor'(Head, HeadF).
'$get_clause_head_functor'((Head -> _Body), HeadF) :-
    !,
    '$get_ultimate_functor'(Head, HeadF).
'$get_clause_head_functor'((Head :: _Test <= _Body), HeadF) :-
    !,
    '$get_ultimate_functor'(Head, HeadF).
'$get_clause_head_functor'((Head <= _Body), HeadF) :-
    !,
    '$get_ultimate_functor'(Head, HeadF).
'$get_clause_head_functor'((Head :: _Test ~> _Body), HeadF) :-
    !,
    '$get_ultimate_functor'(Head, HeadF).
'$get_clause_head_functor'((Head ~> _Body), HeadF) :-
    !,
    '$get_ultimate_functor'(Head, HeadF).
'$get_clause_head_functor'((Head :: _Test), HeadF) :-
    !,
    '$get_ultimate_functor'(Head, HeadF).
'$get_clause_head_functor'((Head), HeadF) :-
    !,
    '$get_ultimate_functor'(Head, HeadF).


%%
% '$gen_should_infer_remember_fact'(Types) :-
%     member('$decl'(A, '$tuple_type'(B)), Types),
%     !,
%     length(B, N),
%     '@functor'(Template, A, N),
%     '$add_file_info'(should_infer_remember_(Template)),
%     assert(should_infer_remember_(Template)).
 



%% "Compile" all rules to QuProlog rules
'$translate_all_clauses' :-
    findall(Head, '$tmp_clause'(Head, _), AllHeads),
    sort(AllHeads, SortedHeads),
    '$translate_all_clauses_aux'(SortedHeads),
    retractall('$tmp_clause'(_, _)).

%%'$translate_all_clauses_aux'(C) :- errornl('$translate_all_clauses_aux'(C)),fail.
'$translate_all_clauses_aux'([]).
'$translate_all_clauses_aux'([H|Heads]) :-
    findall(Clause, '$tmp_clause'(H, clause(Clause)), HClauses),
    '$translate_rule_definition'(HClauses, LastClause),
    '$add_default_exception_clause'(LastClause),
    '$translate_all_clauses_aux'(Heads).

'$translate_rule_definition'([H], LastClause) :-
    !,
    freeze_term(H),
    '$translate_clause'(H),
    thaw_term(H),
    LastClause = H.
'$translate_rule_definition'([H|HClauses], LastClause) :-
    freeze_term(H),
    '$translate_clause'(H),
    thaw_term(H),
    '$translate_rule_definition'(HClauses, LastClause).

'$add_default_exception_clause'((Head :: Test -> _Body)) :-
    '$make_var_template'(Head, Template),
    ( Test \= true ; '$not_all_distinct_var_args'(Head, Template) ), !,
    '$add_default_exception_rule'(Template, fun).
'$add_default_exception_clause'((Head  -> _Body)) :-
    '$make_var_template'(Head, Template),
    '$not_all_distinct_var_args'(Head, Template), !,
    '$add_default_exception_rule'(Template, fun).
'$add_default_exception_clause'((Head :: Test ~> _Body)) :-
    '$make_var_template'(Head, Template),
    ( Test \= true ; '$not_all_distinct_var_args'(Head, Template) ), !,
    '$add_default_exception_rule'(Template, act).
'$add_default_exception_clause'((Head  ~> _Body)) :-
    '$make_var_template'(Head, Template),
    '$not_all_distinct_var_args'(Head, Template), !,
    '$add_default_exception_rule'(Template, act).
'$add_default_exception_clause'(_).

'$make_var_template'(Head, Template) :-
    atom(Head), !,
    Template = Head.
'$make_var_template'(Head, Template) :-
    Head '@=..' [F|Args],
    length(Args, N),
    length(TempArgs, N),
    '$make_var_template'(F, TempF),
    Template '@=..' [TempF|TempArgs].

'$not_all_distinct_var_args'(Head, Template) :-
    freeze_term(Template),
    Template = Head, !, fail.
'$not_all_distinct_var_args'(_Head, _Template).

%%'$add_default_exception_rule'(A,B) :- errornl('$add_default_exception_rule'(A,B)),fail.
'$add_default_exception_rule'(Head, fun) :-
    '$get_function_type_info'(Head,HeadT, _, ResultType),
    length(HeadT, Len),
    Len \= 1, !,
    '@=..'(Head, _P),
    '$add_fun_args'(ResultType, Head, FullHead, 0, _),
    term2string(Head, HeadStr),
    FBody =  throw(no_matching_function_rule(HeadStr)),
    FullHead =.. [F|Args],
    '$generate_apply_name'(F, ApplyName),
    A1 = ApplyName(F, Args, _X),
    '$process_apply_clause'(A1, FBody).
    

'$add_default_exception_rule'(Template, fun) :-
    '@=..'(Template, P),
    append(P, [_X], P1),
    A1 =.. P1,
    assert((A1 :-  name_vars(Template), term2string(Template, TempStr),
            throw(no_matching_function_rule(TempStr)))).
'$add_default_exception_rule'(Template, act) :-
    assert((Template :- name_vars(Template), term2string(Template, TempStr),
            throw(no_matching_action_rule(TempStr)))).
	  
'$check_no_repeated_trs' :-
    findall(TRFunctor,
	    ('$tel'(Head, _Body), '@functor'(Head, TRFunctor, _)),
	    AllTRFunctors),
    (
     append(_, [Fun|Rest], AllTRFunctors), member(Fun, Rest)
    ->
     '$display_error'(repeated_TR_defn(Fun))
    ;
     true
    ).

'$check_contiguous_defs' :-
    findall(F, retract('$head_order_check'(F)), AllFs),
    (
     '$is_non_contiguous_in'(AllFs, H)
    ->
     '$display_error'(noncontiguous_defn(H))
    ;
     true
    ).

'$is_non_contiguous_in'(Xs, X) :-
    append(_, [X|Rest], Xs),
    X \= '$ok',
    Rest \= [X|_],
    member(X, Rest).
	    
'$declare_percepts_top' :-
    retract('$tmp_percepts'(P)),
    '$declare_percept'(P),
    fail.
'$declare_percepts_top'.


'$declare_percept'(P) :-
    '$tmp_clause'(P, _),
    %% percept rule - deduced from other percepts
    !,
    '$user_type_info'(P, _, _, PTerm, _),
    '$add_file_info'('$percept'(_)),
    functor(PTerm, _, N),
    dynamic(P/N, 0),    
    assert('$percept'(PTerm)).
'$declare_percept'(P) :-
    '$user_type_info'(P, _, _, PTerm, _),
    functor(PTerm, _, N),
    dynamic(P/N, 0),
    '$add_file_info'('$belief'(PTerm)),
    assert('$belief'(PTerm)),
    '$add_file_info'('$percept'(PTerm)),
    assert('$percept'(PTerm)).



'$strip_decl_vars'([], []).
'$strip_decl_vars'([H|Rest], [T|SRest]) :-
    compound(H),
    functor(H, ':', 2),
    H = (_:T),
    !,
    '$strip_decl_vars'(Rest, SRest).
'$strip_decl_vars'([T|Rest], [T|SRest]) :-
    '$strip_decl_vars'(Rest, SRest).

%%'$check_decl'(Decl, Vars) :- errornl('$check_decl'(Decl, Vars)),fail.
'$check_decl'(Decl, VarNames) :-
    \+ '$check_repeated_poly_types'(Decl, VarNames),
    !,fail.
'$check_decl'(Decl, VarNames) :- 
    Decl = '$code_decl'(_, [T], _),
    T = '$decl'(F, _),
    !,
    %% a single declaration
    %% check for re declaration
    (
      '$type_info'(F, _, Type, _, V)
    ->

      '$real_kind'(F, Kind),
      '$vars2names'(Decl, VarNames),
      '$vars2names'(Type, V),
      '$display_error'(re_declaration_type(F, Decl, Kind, Type))
    ;
      true
    ).
    
'$check_decl'(Decl, VarNames) :-
    Decl = '$code_decl'(Kind, Types, _),
    Kind \= percept,
    Types = ['$decl'(F, '$tuple_type'(Args))|Rest],
    length(Args, N),
    forall(member('$decl'(F1, '$tuple_type'(A)), Rest), (F1 = F, length(A, N))),
    %% intersection type declaration
    !,
    %% check for re declaration
    (
      '$type_info'(F, _, Type, _, V)
    ->
      '$real_kind'(F, Kind),
      '$vars2names'(Decl, VarNames),
      '$vars2names'(Type, V),
      '$display_error'(re_declaration_type(F, Decl, Kind, Type))
    ;
      true
    ).
'$check_decl'(Decl, VarNames) :-
    Decl = '$code_decl'(_, Types, _),
    Types = ['$decl'(F, ('$tuple_type'(Args) -> _))|Rest],
    length(Args, N),
    forall(member('$decl'(F1, '$tuple_type'(A)), Rest), (F1 = F, length(A, N))),
    %% intersection type declaration
    !,
    %% check for re declaration
    (
      '$type_info'(F, _, Type, _, V)
    ->
      '$real_kind'(F, Kind),
      '$vars2names'(Decl, VarNames),
      '$vars2names'(Type, V),
      '$display_error'(re_declaration_type(F, Decl, Kind, Type))
    ;
      true
    ).

    
'$check_decl'(Decl, VarNames) :-
    Decl = '$code_decl'(_, Types, _),
    member(T1, Types),
    member(T2, Types),
    T1 \== T2,
    T1 = '$decl'(F, _),
    T2 = '$decl'(F, _),
    %% not an intersection type but repeated declarations 
    !,
    '$vars2names'(Decl, VarNames),
    '$display_error'(repeats_in_type_decl(Decl)).
'$check_decl'(Decl, VarNames) :-
    Decl = '$code_decl'(_, Types, ""),
    %% multiple declarations without a doc
    !,
    %% check for re declaration
    (
      (member('$decl'(F, _), Types), '$type_info'(F, _, Type, _, V))
    ->

      '$real_kind'(F, Kind),
      '$vars2names'(Decl, VarNames),
      '$vars2names'(Type, V),
      '$display_error'(re_declaration_type(F, Decl, Kind, Type))
    ;
      true
    ).

'$check_decl'(Decl, VarNames) :-
    !,
    %% multiple declarations with a doc
    '$vars2names'(Decl, VarNames),
    '$display_error'(doc_in_multi_type_decl(Decl)).

%%'$check_repeated_poly_types'(Decl, VarNames) :- errornl('$check_repeated_poly_types'(Decl, VarNames)),fail.
'$check_repeated_poly_types'(Decl, VarNames) :-
    fail, %%TODO XXXX
    Decl = '$code_decl'(_, Types, _),
    member(T, Types),
    transform_subterms('$remove_decl_docvars', T, T1),
    collect_vars(T1, Vars),
    member(V, Vars),
    collect_simple_terms('$collect_var_occurrences'(V), T1, [], Voc),
    length(Voc, VocLen),
    VocLen < 2,
    !,
    '$vars2names'(Decl, VarNames),
    '$display_error'(non_repeated_poly_var_decl(Decl, V)).
'$check_repeated_poly_types'(_Decl, _VarNames).

'$collect_var_occurrences'(X, A,B,C) :-
    A == X, C = [A|B].

'$remove_decl_docvars'(T, T1) :-
    compound(T), T = _:T1, !.
'$remove_decl_docvars'(T, T).

'$kind_mapping'(rel, rel).
'$kind_mapping'(act, act).
'$kind_mapping'(fun, fun).
'$kind_mapping'(tel, tel).
'$kind_mapping'(dyn, rel).

%%'$check_mrel_modes'(Decl, VarNames) :- errornl('$check_mrel_modes'(Decl, VarNames)),fail.
'$check_mrel_modes'(Decl, VarNames) :-
    Decl = '$code_decl'(rel, Types, _Doc),
    member('$decl'(_, '$tuple_type'(ArgTypes)), Types),
    member(MT, ArgTypes),
    moded__(MT),
    MT = '!'(_),
    push_modes_in__(MT, T),
    has_non_inner_bang__(T),
    '$vars2names'(Decl, VarNames),
    '$display_error'(mrel_mixed_modes(MT, Decl)).
'$check_mrel_modes'(_, _).


%%has_non_inner_bang__(T) :- errornl(has_non_inner_bang_aux__(T)),fail.
has_non_inner_bang__(M(_T)) :-  M \= '!', !.
has_non_inner_bang__(_M(T)) :-
    constructor_type__(T, _, SubT),
    member(T1, SubT),
    has_non_inner_bang__(T1).

%% Dependency calculation for rem relations
%% For each rem relation we compute the dependencies
%% on belief predicates and generate a rule of the form
%% not_dep_overlap_(Rel, Dep1, Dep2) :-
%%       '$no_overlap'(Dep1, Dep2, Depends)
%% where Rel is a template for the infer and remember predicate
%% and Depends is the list of templates of belief store  predicates
%% its rules depend on.
%% Dep1 and Dep2 are the forget/remember lists from forget_remember
'$add_cache_dependencies' :-
    cache_info__(Call, RemCall, Rem, _),
    findall(Body, clause(RemCall, Body), Bodies),
    '$get_cache_dependencies'(Bodies, Depends, Call, [Call], _),
    functor(Rem, RemF, N),
    (
      Depends = []
    ->
      dynamic( RemF/N)
    ;
      dynamic( RemF/N, 0)
    ),
    '$remove_dep_deplicates'(Depends, SimpDepends),
    '$add_file_info'(not_dep_overlap_(_, _,_)),
    assert((not_dep_overlap_(Call, Dep1, Dep2) :-
           '$no_overlap'(Dep1, Dep2, SimpDepends))),
    fail.
'$add_cache_dependencies'.

%%'$get_cache_dependencies'(A,B,C,D,E) :- errornl('$get_cache_dependencies'(A,B,C,D,E)),fail.
'$get_cache_dependencies'([], [], _, InSeen, InSeen).
'$get_cache_dependencies'([Body|Bodies], Depends, Head, InSeen, OutSeen) :-
    '$get_cache_dependencies_conj'(Body, BodyDepends, Head, InSeen, MidSeen),
    '$get_cache_dependencies'(Bodies, RestDepends, Head, MidSeen, OutSeen),
    append(BodyDepends, RestDepends,  Depends).

%%'$get_cache_dependencies_conj'(A,B,C,D,E) :- errornl('$get_cache_dependencies_conj'(A,B,C,D,E)),fail.
'$get_cache_dependencies_conj'((A,B), Depend, Head, InSeen, OutSeen) :-
    !,
    '$get_cache_dependencies_conj'(A, ADepend, Head, InSeen, MidSeen),
    '$get_cache_dependencies_conj'(B, BDepend, Head, MidSeen, OutSeen),
    append(ADepend, BDepend, Depend).
'$get_cache_dependencies_conj'(Goal, _, Head, _InSeen, _OutSeen) :-
    functor(Goal, F, _),
    var(F),
    '$display_error'(inference_var_functor(Goal, Head)).
'$get_cache_dependencies_conj'(Goal, _, Head, _InSeen, _OutSeen) :-
    functor(Goal, F, _),
    atom_concat(UF, '$apply', F),
    !,
    '$display_error'(inference_ho(UF, Head)).
'$get_cache_dependencies_conj'(forall(A, B), Depend, Head, InSeen, OutSeen) :-
    !,
    '$get_cache_dependencies_conj'(A, ADepend, Head, InSeen, MidSeen),
    '$get_cache_dependencies_conj'(B, BDepend, Head, MidSeen, OutSeen),
    append(ADepend, BDepend, Depend).
'$get_cache_dependencies_conj'(findall(_, B, _), Depend, Head,
                                InSeen, OutSeen) :-
    !,
    '$get_cache_dependencies_conj'(B, Depend, Head, InSeen, OutSeen).
'$get_cache_dependencies_conj'(\+(B), Depend, Head, InSeen, OutSeen) :-
    !,
    '$get_cache_dependencies_conj'(B, Depend, Head, InSeen, OutSeen).
'$get_cache_dependencies_conj'(not(B), Depend, Head, InSeen, OutSeen) :-
    !,
    '$get_cache_dependencies_conj'(B, Depend, Head, InSeen, OutSeen).
'$get_cache_dependencies_conj'(once(B), Depend, Head, InSeen, OutSeen) :-
    !,
    '$get_cache_dependencies_conj'(B, Depend, Head, InSeen, OutSeen).
'$get_cache_dependencies_conj'(B, Depend, _Head, InSeen, InSeen) :-
    ( '$percept'(B) ; '$belief'(B) ), !,
    functor(B,P,N),
    functor(Template, P, N),
    Depend = [Template].
'$get_cache_dependencies_conj'(B, [], _Head, InSeen, OutSeen) :-
    \+ \+ member(B, InSeen), !,
    OutSeen = InSeen.
'$get_cache_dependencies_conj'(B, Depend, Head, InSeen, OutSeen) :-
    cache_info__(B, RemB, _, _),
    functor(RemB,P,N),
    functor(Template, P, N),    
    !,
    findall(Body, clause(Template, Body), Clauses),
    '$get_cache_dependencies'(Clauses, Depend, Head, InSeen, OutSeen).
'$get_cache_dependencies_conj'(B, Depend, Head, InSeen, OutSeen) :-
    '$user_type_info'(_, rel, _, B, _),
    !,
    findall(Body, clause(B, Body), Clauses),
    '$get_cache_dependencies'(Clauses, Depend, Head, [B|InSeen], OutSeen).
'$get_cache_dependencies_conj'(B, Depend, Head, InSeen, OutSeen) :-
    functor(B, F, N),
    '$user_type_info'(F, fun, _, _, _),
    functor(Template, F, N),        
    !,
    findall(Body, clause(Template, Body), Clauses),
    '$get_cache_dependencies'(Clauses, Depend, Head, [Template|InSeen], OutSeen).
'$get_cache_dependencies_conj'(_B, [], _, InSeen, InSeen).


'$remove_dep_deplicates'([], []).
'$remove_dep_deplicates'([X|L1], L2) :-
	member(X, L1),
	!,
	'$remove_dep_deplicates'(L1, L2).
'$remove_dep_deplicates'([X|L1], [X|L2]) :-
	'$remove_dep_deplicates'(L1, L2).    


'$is_state_belief'(B) :-
    '$state_belief'(B).
%'$is_state_belief'(B) :-
%    '$builtin_state_belief'(B).


%% Generate the interface between code at the Qulog level and the corresponding
%% implementation at the QP level.
%% If we have a declaration like rel foo(!nat, ?nat)
%% and a fact qulog2qp_map(foo(X, Y), bar(X, Y))
%% we compile a definition for foo___qulog2qp__(X, Y) in terms of bar(X, Y)
%% wrapped in extra checks and change the above fact to
%% qulog2qp_map(foo(X, Y), foo___qulog2qp__(X, Y))
%% so that any call to foo in Qulog code is compiled to a call to
%% foo___qulog2qp__
%% If qulog2qp_wrap is declared for foo then the call is wrapped with type
%% checks.

%% Add a qulog2qp_map fact for wrapped rels/acts if non exists.
'$add_qulog2qp_map_for_wrap' :-
    qulog2qp_wrap(F),
    \+ (qulog2qp_map(QLG, _), QLG =.. [F|_]),
    '$user_type_info'(F, Kind, _, QLG, _),
    (
      Kind = rel ; Kind = act
    ->
      true
    ;
      '$display_error'(qulog2qp_wrap_kind(F))
    ),
    assert(qulog2qp_map(QLG, QLG)),
    fail.
'$add_qulog2qp_map_for_wrap'.


'$check_and_compile_qulog2qp' :-
    qulog2qp_map(QLG, QP),
    '$check_qulog2qp_map_type'(QLG, Kind),
    '$check_qulog2qp_single_type'(QLG),
    QLG =.. [F|Args],
    atom_concat(F, '__qulog2qp__', MF),
    QLGMap =.. [MF|Args],
    %'$compile_qulog2qp'(Kind, QLGMap, QP, QLG),
    retract(qulog2qp_map(QLG, QP)),
    assert(qulog2qp_map(QLG, QLGMap)),
    qulog2qp_wrap(F),
    '$compile_qulog2qp'(Kind, QLGMap, QP, QLG),
    fail.
'$check_and_compile_qulog2qp'.

%% qulog2qp_map's are only allowed for relations and actions
%%'$check_qulog2qp_map_type'(QLG, Kind) :- errornl('$check_qulog2qp_map_type'(QLG, Kind)),fail.
'$check_qulog2qp_map_type'(QLG, Kind) :-
    '$user_type_info'(_, Kind, _, QLG, _),
    (Kind = rel ; Kind = act), !,
    '$check_qulog2qp_args'(QLG).
'$check_qulog2qp_map_type'(QLG, _) :-
    '$display_error'(qulog2qp_map_kind(QLG)).

%% qulog2qp_map's are only allowed for relations/actions with a single type
%% declaration
'$check_qulog2qp_single_type'(QLG) :-
    findall(T, '$user_type_info'(_, _, T, QLG, _), Types),
    Types = [_], !.
'$check_qulog2qp_single_type'(QLG) :-
    '$display_error'(qulog2qp_multi_types(QLG)).
    
%%  The first arg of qulog2qp_map must contain only distinct variables   
'$check_qulog2qp_args'(QLG) :-
    '@..rev'(QLG, _F, Args),
    length(Args, N),
    length(VArgs, N),
    freeze_term(VArgs),
    Args = VArgs, !.
'$check_qulog2qp_args'(QLG) :-
    '$display_error'(qulog2qp_map_args(QLG)),
    fail.

%% For relations the inteface wraps the QP level call in a catch in order
%% to catch any QP exceptions and raises them as qulog exceptions.
%% If the call succeeds any necessary type checks are carried out to make
%% sure any invalid types don't end up at the qulog level.
'$compile_qulog2qp'(rel, QLGMap, QP, QLG) :-
    '$gen_qulog2qp_type_tests'(QLG, Tests),
    assert((QLGMap :- catch(QP, Pattern, '$throw_qp_exception'(Pattern)),
            Tests)).
'$compile_qulog2qp'(act, QLGMap, QP, QLG) :-
    '$gen_qulog2qp_type_tests'(QLG, Tests),
    assert((QLGMap :- catch(QP, Pattern, '$throw_qp_exception'(Pattern)),!,
            Tests)),
    assert((QLGMap :- term2string(QLG, QLGString), throw(action_failure(QLGString)))).

%% Turn a QP exception into a qp_exception  (a qulog exception)
'$throw_qp_exception'(Pattern) :-
    type(Pattern, exception), !,
    throw(Pattern).
'$throw_qp_exception'(Pattern) :-
    open_string(write, Stream),
    write(Stream, Pattern),
    stream_to_string(Stream, PatternString),
    throw(qp_exception(PatternString)).

%% generate the required tests
'$gen_qulog2qp_type_tests'(QLG, Tests) :-
    '$user_type_info'(_, _, HeadTypes, QLG, _),
    '$add_annotations'(HeadTypes, Types, '!'),
    '@..rev'(QLG, _F, Args),
    has_type_term_index_iterate__(Types, Args, 1, QLG, head, [], VTB, [],
                                  _LE, [],  _Unders, _Err1),
    (
      ground(Types)
    ->
      %% If we don't have any polymorphic types
      %% then we just need to test the ? and ?? types (and check that ? types
      %% are now ! types
      filter('$not_all_!', VTB, NonBangVTB),
      in2out__(NonBangVTB, CheckVTB),
      '$gen_qulog2qp_type_tests_aux'(CheckVTB, Tests, QLG)
    ;
      %% If we have poly types then we figure out what the poly types is
      %% from the input arguments and then use that for checking the
      %% output args
      filter('$not_all_!', VTB, NonBangVTB),
      in2out__(NonBangVTB, CheckVTB),
      diff_list( VTB, NonBangVTB, BangVTB),
      '$split_vtb'(BangVTB, BangFArgs, BangFTypes),
      '$split_vtb'(CheckVTB, FArgs, FTypes),
      
      Tests = '$qulog2qp_type_list_test'(BangFArgs, BangFTypes,
                                         FTypes, FArgs, QLG)
    ).

'$not_all_!'((_:Type)) :-
    '$not_all_!_aux'(Type).

'$not_all_!_aux'('?'(_)).
'$not_all_!_aux'('??'(_)).
'$not_all_!_aux'('!'(Type)) :-
    constructor_type__(Type, _, SubT),
    member(T, SubT), '$not_all_!_aux'(T).

'$gen_qulog2qp_type_tests_aux'([], true, _).
'$gen_qulog2qp_type_tests_aux'([V:T], '$qulog2qp_type_test'(V, T, QLG), QLG) :-
    !.
'$gen_qulog2qp_type_tests_aux'([V:T|CheckVTB],
                               ('$qulog2qp_type_test'(V, T, QLG),Tests), QLG) :-
    '$gen_qulog2qp_type_tests_aux'(CheckVTB, Tests).

'$qulog2qp_type_test'(V, T, _) :- type(V, T), !.
'$qulog2qp_type_test'(V, T, QLG) :-
    open_string(write, S),
    writeL_(S, [V, " is not of type ", T, " in ", QLG]),
    stream_to_string(S, ErrStr),
    throw(qulog2qp_type_exception(ErrStr)).

'$split_vtb'([], [], []).
'$split_vtb'([V:T|Rest], [V|VRest], [T|TRest]) :-
    '$split_vtb'(Rest, VRest, TRest).

'$qulog2qp_type_list_test'(BTypes, BArgs, Types, Args, QLG) :-
    has_type_term_index_iterate__(BTypes, BArgs, 1, QLG, head, [], _VTB, [],
                                  LE, [],  _Unders, Err1),
    check_bind_type_constraints__(LE, Err1),
    var(Err1),
    bind_type_constraints1__(LE, Err), var(Err),
    has_type_term_index_iterate__(Types, Args, 1, QLG, head, [], _, [],
                                  _, [],  _Unders, Err2),
    var(Err2),
!.
'$qulog2qp_type_list_test'(_, _, _Types, _Args, QLG) :-
    open_string(write, S),
    writeL_(S, [QLG, " is not correctly typed"]),
    stream_to_string(S, ErrStr),
    throw(qulog2qp_type_exception(ErrStr)).
    