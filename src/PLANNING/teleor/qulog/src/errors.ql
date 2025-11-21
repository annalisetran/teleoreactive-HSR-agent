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
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Error display
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


'$display_goal_for_error'(OG) :-
    !,
    writeL_(stderr, [show_(OG)]).



%%'$display_error'(T) :- errornl('$display_error'(T)),fail.
'$display_error'(_) :-
    writeL_(stderr, [nl_]),
    global_state_increment(num_errors, _), fail.
'$display_error'(dyn_has_rule(F, Clause, VarNames)) :-
    !,
    '$vars2names'(Clause, VarNames),
    writeL_(stderr, ["Error: ", F, " has been declared as dynamic",nl_,
                     "but the following rule has been defined", nl_,
                     Clause, nl_]),
    fail.
    
'$display_error'(inference_inference(Body, Head)) :-
    !,
    functor(Body, BF, _),
    functor(Head, HF, _),
    writeL_(stderr, ["Error: the infer and remember predicate ", BF, nl_,
                     "is referenced directly or indirectly in a rule",
                     nl_, "for the infer and remember predicate ", HF,
                     ".", nl_, "This is not allowed."]),
    fail.
'$display_error'(inference_ho(F, Head)) :-
    !,
    functor(Head, HF, _),
    writeL_(stderr, ["Error: a rule for the infer and remember predicate ",
                     HF, nl_,
                     "either directly or indirectly contains a call on the higher order function ", F, ".", nl_, "This is not allowed."]),
    fail.
'$display_error'(rem_no_dependencies(Head)) :-
    !,
    functor(Head, HF, _),
    writeL_(stderr, ["Error: the rem relation ",
                     HF, nl_,
                     "has no dependencies on the belief store ", nl_]),
    fail.
'$display_error'(no_tel_atomic) :-
    !,
     writeL_(stderr, ["Error: No tel_atomic declaration", nl_]),
     fail.
'$display_error'(qulog2qp_map_kind(QLG)) :-
    !,
     writeL_(stderr, ["Error: In ", qulog2qp_map(QLG), " ", QLG, " is neither a relation nor an action", nl_]),
     fail.
'$display_error'(qulog2qp_wrap_kind(F)) :-
    !,
     writeL_(stderr, ["Error: In ", qulog2qp_wrap(F), " ", F, " is neither a relation nor an action", nl_]),
     fail.
'$display_error'(qulog2qp_multi_types(QLG)) :-
    !,
     writeL_(stderr, ["Error: In ", qulog2qp_map(QLG), " ", QLG, " has multiple types", nl_]),
     fail.
'$display_error'(qulog2qp_map_args(QLG)) :-
    !,
     writeL_(stderr, ["Error: In ", qulog2qp_map(QLG), " the arguments of ", QLG, " are not unique variables", nl_]),
     fail.
    
'$display_error'(invalid_resources(TR, Index, NRes)) :-
    !,
    TRProg = '$tel'(TR, _Body),
    '$saved_input_with_vars'(TRProg, Vars, _),
    '$vars2names'(TRProg, Vars),
    writeL_(stderr, ["Error: Rule number ", Index, " of ",
			       TR, nl_, 
                              "contains invalid resources ", NRes, nl_]).
'$display_error'(default_args_order(Decl)) :-
    !,
    writeL_(stderr, ["Error: The declaration", nl_, Decl, nl_,
                    "does not have all default arguments at the end", nl_]),
    fail.           
'$display_error'(default_arg_not_ground(Decl, A, T)) :-
    !,
    writeL_(stderr, ["Error: The default argument ", nl_,
                    default(T, A), nl_,
                    "contains variables in", nl_, Decl, nl_]),
    fail.           
'$display_error'(default_arg_mode(Decl, A, T)) :-
    !,
    writeL_(stderr, ["Error: The default argument ", nl_,
                    default(T, A), nl_,
                    "does not have ! mode in", nl_, Decl, nl_]),
    fail.           
'$display_error'(default_arg_type(Decl, A, T)) :-
    !,
    writeL_(stderr, ["Error: The default argument ", A,
                    " does not have type ", T, " in", nl_, Decl, nl_]),
    fail.           
'$display_error'(alt_in_forall(Forall, Clause, VarNames)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, VarNames),
    writeL_(stderr, ["Error:  ", Forall,
                    " contains an alternation in", nl_, Clause, nl_]),
    fail.           
'$display_error'(tr_head_arity_error(Clause, VarNames)) :-
    !,
    Clause = '$tel'(Head, _),
    '$vars2names'(Clause, VarNames),
    writeL_(stderr, ["Arity Error: the TR definition head ", Head,
                     " has the wrong arity", nl_]),
    fail.
    
'$display_error'(arity_mismatch(Term, '$tr'(Head, Rule), N, Kind)) :-
    !,
    writeL_(stderr, ["Error: ", Term,
                    " appearing in", nl_,
                    Rule,nl_,"of the TR procedure ", Head, nl_,
                    "is declared as a ", N,
		    " argument ", Kind, nl_]),
    fail.
'$display_error'(arity_mismatch(_Term, Clause, _N, function_arg)) :-
    !,
    writeL_(stderr, ["Arity Error: ",
                    Clause,nl_,"has the wrong arity", nl_]),
    fail.
'$display_error'(arity_mismatch(Term, Clause, N, Kind)) :-
    !,
    writeL_(stderr, ["Error: ", Term,
                    " appearing in", nl_,
                    Clause,nl_,"is declared as a ", N,
		    " argument ", Kind, nl_]),
    fail.
    
'$display_error'(doc_in_multi_type_decl(Decl)) :-
    !,
     writeL_(stderr, [nl_, "The multiple declaration ", nl_,
                     Decl, nl_,
                     "has an associated doc string ", nl_]),
    fail.
   
'$display_error'(repeats_in_type_decl(Types)) :-
    !,
    writeL_(stderr, [nl_, "Error: The declaration ", nl_,
                    Types, nl_,
                    "contains repeats but is not an intersection type",
                    nl_, nl_]),
    fail.
'$display_error'(no_tel_start) :-
    !,
    writeL_(stderr, ["There are no tel_start declarations", nl_]),
    fail.
'$display_error'(tel_atomic_timeout(TR)) :-
    !,
    '$type_info'(TRName, _, _,TR, _),
    writeL_(stderr, ["Error: " , TRName,
	    " has a timeout and is outside a task atomic", nl_]),
    fail.
'$display_error'(tel_atomic_goal(TR)) :-
    !,
    '$type_info'(TRName, _, _,TR, _),
    writeL_(stderr, ["Error: " , TRName,
	    " has an attached goal and is outside a task atomic", nl_]),
    fail.
'$display_error'(tel_atomic_resource(TR, Index, atomic(TR1, Res), R)) :-
    '$type_info'(TRName, _, _,TR, _),
    !,
    '$type_info'(TR1Name, _, _,TR1, _),
    TRProg = '$tel'(TR1, Body),
    once('$index_member'(Rule, Index, Body)),
    '$saved_input_with_vars'(TRProg, Vars, _),
    '$vars2names'(TRProg, Vars),
    writeL_(stderr, ["Error: " , TRName,
                    " is called in",nl_,
                    Rule, nl_,
                    "of the TR procedure", nl_,
            TR1Name, " with resource arguments ", Res, nl_,
	    "but uses the extra resource ", R, nl_]),
    fail.
'$display_error'(tel_atomic_resource(Act, Index, atomic(TR1, Res), R)) :-
    % must be primitive
    !,
    '@functor'(Act, AF, _),
    '$type_info'(TR1Name, _, _,TR1, _),
    TRProg = '$tel'(TR1, Body),
    once('$index_member'(Rule, Index, Body)),
    '$saved_input_with_vars'(TRProg, Vars, _),
    '$vars2names'(TRProg, Vars),
    writeL_(stderr, ["Error: " , AF,
                    " is called in", nl_,
                    Rule, nl_,
                    "of the TR procedure", nl_,
                    TR1Name, " with resource arguments ", Res, nl_,
                    "but uses the extra resource ", R, nl_]),
    fail.
'$display_error'(tel_atomic_non_nil_first(TR)) :-
    !,
    '$type_info'(TRName, _, _,TR, _),
    writeL_(stderr, ["Error: The first rule for " , TRName,
	    " is not the nil action", nl_]),
    fail.
'$display_error'(non_tel_atomic_robotic(TR)) :-
    !,
    '$type_info'(TRName, _, _,TR, _),
    writeL_(stderr, ["Error: " , TRName,
	    " is not task atomic but contains a teleor action", nl_]),
    fail.


'$display_error'(invalid_macro_union_type_defn(B, T, Vars)) :-
    !,
    '$vars2names'(T, Vars),
    writeL_(stderr, ["Error: In the polymorphic macro type definition ", nl_,
                    '=='(B, T), nl_,
                    "the right hand side is a union type.", nl_,
                    "This is not allowed in a polymorphic type definition",
                    nl_]). 
'$display_error'(invalid_macro_defn(B, T, Vars)) :-
    !,
    '$vars2names'(T, Vars),
    writeL_(stderr, ["Error: ", T, " is not a type in the macro ", '=='(B, T), nl_]).
'$display_error'(only_teleor(T, Vars)) :-
    !,
    '$vars2names'(T, Vars),
    writeL_(stderr, ["Error: ", T, " is only available in the teleor extension", nl_]).
'$display_error'(builtin_redefinition(F, Kind)) :-
    !,
    '$kind2name'(Kind, KindN),
    writeL_(stderr, ["Error: ", F, " is a builtin ", KindN,
                    " and can't be redefined", nl_]),
    fail.
'$display_error'(undeclared_code(F, Kind)) :-
    !,
    %'$kind2name'(Kind, KindN),
    writeL_(stderr, ["Error: ", F, " is defined as "]),
    '$display_a_kind'(Kind),
    writeL_(stderr, [" but has no type declaration", nl_]),
    fail.

'$display_error'(globals_locals_error(Var, Kind, OG, Clause)) :-
    !,
    writeL_(stderr, ["Error: The variables ", Var, " have been quantified earlier and so can't be reused"]),
    dump_in_call__(rule_info(Kind, Clause, _, _), OG, Clause),
    dump_rule_kind__(Kind, Clause),
    writeL_(stderr, [nl_]).
'$display_error'(locals_globals_error(Vars, Kind, OG, Clause)) :-
    !,
    writeL_(stderr, ["Error: The variables ", Vars, " are quantified inside the scope of prior occurrences", nl_, "of the variables"]),
    dump_in_call__(rule_info(Kind, Clause, _, _), OG, Clause),
    dump_rule_kind__(Kind, Clause),
    writeL_(stderr, [nl_]).
'$display_error'(not_underscore_mode_error(Vars, Kind, OG, Clause)) :-
    OG = (A \= B),
    !,
    writeL_(stderr, ["Mode Error: The variables ", Vars, " are required to be quantified or be underscore variables in ", nl_]),
    '$display_goal_for_error'(OG),
    writeL_(stderr, [" (equivalent to "]),
    '$display_goal_for_error'(not(A = B)),
    writeL_(stderr, [")"]),
    '$kind2name'(Kind, KindN),
    writeL_(stderr, [nl_, "in the ", KindN, " rule", nl_,
		    show_(Clause), nl_]).
'$display_error'(not_underscore_mode_error(Vars, Kind, OG, Clause)) :-
    !,
    writeL_(stderr, ["Mode Error: The variables ", Vars, " are required to be quantified or be underscore variables "]),
    dump_in_call__(rule_info(Kind, Clause, _, _), OG, Clause),
    dump_rule_kind__(Kind, Clause),
    writeL_(stderr, [nl_]).
'$display_error'(forall_quant_vars_error(Vars, Kind, OG, Clause)) :-
    !,
    writeL_(stderr, ["Error: The variables ", Vars, " appear after (but not before) the implication of ", nl_]),
    '$display_goal_for_error'(OG),
    dump_rule_kind__(Kind, Clause),
    writeL_(stderr, [nl_]).


'$display_error'(decl_template_error(A, Vars)) :-
    !,
    '$vars2names'(A, Vars),
    writeL_(stderr, ["Error: template has the wrong number of arguments in ",
		    nl_, A, nl_]),
    fail.
'$display_error'(handle_percepts_wrong_kind) :-
    !,
    writeL_(stderr, ["Error: handle_percepts is not declared as an action ",
                    nl_]),
    fail.
'$display_error'(controls_wrong_kind) :-
    !,
    writeL_(stderr, ["Error: mod_controls is not declared as a relation ",
                    nl_]),
    fail.
'$display_error'(controls_wrong_type(Type, Vars)) :-
    !,
    thaw_term(Type),
    '$vars2names'(Type, Vars),
    writeL_(stderr, ["Error: mod_controls must be declared with type", nl_,
                    "act([control_action, control_action]) - it has been declared as type",
                    nl_, Type, nl_]),
    fail.


'$display_error'(kind_mismatch(Clause, Vars, Name, tel, no_body)) :-
    !,
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: The definition", nl_, show_(Clause), nl_,
                    "does not match the declaration", nl_]),
    display_type_desc_(stderr, Name),
    nl(stderr),
    writeL_(stderr, ["A TR definition requires a body", nl_]).
'$display_error'(kind_mismatch(Clause, Vars, Name, function, no_body)) :-
    !,
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: The definition", nl_, show_(Clause), nl_,
                    "does not match the declaration", nl_]),
    display_type_desc_(stderr, Name),
    nl(stderr),
     writeL_(stderr, ["A function body is required", nl_]).
'$display_error'(kind_mismatch(Clause, Vars, Name, action, no_body)) :-
    !,
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: The definition", nl_, show_(Clause), nl_,
                    "does not match the declaration", nl_]),
    display_type_desc_(stderr, Name),
    nl(stderr),
     writeL_(stderr, ["An action body is required", nl_]).
'$display_error'(kind_mismatch(Clause, Vars, Name, _Kind, _)) :-
    !,
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: The definition", nl_, show_(Clause), nl_,
                    "does not match the declaration", nl_]),
    display_type_desc_(stderr, Name),
    nl(stderr).
'$display_error'(tr_head_nonvar(A, Head, Vars)) :-
    !,
    thaw_term(Head),
    '$vars2names'(Head, Vars),
    writeL_(stderr, ["Error: the argument ", A, " of the head ", nl_,
		    Head,
		    " of the TR procedure is not a (unique) variable", nl_]).


'$display_error'(expression_query_mode(Kind, Vars, FVars, _OG)) :-
    !,
    '$do_unifs'(Vars),
    '$kind2name'(Kind, KindN),
    writeL_(stderr, ["Mode Error: the variables ", FVars,
                    " occur unbound in input arguments in the expression ",
                    nl_, KindN, nl_]).


'$display_error'(repeated_enum_type(TName, A)) :-
    !,
    writeL_(stderr, ["Error: ", A, " is repeated in the enumerated type ",
                    TName, nl_]),
    fail.
'$display_error'(repeated_constr_enum_type(TName, A)) :-
    !,
    writeL_(stderr, ["Error: The functor ", A,
                    " is repeated in the constructor enumerated type ",
                    TName, nl_]),
    fail.
'$display_error'(overlapping_types(TName, RName)) :-
    !,
    writeL_(stderr, ["Error: Types ", TName, " and ", RName, " overlap", nl_,
		    "Consider splitting the types into non-overlapping types and using type union", nl_]).

'$display_error'(overlapping_poly_types(TName, RName)) :-
    !,
    writeL_(stderr, ["Error: " , TName, " is polymorphic constructor enumeration. ", nl_,
                    "It overlaps with the type ", RName, nl_,
                    "Overlapping polymorphic constructor type defs are not allowed"]).

'$display_error'(invalid_type(Clause, Vars, T)) :-
    !,
    thaw_term(Clause),
   '$vars2names'(Clause, Vars),
     writeL_(stderr, ["Error: ",T, " is an invalid type in ", nl_,
		     show_(Clause),nl_]).
'$display_error'(non_finite_isa(Clause, Vars, T, Goal)) :-
    !,
    thaw_term(Clause),
   '$vars2names'(Clause, Vars),
     writeL_(stderr, ["Error: ",T, " is not a finite type in ", Goal,
		     " of", nl_, show_(Clause) ,nl_]),
     fail.
'$display_error'(enum_is_type(A, TName, Kind)) :-
    !,
    '$kind2name'(Kind, KindName),
    writeL_(stderr, ["Error: ",A, " in the enumerated type ", TName,
		   " has been declared as a ",  KindName, nl_]),
    fail.
'$display_error'(enum_is_constr_enum(A, TName, tel_action_term,
                                     Defn, Vars)) :-
    !,
    thaw_term(Defn),
   '$vars2names'(Defn, Vars),
    writeL_(stderr, ["Error: ",A, " in the enumerated type ", TName, nl_,		   "also occurs in the declaration of tel_action ", nl_]),
    fail.
'$display_error'(enum_is_constr_enum(A, TName, C, Defn, Vars)) :-
    !,
    thaw_term(Defn),
   '$vars2names'(Defn, Vars),
    writeL_(stderr, ["Error: ",A, " in the enumerated type ", TName, nl_,		   "also occurs in the constructor enumeration type ", C, nl_]),
    fail.
'$display_error'(constr_enum_is_type(A, TName, Kind, Defn, Vars)) :-
    !,
    thaw_term(Defn),
   '$vars2names'(Defn, Vars),
    '$kind2name'(Kind, KindName),
    writeL_(stderr, ["Error: ",A, " in the constructor enumeration type ", TName,
		   " has been declared as a ",  KindName, nl_]),
    fail.
'$display_error'(constr_enum_is_constr_enum(A, TName, Constr,
                                            Defn, Vars, Defn2, Vars2)) :-
    !,
    thaw_term(Defn),
   '$vars2names'(Defn, Vars),
    thaw_term(Defn2),
   '$vars2names'(Defn2, Vars2),
    writeL_(stderr, ["Error: ",A, " in the constructor enumeration type ",
		    TName, nl_,
		   "also occurs in the constructor enumeration type ", Constr,
		    nl_, "with different arguments",
		    nl_]),
    fail.
'$display_error'(constr_different_arities(A, TName, Constr,
                                          Defn, Vars, Defn2, Vars2)) :-
    !,
    thaw_term(Defn),
   '$vars2names'(Defn, Vars),
    thaw_term(Defn2),
   '$vars2names'(Defn2, Vars2),
   writeL_(stderr, ["Error: the constructor enumerator types ", TName, " and ",
                   Constr, nl_, "contain the same constructor ", A, nl_,
                   "but have different arities",  nl_]),
    fail.
    
'$display_error'(invalid_type_unmatched_vars(Clause, Vars)) :-
    !,
    thaw_term(Clause),
   '$vars2names'(Clause, Vars),
     writeL_(stderr, ["Error: The variables on the LHS of", nl_,
		     show_(Clause),nl_,
                     "don't appear on the RHS", nl_]),
     fail.
'$display_error'(invalid_constructor_type(Clause, Vars, T)) :-
    !,
    thaw_term(Clause),
   '$vars2names'(Clause, Vars),
     writeL_(stderr, ["Error: ",T,
		     " is an invalid constructor in the enumeration", nl_,
		     show_(Clause),nl_]),
     fail.
'$display_error'(invalid_atom_enum_type(Clause, Vars, T)) :-
    !,
    thaw_term(Clause),
   '$vars2names'(Clause, Vars),
     writeL_(stderr, ["Error: ",T, " is a type name in the enumeration", nl_,
		     show_(Clause),nl_]),
     fail.
'$display_error'(declaration_mode_error(MT, Name)) :-
    !,
    writeL_(stderr, ["Error: ",MT, " contains an inner mode that is more", nl_,
                    "restrictive than an outer mode in the declaration", nl_]),
    dump_declaration__(Name),
    writeL_(stderr, [nl_]),
     fail.


'$display_error'(eval_in_head(A, Clause, Vars)) :-
    !,
    thaw_term(Clause),      
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Evaluation Error: the function ", A,
                             " occurs in the head of the definition",nl_,
                             show_(Clause),nl_]),
    fail.

'$display_error'(bad_expression(Term, '$tr'(Head, Rule), Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    functor(Term, P, N),
    writeL_(stderr, ["Error: ", P/N,
                             " is undeclared in the rule", nl_]),
    write_a_tr_rule(stderr, Rule),
    writeL_(stderr, [nl_, "of the TR procedure ", Head, nl_]),
    (
      '$get_ultimate_functor'(Term, R),
      '$type_info'(R, Kind, _, _)
    ->
    '$kind2name'(Kind, KindN),
      writeL_(stderr, ["However, there is a ", KindN, " declaration for ", R, nl_])
    ;
      true
    ),
    fail.
'$display_error'(bad_expression(Term, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    functor(Term, P, N),
    writeL_(stderr, ["Error: ", P/N,
                             " is undeclared in", nl_,
                             show_(Clause),nl_]),
    (
      '$get_ultimate_functor'(Term, R),
      '$type_info'(R, Kind, _, _, _)
    ->
    '$kind2name'(Kind, KindN),
      writeL_(stderr, ["However, there is a ", KindN, " declaration for ", R, nl_])
    ;
      true
    ),
    fail.
'$display_error'(bad_show(Show, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", Show, " is not allowed in ", nl_,
		    show_(Clause), nl_]),
    fail.
    
% '$display_error'(bad_goal(Term, '$tr'(Head, Rule), Vars)) :-
%     !,
%     thaw_term((Head,Rule)),
%     '$vars2names'((Head,Rule), Vars),
%     writeL_(stderr, ["Error: ", Term,
%                              " is undeclared in rule", nl_,
%                              Rule,nl_, "of the TR procedure ", Head,nl_]),
%     (
%       '$get_ultimate_functor'(Term, R),
%       '$type_info'(R, Kind, _, RT, _),
%       functor(RT, R, N)
%     ->
%     '$kind2name'(Kind, KindN),
%       writeL_(stderr, ["However, there is a ", KindN, " declaration for ", R/N, nl_])
%     ;
%       true
%     ),
    
%     fail.
'$display_error'(bad_goal(Term, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    '$get_ultimate_functor'(Term, R),
    writeL_(stderr, ["Error: In the call ", Term, " ", R,
                             " is undeclared in", nl_,
                             show_(Clause),nl_]),
    (
      '$type_info'(R, Kind, _, RT, _),
      functor(RT, R, N)
    ->
    '$kind2name'(Kind, KindN),
      writeL_(stderr, ["However, there is a ", KindN, " declaration for ", R/N, nl_])
    ;
      true
    ),
    fail.
'$display_error'(bad_belief_goal(Term, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", Term,
                             " is not a declared dynamic relation in",
                             nl_, show_(Clause),nl_]),
    fail.
'$display_error'(bad_belief_global_goal(Term, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", Term,
                             " is a global value in", nl_,
                             show_(Clause),nl_]),
    fail.
'$display_error'(bad_tel_action_goal(Term, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", Term,
                             " is a teleor action in", nl_,
                             show_(Clause),nl_]),
    fail.
'$display_error'(bad_tel_goal(Term, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", Term,
                             " is a TR procedure in", nl_,
                             show_(Clause),nl_]),
    fail.
'$display_error'(non_action_in_tr_goal(G, '$tr'(Head, Rule), Vars)) :-
    !,
    thaw_term((Head,Rule)),
    '$vars2names'((Head,Rule), Vars),
    writeL_(stderr, ["Error: ", G,
                             " occuring after ++ is not an action in", nl_,
                             Rule,nl_, "of the TR procedure ", Head,nl_]),
    fail.
'$display_error'(non_pa_in_tr_actions(G, '$tr'(Head, Rule), Vars)) :-
    !,
    thaw_term((Head,Rule)),
    '$vars2names'((Head,Rule), Vars),
    writeL_(stderr, ["Error: ", G,
                             " occurs in the TR actions and is not a teleor action in ", nl_,
                             Rule,nl_, "of the TR procedure ", Head,nl_]),
    fail.
% '$display_error'(tr_in_tr_actions(G, '$tr'(Head, Rule), Vars)) :-
%     !,
%     thaw_term((Head,Rule)),
%     '$vars2names'((Head,Rule), Vars),
%     writeL_(stderr, ["Error: the TR procedure ", G,
%                              " is mixed with other actions in ", nl_,
%                              Rule,nl_, "of the TR procedure ", Head,nl_]),
%     fail.

'$display_error'(action_in_rel(G, '$tr'(Head, Rule), Vars)) :-
    !,
    thaw_term((Head,Rule)),
    '$vars2names'((Head,Rule), Vars),
    writeL_(stderr, ["Error: ", G,
                    " is an action call - expecting relation call in rule", nl_,
                    Rule,nl_, "of the TR procedure ", Head,nl_]),
    fail.
'$display_error'(action_in_rel(G, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", G,
                    " is an action call - expecting relation call in", nl_,
                    show_(Clause),nl_]),
    fail.
'$display_error'(expression_in_relact(G, '$tr'(Head, Rule), Vars, _RelAct)) :-
    !,
    thaw_term((Head,Rule)),
    '$vars2names'((Head,Rule), Vars),
    writeL_(stderr, ["Error: ", G,
                    " is an expression", nl_,
                    "- expecting relation or action call in rule", nl_,
                    Rule,nl_, "of the TR procedure ", Head,nl_]),
    fail.
'$display_error'(expression_in_relact(G, Clause, Vars, any)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", G,
                    " is an expression", nl_,
                    "- expecting relation or action in", nl_,
                    show_(Clause),nl_]),
    fail.
'$display_error'(expression_in_relact(G, Clause, Vars, RelAct)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    '$kind2name'(RelAct, RAName),
    writeL_(stderr, ["Error: ", G,
                    " is an expression", nl_,
                    "- expecting ", RAName, " in", nl_,
                    show_(Clause),nl_]),
    fail.
'$display_error'(rel_in_act(Call, Clause, VarNames)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, VarNames),
    writeL_(stderr, ["Error: ", Call,
                    " is a relation call - expecting action call in rule", nl_,
                    show_(Clause),nl_]),
    fail.
'$display_error'(not_a_global(A, G, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", A,
                             " is not a global value in ", G, " of",nl_,
                             show_(Clause),nl_]),
    fail.

'$display_error'(undeclared_function(Clause, Vars)) :-
    !,
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: undeclared function ", nl_,
                             show_(Clause),nl_]),
    fail.
'$display_error'(wrong_num_args(Term, '$tr'(Head, Rule), Vars)) :-
    !,
    thaw_term('$tr'(Head, Rule)),
    '$vars2names'('$tr'(Head, Rule), Vars),
    writeL_(stderr, ["Error: ", Term,
                    " has the wrong number of arguments in the rule", nl_,
		    Rule, nl_, "of the TR procedure ", Head,nl_]),
    fail.
% '$display_error'(wrong_num_args(Term, Clause, Vars)) :-
%     !,
%     thaw_term(Clause),
%     '$vars2names'(Clause, Vars),
%     writeL_(stderr, ["Error: ", Term,
%                     " has the wrong number of arguments in", nl_,
%                     show_(Clause),nl_]),
%     fail.
'$display_error'(non_atom_functor(F, Head, Clause, Vars)) :-
    !,
    thaw_term(Clause),
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", F, " of " , Head,
		    " is not an atom", nl_, "in", nl_,
		    show_(Clause),nl_]),
    fail.
'$display_error'(mismatched_function_types(Clause, Vars)) :-
    !,
    '$vars2names'(Clause, Vars),
    writeL_(stderr, ["Error: ", nl_,
		    show_(Clause),nl_, "does not match type declaration", nl_]),
    fail.
'$display_error'(already_declared(Term, Clause, Vars)) :-
    !,
    thaw_term((Clause,Vars)),
    '$vars2names'(Clause, Vars),
    '$get_ultimate_functor'(Term, F),
    writeL_(stderr, ["Error: ", Term,
                    " contains the wrong number of arguments in", nl_,
                    show_(Clause),nl_,"The type of ", F, " is", nl_]),
    display_type_(stderr, F),nl(stderr),
    
    fail.
'$display_error'(already_declared(Term,
                                  tr_action('$tr'(Head, Rule), Vars))) :-
    !,
    thaw_term((Head, Rule,Vars)),
    thaw_term('$tr'(Head, Rule)),
    '$vars2names'('$tr'(Head, Rule), Vars),
    '$get_ultimate_functor'(Term, F),
    writeL_(stderr, ["Error: ", Term,
                    " contains the wrong number of arguments in the rule",nl_,
                    Rule,nl_,
                    "of the TR procedure ", Head,nl_,
                    "The type of ", F, " is", nl_]),
    display_type_(stderr, F),nl(stderr),
     fail.
'$display_error'(re_declaration_type(A, Term, Kind, _Type)) :-
    !,
    '$kind2name'(Kind, KindN),
    writeL_(stderr, ["Error: ", nl_, Term, nl_,
                    "contains a re-declaration.", nl_,
                    "There is already a ", KindN,
                    " declaration for ", A, ":", nl_]),
    display_type_desc_(stderr, A),nl(stderr).
'$display_error'(re_declaration(A, B)) :-
    !,
    writeL_(stderr, ["Error: ", (A:B),
                    " is a re-declaration.", nl_,
                    "There is already a declaration for ", A, nl_, nl_, nl_]).
'$display_error'(re_declaration_kind(A, B, Kind)) :-
    !,
    '$kind2name'(Kind, KindN),
    writeL_(stderr, ["Error: ", B,
                    " is a re-declaration.", nl_,
                    "There is already a ", KindN,
                    " declaration for ", A, nl_]).
'$display_error'(mrel_mixed_modes(MT, Decl)) :-
    !,
    writeL_(stderr, ["Error: ", Decl, nl_,
                    "contains the mixed mode type ", MT, nl_]).

'$display_error'(noncontiguous_defn(D)) :-
    !,
    writeL_(stderr, ["Error: the rules for ", D, " are not contiguous.",nl_]),
    fail.
'$display_error'(repeated_TR_defn(TR)) :-
    !,
    writeL_(stderr, ["Error: the TR program ", TR, " has two definitions.",nl_]),
    fail.
'$display_error'(ho_default_args(F)) :-
    !,
    writeL_(stderr, ["Error: default argument in the higher order function ",
                     F, " with declaration", nl_]),
    display_type_desc_(stderr, F),
    fail.
'$display_error'(E) :-
    writeL_(stderr, ["unmatched error ", E, nl_]),fail.


    
process_non_finite_isa(Type, Goal) :-
    ip_lookup(valid_type_context, valid_type_check(Clause, Vars)),
    '$display_error'(non_finite_isa(Clause, Vars, Type, Goal)).




'$is_query_kind'(relation_query).
'$is_query_kind'(action_query).
'$is_query_kind'(expression_query).

'$kind2name'(rel, relation) :- !.
'$kind2name'(act, action) :- !.
'$kind2name'(fun, function) :- !.
'$kind2name'(tel, "teleor procedure") :- !.
'$kind2name'(defined_type, "defined type") :- !.
'$kind2name'(X, X).



%%?- global_state_set(num_errors, 0).

%%do_error_with_decl__(E, RuleInfo, Decl, Num) :- errornl(do_error_with_decl___(E, RuleInfo, Decl, Num)),fail.

do_error_with_decl__(Error, RuleInfo, Decl, Num) :-
    bind_vars__(RuleInfo),
    do_error_aux_0__(Error, RuleInfo, Decl, Num),
    %%dump_declaration__(Decl),
    fail.

do_error_aux_0__(body(body_call_error(_Call, not_le_type_test(Call, _))),
               RuleInfo, _Decl, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %get_full_kind__(Kind, FullKind),
    writeL_(stderr, ["Warning: the type test ", show_(Call), " always fails "]),
    dump_in_call__(RuleInfo, Call, Clause),
    dump_rule_kind__(Kind, Clause),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux_0__(_Error, _RuleInfo, _Decl, _Num) :-
    global_state_increment(num_errors, _), fail.
do_error_aux_0__(Error, RuleInfo, Decl, Num) :-
    do_error_aux__(Error, RuleInfo, Decl, Num).

%%do_error_aux__(Error, RuleInfo, Decl, Num) :- errornl(do_error_aux__(Error, RuleInfo, Decl, Num)),fail.
do_error_aux__(head(function_call_in_head(Term)), RuleInfo, _Decl, _) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    %get_full_kind__(Kind, FullKind),
    writeL_(stderr, ["Error: ", wr_(Term),
                    ", being a function call, is not allowed in the head of the rule ", nl_,
                     show_(Clause), nl_]).
do_error_aux__(head(head_type_mismatch), RuleInfo, _, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Error:  The rule head is not declared as "]),
    '$display_a_kind'(Kind),
    writeL_(stderr, [" in", nl_, show_(Clause), nl_]).

do_error_aux__(head(arity_error(Term, _Type)),
               RuleInfo, _Decl, _) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    functor(Term, F, _),
    writeL_(stderr,
           ["Arity Error: In ", wr_(Term), ", ", F,
            " does not have the correct arity in"]),
    writeL_(stderr,
           [nl_,"in the head of the rule",  nl_,
           show_(Clause), nl_, nl_]).
do_error_aux__(head(type_error(0, Head, Term, Type, default, _)),
               RuleInfo, Decl, _) :-
    RuleInfo = rule_info(_Kind, Clause, _, _),
    rule_head__(Clause, Head),
    !,
    writeL_(stderr, ["Type Error: ", wr_(Term),  
                    ", is not of type ",
                    wr_(Type), " in the head of the rule",  nl_,
                    show_(Clause), nl_,
                    "where", nl_]),
    dump_declaration__(Decl),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(head(type_error(Index, Head, Term, Type, default, _)),
               RuleInfo, Decl, _) :-
    RuleInfo = rule_info(_Kind, Clause, _, _),
    rule_head__(Clause, Head),
    !,
    writeL_(stderr, ["Type Error: ", wr_(Term),  ", at argument ", Index,
                    ", is not of type ",
                    wr_(Type), " in the head of the rule",  nl_,
                    show_(Clause), nl_,
                    "where", nl_]),
    dump_declaration__(Decl),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(head(type_error(Index, PTerm, Term, Type, default, _)),
               RuleInfo, Decl, _) :-
    RuleInfo = rule_info(_Kind, Clause, _, _),
    rule_head__(Clause, _Head),
    !,
    writeL_(stderr, ["Type Error: ", wr_(Term),  ", at argument ", Index,
                    " of ", wr_(PTerm),
                    ", is not of type ",
                    wr_(Type),
                    " in the head of the rule",  nl_,
                    show_(Clause), nl_,
                    "where", nl_]),
    dump_declaration__(Decl),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(head(type_error(0, _Term, Term, term, not_a_term, _)),
               RuleInfo, _Decl, _) :-
    RuleInfo = rule_info(_Kind, Clause, _, _),
    rule_head__(Clause, _Head),
    !,
    writeL_(stderr, ["Type Error: ", wr_(Term), " is not a valid constructor term (it is neither a value of a primitive type nor a program defined type) nor a function application in the head of the rule",  nl_,
                    show_(Clause)]),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(head(type_error(Index, PTerm, Term, term, not_a_term, _)),
               RuleInfo, _Decl, _) :-
    RuleInfo = rule_info(_Kind, Clause, _, _),
    rule_head__(Clause, _Head),
    !,
    writeL_(stderr, ["Type Error: ", wr_(Term),  ", at argument ", Index,
                    " of ", wr_(PTerm),
                    ", is not a valid constructor term (it is neither a value of a primitive type nor a program defined type) nor a function application in the head of the rule",  nl_,
                    show_(Clause)]),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(head(type_error(0, Term, Term, T1, head_merge_conflict(T2), _)),
               RuleInfo, Decl, _) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ", wr_(Term), " cannot be both of moded type ",
            wr_(T1), " and ", wr_(T2), nl_, 
           "in the head of the rule",  nl_,
            show_(Clause), nl_, "where", nl_]),
    dump_declaration__(Decl),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(head(mode_error(_Index, Term, Term, '@'(term),
                               repeated_at_term_mode_error)),
               RuleInfo, Decl, _) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error: ", wr_(Term),
                    " is an @ moded variable repeated in the head of the rule ",
                    nl_,
                    show_(Clause), nl_, "where", nl_]),
    dump_declaration__(Decl),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(head(mode_error(Index, _PTerm, Term, '@'(term),
                               at_term_mode_error)),
               RuleInfo, Decl, _) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error: ", wr_(Term), " at argument ", Index,
                    " is a non-variable in an @ moded position in the head of the rule ",
                    nl_,
                    show_(Clause), nl_, "where", nl_]),
    dump_declaration__(Decl),
    writeL_(stderr, [nl_, nl_]).

%% body errors
do_error_aux__(body(body_call_error(Call, Error)),
               RuleInfo, Decl, Num) :-
    act_alt_error__(Error), !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Action alternatives error :", nl_, "Execution path for error"]),
    display_alt_path__(Error, InnerError),
    writeL_(stderr, [nl_]),
    do_error_aux__(body(body_call_error(Call, InnerError)),
                   RuleInfo, Decl, Num),
    writeL_(stderr, ["in", nl_, show_(Clause)]).

do_error_aux__(body(body_call_error(Call,
                                    act_input_error(Index, PTerm, Term, _))),
               _RuleInfo, _Decl, _Num) :-
    Term \= '$$var$$'(_),
    !,
    writeL_(stderr, ["Action Output Error:  In ", show_(Call),
                    ", ", wr_(Term), " is not a variable at argument ",Index, " of ", show_(PTerm)]),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    act_input_error(Index, PTerm, Term, _))),
               _RuleInfo, _Decl, _Num) :-
    !,
    writeL_(stderr, ["Action Output Error:  In ", show_(Call),
                     ", ", wr_(Term), 
                     " is either a repeated variable or is not guaranteed to be a variable at argument ",Index, " of ", show_(PTerm)]),
    writeL_(stderr, [nl_]).
    
do_error_aux__(body(body_call_error(Call, call_error(_, mode_error(C, _)))),
               RuleInfo, Decl, _Num) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error: The functor ", wr_(C), " of the call ",
                    wr_(Call), nl_,
                    "might not be ground in the rule",  nl_,
                    show_(Clause), nl_, "where",nl_]),
    dump_declaration__(Decl).
do_error_aux__(body(body_call_error(Call, call_error(F, new_variable))),
               RuleInfo, Decl, _Num) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: The functor ",
                    wr_(F), nl_,
                    "is a new variable and so ", wr_(Call), nl_,
                    "is not of code type in the rule",  nl_,
                    show_(Clause), nl_, "where",nl_]),
    dump_declaration__(Decl).
do_error_aux__(body(body_call_error(AC(Var),
                                    new_var_allowed_call(_))),
               RuleInfo, _Decl, _Num) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error: ",
                    wr_(Var),
                     " is a new variable and so is not allowed in the call ",
                     wr_(AC(Var)), " in", nl_,
                    show_(Clause), nl_]).
do_error_aux__(body(body_call_error(Call,
                                    call_error(_, not_a_code_type(F, MT)))),
               RuleInfo, Decl, _Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    get_full_kind__(Kind, FullKind),
    writeL_(stderr, ["Type Error: The functor ", wr_(F), nl_,
                    "is of moded type ", wr_(MT), " and so ",
                    wr_(Call), nl_,
                    "is not a ",  FullKind, " call in the rule",  nl_,
                    show_(Clause), nl_, "where",nl_]),
    dump_declaration__(Decl),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(body(body_call_error(Call,
                                    call_error(_, FT, 
                                               mode_error(Index, _Call, Term,
                                                          _MT1, MT2)))),
               RuleInfo, Decl, Num) :-
    RuleInfo = rule_info(Kind, Clause, _, _),
    functor(Call, CallF, _),
    !,
    writeL_(stderr, ["Mode Error:  In ", show_(Call),
                    ", ", wr_(Term), " will have moded type ",
                    wr_(MT2), nl_,
                    "and so will not satisfy the required moded type at argument ",
                    Index, " of ", show_(Call), nl_,
                    "where ", wr_(CallF : FT)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    call_error(_, FT, Error))),
               RuleInfo, _Decl, Num) :-
    %RuleInfo = rule_info(Kind, Clause, _, _),
    functor(Call, CallF, _),
    !,
    do_error_aux__(body(body_call_error(Call, Error)),
                   RuleInfo, (CallF:FT), Num).
do_error_aux__(body(body_call_error(Call,
                                    kind_error(_, rel, Kind2))),
               RuleInfo, _Decl, _Num) :-
    RuleInfo = rule_info(Kind, Clause, _, _),
    Kind \= rel, !,
    get_full_kind__(Kind, FullKind),
    get_full_kind__(Kind2, FullKind2),
    writeL_(stderr, ["Error: The ", FullKind2, " call ",
                    wr_(Call), nl_,
                    "is being used in relation part of the ",
                    FullKind, " rule", nl_, show_(Clause), nl_]).
do_error_aux__(body(body_call_error(Call,
                                    kind_error(_, Kind1, Kind2))),
               RuleInfo, _Decl, _Num) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    get_full_kind__(Kind1, FullKind1),
    get_full_kind__(Kind2, FullKind2),
    writeL_(stderr, ["Error: The ", FullKind2, " call ",
                    wr_(Call), nl_,
                    "is being used in the body of the ",  FullKind1, " rule",
                    nl_,
                    show_(Clause), nl_]).
do_error_aux__(body(body_call_error(_,
                                    mode_error(0, saved, Term,
                                               Type1, Type2))),
               RuleInfo, Decl, _Num) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error: At the end of the rule, ", wr_(Term),
                    " will be of moded type ",
                    wr_(Type2), nl_, "but must be of moded type ",
                    wr_(Type1), nl_,
                    "in the rule",  nl_,
                    show_(Clause), nl_, "where",nl_]),
    dump_declaration__(Decl).
do_error_aux__(body(body_call_error(Call, unify_new_var(_V1, _V2))),
               RuleInfo, Decl, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Error: the unification of two variables where at least one is new is not allowed"]),
    dump_in_call__(RuleInfo, Call, Clause),
    dump_rule_kind__(Kind, Clause),
    dump_call_declaration__(Call, Decl),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call, unify_contains_at_term_var(V))),
               RuleInfo, Decl, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Error: the unification ", show_(Call),
                    " contains the @ moded variable ", wr_(V),nl_,
                    "that might be instantiated"]),
    dump_in_call__(RuleInfo, Call, Clause),
    dump_rule_kind__(Kind, Clause),
    dump_call_declaration__(Call, Decl),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(_Call, type_error(0, _Term, Term, term,
                                                    not_a_term, _))),
               RuleInfo, _Decl, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ", wr_(Term), " is not a valid constructor term (it is neither a value of a primitive type nor a program defined type) nor a function application "]),
    dump_rule_kind__(Kind, Clause),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(_Call, type_error(Index, PTerm, Term, term,
                                                    not_a_term, _))),
               RuleInfo, _Decl, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ", wr_(Term),  ", at argument ", Index,
                    " of ", wr_(PTerm),
                    ", is not a valid constructor term (it is neither a value of a primitive type nor a program defined type) nor a function application "]),
    dump_rule_kind__(Kind, Clause),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call, type_error(0, Term, Term, T1,
                                                     merge_conflict(T2), _))),
               RuleInfo, Decl, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ", wr_(Term), " cannot be both of moded type ",
                    wr_(T1), " and ", wr_(T2)]),
    dump_in_call__(RuleInfo, Call, Clause),
    dump_rule_kind__(Kind, Clause),
    writeL_(stderr, ["where", nl_]),
    dump_declaration__(Decl),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call, body_merge_conflict(Term, T1, T2, _))),
               RuleInfo, _Decl, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ", wr_(Term), " cannot be both of moded type ",
                    wr_(T1), " and ", wr_(T2)]),
    dump_in_call__(RuleInfo, Call, Clause),
    dump_rule_kind__(Kind, Clause),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call, arity_error(Term, _Type))),
               RuleInfo, _Decl, _) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    functor(Term, F, _),
    writeL_(stderr,
           ["Arity Error: In ", wr_(Term), ", ", F, 
            " does not have the correct arity"]),
    dump_in_call__(RuleInfo, Call, Clause),
    dump_rule_kind__(Kind, Clause),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(_Call,
                                    mode_error(Index, Call, Term,
                                               _MT1, dyn_term_mode_error))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    writeL_(stderr, ["Mode Error:  In ", show_(Call), ", ",
                    wr_(Term), nl_,
                    "will not be sufficiently instantiated",
                    nl_, "at argument ", Index, " of ",
                    show_(Call)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).

do_error_aux__(body(body_call_error(_, new_var_error(Call, new_var(Var)))),
               RuleInfo, _Decl, _Num) :-
    !,
    RuleInfo = rule_info(_, Clause, _, _),
    writeL_(stderr, ["Error: ", Var,
                     " is a new variable and so can't be ground in ", Call,nl_,
                     "of the rule",
                     nl_,show_(Clause), nl_]).
do_error_aux__(body(body_call_error(_Call,
                                    mode_error(Index, Call, Term,
                                               _MT1, new_variable))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl), 
    writeL_(stderr, ["Mode Error:  In ", show_(Call), ", ",
                    wr_(Term),
                    " is a new variable", nl_,
                    "and so will not satisfy the required moded type",
                    nl_, "at argument ", Index, " of ",
                    show_(Call)]),
    dump_call_declaration__(dummy, CallDecl),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    mode_error(_Index, Patt, Var,
                                               _MT1, new_variable_compr))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error:  In the comprehension pattern ", wr_(Patt),
                    " of ", nl_,show_(Call), ",", nl_,
                    wr_(Var),
                     " will not be ground"]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    mode_error(Index, _PTerm, Term, _MT,
                                               body_at_term_mode_error))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl), 
    writeL_(stderr, ["Mode Error:  In ", show_(Call),
                    ", ", wr_(Term), " has moded type @term",
                     nl_,
                    "but may be instantiated at argument ",
                    Index, " of ", Call]),
    dump_call_declaration__(Call, CallDecl),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    mode_error(_Index, _Call, Term, _, _MT2))),
               RuleInfo, Decl, Num) :-
    Call = '$remote_query'(_,_),
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    '$get_decl'(Call, CallDecl), 
    writeL_(stderr, ["Mode Error:  In ", show_(Call),
                    ", ", nl_, wr_(Term), " might not be ground at the end of the query"]),
    dump_call_declaration__(Call, CallDecl),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).

do_error_aux__(body(body_call_error(Call,
                                    mode_error(Index, _Call, Term, _MT1,
                                               non_ground_var_functor(_F)))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl), 
    writeL_(stderr, ["Mode Error:  In ", show_(Call),
                    ", ", wr_(Term), " has a non-ground functor at argument ",
                    Index, " of ", show_(Call)]),
    dump_call_declaration__(Call, CallDecl),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    mode_error(0, _Call, Term, _MT1, MT2))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl), 
    writeL_(stderr, ["Mode Error:  In ", show_(Call),
                    ", ", wr_(Term), " will have moded type ",
                    wr_(MT2), nl_,
                    "and so will not satisfy the required moded type in ", 
                    show_(Call)]),
    dump_call_declaration__(Call, CallDecl),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    mode_error(Index, _Call, Term, _MT1, MT2))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl), 
    writeL_(stderr, ["Mode Error:  In ", show_(Call),
                    ", ", wr_(Term), " will have moded type ",
                    wr_(MT2), nl_,
                    "and so will not satisfy the required moded type at argument ",
                    Index, " of ", show_(Call)]),
    dump_call_declaration__(Call, CallDecl),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error((A=B),
                                    unify_unify_err(_, _,
                                                    unify_always_fails(X, Y)))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
               writeL_(stderr, ["Error: In ", show_((A=B)), " the unification ",
                               show_((X=Y)),
                              " always fails"]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error((A=B),unify_always_fails(X, Y))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
               writeL_(stderr, ["Error: In ", show_((A=B)), " the unification ",
                               show_((X=Y)),
                              " always fails"]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error((A=B),
                                    type_error(0, Term, Term, T2,
                                               conflict(T1), _))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: In the unification ", show_((A=B)), " the type ", wr_(T1), " for ", wr_(Term),
                     nl_,
                    "is incompatible with its earlier type ", wr_(T2)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(PTerm,
                                    type_error(0, PTerm,
                                               Term, Type, default, _))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ", Term,
                    " does not match the required type ", wr_(Type)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(_Call,
                                    type_error(0, Term, Term, T2,
                                               conflict(T1), _))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: The type ", wr_(T2), " for ", wr_(Term),
                     nl_,
                    "is incompatible with its earlier type ", wr_(T1)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(_Call,
                                    type_error(Index, PTerm, Term, T2,
                                               conflict(T1), _))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: The type ", wr_(T2), " for ", wr_(Term),
                    " at argument ", Index, " of ", wr_(PTerm), nl_,
                    "is incompatible with its earlier type ", wr_(T1)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,  type_error(0, _, Term, T1,
                                                      unify_conflict(T2), _))),
               RuleInfo, _Decl, _Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: The unification ", show_(Call),nl_,
                    "produces a type conflict (",
                     wr_(T1), " and ", wr_(T2),
                    ") for ", wr_(Term)]),
    dump_rule_kind__(Kind, Clause),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(_Call,  unify_err(Var, _, _,
                                                     type_error(Index, PTerm,
                                                                _, _, _, _)))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(PTerm, F),
    writeL_(stderr, ["Type Error: In ", show_(PTerm),
                    ", after the earlier unifications, ", nl_, wr_(Var) ,
                    " will not satisfy the required moded type at argument ",
                    Index, " of ", PTerm, " where",  nl_]),
    display_call_type__(stderr, PTerm),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call, 
                                    type_error(0, _, Call,
                                               no_suitable_moded_type,
                                               multidecl, _))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: There are no suitable moded types for the arguments of ",
                    show_(Call), nl_,
                     "with declaration", nl_]),
    dump_alt_declarations__(Call),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call, 
                                    type_error(0, _, Term,
                                               no_suitable_moded_type,
                                               _, _))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ",
                    wr_(Term), 
                    " does not match any of the types required in ",
                    show_(Call), nl_,
                     "with declaration", nl_]),
    dump_alt_declarations__(Term),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(_Call, 
                                    type_error(_, X, Term,
                                               no_suitable_moded_type,
                                               multidecl, _))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ",
                    wr_(X), 
                    " does not match any of the types required in ",
                    wr_(Term), nl_,
                     "with declaration", nl_]),
    dump_alt_declarations__(Term),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    type_error(_Index, _Call,
                                               Term, no_suitable_moded_type,
                                               unify_var_term_error, DecI))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    '$get_decl'(Call, CallDecl, DecI), 
    writeL_(stderr, ["Type Error: In ", show_(Call), ", ", Term,
                    " does not match any of the required types"]),
    dump_call_declaration__(Call, CallDecl),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    type_error(_Index, _Call,
                                               Term, Type,
                                               unify_var_term_error, DecI))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    '$get_decl'(Call, CallDecl, DecI), 
    writeL_(stderr, ["Type Error: In ", show_(Call), ", ", Term,
                    " does not match the required type ", wr_(Type)]),
    dump_call_declaration__(Call, CallDecl),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    type_error(Index, Call,
                                               Term, no_suitable_moded_type,
                                               default, DecI))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl, DecI), 
    writeL_(stderr, ["Type Error: In ", show_(Call), ", ", Term,
                    " does not match any of the required types ", nl_,
                    "at argument ", Index, " of ",
                    wr_(Call)]),
    dump_call_declaration__(Call, CallDecl, DecI),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    type_error(Index, Call,
                                               Term, Type, default, DecI))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl, DecI), 
    writeL_(stderr, ["Type Error: In ", show_(Call), ", ", Term,
                    " does not match the required type ", wr_(Type), nl_,
                    "at argument ", Index, " of ",
                    wr_(Call)]),
    dump_call_declaration__(Call, CallDecl, DecI),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error('tr~>'(_, _),
                                    type_error(Index, Call,
                                               Term, no_suitable_moded_type,
                                               default, DecI))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl, DecI), 
    writeL_(stderr, ["Type Error: In ", show_(Call), ", ", Term,
                    " does not match any of the required types", nl_,
                    "at argument ", Index, " of ",
                    wr_(Call)]),
    dump_call_declaration__(Call, CallDecl, DecI),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error('tr~>'(_, _),
                                    type_error(Index, Call,
                                               Term, Type, default, DecI))),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(Call, CallF),
    '$get_decl'(Call, CallDecl, DecI), 
    writeL_(stderr, ["Type Error: In ", show_(Call), ", ", Term,
                    " does not match the required type ", wr_(Type), nl_,
                    "at argument ", Index, " of ",
                    wr_(Call)]),
    dump_call_declaration__(Call, CallDecl, DecI),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    type_error(Index, PTerm,
                                               Term, no_suitable_moded_type,
                                               default, _))) ,
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(PTerm, PTermF),
    writeL_(stderr, ["Type Error: ", Term,
                    " does not match any of the required types ", nl_,
                    "at argument ", Index, " of ",
                    wr_(PTerm)]),
    display_compound_type__(stderr, PTerm),
    dump_in_call__(RuleInfo, Call, Clause),
    %%writeL_(stderr, [nl_, "in ", show_(Call)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    type_error(Index, PTerm,
                                               Term, Type, default, _))) ,
               RuleInfo, Decl, Num) :-
    PTerm = '$catch'(_, _, _),
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(PTerm, PTermF),
    writeL_(stderr, ["Type Error: ", Term,
                    " does not match the required type ", wr_(Type), nl_,
                     "at argument ", Index, " of ", nl_]),
    '$show_except_alt'(PTerm, 0, 4, stderr),
    dump_in_call__(RuleInfo, Call, Clause),
    %%writeL_(stderr, [nl_, "in ", show_(Call)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(Call,
                                    type_error(Index, PTerm,
                                               Term, Type, default, _))) ,
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    %%'$get_ultimate_functor'(PTerm, PTermF),
    writeL_(stderr, ["Type Error: ", Term,
                    " does not match the required type ", wr_(Type), nl_,
                    "at argument ", Index, " of ",
                    wr_(PTerm)]),
    display_compound_type__(stderr, PTerm),
    dump_in_call__(RuleInfo, Call, Clause),
    %%writeL_(stderr, [nl_, "in ", show_(Call)]),
    dump_rule_kind__(Kind, Clause),
    dump_declaration__(Decl, Num),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(body(body_call_error(_,
                                    non_ground_call(Vars, Call))),
               RuleInfo, _Decl, _Num) :-
    !,
    RuleInfo = rule_info(Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error: the variables ", Vars, " in ", show_(Call),
                    nl_, "will not be ground at the time of call"]),
    dump_rule_kind__(Kind, Clause),
    dump_tr_head__(RuleInfo),
    writeL_(stderr, [nl_]).
do_error_aux__(saved(mode_error(0, _, Term, Type1, Type2)),
               RuleInfo, Decl, _Num) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error: At the end of the rule, ", wr_(Term),
                    " will be of moded type ",
                    wr_(Type2), nl_, "but must be of moded type ",
                    wr_(Type1), nl_,
                    "in the rule",  nl_,
                    show_(Clause), nl_, "where",nl_]),
    dump_declaration__(Decl).



%%% function errors
do_error_aux__(fun_result(mode_error(_Index, PTerm, Term, Type, _)),
               RuleInfo, Decl, Num) :-
    PTerm == Term,
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Mode Error: ", wr_(Term), " is not of moded type ", Type,
                    " in the result of the function rule", nl_,
                    show_(Clause), nl_]),
    dump_declaration__(Decl, Num),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(fun_result(mode_error(Index, PTerm, Term, Type, _)),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    %%'$get_ultimate_functor'(PTerm, PTermF),
    writeL_(stderr, ["Mode Error: ", wr_(Term), " is not of moded type ",
                    Type, nl_,
                    "at argument ", Index, " of ", PTerm,
                    " in the result of the function rule", nl_,
                    show_(Clause), nl_]),
    dump_declaration__(Decl, Num),
    writeL_(stderr, [nl_, nl_]).
  do_error_aux__(fun_result(type_error(_Index, PTerm, Term, Type, _, _)),
               RuleInfo, Decl, Num) :-
    PTerm == Term,
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    writeL_(stderr, ["Type Error: ", wr_(Term), " is not of type ", Type,
                    " in the result of the function rule", nl_,
                    show_(Clause), nl_]),
    dump_declaration__(Decl, Num),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(fun_result(type_error(Index, PTerm, Term, Type, _, _)),
               RuleInfo, Decl, Num) :-
    !,
    RuleInfo = rule_info(_Kind, Clause, _, _),
    %%'$get_ultimate_functor'(PTerm, PTermF),
    writeL_(stderr, ["Type Error: ", wr_(Term), " is not of type ", Type, nl_,
                    "at argument ", Index, " of ", PTerm,
                    " in the result of the function rule", nl_,
                    show_(Clause), nl_]),
    dump_declaration__(Decl, Num),
    writeL_(stderr, [nl_, nl_]).
do_error_aux__(Error, RuleInfo, Decl, Num) :-
    errornl(unprocessed_error(Error, RuleInfo, Decl, Num)).

%%bind_vars__(RI) :- errornl(bind_vars__(RI)),fail.
bind_vars__(rule_info(_, Clause, Vars, _)) :-
    '$set_var_names'(Clause, Vars, _Thawed).

dump_in_call__(rule_info(tel, tel(_,Call), _, _), Call, _) :- !.
dump_in_call__(_, Call, Clause) :- Call == Clause, !.
dump_in_call__(_, Call, _) :-
    writeL_(stderr, [nl_, "in ", show_(Call)]).

dump_tr_head__(rule_info(tel, tel(Head, _), _, _)) :-
    writeL_(stderr, ["in the TR procedure ", show_(Head)]).
dump_tr_head__(_).

%%dump_rule_kind__(A, B) :- errornl(dump_rule_kind__(A, B)),fail.
dump_rule_kind__(rel_query, Clause) :-
    !,
    writeL_(stderr, [nl_, "in the relation query",  nl_,
                    show_(Clause), nl_]).
dump_rule_kind__(act_query, Clause) :-
    !,
    writeL_(stderr, [nl_, "in the action query",  nl_,
                    show_(Clause), nl_]).
dump_rule_kind__(tel, tel(_, Clause)) :-
    !,
    writeL_(stderr, [nl_, "in the TR rule",  nl_,
                    show_(Clause), nl_]).
dump_rule_kind__(_, Clause) :-
    !,
    writeL_(stderr, [nl_, "in the rule",  nl_,
                    show_(Clause), nl_]).

dump_call_declaration__((_ = _), _Decl) :- !.
dump_call_declaration__(dummy, set) :- !.
dump_call_declaration__('$forall'(_, _, _), _Decl) :- !.
dump_call_declaration__('$remote_query'(_, _), _) :- !.
dump_call_declaration__('$forall_actions'(_, _, _), _Decl) :- !.
dump_call_declaration__('$catch'(_,_,_), _) :- !.
dump_call_declaration__(_, '$var_functor') :- !.
dump_call_declaration__(_Call, Decl) :-
    writeL_(stderr, [", where", nl_]),
    dump_declaration__(Decl).

dump_call_declaration__(Call, Decl, DecI) :-
    var(DecI), !,
    dump_call_declaration__(Call, Decl).
dump_call_declaration__(_Call, Decl, _DecI) :-
     writeL_(stderr, [", using the applicable type declaration", nl_]),
     dump_declaration__(Decl).

%%dump_declaration__(D) :- errornl(dump_declaration__(D)),fail.
dump_declaration__(fun((F(Dom) -> Ran))) :-
    !,
    writeL_(stderr, [F, " : ", (Dom -> Ran)]).
dump_declaration__(_Kind(F((Dom -> Ran)))) :-
    !,
    writeL_(stderr, [F, " : ", (Dom -> Ran)]).
dump_declaration__(Kind(F(Args))) :-
    '@..'(F, Args, D),
    writeL_(stderr, [Kind, " ", D]).

dump_declaration__(_Decl, single) :- !.
dump_declaration__((_ = _), _) :- !.
dump_declaration__(Decl, _) :-
    writeL_(stderr, ["for rule declaration",nl_]),
    dump_declaration__(Decl).

dump_alt_declarations__(Term) :-
    '$get_ultimate_functor'(Term, P),
    once('$type_descr'(P, _, Kind, _)),
    findall(Type,
            ('$type_descr'(P, Type, _, _)), AllT),
    (
      AllT = [T]
    ->
      writeL_(stderr, [Kind, " ", T])
    ;
      AllT = [T|Rest],
      writeL_(stderr, [Kind, " ", T, ",",nl_]),
      '$display_alt_types'(stderr, Rest)
    ).



get_full_kind__(rel, relation).
get_full_kind__(act, action).
get_full_kind__(fun, function).
get_full_kind__(tel, teleor).

display_call_type__(Stream, Call) :-
    '$get_ultimate_functor'(Call, F),
    once('$type_descr'(F, _, Kind, _)),
    findall(Type,
            ('$type_descr'(F, Type, _, _)), AllT),
    (
      AllT = [T]
    ->
      writeL_(Stream, [Kind, " ", T])
    ;
      AllT = [T|Rest],
      writeL_(Stream, [Kind, " ", T, ",",nl_]),
      '$display_alt_types'(Stream, Rest)
    ).

display_compound_type__(Stream, Term) :-
    '$get_ultimate_functor'(Term, C),
    '$type_info'(_, defined_type, Type, '$constr_enum_type'(Enum), _),
    member(E, Enum),
    functor(E, C, _), !,
    writeL_(stderr, [" where", nl_]),
    display_constructor_type__(Stream, Type).
display_compound_type__(Stream, Term) :-
    '$get_ultimate_functor'(Term, F),
    '$type_info'(F, fun, _, _, _), !,
    writeL_(stderr, [" where", nl_]),
    display_call_type__(Stream, Term).
display_compound_type__(_Stream, _Term).    
    
display_constructor_type__(Stream, Type) :-
    '$type_info'(_, defined_type, Type, Enum, Vars),
    Defn = '::='(Type,Enum),
    '$vars2names'(Defn, Vars),
    write(Stream, 'def '),
    writeTerm_(Stream, Defn).


act_alt_error__(try_error(_, _)).
act_alt_error__(except_error(_, _)).
act_alt_error__(case_error(_, _)).
act_alt_error__(wait_case_error(_, _)).
act_alt_error__(receive_error(_, _)).

%%display_alt_path__(A,B) :- errornl(display_alt_path__(A,B)),fail.
display_alt_path__(try_error(_, Error), InnerError) :-
    !,
    writeL_(stderr, [" : try"]),
    display_alt_path__(Error, InnerError).
display_alt_path__(except_error('$catch'(Patt, '$empty', _), Error),
                   InnerError) :-
    !,
    writeL_(stderr, [" : except ", wr_(Patt)]),
    display_alt_path__(Error, InnerError).
display_alt_path__(except_error('$catch'(Patt, Test, _), Error),
                   InnerError) :-
    !,
    writeL_(stderr, [" : except ", wr_(Patt), "::", show_(Test)]),
    display_alt_path__(Error, InnerError).
display_alt_path__(case_error('$case_alt'(Test, _), Error),
                   InnerError) :-
    !,
    writeL_(stderr, [" : case ", show_(Test)]),
    display_alt_path__(Error, InnerError).
display_alt_path__(wait_case_error('$case_alt'(Test, _), Error),
                   InnerError) :-
    !,
    writeL_(stderr, [" : wait_case ", show_(Test)]),
    display_alt_path__(Error, InnerError).
display_alt_path__(wait_case_error(timeout(_, _), Error),
                   InnerError) :-
    !,
    writeL_(stderr, [": wait_case timeout"]),
    display_alt_path__(Error, InnerError).
display_alt_path__(receive_error('$normal'('$receive_from'(M, from(A),
                                                           '$empty', _)),
                                 Error),
                   InnerError) :-
    !,
    writeL_(stderr, [" : receive ", wr_(M), " from ", wr_(A)]),
    display_alt_path__(Error, InnerError).
display_alt_path__(receive_error('$normal'('$receive_from'(M, from_thread(A),
                                                           '$empty', _)),
                                 Error),
                   InnerError) :-
    !,
    writeL_(stderr, [" : receive ", wr_(M), " from_thread ", wr_(A)]),
    display_alt_path__(Error, InnerError).
display_alt_path__(receive_error('$normal'('$receive_from'(M, from(A),
                                                           Test, _)),
                                 Error),
                   InnerError) :-
    !,
    writeL_(stderr, [" : receive ", wr_(M), " from ", wr_(A), " :: ",
                    show_(Test)]),
    display_alt_path__(Error, InnerError).
display_alt_path__(receive_error('$normal'('$receive_from'(M, from_thread(A),
                                                           Test, _)),
                                 Error),
                   InnerError) :-
    !,
    writeL_(stderr, [" : receive ", wr_(M), " from_thread ", wr_(A), " :: ",
                    show_(Test)]),
    display_alt_path__(Error, InnerError).
display_alt_path__(receive_error(timeout(_, _), Error),
                   InnerError) :-
    !,
    writeL_(stderr, [": receive timeout"]),
    display_alt_path__(Error, InnerError).
display_alt_path__(Error, Error).


'$get_decl'(Call, Decl) :-
    '$get_decl'(Call, Decl, _).

'$get_decl'(Call, '$var_functor', _) :-
    compound(Call),
    functor(Call, F, _),
    F = '$$var$$'(_), !.
'$get_decl'(remember(_), Decl, _) :- !,
    Decl = act(remember(['!'(list('!'(dyn_term)))])).
'$get_decl'('$remote_query'(A,B), Decl, _) :- !,
    Decl = '$remote_query'(A,B).
'$get_decl'((_A = _B), Decl, _) :- !,
    Decl = rel(=(['??'(term), '??'(term)])).
'$get_decl'(ground(_Term), Decl, _) :- !,
    Decl = rel(ground(['??'(term)])).
'$get_decl'('$set_enum'(_L), Decl, _) :- !,
    Decl = set.
'$get_decl'(Call, Decl, _) :-
    '$get_ultimate_functor'(Call, F),
    '$type_info'(F, fun, Types, _, VarNames), !,
    '$set_var_names'(Types, VarNames, _),
    Types = (D -> R),
    Decl = fun((F(D) -> R)).
'$get_decl'(Call, Decl, DecIndex) :-
    integer(DecIndex), !,
    '$get_ultimate_functor'(Call, F),
    findall(DecTypes, '$type_info'(F, _, DecTypes, _, _), AllDecTypes),
    get_item_at_index__(1, DecIndex, AllDecTypes, Types),
    once('$type_info'(F, Kind, Types, _, VarNames)),
    '$set_var_names'(Types, VarNames, _),
    '$add_annotations'(Types, ATypes, '!'),
    Decl = Kind(F(ATypes)).    
'$get_decl'(Call, Decl, _) :-
    '$get_ultimate_functor'(Call, F),
    '$type_info'(F, Kind, Types, _, VarNames),
    '$set_var_names'(Types, VarNames, _),
    '$add_annotations'(Types, ATypes, '!'),
    Decl = Kind(F(ATypes)), !.

%%get_item_at_index__(N, I, Lst, V) :- errornl(get_item_at_index__(N, I, Lst, V)),fail.
get_item_at_index__(N, I, Lst, V) :-
    N = I, !, Lst = [V|_].
get_item_at_index__(N, I, [_|Lst], V) :-
    N1 is N+1,
    get_item_at_index__(N1, I, Lst, V).

'$display_a_kind'(rel) :-
    writeL_(stderr, ["a relation"]).
'$display_a_kind'(act) :-
    writeL_(stderr, ["an action"]).
'$display_a_kind'(fun) :-
    writeL_(stderr, ["a function"]).
'$display_a_kind'(tel) :-
    writeL_(stderr, ["a TR procedure"]).

'$display_warning'(non_allowed_action_in_tr_goal(G, '$tr'(Head, Rule), Vars)) :-
    thaw_term((Head,Rule)),
    '$vars2names'((Head,Rule), Vars),
    functor(G, GF, _),
    writeL_(stderr, ["Warning: A call to disallowed QuLog action '", GF ,
                     "' occurs directly or indirectly after ++ in", nl_,
                             Rule,nl_, "of the TR procedure ", Head,nl_]),
    fail.
'$display_warning'(_).