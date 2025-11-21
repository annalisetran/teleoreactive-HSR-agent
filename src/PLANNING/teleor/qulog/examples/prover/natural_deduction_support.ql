
%%
start_proof(Formula, Status) :-
    interact_proof([ps(init_proof, [|-([], Formula)])], 1,  Status).
start_proof(_, failed).

%% All the predicates below are nondeterministic and contain actions
%% and so cannot be either Qulog relations or actions.
interact_proof(Steps, _, Status) :-
    Steps = [ps(_, [])|_], !,
    Status = completed,
    write_proof(Steps).
interact_proof(Sequents, Level, Status) :-
    get_command(Sequents, Level, Cmd),
    interact_proof_step(Cmd, Sequents, Level, Status).

interact_proof_step(quit, _, _, abandoned) :- !.
interact_proof_step(undo, _, _, _) :- !, fail.
interact_proof_step(auto, Sequents, Level, Status)  :- 
    do_auto(Sequents, NewSequents),
    Level1 is Level+1,
    interact_proof(NewSequents, Level1, Status).    
interact_proof_step(Tactic, Sequents, Level, Status)  :-
    Tactic \= auto,
    apply(Tactic, Sequents, NewSequents),
    ( true ;  write_list(["Retrying ", Tactic, nl_]), fail),
    Level1 is Level+1,
    interact_proof(NewSequents, Level1, Status).
interact_proof_step(Tactic, Sequents, Level, Status) :-
    write_list([Tactic, " failed ", nl_]),
    interact_proof(Sequents, Level, Status).

do_auto(Sequents, Sequents) :- finished(Sequents, _), !.
do_auto(Sequents, NewSequents) :-
    auto_step(Sequents, Tac, Sequents1),
    write_list(["Applying ", Tac, nl_, #Sequents1, ":", nl_]),
    Sequents1 = [ps(_, State)|_],
    write_sequents(State),
    ( true ;  write_list(["Retrying ", Tac, nl_]), fail),
    do_auto(Sequents1, NewSequents).

