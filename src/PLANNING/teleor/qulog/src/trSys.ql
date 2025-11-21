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

?- dynamic('__running__'/1, 0).
?- dynamic('__waiting__'/1, 0).
?- dynamic('__resources__'/2, 0).
?- dynamic('__time__'/1, 0).
?- dynamic('__prev_time__'/1, 0).
?- dynamic('$delayed_tr_call'/1, 0).
?- dynamic('$agent_info'/2).
?- dynamic(overlapping_resources_/2).
?- dynamic(logger_name_/1).
?- dynamic(agent_suspended_/0).

?- dynamic('__percept_interface__'/1).

?- dynamic('$should_kill'/1).
?- dynamic('$percept_thread_running'/0).

?- dynamic('$can_run_handlers'/0).

?- dynamic(new_task_/2).
?- dynamic(end_task_/1).

?- dynamic(end_agent_/0).

'$delay_tr_call'(G) :-
    assert('$delayed_tr_call'(G)).

get_attached_actions_(Actions) :-
    '$delayed_tr_call'(_), !,
    findall(G, retract('$delayed_tr_call'(G)), Actions).
get_attached_actions_([]).


%% If task atomic is not declared an we have multiple tasks then we use all_
%% as the collection of resources used.
all_resources_used_(R) :-
    '$tel_atomic'(_), !,
    R = no.
all_resources_used_(yes).


%% upate the resources needed by Task to Res
'$update_resources'(Task , Res) :-
    '__resources__'(Task, Res), !.
'$update_resources'(Task, Res) :-
    retractall('__resources__'(Task, _)),
    assert('__resources__'(Task, Res)).


    
%%'__can_run__'(Task, Resources) :- ip_lookup('$processed', Processed),log_list(['__can_run__'(Task, Resources, Processed), nl_]) ,'__waiting__'(Task1), log_list([waiting___(Task1),nl_]), fail.
%% Check to see if Task, running TR, that needs Resources can become a
%% running task.
%% Note: this is called (as a negated test) in task atomic after first rule
%% is tested - failure of can_run causes the task to wait
'__can_run__'(Task, Resources) :-
    %% save away required resources
    '$update_resources'(Task, Resources),
    ip_lookup('$processed', Processed),
    %% Processed is the list of all running and waiting processes that have
    %% been processed. If Task is a non-running task then Processed includes
    %% all the waiting tasks earlier than Task in the wait queue (or Task
    %% is not yet added to the wait queue)
    '__can_run__aux'(Task, Resources, Processed).

%% If already running then can run
'__can_run__aux'(Task, _Resources, _) :-
    '__running__'(Task), !. %%,log_list([Task, " running",nl_]).
%% Task is not running or waiting - this occurs when Task was running
%% and the re-eval jumps out of the top-level task atomic - Task becomes
%% tentatively waiting and is to be rechecked when the tasks in the wait queue
%% have been checked if they can become running before this Task is checked
'__can_run__aux'(Task, _Resources, _) :-
    \+ '__waiting__'(Task), !,
    %%log_list([Task, " pending",nl_]),
    fail.
%% If Task is in the wait queue and its resources overlap with any running tasks
%% or tasks earlier in the wait queue (i.e. those in Pending)
'__can_run__aux'(Task, Resources, Processed) :-
    member(T, Processed),
    '__resources__'(T, TRes),
    overlapping_resources_interface_(TRes, Resources), !,
    (
      '__waiting__'(Task)
    ->
      true
    ;
      assert('__waiting__'(Task))
    ), fail.
%% Task does not overlap with resources in running or waiting tasks earlier
%% in the wait queue so becomes running
'__can_run__aux'(Task, _Resources, _Processed) :-
    %%log_list([Task, " becomes running",nl_]),
    retractall('__waiting__'(Task)), 
    assert('__running__'(Task)).
 


%% If all is used then resources always overlap
default_overlapping_resources(all, _) :- !.
default_overlapping_resources(_, all) :- !.
%% Overlap if there is a non-empty intersection
default_overlapping_resources(Res1,Res2) :-
   member(R,Res1),
   member(R,Res2),!.
%% If the user defines overlapping_resources then this is also used to determine
%% if there is an overlap
%%overlap_res_(Res1,Res2) :-
%%    overlapping_resources_interface_(Res1, Res2), !.
%%overlap_res_(Res1,Res2) :-
%%    overlapping_resources_interface_(Res2, Res1), !.


%% Times is a list of times, Now is the current time. MinTimeout is the earliest
%% time in the future (relative to Now). 1e20 represents (here and elsewhere)
%% a far distant future time.
min_timeout_(Now, Times, MinTimeout) :-
    get_min_time_(Times, Now, 1e20, MinTimeout).


get_min_time_([], _, M, M).
get_min_time_([T|Rest], Now, InMin, Min) :-
    T >= Now, T < InMin, !,
    get_min_time_(Rest, Now, T, Min).
get_min_time_([_|Rest], Now, InMin, Min) :-
    get_min_time_(Rest, Now, InMin, Min).


%% This is called in non-task atomic TRs and for the first rule of a
%% task atomic when the Tr rule is either a refiring or an initial
%% firing

%% check_if_should_clean_up_(A, B) :- log_list([check_if_should_clean_up_(A,B), nl_]), fail.
check_if_should_clean_up_(yes, _) :- !.
check_if_should_clean_up_(_, none) :- !.
%% cleanup required
check_if_should_clean_up_(_, _) :-
    this_task_name(Task),
    '$update_resources'(Task, []),
    (
      retract('__running__'(Task))
    ->
      %% the task was running - becomes tentatively waiting
      %% gets retried after waiting tasks get tried
      true
    ;
      '__waiting__'(Task)
    ->
      %% already waiting - do nothing
      true
    ;
      %% was tentatively waiting - becomes waiting
      assert('__waiting__'(Task))
    ).

%% The eval thread is responsible for re-evaluating the call stacks of each
%% agent task in an appropriate order.
%% The eval thread maintains a list of task info that is used for call stack
%% evaluation. Because it uses destructive updates for efficiency the list
%% needs to start with a sentinal value so the tail can be
%% destructively updated.

start_task_eval_thread_ :-
    errornl("Starting task eval thread ..."),
    (
      '$tel_atomic'(_)
    ->
      %% the tasks need to be retried in running and then waiting queue order
      %% eval_resources_handler_
      thread_fork(eval_handler_, eval_resources_handler_([sentinal], -1))
    ;
      %% when no resources are used the tasks are re-evaluated in the
      %% order they appear in the task info list
      thread_fork(eval_handler_, eval_no_resources_handler_([sentinal], -1))
    ),
    thread_yield,
    errornl("Task eval thread started").

%% The eval thread is woken if the timestamp on the belief store has changed
%% since the last task eval
bs_changed_ :-
    '__prev_time__'(PrevT),
    '__time__'(T),
    PrevT < T.

%% When all the task call stacks have been re-evaluated the timestamp on the
%% belief store is reset and any ++ actions are called. Note: these actions
%% might include a BS update and so happens after the timestamp reset so
%% that the eval thread will kick in again.
update_time_do_actions_(Actions) :-
    '__time__'(T),
    retract('__prev_time__'(_)),
    assert('__prev_time__'(T)),
    forall(member(Call, Actions), call(Call)).

%% For logging messages from the task evaluator thread
log_eval_thread_message_(Terms) :-
    open_string(write, Stream),
    writeL_(Stream, ["Task evaluator ::: "|Terms]),
    send_log_message_(Stream).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% eval handler when no resources/thread_atomic are defined
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%eval_no_resources_handler_(TaskInfo, D) :- errornl(eval_no_resources_handler_(TaskInfo, D)),fail.
eval_no_resources_handler_(TaskInfo, -1) :- !, %% no delays
    thread_wait_on_goal(bs_changed_, [wait_preds(['__time__'/1])]),
    %% We first check if the agent has been suspended and if so wait until it
    %% it is resumed before evaluating the call stacks
    check_if_should_suspend_,
    thread_atomic_goal(eval_tasks_bs_(TaskInfo, Delay, 1e20)),
    %% give other threads a chance to run
    ( end_agent_ -> kill_agent(_); true),
    thread_yield,
    eval_no_resources_handler_(TaskInfo, Delay).
eval_no_resources_handler_(TaskInfo, Delay) :-
    thread_wait_on_goal(bs_changed_,
                        [wait_preds(['__time__'/1]), wait_for(Delay)]),
    !,                          % timeout didn't happen - BS changed
    check_if_should_suspend_,
    thread_atomic_goal(eval_tasks_(TaskInfo, NewDelay, 1e20)),
    ( end_agent_ -> kill_agent(_); true),
    thread_yield,
    eval_no_resources_handler_(TaskInfo, NewDelay).
eval_no_resources_handler_(TaskInfo, _Delay) :-
    check_if_should_suspend_,
    gettimeofday(Now),
    thread_atomic_goal(eval_tasks_tout_(TaskInfo, NewDelay, Now)),
    ( end_agent_ -> kill_agent(_); true),
    thread_yield,
    eval_no_resources_handler_(TaskInfo, NewDelay).

check_if_should_suspend_ :-
    agent_suspended_,
    !,
    suspend_eval_.
check_if_should_suspend_.

suspend_eval_ :-
    log_eval_thread_message_(["Agent suspended", nl_]),
    thread_wait_on_goal(\+agent_suspended_),
    log_eval_thread_message_(["Agent resumed", nl_]).
    
send_stop_action_(Name) :-
    ip_set('$task_name', Name),
    controls_([]).


eval_tasks_bs_(TaskInfo, Delay, Time) :-
    log_eval_thread_message_(["Evaluator wakes up because of change to the belief store", nl_]),
    '$display_beliefs',
    eval_tasks_(TaskInfo, Delay, Time).
eval_tasks_tout_(TaskInfo, Delay, Time) :-
    log_eval_thread_message_(["Evaluator wakes up because of timeout", nl_]),
    eval_tasks_(TaskInfo, Delay, Time).

%%eval_tasks_(TaskInfo, Delay,Time) :- errornl(eval_tasks_(TaskInfo, Delay, Time)), fail.
%% eval_tasks_(TaskInfo, Delay, Time):
%% TaskInfo is the list of info about tasks,
%% Delay is instantiated to the minimum delay for all of the tasks (or -1
%% if no tasks have a timeout
%% Time is either 1e20 if the evaluator was not woken up by a timeout or
%% is the current time otherwise
%% Throughout processing we use the implicit parameter '$task_name'
%% to store the current task name of the task being processed. This is
%% used when sending action messages the task name is included in the message
%% term : actions(TaskName, ActionList)

eval_tasks_(TaskInfo, Delay, Time) :-
    process_start_tasks_(TaskInfo),
    process_end_tasks_(TaskInfo),
    TaskInfo = [_sentinal|TaskInfoRest],
    gettimeofday(Now),
    thread_atomic_goal(eval_tasks_aux_(TaskInfoRest, Timeout, Now, Time)),
    get_attached_actions_(Actions),
    update_time_do_actions_(Actions),
    get_delay_(Now, Timeout, Delay).

%% When a new task is created new_task(TaskName, TR) is asserted.
%% If there are new tasks this predicate updates TaskInfo with the info
%% for the new task
process_start_tasks_(TaskInfo) :-
    new_task_(_, _), !,
    findall(new_task_(Name, TR), retract(new_task_(Name, TR)), NewTasks),
    process_start_tasks_aux_(NewTasks, TaskInfo).
process_start_tasks_(_).

%%process_start_tasks_aux_(A,B) :- errornl(process_start_tasks_aux_(A,B)),fail.
process_start_tasks_aux_([], _TaskInfo).
process_start_tasks_aux_([new_task_(Name, TR)|NewTasks], TaskInfo) :-
    assert(task(Name,TR)),
    %% Initialize the task info for the new task. Note: we don't need
    %% to eval the call stack here as it will be done later when all the
    %% call stacks are evaluated.
    LastTerm = rule_choice(TR, none, [], no, _),
    TermSeq = [sentinal, LastTerm],
    TaskI = task_info(Name, TermSeq, LastTerm, 1e20),
    %% Update TaskInfo with this new task info
    TaskInfo = [_sentinal|TaskInfoRest],
    UpdatedInfo = [TaskI|TaskInfoRest],
    setarg(2, TaskInfo, UpdatedInfo),
    (
      '$tel_atomic'(_)
    ->
      assert('__waiting__'(Name)),
      '$update_resources'(Name, [])
    ;
      true
    ),
    process_start_tasks_aux_(NewTasks, TaskInfo).

%% When a task is to be deleted and_task_(TaskName) is asserted.
%% If there are tasks to be deleted this predicate updates TaskInfo
process_end_tasks_(TaskInfo) :-
    end_task_(_), !,
    findall(end_task_(Name), retract(end_task_(Name)), NewTasks),
    process_end_tasks_aux_(NewTasks, TaskInfo).
process_end_tasks_(_).

%%process_end_tasks_aux_(A,B) :- errornl(process_end_tasks_aux_(A,B)),fail.
process_end_tasks_aux_([], _TaskInfo).
process_end_tasks_aux_([end_task_(Name)|EndTasks], TaskInfo) :-
    retract(task(Name, _)),
    TaskInfo = [_sentinal|TaskInfoRest],
    remove_task_(Name, TaskInfoRest, TaskInfo, TermSeq),
    (
      '$tel_atomic'(_)
    ->
      retractall('__running__'(Task)),
      retractall('__waiting__'(Task)),
      retractall('__resources__'(Task, _))
    ;
      true
    ),
    %% stop all actions related to this task
    ip_set('$task_name', Task),
    do_action_([], TermSeq),
    process_end_tasks_aux_(EndTasks, TaskInfo).
    
remove_task_(Name, [task_info(Name, TermSeq, _, _)|Rest], TaskInfo, TermSeq) :-
    !,
    setarg(2, TaskInfo, Rest).
remove_task_(Name, [_|Rest], [_|TaskInfo], TermSeq) :-
    remove_task_(Name, Rest, TaskInfo, TermSeq).

get_delay_(_, 1e20, -1) :- !.
get_delay_(Now, Timeout, Delay) :-
    Delay is max(0, Timeout - Now).

%%eval_tasks_aux_(A,B,C,D) :- errornl(eval_tasks_aux_(A,B,C,D)),fail.
eval_tasks_aux_([], 1e20, _, _). % no timeout
eval_tasks_aux_([TaskI|TaskInfo], Timeout, Now, Time) :-
    TaskI = task_info(Task, TermSeq, LastTerm, LastTimeout),
    LastTimeout =< Time, 
    ip_set('$task_name', Task),
    catch(process_call_stack_(no_retry, TermSeq, LastTerm,
                              NewLastTerm,TTimeout, Now),
          ErrorTerm, process_error_(ErrorTerm, Task)),
    !, %% process_call_stack_ succeeds so change to call stack
    setarg(3, TaskI, NewLastTerm),
    setarg(4, TaskI, TTimeout),
    eval_tasks_aux_(TaskInfo, Timeout1, Now, Time),
    Timeout is min(TTimeout, Timeout1).
eval_tasks_aux_([TaskI|TaskInfo], Timeout, Now, Time) :-
    %% no change to call stack so no change to task_info, use same timeout
    TaskI = task_info(Task, _TermSeq, _LastTerm, TTimeout),
    ip_set('$task_name', Task),
    log_eval_thread_message_(["No change to call stack", nl_]),
    eval_tasks_aux_(TaskInfo, Timeout1, Now, Time),
    Timeout is min(TTimeout, Timeout1).

%%process_call_stack_(Retry, TermSeq, LastTerm, NewLastTerm,TTimeout, Now) :- errornl(process_call_stack_(Retry, TermSeq, LastTerm, NewLastTerm,TTimeout, Now)),fail.
%% process_call_stack_ fails if there is no change to the call stack
%% the first arg is used to determine if we are doing a retry because a running
%% task has become waiting - we do a simpler call stack reeval in this case
process_call_stack_(no_retry, TermSeq, _LastTerm, NewLastTerm, Timeout, Now) :-
    TermSeq = [_|StrippedSeq],
    update_call_chain_aux_(StrippedSeq, TermSeq, NewLastTerm,
                           NewActions, Timeout, Now),
    !,
    do_action_(NewActions, TermSeq).
process_call_stack_(retry, TermSeq, _LastTerm, NewLastTerm, Timeout, Now) :-
    TermSeq = [_|StrippedSeq],
    update_call_chain_aux_waiting_(StrippedSeq, TermSeq, NewLastTerm,
                           NewActions, Timeout, Now),
    !,
    do_action_(NewActions, TermSeq).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% eval handler when resources/thread_atomic are defined
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%eval_resources_handler_(TaskInfo, D) :- errornl(eval_resources_handler_(TaskInfo, D)),fail.
eval_resources_handler_(TaskInfo, -1) :- !, %% no delays
    log_eval_timeout_(Delay),
    thread_wait_on_goal(bs_changed_, [wait_preds(['__time__'/1])]),
    check_if_should_suspend_,
    thread_atomic_goal(eval_resources_tasks_bs_(TaskInfo, Delay, 1e20)),
    ( end_agent_ -> kill_agent(_); true),
    log_resources_,
    resources_callback,
    thread_yield,
    eval_resources_handler_(TaskInfo, Delay).
eval_resources_handler_(TaskInfo, Delay) :-
    log_eval_timeout_(Delay),
    thread_wait_on_goal(bs_changed_,
                        [wait_preds(['__time__'/1]), wait_for(Delay)]),
    !, % timeout didn't happen - BS changed
    check_if_should_suspend_,
    thread_atomic_goal(eval_resources_tasks_bs_(TaskInfo, NewDelay, 1e20)),
    ( end_agent_ -> kill_agent(_); true),
    log_resources_,
    thread_yield,
    eval_resources_handler_(TaskInfo, NewDelay).
eval_resources_handler_(TaskInfo, _Delay) :-
    check_if_should_suspend_,
    gettimeofday(Now),
    thread_atomic_goal(eval_resources_tasks_tout_(TaskInfo, NewDelay, Now)),
    ( end_agent_ -> kill_agent(_); true),
    log_resources_,
    thread_yield,
    eval_resources_handler_(TaskInfo, NewDelay).

log_eval_timeout_(-1) :- !.
log_eval_timeout_(Delay) :-
    '$logging',!,
    open_string(write, Stream),
    gettimeofday(Now),
    '$start_time'(T1),
    DelayTill is Now - T1 + Delay,
    writeL_(Stream, [nl_, "Next call stack evaluation time event at ",
                     DelayTill,nl_]). 
log_eval_timeout_(_).

eval_resources_tasks_bs_(TaskInfo, Delay, Time) :-
    log_eval_thread_message_(["Evaluator wakes up because of change to the belief store", nl_]),
    '$display_beliefs',
    eval_resources_tasks_(TaskInfo, Delay, Time).
eval_resources_tasks_tout_(TaskInfo, Delay, Time) :-
    log_eval_thread_message_(["Evaluator wakes up because of timeout", nl_]),
    eval_resources_tasks_(TaskInfo, Delay, Time).

%% Similar to eval_no_resources_tasks_ except that the order of evaluation
%% is important: first the running tasks then the waiting tasks in wait queue
%% order
%%eval_resources_tasks_(TaskInfo, Delay, Time) :- errornl(eval_resources_tasks_(TaskInfo, Delay, Time)),fail.
eval_resources_tasks_(TaskInfo, Delay, Time) :-
    process_start_tasks_(TaskInfo),
    process_end_tasks_(TaskInfo),
    TaskInfo = [_sentinal|TaskInfoRest],
    gettimeofday(Now),
    thread_atomic_goal(eval_resources_tasks_aux_0_(TaskInfoRest, Timeout,
                                                   Now, Time)),
    get_attached_actions_(Actions),
    update_time_do_actions_(Actions),
    get_delay_(Now, Timeout, Delay).


eval_resources_tasks_aux_0_(TaskInfo, Timeout, Now, Time) :-
    findall(RTask, '__running__'(RTask), RunningTasks),
    findall(WTask, '__waiting__'(WTask), WaitingTasks),
    eval_resources_tasks_aux_(RunningTasks, WaitingTasks, [], [],
                              TaskInfo, Timeout, Now, Time).

%% eval_resources_tasks_aux_(Running, Waiting, Retry, Processed,
%%                           TaskInfo, Timeout, Now, Time)
%% Running is the list of running tasks,
%% Waiting is the list of waiting tasks in queue order
%% Retry is the list of tasks that were running but become waiting ready
%%   for retrying
%% Processed is the list of tasks that have already been processed - this is
%% used to find out if there are any overlapping resources with
%% tasks that have already been processing - to prevent "queue jumping"
%% Timeout is instantiated to the minimum timeout is the call stack.
%%eval_resources_tasks_aux_(A,B,C,D,E,F,G,H) :- errornl(eval_resources_tasks_aux_(A,B,C,D,E,F,G,H)),fail.
eval_resources_tasks_aux_([], [], [], _, _, 1e20, _, _) :- !. 
eval_resources_tasks_aux_([], [], [Task|Rest], Processed,
                          TaskInfo, Timeout, Now, _Time) :-
    %% retry a task that was running and becomes waiting
    member(TaskI, TaskInfo), TaskI = task_info(Task, _, _, _), !,
    ip_set('$processed', Processed),
    eval_resources_tasks_aux_2_(TaskI, TTimeout, Now, retry, 1e20, _),
    eval_resources_tasks_aux_([], [], Rest, [Task|Processed],
                              TaskInfo, Timeout1, Now, 1e20),
    Timeout is min(TTimeout, Timeout1).
eval_resources_tasks_aux_([], [Task|Rest], ReEval, Processed,
                          TaskInfo, Timeout, Now, Time) :-
    %% reeval the next task in the wait queue
    member(TaskI, TaskInfo), TaskI = task_info(Task, _, _, _), !,
    ip_set('$processed', Processed),
    eval_resources_tasks_aux_2_(TaskI, TTimeout, Now, no_retry, Time, OutTime),
    eval_resources_tasks_aux_([], Rest, ReEval, [Task|Processed],
                              TaskInfo, Timeout1, Now, OutTime),
    Timeout is min(TTimeout, Timeout1).
eval_resources_tasks_aux_([Task|Rest], Waiting, ReEval, Processed,
                          TaskInfo, Timeout, Now, Time) :-
    %% reeval the "next" running task
    member(TaskI, TaskInfo), TaskI = task_info(Task, _, _, _), !,
    ip_set('$processed', Processed),
    eval_resources_tasks_aux_2_(TaskI, TTimeout, Now, no_retry, Time, OutTime),
    (
      '$not_running_or_waiting'(Task)
    ->
      %% the task becomes waiting
      assert('__waiting__'(Task)),
      eval_resources_tasks_aux_(Rest, Waiting, [Task|ReEval], Processed,
                                TaskInfo, Timeout1, Now, OutTime)
    ;
      eval_resources_tasks_aux_(Rest, Waiting, ReEval, [Task|Processed],
                                TaskInfo, Timeout1, Now, OutTime)
    ),
    Timeout is min(TTimeout, Timeout1).


eval_resources_tasks_aux_2_(TaskI, TTimeout, Now, Retry, Time, 1e20) :-
    TaskI = task_info(Task, TermSeq, LastTerm, LastTimeout),
    LastTimeout =< Time, 
    ip_set('$task_name', Task),
    process_call_stack_(Retry, TermSeq, LastTerm, NewLastTerm,TTimeout, Now),
    !,
    setarg(3, TaskI, NewLastTerm),
    setarg(4, TaskI, TTimeout).
eval_resources_tasks_aux_2_(TaskI, TTimeout, _Now, _, Time, Time) :-
    %% no change - same timeout
    TaskI = task_info(_Task, _TermSeq, _LastTerm, TTimeout).


'$not_running_or_waiting'(Task) :-
    '__running__'(Task), !, fail.
'$not_running_or_waiting'(Task) :-
    '__waiting__'(Task), !, fail.
'$not_running_or_waiting'(_).

    

%% update_call_chain_aux_(TermSeq, Trailing, LastTerm,
%%                        NewActions, Delay, Now)
%% TermSeq is the call stack, Trailing is such that Trailing = [_|TermSeq]
%% This is so the tail of Trailing (which becomes the new extension of the
%% call stack can be destructively updated

%%update_call_chain_aux_(A,B,C,D,E,F) :- errornl(update_call_chain_aux_(A,B,C,D,E,F)),fail.
update_call_chain_aux_([], _TermSeq, _, _Actions, _Delay, _Now) :-
    !,
    %% no changes
    fail.
update_call_chain_aux_([Term|_TermSeq], Trailing, NewLastTerm,
                       NewActions, Timeout, Now) :-
    update_call_chain_aux_term_(Term, NewTermSeq, NewLastTerm, NewActions,
                               Now, Timeout),
    %% Term on the call stack is updated to NewTermSeq and so the call
    %% stack from term on needs to be replaced by NewTermSeq
    '$set_refire'(Term, NewTermSeq),
    setarg(2, Trailing, NewTermSeq),
    !.
update_call_chain_aux_([Term|TermSeq], [_|Trailing], LastTerm,
                       NewActions, Timeout, Now) :-
    update_call_chain_aux_(TermSeq, Trailing, LastTerm,
                           NewActions, Timeout, Now),
    '$set_to_unchanged'(Term).

%% This is purely for logging so we can see what part of the call stack is
%% continuing and where it changes to a new rule choice.
%'$set_to_unchanged'(rule_choice(_, _, _, _, true)) :- !.
'$set_to_unchanged'(RuleChoice) :-
    setarg(5, RuleChoice, true).

'$set_refire'(Term, NewTermSeq) :-
    '$logging',
    NewTermSeq = [rule_choice(NewCall, rid(NewRuleNum, _, _), _, _, Cont)|_],
    Term = rule_choice(Call, rid(RuleNum, _, _), _, _, _),
    Call = NewCall, RuleNum = NewRuleNum, !,
    Cont = refire.
'$set_refire'(_, _).

%% This is the same as above but is used for re-re-evaluation - i.e.
%% when a re-evaluation causes the task to change from running to waiting.
%% After all other task call stacks are re-evaluated and we re-re-evaluate
%% in case the resources being used have changed in such a way that this
%% task can become running. Since the BS has not changed since the re-eval
%% then we don't need to re-evaluate every term in the call stack - only the
%% last which will be a wait_choice term
%%update_call_chain_aux_waiting_(A,B,C,D,E,F) :- errornl(update_call_chain_aux_waiting_(A,B,C,D,E,F)),fail.
update_call_chain_aux_waiting_([], _TermSeq, _, _Actions, _Delay, _Now) :-
    !,
    %% no changes
    fail.
update_call_chain_aux_waiting_([Term|_TermSeq], Trailing,
                               NewLastTerm, NewActions, Delay, Now) :-
    Term = wait_choice(_TR),
    update_call_chain_aux_term_(Term, NewTermSeq, NewLastTerm, NewActions,
                                Now, Delay),
    setarg(2, Trailing, NewTermSeq),
    !.
update_call_chain_aux_waiting_([Term|TermSeq], [_|Trailing], LastTerm,
                       NewActions, Delay, Now) :-
    update_call_chain_aux_waiting_(TermSeq, Trailing, LastTerm,
                           NewActions, Delay, Now),
    '$set_to_unchanged'(Term).


%%update_call_chain_aux_term_(A,B,C,D,E,F) :- errornl(update_call_chain_aux_term_(A,B,C,D,E,F)),fail.
%% The entry is a timed sequence - has the timeout expired?
update_call_chain_aux_term_(timed_seq(Timeout, S, All, Time, InResources),
                            NewTermSeq, LastTerm, NewActions,
                            Now, OutTimeout) :-
    Timeout < Now,
    %% timeout expired - 
    %% get the next element in the timed sequence
    update_timed_seq_(S, All, S1), 
    S1 = [TR:T|_],
    update_call_chain_TS_(T, Now, S1, All, Time, OutTime,
                         InResources,  Term),
    update_call_chain_aux2_(TR, Term, RestTermSeq, LastTerm,
                           OutTime, Now, OutTimeout,
                           NewActions, InResources),
    NewTermSeq = [Term|RestTermSeq].     
%% re-eval TR - was waiting
update_call_chain_aux_term_(wait_choice(TR), NewTermSeq, LastTerm,
                           NewActions, Now, Timeout) :-
    %% The update is at a wait_choice
    tr_rule(TR, wait, Now, NewTermSeq, LastTerm, [], Timeout,  NewActions, no).
%% re-eval TR
update_call_chain_aux_term_(rule_choice(TR,  RID, InTimes, ResourcesUsed, _),
                           NewTermSeq, LastTerm, NewActions, Now, Timeout) :-
    tr_rule(TR, RID, Now, NewTermSeq, LastTerm, InTimes, Timeout,
             NewActions, ResourcesUsed).


%%update_call_chain_TS_(A,B,C,D,E,F,G,H) :- errornl(update_call_chain_TS_(A,B,C,D,E,F,G,H)),fail.
%% at the last TS element with -1 as time - i.e. no timeout
update_call_chain_TS_(-1, _Now, S1, All, Times, OutTimes,
                     InResources,  NewTerm) :-
    !,
    Times = OutTimes,
    %% 1e20 is so far into the future that it's not a viable timeout
    NewTerm = timed_seq(1e20, S1, All, Times, InResources).
update_call_chain_TS_(T, Now, S1, All, Times, OutTimes,
                     InResources, NewTerm) :-
    TSNow = Now + T,
    OutTimes = [TSNow|Times],
    NewTerm = timed_seq(TSNow, S1, All, Times, InResources).

%%update_call_chain_aux2_(A,B,C,D,E,F,G,H,I) :- errornl(update_call_chain_aux2_(A,B,C,D,E,F,G,H,I)),fail.

update_call_chain_aux2_(TR, Term, NewTermSeq, Term, 
                       InTimes, Now,  Timeout,
                        Actions, _InResources) :-
      TR = pa(Actions), !,
      %% the timed sequence element is a primitive action
      NewTermSeq = [],
      min_timeout_(Now, InTimes, Timeout).
update_call_chain_aux2_(TR, _, NewTermSeq, LastTerm, 
                       InTimes, Now, Timeout,
                       Actions, InResources) :-
    %% otherwise a TR so update the call stack
    %% note if multi-tasking then the timed sequence must be inside
    %% a task atomic and therefore is running
    tr_rule(TR, none, Now, NewTermSeq, LastTerm,
            InTimes, Timeout,  Actions, InResources).


'$display_beliefs' :-
    '$start_time'(T1),
    '__time__'(T),
    Delta is T - T1,
    display_beliefs_(Delta).


%% update the list containing the remaining elements in timed sequence
update_timed_seq_(S, All, S1) :-
    S = [_], !, S1 = All.
update_timed_seq_(S, _All, S1) :-
    S = [_|S1].


do_action_(Actions, CallList) :-
    ip_lookup('$task_name', Task),
    %% log the call stack and actions for Task
    log_call_stack_and_action_(Task, CallList, Actions),
    %% Send off Actions to controller
    controls_(Actions),
    !.  % to eliminate choice points

process_error_(ErrorTerm, Task) :-
    assert(end_agent_),
    display_task_error_(ErrorTerm, Task),
    fail.

display_task_error_(no_matching_tr_rule(Prog), Task) :-
    !,
    writeL_(stderr, ["No matching rule for ", Prog,
                             " - task ", Task, " terminated.", nl_]),
    log_error_(error(Task, no_matching_tr_rule, Prog)).

display_task_error_(ErrorTerm, Task) :-
    write_term_list(stderr, ['Unhandled error ',
                             ErrorTerm,
                             ' - task ', Task, ' terminated.', nl]),
    log_error_(error(Task, ErrorTerm)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% logging

?- dynamic('$logging'/0).

%% all log messages are created by writing terms to a string stream
%% this predicate extracts the string representing the log message from Stream
%% and sends it to the logger
send_log_message_(Stream) :-
    logger_name_(Name), !,
    stream_to_string(Stream, String),
    String ->> Name.
%% if a logger had not been declared then simply close the stream
send_log_message_(Stream) :-
    close(Stream).


%% log messages from agents - called in handle_message_interface
log_message_(message(M, A)) :-
    '$logging', !,
    open_string(write, Stream),
    writeL_(Stream, ["Received message from ", A, " :", nl_, M]),
    send_log_message_(Stream).
log_message_(_).


%% for logging task error messages
log_error_(T) :-
    %%do_log(errors),!,
    open_string(write, Stream),
    writeL_(Stream, [T]),
    send_log_message_(Stream).
%%log_error_(_).

%% For agents with resources logs the resources being used (or waited on)
log_resources_ :-
    '$logging',
    once(( '__running__'(_) ; '__waiting__'(_) )),
     !,
    open_string(write, Stream),
    thread_atomic_goal(
      (
        writeL_(Stream, ["Tasks using resources:", nl_]),
        forall('__running__'(Task),'$write_resources'(Task, Stream)),
        write(Stream, 'Tasks waiting on resources:\n'),
        forall('__waiting__'(Task), '$write_resources'(Task, Stream))
       )
     ),
    send_log_message_(Stream).
    
log_resources_.

'$write_resources'(Task, Stream) :-
    '__resources__'(Task, Res),
    (
     Res = [all__]
    ->
     writeTerm_(Stream, Task-all), nl(Stream)
    ;
     writeTerm_(Stream, Task-Res), nl(Stream)
    ).

%%log_call_stack_and_action_(Task, CallList, Actions) :- errornl(log_call_stack_and_action_(Task, CallList, Actions)),fail.


%% Log the call stack and actions
log_call_stack_and_action_(Task, [_|Chain], Actions) :-
    '$logging',!,
    open_string(write, Stream),
    writeL_(Stream, [Task, ": New call stack evaluation:", nl_]),
    %% print the call stack starting with an indent of 0 and bind D to
    %% the last indent
    log_action_chain_(Chain, 0, D, continue, Stream),
    D1 is D+1,
    writeL_(Stream, [sp_(D1), Actions, nl_]),
    writeL_(Stream, [nl_, "Robotic actions for ",Task," are: ",Actions, nl_]),
    findall(A, '$delayed_tr_call'(A), QA),
    (
      QA = []
    ->
      true
    ;
      writeL_(Stream, ["Qulog actions: ", QA,nl_])
    ),
    send_log_message_(Stream).
log_call_stack_and_action_(_, _D, _).

log_manual_actions_(Controls) :-
    '$logging',!,
    open_string(write, Stream),
    writeL_(Stream, [nl_, "Robotic actions are: ",Controls, nl_]),
    send_log_message_(Stream).
log_manual_actions_( _Controls).

%%log_action_chain_(A,B,C,D,E) :- errornl(log_action_chain_(A,B,C,D,E)),fail.
log_action_chain_([], D, D, _, _).
log_action_chain_([no_matching_tr_rule(TR)|_Rest], D, D, _, Stream) :-
    !,
    writeL_(Stream, [sp_(D), TR, "- no matching rule", nl_]).
log_action_chain_([timed_seq(1e20, This, A, _, _)|Rest],
                 InD, OutD, Continue, Stream) :-
    !,
    length(This, ThisLen),
    length(A, ALen),
    Index is ALen - ThisLen + 1,
    writeL_(Stream, [sp_(InD), "["]),
    '$write_timed_seq_elems'(Stream, A),
    writeL_(Stream, ["] TS action ", Index,nl_]),
    MidD is InD + 1,
    log_action_chain_(Rest, MidD, OutD, Continue, Stream).
log_action_chain_([timed_seq((_ + Delay), This, A, _, _)|Rest],
                 InD, OutD, Continue, Stream) :-
    !,
    '$start_time'(T1),
    gettimeofday(T),
    NextTime is T - T1 + Delay,
    length(This, ThisLen),
    length(A, ALen),
    Index is ALen - ThisLen + 1,
    writeL_(Stream, [sp_(InD), "["]),
    '$write_timed_seq_elems'(Stream, A),
    writeL_(Stream, ["] TS action ", Index, " next TS action at ", NextTime,nl_]),
    MidD is InD + 1,
    log_action_chain_(Rest, MidD, OutD, Continue, Stream).
log_action_chain_([rule_choice(A, rid(I,_,_), _, _, New)|Rest],
                 InD, OutD, Continue, Stream) :-
    New == refire, Continue = continue, !,
    %% This is a new firing and if continue then this is the
    %% start of the new part of the call stack and it's a refire
    writeL_(Stream, [sp_(InD), "********************", nl_]),
    writeL_(Stream, [sp_(InD), A-I, " refired",nl_]),
    MidD is InD + 1,
    log_action_chain_(Rest, MidD, OutD, not_continued, Stream).
log_action_chain_([rule_choice(A, rid(I,_,_), _, _, New)|Rest],
                 InD, OutD, Continue, Stream) :-
    var(New), Continue = continue, !,
    %% var New means this is a new firing and if continue then this is the
    %% start of the new part of the call stack
    writeL_(Stream, [sp_(InD), "********************", nl_]),
    writeL_(Stream, [sp_(InD), A-I, " fired",nl_]),
    MidD is InD + 1,
    log_action_chain_(Rest, MidD, OutD, not_continued, Stream).
log_action_chain_([rule_choice(A, rid(I,_,_), _, _, New)|Rest],
                 InD, OutD, Continue, Stream) :-
    var(New),  !,
    %% a new firing but not the first one in the call stack
    writeL_(Stream, [sp_(InD), A-I, " fired",nl_]),
    MidD is InD + 1,
    log_action_chain_(Rest, MidD, OutD, Continue, Stream).
log_action_chain_([rule_choice(A, rid(I,_,_), _, _, _New)|Rest],
                 InD, OutD, Continue, Stream) :-
    Continue = not_continued,  !,
    %% a new firing but not the first one in the call stack
    writeL_(Stream, [sp_(InD), A-I, " fired",nl_]),
    MidD is InD + 1,
    log_action_chain_(Rest, MidD, OutD, Continue, Stream).
log_action_chain_([rule_choice(A, rid(I,_,_), _, _, _New)|Rest],
                 InD, OutD, Continue, Stream) :-
    !,
    %% continuing at this level
    writeL_(Stream, [sp_(InD), A-I, " continued",nl_]),
    MidD is InD + 1,
    log_action_chain_(Rest, MidD, OutD, Continue, Stream).
log_action_chain_([wait_choice(TR)|_Rest],
                 InD, InD, _Continue, Stream) :-
    !,
    %% call stack terminates in a wait
    writeL_(Stream, [sp_(InD), wait_choice(TR), nl_]).


'$write_timed_seq_elems'(_Stream, []).
'$write_timed_seq_elems'(Stream, [A]) :-
    !, '$write_timed_seq_elems_aux'(Stream, A).
'$write_timed_seq_elems'(Stream, [A|As]) :-
    '$write_timed_seq_elems_aux'(Stream, A),
    writeL_(Stream, [", "]),
    '$write_timed_seq_elems'(Stream, As).

'$write_timed_seq_elems_aux'(Stream, pa(As) : -1) :-
    !,
    write_a_tr_action_(Stream, As).
'$write_timed_seq_elems_aux'(Stream, pa(As) : T) :-
    !,
    write_a_tr_action_(Stream, As),
    writeL_(Stream, [" : ", T]).
'$write_timed_seq_elems_aux'(Stream, As : -1) :-
    !,
    writeTerm_(Stream, As).
'$write_timed_seq_elems_aux'(Stream, As : T) :-
    !,
    writeTerm_(Stream, As),
    writeL_(Stream, [" : ", T]).

    
log(all) :-
    assert('$logging').


logger_(Name) :-
    retract(logger_name_(OldName)) , !,
    writeL_(["Changing logger from ", OldName, " to ", Name, nl_]),
    assert(logger_name_(Name)).
logger_(Name) :-
    writeL_(["Starting logging to ", Name, nl_]),
    assert(logger_name_(Name)),
    assert('$logging').
unlog :-
    writeL_(["Stoping logging",nl_]),
    retractall('$logging'),
    retractall(logger_name_(_)).

%% Used to enter a list of beliefs that replace the existing beliefs.
%% Useful for debugging teleor programs as the user can check which TR
%% procedure guards are true for a given state of the belief store.
refresh_bs(_) :-
    forall(get_percepts_belief_(B), retract(B)),
    writeL_(stdout, ["Paste a list of beliefs from logger", nl_]),
    '$read_term_'(Beliefs),
    forall(member(B, Beliefs), remember(B)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Display beliefs

?- dynamic(task/2).

display_beliefs_(T) :-
    '$logging',
    !,
    open_string(write, Stream),
    thread_atomic_goal((display_all_beliefs_(T, Stream),
                        send_log_message_(Stream))).
display_beliefs_(_).

display_all_beliefs_(T, Stream) :-
    writeL_(Stream, [
                    "------ Current Percepts and Dynamic Beliefs at Time: ", T,
                    " ------",nl_]),
    findall(B, get_percepts_belief_(B), Beliefs),
    sort(Beliefs, SortedBeliefs, '$percept_order'),
    writeL_(Stream, [SortedBeliefs]).

%% used in sort above to list percepts in reasonable order
'$percept_order'(A, B) :-
    compound(A), compound(B), !,
    A =.. AArgs,
    B =.. BArgs,
    '$percept_order_args'(AArgs, BArgs).
'$percept_order'(A, B) :-
    A @=< B.

'$percept_order_args'([], _).
'$percept_order_args'([A|Arest], [B|BRest]) :-
    A = B, !,
    '$percept_order_args'(Arest, BRest).
'$percept_order_args'([A|_Arest], [B|_BRest]) :-
    '$percept_order'(A, B).


get_percepts_belief_(G) :-
    '$belief_type_info'(_, _, G),
    \+ '$nondisplay_belief'(G),
    call(G).

'$nondisplay_belief'(time_(_)).
'$nondisplay_belief'(task(_,_)).
'$nondisplay_belief'(running(_)).
'$nondisplay_belief'(waiting(_)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Percept management
%%

%% Keep asking the percept process to send initial percepts until
%% init percepts are sent - this does not read the percept message - this is
%% done by the caller
do_initialise_ :-
    '$agent_info'(agent, embedded), !.
do_initialise_ :-
    '$agent_info'(_, Handle),
    same_handle(Handle, B@Host),
    repeat,
    to(initialise_, Handle),
    writeL_(["Waiting for initial percepts from ", Handle, nl_]),
    ipc_peek(_, H, [timeout(5)]),
    same_handle(H, B@Host), !,
    writeL_(["Received initial percepts from ", Handle, nl_]).

%% Embedded agents take care of percepts and control actions directly
%% rather via communicating with other agents.
%% This means that the percept handler needs to access sensors directly
%% and so polls the sensors every Reactivity seconds
start_embedded_percept_handler_(Reactivity) :-
    %% initialize the percept handler - default is to do nothing
    thread_atomic_goal(init_percept_handler_interface),
    %% get the initial sensors by calling the user defined poll_sensors
    assert('$can_run_handlers'),
    '$reset_time_fact',
    poll_sensors(Percepts),
    '$process_embedded_percepts_init'(Percepts),
    %% let other threads know handler is running
    assert('$percept_thread_running'),
    thread_yield,
    %% enter percept handler loop
    handle_embedded_percepts_(Reactivity).

%% For embedded agents: sleep then poll sensors and process them
handle_embedded_percepts_(Reactivity) :-
    repeat,
    thread_sleep(Reactivity),
    poll_sensors(P),
    thread_atomic_goal('$process_embedded_percepts_init'(P)),
    fail.
    %handle_embedded_percepts_(Reactivity).

%% for initialization we don't want failure of '$process_embedded_percepts'
%% to cause failure of the caller
'$process_embedded_percepts_init'(P) :-
    '$process_embedded_percepts'(P), !.
'$process_embedded_percepts_init'(_).

%% For embedded agents we use the 'all' percept interface
'$process_embedded_percepts'(Percepts) :-
    '$log_process_percepts'(Percepts),
    '__time__'(BSTime),
    %% any remember/forget will cause __time__ to increase as
    %% this is the timestamp on BS updates
    handle_percepts_interface_(Percepts),
    '__time__'(NewBSTime),
    NewBSTime > BSTime,
    %% timestamp increased so need to check tasks   
    task(_, _),
    %% log new BS
    '$display_beliefs'.


%%'$send_eval_now_message'(A,B) :- log_list(['$send_eval_now_message'(A,B),nl_]),fail.
'$send_eval_now_message'([], [FirstTask|Rest]) :- !,
    %%log_list([eval_now([], Rest, [], FirstTask, []), " ->> ", FirstTask, nl_]),
    eval_now([], Rest, [], FirstTask, [], no_timeout) ->> FirstTask.
'$send_eval_now_message'([FirstTask|Rest], Waiting) :-
    %%log_list([eval_now(Rest, Waiting, [], FirstTask, []), " ->> ", FirstTask, nl_]),
    eval_now(Rest, Waiting, [], FirstTask, [], no_timeout) ->> FirstTask.

%% Start the percepts handling thread
start_percept_handler_ :-
    (
     current_predicate(handle_percepts_interface_/1)
    ->
     true
    ;
     '$assert_percepts_interface'(updates)
    ),
    '$reset_time_fact',
    %% user defined interface - mostly not needed but the user
    %% can define if, for example, the percepts comms channel needs to be set up
    %% EG ROS interface
    init_percept_handler_interface,
    %% Send an initialise_ message to "robot" and wait for a response
    %% to appear in the percept thread message buffer (using peek)
    %%do_initialise_,
    '$get_percepts'(P),
    '__percept_interface__'(Interface),
    %% process the read in percepts using the user declared interface
    %% (all or updates)
    '$process_percepts_init'(Interface, P),
    %% Signal to the forking thread that the percept handler is ready
    assert('$percept_thread_running'),
    %% Given up its timeslice so othere threads can run
    thread_yield,
    %% Enter percept handler loop
    handle_percepts_(Interface).       

%% The percept handler is a repeat fail loop rather than a recursive loop
%% to avoid garbage collection
handle_percepts_(Interface) :-
    repeat,
    '$get_percepts'(P),
    %% there are percepts to process - wait until task re-eval finished 
    thread_atomic_goal('$process_percepts'(Interface, P)),
    fail.
   %% handle_percepts(Interface).

%% for initialization we don't want failure of '$process_percepts'
%% to cause failure of the caller
'$process_percepts_init'(Interface, P) :-
    '$process_percepts'(Interface, P),
    !.
'$process_percepts_init'(_, _).

'$process_percepts'(Interface, P) :-
    %% there might have been more percepts while waiting - get the rest
    %% and filter out invalid percepts terms
    '$get_all_filtered_percepts'(Interface, P, GPs),
    %'__time__'(BSTime),
    '$log_process_percepts'(GPs),
    handle_percepts_interface_(GPs),
    !. %, '$display_beliefs'.
    % '__time__'(NewBSTime),
    % NewBSTime > BSTime,
    % '$display_beliefs', !.

%% get perepts list and filter out any terms that are not ground percept terms
%% using all interface
'$get_all_filtered_percepts'(all, P, GPs) :-
    '$get_all_all_percepts'(P, P2),
    filter_ground_all_percept__(P2, GPs).

%% get perepts list and filter out any terms that are not  wrapped
%% ground percept terms using updates interface
'$get_all_filtered_percepts'(updates, P, GPs) :-
    '$get_all_updates_percepts'(P, P2),
    filter_ground_updates_percept__(P2, GPs). 

'$log_process_percepts'(P) :-
    '$logging', !,
    open_string(write, StreamP),
    write(StreamP, 'Received new percept message:\n'),
    writeL_(StreamP, [P]), nl(StreamP),
    send_log_message_(StreamP).
'$log_process_percepts'(_).

'$get_all_all_percepts'(_P, P2) :-
    ipc_peek(_, _, [timeout(poll)]), !,
    '$get_percepts'(P1),
    '$get_all_all_percepts'(P1, P2).
'$get_all_all_percepts'(P, P).

'$get_all_updates_percepts'(P, P2) :-
    ipc_peek(_, _, [timeout(poll)]), !,
    '$get_percepts'(P0),
    append(P, P0, P1),
    '$get_all_updates_percepts'(P1, P2).
'$get_all_updates_percepts'(P, P).



%%filter_ground_updates_percept__(A,B) :- errornl(filter_ground_updates_percept__(A,B)),fail.
filter_ground_updates_percept__([], []).
filter_ground_updates_percept__([r_(P)|Rest], [r_(P1)|Grest]) :-
    atom(P),
    P1 = P('$none_'),
    ground_percept_(P1), !,
    filter_ground_updates_percept__(Rest, Grest).
filter_ground_updates_percept__([r_(P)|Rest], [r_(P)|Grest]) :-
    ground_percept_(P), !,
    filter_ground_updates_percept__(Rest, Grest).
filter_ground_updates_percept__([u_(P)|Rest], [u_(P)|Grest]) :-
    gen_percept_pattern_(P, _Patt, Stripped),
    ground_percept_(Stripped), !,
    filter_ground_updates_percept__(Rest, Grest).
filter_ground_updates_percept__([f_(P)|Rest], [f_(P1)|Grest]) :-
    atom(P),
    P1 = P('$none_'),
    ground_percept_(P1), !,
    filter_ground_updates_percept__(Rest, Grest).
filter_ground_updates_percept__([f_(P)|Rest], [f_(P)|Grest]) :-
    percept_(P), !,
    filter_ground_updates_percept__(Rest, Grest).
filter_ground_updates_percept__([fa_(P)|Rest], [fa_(P1)|Grest]) :-
    atom(P),
    P1 = P('$none_'),
    ground_percept_(P1), !,
    filter_ground_updates_percept__(Rest, Grest).
filter_ground_updates_percept__([fa_(P)|Rest], [fa_(P)|Grest]) :-
    percept_(P), !,
    filter_ground_updates_percept__(Rest, Grest).
filter_ground_updates_percept__([P|Rest], Grest) :-
    log_error_(error(invalid_updates_percept_term(P))),
    filter_ground_updates_percept__(Rest, Grest).

filter_ground_all_percept__([], []).
filter_ground_all_percept__([P|Rest], [P1|Grest]) :-
    atom(P),
    P1 = P('$none_'),
    ground_percept_(P1), !,
    filter_ground_all_percept__(Rest, Grest).
filter_ground_all_percept__([P|Rest], [P|Grest]) :-
    ground_percept_(P), !,
    filter_ground_all_percept__(Rest, Grest).
filter_ground_all_percept__([P|Rest], Grest) :-
    log_error_(error(invalid_all_percept_term(P))),
    filter_ground_all_percept__(Rest, Grest).


'$get_percepts'(Ps) :-
    Ps1 <<- _,
    (
      list(Ps1)
    ->
      Ps = Ps1
    ;
      Ps = [Ps1]
    ) .
      


%% add resources hook if defined by user
'$assert_resources_hook_interface' :-
    current_predicate(resources_hook/1), !,
    assert((resources_callback :-  resources_hook(_))).
'$assert_resources_hook_interface' :-
    assert(resources_callback).


%% return the active (running) resources used
get_active_resources(Resources) :-
    findall(res_(Task, Res),
            ('__running__'(Task),
                '__resources__'(Task,Res)),
            Resources).

%% return resources being waited on
get_waiting_resources(Resources) :-
    findall(res_(Task, Res),
            ('__waiting__'(Task),
                '__resources__'(Task,Res)),
            Resources).


%% add interface to robotic messages if defined by user or used the
%% default handler
'$assert_send_robotic_message_interface' :-
    current_predicate(send_robotic_message/1), !,
    assert((send_robotic_message_interface_(A) :- send_robotic_message(A))).
'$assert_send_robotic_message_interface' :-
    assert((send_robotic_message_interface_(A) :- send_robotic_message_default_(A))).

%% add interface for post processing percepts
%% if defined by the user
'$assert_post_percepts_interface' :-
    current_predicate(post_process_percepts/2), !,
    assert((post_percepts_interface(A,B) :- post_process_percepts(A,B))).
'$assert_post_percepts_interface' :-
    assert((post_percepts_interface(_A,_B) :- true)).



controls_(C) :-
    send_robotic_message_interface_(C).

send_robotic_message_default_(Controls) :-
    '$agent_info'(_, embedded), !,
    ip_lookup('$task_name', Task),
    actions(Task, Controls) ->> embedded.
send_robotic_message_default_(Controls) :-
    ip_lookup('$task_name', Task),
    '$agent_info'(_, ToHandle),
    to(actions(Task, Controls), ToHandle).

'$update_interfaces'(Interface) :-
    '$running_teleo', !,
    %% retract all interfaces
    '$retract_interface'(handle_percepts_interface_(_)),
    '$retract_interface'(handle_message_interface_(_, _)),
    '$retract_interface'(init_message_handler_interface),
    '$retract_interface'(init_percept_handler_interface),
    '$retract_interface'(overlapping_resources_interface_(_, _)),
    '$retract_interface'(resources_callback),
    '$retract_interface'(send_robotic_message_interface_(_)),
    '$retract_interface'(post_percepts_interface(_, _)),
    retractall('__percept_interface__'/1),
    
    %% set the percept handling interface (all or updates)
    '$assert_percepts_interface'(Interface),
    %% set the message handler for user defined handlers for:
    %% handle_message/2, handle_template_message/2, (handle_invalid_message/2
    %% if not defined do nothing
    '$assert_message_interface',
    %% set the handler to initialize message handler if defined by the user
    %% handler does nothing if not user defined
    '$assert_init_message_handler_interface',
    %% ditto for the percept handler
    '$assert_init_percept_handler_interface',
    %% set that handler for user defined overlapping_resources
    %% the handler fails if not user defined
    '$assert_overlapping_resources_interface',
    %% add resources hook if defined by user
    %% do nothing if not defined
    '$assert_resources_hook_interface',
    %% add interface for post processing percepts 
    %% if defined by the user - do nothing if not user defined
    '$assert_post_percepts_interface',
    %% add interface for sending robotic control messages
    '$assert_send_robotic_message_interface'.
'$update_interfaces'(_).

'$retract_interface'(Call) :-
    retractall(Call).

%% start_agent(Handle, Interface)
%% Start a teleor agent using Interface (all or updates) where
%% Handler is the address to send robot control messages to
start_agent(pubsub, Interface) :-
    '$running_teleo', !,
    set_default_message_thread(messages),
    %% agent_info is used for sending messages to robot
    assert('$agent_info'(agent, pubsub)),
    %% set up user defined interfaces
    '$update_interfaces'(Interface),
    %% initialize __time__ and __prev_time__
    '$reset_time_fact',
    '__time__'(T), retractall('__prev_time__'(_)), assert('__prev_time__'(T)),
    start_task_eval_thread_,
    start_percept_thread__,
    thread_fork(messages, handle_messages_),
    thread_yield, %% to give the message thread a chance to init
    writeL_(["Messages thread started", nl_]),
    writeL_(["Agent initialised", nl_]).  
start_agent(_Handle, _Interface) :-
    \+ pedro_is_registered,
     !,
    writeL_(["Error: Agent is not registered with pedro", nl_]).
start_agent(Handle, Interface) :-
    '$running_teleo', !,
    %%thread_handle((_:MyName@_)),
    %% agent messages from other agents are put in the messages threads buffer
    set_default_message_thread(messages),
    %% if the agent handle is an atom then localhost is added as the machine
    %% name otherwise Hndle will be treated as a local thread name
    ( atom(Handle) -> Handle1 = (Handle@localhost) ; Handle1 = Handle ),
    %% agent_info is used for sending messages to robot
    assert('$agent_info'(agent, Handle1)),
    %% set up user defined interfaces
    '$update_interfaces'(Interface),
    %% initialize __time__ and __prev_time__
    '$reset_time_fact',
    '__time__'(T), retractall('__prev_time__'(_)), assert('__prev_time__'(T)),
    start_task_eval_thread_,
    start_percept_thread__,
    thread_fork(messages, handle_messages_),
    thread_yield, %% to give the message thread a chance to init
    writeL_(["Messages thread started", nl_]),
    writeL_(["Agent initialised", nl_]).     
start_agent(_Handle, _Interface) :-
    writeL_(["Error: Attempting to start agent outside of the teleor interface",
	    nl_]).

%% start_embedded_agent(Reactivity) start an embedded agent using the all
%% interface. Reactivity is the time between polling perepts
start_embedded_agent(Reactivity) :-
    '$running_teleo', !,
    %%thread_handle((_:MyName@_)),
    set_default_message_thread(messages),
    assert('$agent_info'(agent, embedded)),
    '$update_interfaces'(all),
    %% start a small thread
    thread_fork(embedded, start_embedded_robot_control_,
                [choice_size(10),
                 env_size(10),
                 heap_size(50),
                 binding_trail_size(10),
                 other_trail_size(10),
                 scratchpad_size(10),
                 name_table_size(100),
                 ip_table_size(100)]),
    '$reset_time_fact',
    '__time__'(T), retractall('__prev_time__'(_)), assert('__prev_time__'(T)),
    start_task_eval_thread_,
    writeL_(["Starting embedded thread", nl_]),
    start_embedded_percept_thread__(Reactivity),
    thread_fork(messages, handle_messages_),
    thread_yield, %% to give the message a thread a chance to init
    writeL_(["Messages thread started", nl_]),
    writeL_(["Agent initialised", nl_]).
     
start_embedded_agent(_) :-
    writeL_(["Error: Attempting to start agent outside of the teleor interface",
	    nl_]).

start_embedded_percept_thread__(Reactivity) :-
    '$percept'(_), !,
    thread_fork(percepts,start_embedded_percept_handler_(Reactivity)),
    writeL_(["Starting percepts thread", nl_]),
    thread_wait_on_goal('$percept_thread_running').
start_embedded_percept_thread__(_Reactivity)  :-
    writeL_(stderr, ["Warning: the percept handling thread has not been started because there is no percept declaration", nl_]).

start_percept_thread__ :-
    '$percept'(_), !,
    thread_fork(percepts,start_percept_handler_),
    writeL_(["Starting percepts thread", nl_]),
    thread_wait_on_goal('$percept_thread_running').
start_percept_thread__ :-
    writeL_(stderr, ["Warning: the percept handling thread has not been started because there is no percept declaration", nl_]).


start_embedded_robot_control_ :-
    embedded_robot_control_loop_([]).

embedded_robot_control_loop_(OldActions) :- 
    Actions <<- _,
    Actions = actions(_, NewActions),
    control_device(OldActions, NewActions),
    embedded_robot_control_loop_(NewActions).


%% Kill the agent:
%% Send the message "finish" to the robot process
%% Kill all tasks
%% Exit "helper" threads
kill_agent(_) :-
    '$agent_info'(agent, Handle), !,
    forall(task(Name, _), kill_task(Name)),
    ( thread_is_thread(messages) -> thread_exit(messages) ; true ),
    thread_exit(percepts),
    errornl('Agent killed'),
    finish ->> Handle,
    thread_exit(eval_handler_),
    retract('$agent_info'(agent, Handle)).
kill_agent(_) :-
    throw(no_agent_to_kill('$none_')).



'$assert_percepts_interface'(all) :-
    !,
    assert('__percept_interface__'(all)),
    assert((handle_percepts_interface_(A) :- handle_percepts_all_(A, F, R), post_percepts_interface(F, R))).
'$assert_percepts_interface'(_) :-
    !,
    assert('__percept_interface__'(updates)),
    assert((handle_percepts_interface_(A) :- handle_percepts_updates_(A, F, R),post_percepts_interface(F, R))).

%%handle_percepts_all_(Ps, Forgets, Remembers) :- errornl(handle_percepts_all_(Ps, Forgets, Remembers)),fail.
handle_percepts_all_(Ps, Forgets, Remembers) :-
    handle_percepts_all_1_(Ps, Forgets, Remembers),
    %% Forgets are the current percepts that need to be forgotten
    %% Remebers are the new precepts that are not current and so need
    %% remembering
    '$forget_remember_percepts'(Forgets, Remembers).

handle_percepts_all_1_(Pers, Forgets, Remembers) :-
    findall(P, to_forget_percept_(P, Pers), Forgets),
    handle_percepts_all_aux_(Pers, Remembers).

to_forget_percept_(P, Pers) :-
    a_percept_(P),
    \+member(P, Pers).

%% Extract the new percepts - i.e. percepts that are in the percepts list
%% but are not current (so need to be remembered)
handle_percepts_all_aux_([], []).
handle_percepts_all_aux_([P|Ps], OutRemembers) :-
    call(P), !,
    OutRemembers = Remembers,
    handle_percepts_all_aux_(Ps, Remembers).
handle_percepts_all_aux_([P|Ps], OutRemembers) :-
    OutRemembers = [P|Remembers],
    handle_percepts_all_aux_(Ps, Remembers).

percept_(P) :- type(P, '??'(dyn_term)), '$percept_type_info'(_, _, P).
ground_percept_(P) :- type(P, '!'(dyn_term)), '$percept_type_info'(_, _, P).

%% See comment before strip_percept_patterns in consult.ql
%% Ps is a list consisting of terms of the form:
%% r_(P) - P is a ground percept that needs to be remembered
%% f_(P) - P is a percept that needs to be forgotten - if P
%%         contains variable the first matching current percept will
%%         be forgotten
%% fa_(P) - P is a percept that needs to be forgotten - typically P
%%          contains variables and all matching current percepts will
%%          be forgotten
%% u_(P) -  P is a ground percept - if it matches a current percept in
%%         non '^' tagged arguments then the matching percept will be forgotten
%%         and P remembered. OW P is remembered and nothing is forgotten
handle_percepts_updates_(Ps, Forgets3, Remembers4) :-
    handle_percepts_updates_aux_(Ps, Forgets1, Remembers1),
    %% Forgets1 and Remembers1 are the "raw" percepts to forget and remember
    %% but there can be repeats because these lists have been constructed
    %% from various percept update terms - the next 4 calls remove repeats
    diff_list(Forgets1, Remembers1, Forgets2),
    diff_list(Remembers1, Forgets1, Remembers2),
    remove_duplicates(Forgets2, Forgets3),
    remove_duplicates(Remembers2, Remembers3),
    handle_percepts_updates_aux_2__(Remembers3, Remembers4),
    %% Remembers4 are the percepts in Remembers3 that actually need
    %% to be remembered (not the ones that are already current)
    '$forget_remember_percepts'(Forgets3, Remembers4).

%%handle_percepts_updates_aux_(A,B,C) :- errornl(handle_percepts_updates_aux_(A,B,C)),fail.
handle_percepts_updates_aux_([], [], []).
handle_percepts_updates_aux_([u_(P)|Ps], AllForgets, [P2|Remembers]) :-
    gen_percept_pattern_(P, P1, P2),
    ( 
      call(P1)
    ->
      AllForgets = [P1|Forgets]
    ;
      AllForgets = Forgets
    ),      
    !, handle_percepts_updates_aux_(Ps, Forgets, Remembers).
handle_percepts_updates_aux_([r_(P)|Ps], Forgets, [P|Remembers]) :-
    !, handle_percepts_updates_aux_(Ps, Forgets, Remembers).
handle_percepts_updates_aux_([f_(P)|Ps],  [P|Forgets], Remembers) :-
    call(P), !,
    handle_percepts_updates_aux_(Ps, Forgets, Remembers).
handle_percepts_updates_aux_([fa_(P)|Ps], AllForgets, Remembers) :-
    !,
    findall(P, call(P), FPs),
    append(FPs, Forgets, AllForgets),
    handle_percepts_updates_aux_(Ps, Forgets, Remembers).
handle_percepts_updates_aux_([_|Ps], Forgets, Remembers) :-
     handle_percepts_updates_aux_(Ps, Forgets, Remembers).

handle_percepts_updates_aux_2__([], []).
handle_percepts_updates_aux_2__([R|Rems], Remembers) :-
    call(R), !,
    handle_percepts_updates_aux_2__(Rems, Remembers).
handle_percepts_updates_aux_2__([R|Rems], [R|Remembers]) :-
    handle_percepts_updates_aux_2__(Rems, Remembers).

gen_percept_pattern_(P, Patt, Stripped) :-
    P '@=..' [F|Args],
    gen_percept_pattern_aux_(Args, PArgs, SArgs),
    Patt '@=..' [F|PArgs],
    Stripped '@=..' [F|SArgs].

gen_percept_pattern_aux_([], [], []).
gen_percept_pattern_aux_(['!'(T)|Args], [T|PArgs], [T|SArgs]) :-
    !,
    gen_percept_pattern_aux_(Args, PArgs, SArgs).
gen_percept_pattern_aux_([A|Args], [_|PArgs], [A|SArgs]) :-
    !,
    gen_percept_pattern_aux_(Args, PArgs, SArgs).


%%'$forget_remember_percepts'(Forgets, Remembers)
%% Forgets is the list of percepts to forget and
%% Remembers is the list of percepts to remember
'$forget_remember_percepts'([], []) :- !.
'$forget_remember_percepts'(Fs, _Rs) :-
    %% retract all the elements of Fs
    member(C, Fs),
    retract(C),
    fail.
'$forget_remember_percepts'(_Fs, Rs) :-
    %% asserta all elements of Rs
    member(C, Rs),
    asserta(C),
    fail.
'$forget_remember_percepts'(Fs, Rs) :-
    %% since the percepts have been updated we need to run infer_and_remember
    '$infer_and_remember'(Fs, Rs),
    '$reset_time_fact'.

%% on backtracking lists all percept facts in the BS
a_percept_(P) :-
    '$percept_type_info'(_, _, P),
    call(P).

start_named_task(_TR, Name) :-
    task(Name, _), !,
    throw(task_name_in_use(Name)).
start_named_task(TR, Name) :-
    remember([new_task_(Name, TR)]).


start_task(TR, _Name, _Root) :-
    '$tel_atomic'(_),
    functor(TR, TRF, _),
    \+'$tel_start'(TRF), !,
    term2string(TR, TRS),
    throw(not_start_task_TR_procedure(TRS)).
start_task(TR, Name, Root) :-
    gen_task_name_(Root, Name),
    remember([new_task_(Name, TR)]).

% gen_task_name_(Root, Name) :-
%     \+task(Root, _),!,
%     Name = Root.
gen_task_name_(Root, Name) :-
    atom_codes(Root, RootCodes),
    between(1, 100000, N),
    number_codes(N, NCodes),
    append(RootCodes, NCodes, Codes),
    atom_codes(Name, Codes),
    \+task(Name, _),
    \+new_task_(Name, _),
    !.


kill_task(Name) :-
    task(Name, _), !,
    send_stop_action_(Name),
    remember([end_task_(Name)]).
kill_task(Name) :-
    throw(no_matching_task_to_kill(Name)).


%% Run in the messages thread to handle messages
handle_messages_ :-
    %% possible user defined initialization - e.g. subscribe
    thread_atomic_goal(init_message_handler_interface),
    '$handle_messages'.

'$handle_messages' :-
    repeat,
    from(M, Addr, true),
    %% there are messages to process - wait until task re-eval finished 
    thread_atomic_goal('$process_messages'(M, Addr)),
    thread_yield,
    fail.

% '$process_messages'(M, Addr) :-
%     '$process_messages_aux'(M, Addr),
%     '$eval_tasks'.


'$process_messages'(terminate_agent, _Addr) :-
    !,
    forall(task(Name, _), send_stop_action_(Name)),
    thread_exit(thread0).
'$process_messages'(suspend_agent, _Addr) :-
    !,
    remember([agent_suspended_]).
'$process_messages'(resume_agent, _Addr) :-
    !,
    forget([agent_suspended_]).    
'$process_messages'(M, Addr) :-
    %% process messages
    thread_atomic_goal(handle_message_interface_(M, Addr)),
    (
      ipc_peek(_, _, [timeout(poll)])
    ->
      from(M1, Addr1, true),
      %% keep going if more have turned up
      '$process_messages'(M1, Addr1)
    ;
      true
    ).

'$set_can_run_handlers' :-
    '$can_run_handlers', !.
'$set_can_run_handlers' :-
    assert('$can_run_handlers').



'$assert_message_interface' :-
    (
      current_predicate(handle_message/2)
    ;
      current_predicate(handle_template_message/2)
    ),
    !,
    (
      current_predicate(handle_message/2),
      \+ '$user_type_info'(message, defined_type, _, _, _)
    ->
      '$display_error'(message_type_undefined)
    ;
      current_predicate(handle_message/2)
    ->
      assert((handle_message_interface_(M, A) :-
             type(M, '!'(message)), !,
              log_message_(message(M, A)),
              handle_message(M, A)))
    ;
      true
    ),
    (
      current_predicate(handle_template_message/2),
      \+ '$user_type_info'(template_message, defined_type, _, _, _)
    ->
      '$display_error'(template_message_type_undefined)
    ;
      current_predicate(handle_template_message/2)
    ->
      assert((handle_message_interface_(M, A) :-
             type(M, '?'(message)),
              template(M), !,
             log_message_(message(M, A)),
              handle_template_message(M, A)))
    ;
      true
    ),
    
    (
      current_predicate(handle_invalid_message/2)
    ->
      assert((handle_message_interface_(M, A) :-
             term2string(M, MString),
             log_message_(message(MString, A)),
             handle_invalid_message(MString, A)))
    ;
      assert(handle_message_interface_(_M, _A))
    ).
'$assert_message_interface' :-
    assert((handle_message_interface_(M, A) :-
           log_message_(message(M, A)),
	   '$default_handle_message'(M,A))).



'$assert_init_message_handler_interface' :-
    current_predicate(init_message_handler/1), !,
    assert((init_message_handler_interface :- init_message_handler(_))).
'$assert_init_message_handler_interface' :-
    assert(init_message_handler_interface).

'$assert_init_percept_handler_interface' :-
    current_predicate(init_percept_handler/1), !,
    assert((init_percept_handler_interface :- init_percept_handler(_))).
'$assert_init_percept_handler_interface' :-
    assert((init_percept_handler_interface :- do_initialise_)).


'$assert_overlapping_resources_interface' :-
    current_predicate(overlapping_resources/2), !,
    assert((overlapping_resources_interface_(A, B) :-
           overlapping_resources(A,B))).
'$assert_overlapping_resources_interface' :-
    assert((overlapping_resources_interface_(A, B) :-
           default_overlapping_resources(A,B))).

'$default_handle_message'(M,A) :-
    ground(M), !,
    '__time__'(Time),
    remember(received_message(M, A, Time)).
'$default_handle_message'(_, _).
	    
