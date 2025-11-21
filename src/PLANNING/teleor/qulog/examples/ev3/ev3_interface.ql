?- load_foreign(
 [
 'ev3_interface.o',
 'c_ev3.o'
 ],
 [
 ev3_init/0 = p_ev3_init_interface,
 ev3_finish/0 = p_ev3_finish_interface,
 ev3_run_forever/2 = p_ev3_run_forever_interface,
 ev3_run_to_rel_pos/4 = p_ev3_run_to_rel_pos_interface,
 ev3_run_timed/3 = p_ev3_run_timed_interface,
 ev3_set_position/1 = p_ev3_set_position_interface,
 ev3_is_running/1 = p_ev3_is_running_interface,
 ev3_get_tacho_speed/2 = p_ev3_get_tacho_speed_interface,
 ev3_stop/0 = p_ev3_stop_interface,
 ev3_touch/2 = p_ev3_touch_interface,
 ev3_ultrasonic/1 = p_ev3_ultrasonic_interface,
 ev3_gyro/1 = p_ev3_gyro_interface
 ],
 ['/usr/local/lib/libev3dev-c.so']).


%% linking zero arity Qulog definitions to QuProlog definitions.
%% The zero arity Qulog predicates are compiled to QuProlog as an arity 1
%% predicate
ev3_init(_) :- ev3_init.
ev3_finish(_) :- ev3_finish.
ev3_stop(_) :- ev3_stop.