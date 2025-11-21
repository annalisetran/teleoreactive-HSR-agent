/*
** This is an older version where the scan was passed in to the wall follower predicate
** and the linear and angular velosities were output variables.
** Kept as an example of how to construct a more complex query.
*/


void Planner::update_callback()
{
	term *old_global = global;

	current_node = this;
	pl_near = near_start;
	term Q = new_g_fn(3);
	ARG(0, Q) = intern((char *) "planner");
	ARG(1, Q) = make_scan(scan_data_);
	term v1 = lookup_var(intern((char *) "LV"));
	term v2 = lookup_var(intern((char *) "AV"));
	ARG(2, Q) = v1;
	ARG(3, Q) = v2;
	term vars = gcons(v1, gcons(v2, _nil));

	term result = pl_query(Q, vars, 1);

	double lv = RVAL(CAR(CAR(result)));
	double av = RVAL(CAR(CDR(CAR(result))));
	update_cmd_vel(lv, av);

	global = old_global;
}

