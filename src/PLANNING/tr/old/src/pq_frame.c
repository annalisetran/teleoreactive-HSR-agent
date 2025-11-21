/************************************************************************/
/*		Postgres version of persistent frame store		*/
/************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <libpq-fe.h>
#include <stdbool.h>
#include <stdint.h>
#include <prolog.h>

term read_expr_from_string(char *);

static PGconn *frame_db;			// connection to frame store

static term _value, _default, _if_needed, _if_added, _if_removed, _if_new,
		_if_deleted, _if_replaced, _range, _multivalued, _cache, _help;

static term self = NULL;


/************************************************************************/
/* 	Apply a daemon, taking care of "new value" and "old value"	*/
/************************************************************************/

static term current_object = NULL;
static term current_slot = NULL;
static term new_value = NULL;
static term old_value = NULL;

static term apply(term daemon, term obj, term slot, term new_val, term old_val)
{
	term store_current_object = current_object;
	term store_current_slot = current_slot;
	term store_new_value = new_value;
	term store_old_value = old_value;
	term rval;

	current_object = obj;
	current_slot = slot;
	new_value = new_val;
	old_value = old_val;

	rval = progn(daemon, NULL);

	current_object = store_current_object;
	current_slot = store_current_slot;
	new_value = store_new_value;
	old_value = store_old_value;

	return rval;
}

/************************************************************************/
/*			Convert a term to a string			*/
/************************************************************************/

static char *to_string(term t)
{
	extern int display;
	extern char *prin_to_string(term);

	display = true;
	char *rval = prin_to_string(t);
	display = false;
	return rval;
}

/************************************************************************/
/*		Handle Postgres error then throw to Prolog error	*/
/************************************************************************/

static void db_error(PGresult *res)
{
	fprintf(stderr, "%s\n", PQerrorMessage(frame_db));

	PQclear(res);
	PQfinish(frame_db);

	exit(1);
}


/************************************************************************/
/*		  Open and initialise frame store			*/
/************************************************************************/

static const char *_fput      = "fput";
static const char *_fget      = "fget";
static const char *_fremove   = "fremove";
static const char *_find_demon  = "find_demon";

static bool frame_db_open(term goal, term *frame)
{
	// Open frame store

	frame_db = PQconnectdb("user=claude dbname=testdb2");

	if (PQstatus(frame_db) == CONNECTION_BAD)
	{
		fprintf(stderr, "Connection to database failed: %s\n", PQerrorMessage(frame_db));
		PQfinish(frame_db);
		exit(1);
	}

	// Create new frame table, deleting any previous one

	PGresult *res = PQexec(frame_db, "DROP TABLE IF EXISTS frame");

	if (PQresultStatus(res) != PGRES_COMMAND_OK)
		db_error(res);

	PQclear(res);

	res = PQexec(frame_db, "CREATE TABLE frame(frame_name VARCHAR, slot VARCHAR, facet VARCHAR, datum VARCHAR)");

	if (PQresultStatus(res) != PGRES_COMMAND_OK)
		db_error(res);

	PQclear(res);

	// Prepare statements

	PQprepare(frame_db, _fput,     "INSERT INTO frame VALUES ($1, $2, $3, $4)", 4, NULL);
	PQprepare(frame_db, _fget,     "SELECT datum FROM frame WHERE frame_name = $1 AND slot = $2 AND facet = $3", 3, NULL);
	PQprepare(frame_db, _fremove,  "DELETE FROM frame WHERE frame_name = $1 AND slot = $2 AND facet = $3", 3, NULL);
	PQprepare(frame_db, _find_demon, "SELECT datum FROM frame WHERE frame_name = $1 AND (slot = 'isa' OR slot = 'ako')", 1, NULL);

	return true;
}


/************************************************************************/
/*		  Close frame store at end of session			*/
/************************************************************************/

static bool frame_db_close(term goal, term *frame)
{
	PQfinish(frame_db);
}


/************************************************************************/
/*	Get slot matching frame(frame_name, slot, facet, ?value)	*/
/************************************************************************/

static term get_facet(term frame_name, term slot, term facet)
{
	term rval = NULL;
	const char *param[3];
	param[0] = NAME(frame_name);
	param[1] = NAME(slot);
	param[2] = NAME(facet);

	PGresult *res = PQexecPrepared(frame_db, _fget, 3, param, NULL, NULL, 0);

	if (PQresultStatus(res) != PGRES_TUPLES_OK)
		db_error(res);

	int rows = PQntuples(res);

	if (rows == 1)
	{
		rval = read_expr_from_string(PQgetvalue(res, 0, 0));
	}

	PQclear(res);
	return rval;
}


/************************************************************************/
/*	Get slot matching frame(frame_name, slot, facet, ?value)	*/
/*	Climb inheritance hierarchy, if _if_needed			*/
/************************************************************************/

static term find_demon(term frame_name, term slot, term facet)
{
	term rval = get_facet(frame_name, slot, facet);

	if (rval != NULL)
		return rval;

	const char *param[1];
	param[0] = NAME(frame_name);
	PGresult *res = PQexecPrepared(frame_db, _find_demon, 1, param, NULL, NULL, 0);

	if (PQresultStatus(res) != PGRES_TUPLES_OK)
		db_error(res);

	int rows = PQntuples(res);

	for (int i = 0; i < rows; i++)
	{
 		term parent = read_expr_from_string(PQgetvalue(res, i, 0));
		rval = find_demon(parent, slot, facet);
		if (rval != NULL)
 			break;
	}

	PQclear(res);
	return rval;
}


/****************************************************************************************/
/* Predicate helper to find slots matching frame(frame_name, ?slot, ?facet, ?datum)	*/
/****************************************************************************************/

static bool pq_fget(term frame_name, term slot, term facet, term datum, term *stack_frame)
{
	bool rval = false;
	const char *param[3];
	param[0] = NAME(frame_name);
	param[1] = NAME(slot);
	param[2] = NAME(facet);

	PGresult *res = PQexecPrepared(frame_db, _fget, 3, param, NULL, NULL, 0);

	if (PQresultStatus(res) != PGRES_TUPLES_OK)
		db_error(res);

	int rows = PQntuples(res);

	if (rows == 0  && facet == _value)
	{
		term d = find_demon(frame_name, slot, _default);

		if (d == NULL)
		{
			d = find_demon(frame_name, slot, _if_needed);
			if (d != NULL)
				d = apply(d, frame_name, slot, NULL, NULL);
			else
			{
				PQclear(res);
				return false;
			}
		}
		if (unify(datum, stack_frame, d, stack_frame))
			rval = true;
		else
			_untrail();
	}
	else
	{
		for (int i = 0; i < rows; i++)
		{
			term d = read_expr_from_string(PQgetvalue(res, i, 0));

			if (unify(datum, stack_frame, d, stack_frame))
				if (rest_of_clause())
					break;
			_untrail();
		}
	}

	PQclear(res);
	return rval;
}


/************************************************************************/
/*	Built-in preciate to fetch from frame database entry		*/
/************************************************************************/

static bool p_fget(term goal, term *stack_frame)
{
	switch (ARITY(goal))
	{
		case 3:
		{
			term frame_name	= check_arg(1, goal, stack_frame, ATOM, IN);
			term slot	= check_arg(2, goal, stack_frame, ATOM, IN);
			term datum	= check_arg(3, goal, stack_frame, ANY,  OUT);

			return pq_fget(frame_name, slot, _value, datum, stack_frame);
		}
		case 4:
		{
			term frame_name	= check_arg(1, goal, stack_frame, ATOM, IN);
			term slot	= check_arg(2, goal, stack_frame, ATOM, IN);
			term facet	= check_arg(3, goal, stack_frame, ATOM, IN);
			term datum	= check_arg(4, goal, stack_frame, ANY,  OUT);

			return pq_fget(frame_name, slot, facet, datum, stack_frame);
		}
		default:
			fail("Incorrect number of arguments");
	}

}


/****************************************************************************************/
/* Function helper to find slot matching frame(frame_name, ?slot, ?facet, ?datum)	*/
/****************************************************************************************/

static term get(term frame_name, term slot)
{
	term rval = NULL;
	const char *param[3];
	param[0] = NAME(frame_name);
	param[1] = NAME(slot);
	param[2] = NAME(_value);

	PGresult *res = PQexecPrepared(frame_db, _fget, 3, param, NULL, NULL, 0);

	if (PQresultStatus(res) != PGRES_TUPLES_OK)
		db_error(res);

	int rows = PQntuples(res);

	if (rows == 0)
	{
		rval = find_demon(frame_name, slot, _default);

		if (rval == NULL)
		{
			rval = find_demon(frame_name, slot, _if_needed);
			if (rval != NULL)
				rval = apply(rval, frame_name, slot, NULL, NULL);
			else
				PQclear(res);
		}
	}
	else if (rows == 1)
	{
		rval = read_expr_from_string(PQgetvalue(res, 0, 0));
	}
	else if (rows > 1)
	{
		rval = _nil;
		for (int i = rows-1; i >= 0; i--)
		{
			term d = read_expr_from_string(PQgetvalue(res, i, 0));

			rval = gcons(d, rval);
		}
	}

	PQclear(res);
	return rval;
}


/************************************************************************/
/* 	Built-in funciton to get from frame database entry		*/
/************************************************************************/

static term f_get(term goal, term *stack_frame)
{
	term rval = NULL;

	if (TYPE(goal) == ATOM)
	{
		rval = get(current_object, goal);
	}
	else
	{
		term slot = check_arg(0, goal, stack_frame, ATOM, IN);
		term frame_name	= check_arg(1, goal, stack_frame, ATOM, IN);
		rval = get(frame_name, slot);
	}

	if (rval == NULL)
		fail("Empty slot");
	else
		return rval;
}


/************************************************************************/
/* when a new value is put into a slot a range check is performed	*/
/* if the value is out of range a help demon is invoked (if present)	*/
/************************************************************************/

static bool in_range(term frame_name, term slot, term datum)
{
	term d = find_demon(frame_name, slot, _range);

	if (d != NULL)
	{
		if (apply(d, frame_name, slot, datum, NULL) != _true)
		{
			if ((d = find_demon(frame_name, slot, _help)) != NULL)
			{
				apply(d, frame_name, slot, datum, NULL);
				return false;
			}
			fail("Could not put value");
		}
	}
	return true;
}

/************************************************************************/
/* Adds a triple generic(frame_name, slot, value)			*/
/* If the value is a string starting with '@', it is assumed to be the	*/
/*	name of	another frame						*/
/************************************************************************/

static bool pq_fput(term frame_name, term slot, term facet, term datum)
{

	if (PROC(slot) == NULL)
		new_subr(f_get, NAME(slot));		// Turns a slot name into a function
	else if (C_CODE(PROC(slot)) != f_get)
		fail("slot name already defined");

	if (facet == _value)
	{
		if (! in_range(frame_name, slot, datum))
			return false;

		term d = get_facet(frame_name, slot, _value);
		if (d != NULL)
		{
			if (find_demon(frame_name, slot, _multivalued) == NULL)
				fail("Cannot add more than one value to a slot that is not multivalued");
		}
	}

	const char *param[4];
	param[0] = NAME(frame_name);
	param[1] = NAME(slot);
	param[2] = NAME(facet);
	param[3] = to_string(datum);
	PGresult *res = PQexecPrepared(frame_db, _fput, 4, param, NULL, NULL, 0);
	free(param[3]);

//	char stm[512];
//	sprintf(stm, "INSERT INTO frame VALUES ('%s','%s','%s','%s')", frame_name, slot, facet, datum);
//	PGresult *res = PQexec(frame_db, stm);

	if (PQresultStatus(res) != PGRES_COMMAND_OK)
		db_error(res);

	PQclear(res);

	if (facet == _value)
	{
		term d = find_demon(frame_name, slot, _if_added);
		if (d != NULL)
			d = apply(d, frame_name, slot, datum, NULL);
	}
	return true;
}

static bool p_fput(term goal, term *frame)
{
	switch (ARITY(goal))
	{
		case 3:
		{
			term frame_name = check_arg(1, goal, frame, ATOM, IN);
			term slot = check_arg(2, goal, frame, ATOM, IN);
			term datum = check_arg(3, goal, frame, ANY, IN);

			return pq_fput(frame_name, slot, _value, datum);
		}
		case 4:
		{
			term frame_name = check_arg(1, goal, frame, ATOM, IN);
			term slot = check_arg(2, goal, frame, ATOM, IN);
			term facet = check_arg(3, goal, frame, ATOM, IN);
			term datum = check_arg(4, goal, frame, ANY, IN);

			return pq_fput(frame_name, slot, facet, datum);
		}
		default:
			fail("Incorrect number of arguments");
	}
}


/************************************************************************/
/* 	Remove facet matching generic(frame_name, slot, facet, ?value)	*/
/************************************************************************/

static void pq_fremove(term frame_name, term slot, term facet)
{
	if (facet == _value)
	{
		term v = get_facet(frame_name, slot, _value);

		if (v == NULL)
		{
			fail("Tried to remove non-existent slot");
		}

		term d = find_demon(frame_name, slot, _if_removed);
		if (d != NULL)
		{
			apply(d, frame_name, slot, NULL, v);
		}
	}

	const char *param[3];
	param[0] = NAME(frame_name);
	param[1] = NAME(slot);
	param[2] = NAME(facet);
	PGresult *res = PQexecPrepared(frame_db, _fremove, 3, param, NULL, NULL, 0);

	if (PQresultStatus(res) != PGRES_COMMAND_OK)
		db_error(res);
	PQclear(res);
}


/************************************************************************/
/* 		Prolog built-in to remove triples from database		*/
/************************************************************************/

static bool p_fremove(term goal, term *frame)
{
	switch (ARITY(goal))
	{
		case 2:
		{
			term frame_name = check_arg(1, goal, frame, ATOM, IN);
			term slot = check_arg(2, goal, frame, ATOM, OUT);

			pq_fremove(frame_name, slot, _value);
			return true;
		}
		case 3:
		{
			term frame_name = check_arg(1, goal, frame, ATOM, IN);
			term slot = check_arg(2, goal, frame, ATOM, OUT);
			term facet = check_arg(3, goal, frame, ATOM, OUT);

			pq_fremove(frame_name, slot, facet);
			return true;
		}
		default:
			fail("Incorrect number of arguments");
	}
}


/************************************************************************/
/* 	Remove facet matching generic(frame_name, slot, facet, ?value)	*/
/************************************************************************/

static bool pq_freplace(term frame_name, term slot, term facet, term datum)
{
	if (facet == _value)
	{
		if (! in_range(frame_name, slot, datum))
			return false;

		term v = get_facet(frame_name, slot, _value);

		if (v == NULL)
			fail("Tried to replace non-existent slot");

		term d = find_demon(frame_name, slot, _if_replaced);
		if (d != NULL)
			apply(d, frame_name, slot, datum, v);
	}

	const char *param[4];
	param[0] = NAME(frame_name);
	param[1] = NAME(slot);
	param[2] = NAME(facet);
	param[3] = to_string(datum);

	PGresult *res = PQexecPrepared(frame_db, _fremove, 3, param, NULL, NULL, 0);

	if (PQresultStatus(res) != PGRES_COMMAND_OK)
		db_error(res);
	res = PQexecPrepared(frame_db, _fput, 4, param, NULL, NULL, 0);

	if (PQresultStatus(res) != PGRES_COMMAND_OK)
		db_error(res);

	free(param[3]);
	PQclear(res);
	return true;
}


/************************************************************************/
/* 		Prolog built-in to remove triples from database		*/
/************************************************************************/

static bool p_freplace(term goal, term *frame)
{
	switch (ARITY(goal))
	{
		case 3:
		{
			term frame_name = check_arg(1, goal, frame, ATOM, IN);
			term slot = check_arg(2, goal, frame, ATOM, IN);
			term datum = check_arg(3, goal, frame, ANY, IN);

			pq_freplace(frame_name, slot, _value, datum);
			return true;
		}
		case 4:
		{
			term frame_name = check_arg(1, goal, frame, ATOM, IN);
			term slot = check_arg(2, goal, frame, ATOM, IN);
			term facet = check_arg(3, goal, frame, ATOM, IN);
			term datum = check_arg(4, goal, frame, ANY, IN);

			pq_freplace(frame_name, slot, facet, datum);
			return true;
		}
		default:
			fail("Incorrect number of arguments");
	}
}


/************************************************************************/
/* 			Hooks for new and old values			*/
/************************************************************************/

static term p_self(term goal, term *frame)
{
	return current_object;
}

static term p_this_slot(term goal, term *frame)
{
	return current_slot;
}

static term p_new_value(term goal, term *frame)
{
	return new_value;
}


static term p_old_value(term goal, term *frame)
{
	return old_value;
}

/************************************************************************/
/*	Store a frame in the data base					*/
/*	A frame is a set of attribute/value pairs			*/
/************************************************************************/

static void add_pair(term frame_name, term pair)
{
	if (TYPE(pair) != FN || ARITY(pair) != 2)
		fail("each element must be an attribute value pair");
	if (TYPE(ARG(1, pair)) != ATOM)
		fail("attribute must be an atom");

	term slot = ARG(1, pair);
	term datum = ARG(2, pair);

	switch (TYPE(datum))
	{
		case LIST:
		{
			for (term t = datum; t != _nil; t = CDR(t))
			{
				pq_fput(frame_name, slot, _value, CAR(t));
			}
			break;
		}
		case FN:
		{
			if (TYPE(datum) == FN && ARG(0, datum) == _lbrace)
			{
				term plist = ARG(1, datum);
				while (TYPE(plist) == FN && ARG(0, plist) == _semi_colon)
				{
					term pair = ARG(1, plist);
					print(pair);
					if (TYPE(ARG(1, pair)) != ATOM)
						fail("Facet must be an atom");
					pq_fput(frame_name, slot, ARG(1, pair), ARG(2, pair));
					plist = ARG(2, plist);
				}
				if (TYPE(plist) != FN || TYPE(ARG(1, plist)) != ATOM)
					fail("Facet must be an atom");
				pq_fput(frame_name, slot, ARG(1, plist), ARG(2, plist));
				break;
			}
		}
		default:
		{
			pq_fput(frame_name, slot, _value, datum);
			break;
		}
	}
}


static bool set_frame(term goal, term *frame)
{
	term frame_name = check_arg(1, goal, frame, ATOM, IN);
	term plist	= check_arg(2, goal, frame, FN, IN);

	if (frame_db == NULL)
		fail("No open database");
	if (ARG(0, plist) == _lbrace)
		plist = ARG(1, plist);
	else
		fail("Frame definition must be a property/value list");

	while (TYPE(plist) == FN && ARG(0, plist) == _semi_colon)
	{
		add_pair(frame_name, ARG(1, plist));
		plist = ARG(2, plist);
	}
	add_pair(frame_name, plist);
	return true;
}


/************************************************************************/
/*		Initialise Postgress frame database library		*/
/************************************************************************/

void init()
{
	void control_init();

	control_init();

	_value = intern("value");
	_default = intern("default");
	_if_needed = intern("if_needed");
	_if_added = intern("if_added");
	_if_removed = intern("if_removed");
	_if_replaced = intern("if_replaced");
	_if_new = intern("if_new");
	_if_deleted = intern("if_deleted");
	_if_new = intern("if_new");
	_multivalued = intern("multivalued");
	_cache = intern("cache");
	_help = intern("help");
	_range = intern("range");

	new_pred(frame_db_open, "open_db");
	new_pred(frame_db_close, "close_db");
	new_pred(p_fput,  "fput");
	new_pred(p_fget, "fget");
	new_pred(p_fremove, "fremove");
	new_pred(p_freplace, "freplace");

	new_subr(p_self,	"self");
	new_subr(p_this_slot,	"this_slot");
	new_subr(p_new_value,	"new_value");
	new_subr(p_old_value,	"old_value");

	defop(998, XFX, new_pred(set_frame, ":"));

	if (isatty(fileno(stdin)))
	{
		// Silliness to turn US date format into international format
		char day[3], month[4], year[5];
		sscanf(__DATE__, "%s %s %s", month, day, year);

		fprintf(stderr, "\nPostgres frame database (%s %s %s)\n", day, month, year);
	}
}
