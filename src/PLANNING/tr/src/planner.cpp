// Author: Claude Sammut

#include "planner/planner.hpp"

#include <memory>


using namespace std::chrono_literals;

static bool pl_drive(term, term *);
Planner *current_node;

Planner::Planner() : Node("tr_planner_node")
{
	/************************************************************
	** Initialise iProlog
	************************************************************/
	pl_init();
	new_pred(wm_db_open, (char *) "wm_open");
	new_pred(pl_drive, (char *) "drive");
	ONE((char *) "[]", (char *) "consult('tr_planner.pro')");
	RCLCPP_INFO(this->get_logger(), "TR planner loaded");

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);


	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&Planner::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "iProlog interface node has been initialised");
}

Planner::~Planner()
{
	RCLCPP_INFO(this->get_logger(), "iProlog interface node has been terminated");
}


/********************************************************************************
** Open database
********************************************************************************/

static bool wm_db_open(term goal, term *frame)
{
	// Open frame store

	wm_db = PQconnectdb("user=posgres dbname=world_model");

	if (PQstatus(frame_db) == CONNECTION_BAD)
	{
		fprintf(stderr, "Connection to database failed: %s\n", PQerrorMessage(frame_db));
		PQfinish(frame_db);
		exit(1);
	}

	// Create new frame table, deleting any previous one

	// Prepare statements

	PQprepare(wm_db, _put,     "INSERT INTO frame VALUES ($1, $2, $3, $4)", 4, NULL);
	PQprepare(wm_db, _get,     "SELECT datum FROM frame WHERE frame_name = $1 AND slot = $2 AND facet = $3", 3, NULL);
	PQprepare(wm_db, _remove,  "DELETE FROM frame WHERE frame_name = $1 AND slot = $2 AND facet = $3", 3, NULL);


	return true;
}


/************************************************************************/
/* Adds a tuple to the world model					*/
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

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

void Planner::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

void Planner::update_callback()
{
	current_node = this;
	pl_query(intern((char *) "planner"), _nil, 1);
}


/********************************************************************************
** Prolog hooks
********************************************************************************/


bool pl_drive(term goal, term *frame)
{
	term lv = check_arg(1, goal, frame, REAL, EVAL);
	term av = check_arg(2, goal, frame, REAL, EVAL);

	current_node -> update_cmd_vel(RVAL(lv), RVAL(av));
	return true;
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Planner>());
	rclcpp::shutdown();

	return 0;
}
