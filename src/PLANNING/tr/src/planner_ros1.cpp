// Author: Claude Sammut

// TODO: directory for prolog folders 

#include "planner/planner_ros1.hpp"
#include <memory>

using namespace std::chrono_literals;

static PGconn *wm_db;			// connection to world model
static bool wm_db_open(term, term *);
static bool pl_can_see(term, term *);
static bool pl_drive(term, term *);
static bool pl_can_see(term, term *);

static const char *_can_see = "can_see";

Planner *current_node;

Planner::Planner(ros::NodeHandle nh) : nh_(nh)
{
	/************************************************************
	** Initialise iProlog
	************************************************************/
	pl_init();
	new_pred(pl_drive, (char *) "drive");
	new_pred(pl_can_see, (char *) "can_see");
	ONE((char *) "[]", (char *) "consult('tr_planner.pro')"); // absolute path
	ROS_INFO("TR planner loaded");

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	// Initialise publishers
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	/************************************************************
	** Initialise ROS timers
	** Open world model
	************************************************************/

	wm_db = PQconnectdb("dbname=world_model user=postgres password=password host=localhost port=5432");

	if (PQstatus(wm_db) == CONNECTION_BAD)
	{
		fprintf(stderr, "Connection to database failed: %s\n", PQerrorMessage(wm_db));
		PQfinish(wm_db);
		exit(1);
	}

	// Prepare statements
	PQprepare(wm_db, _can_see, "SELECT can_see($1)", 1, NULL);

	/************************************************************
	** Initialise ROS publishers
	************************************************************/

	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Planner::update_callback, this));

	ROS_INFO("iProlog interface node has been initialised");
}

Planner::~Planner()
{
	ROS_INFO("iProlog interface node has been terminated");
}


/********************************************************************************
** Callback functions for ROS publishers
********************************************************************************/

void Planner::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_.publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

void Planner::update_callback()
{
	current_node = this;
	pl_query(intern((char *) "planner"), _nil, 1);
}

/************************************************************************/
/*		Handle Postgres error then throw to Prolog error	*/
/************************************************************************/

static void db_error(PGresult *res)
{
	fprintf(stderr, "%s\n", PQerrorMessage(wm_db));

	PQclear(res);
	PQfinish(wm_db);

	exit(1);
}


/********************************************************************************
** Prolog hooks
********************************************************************************/

static bool pl_can_see(term goal, term *frame)
{
	term obj_class = check_arg(1, goal, frame, ATOM, IN);

	const char *param[1];
	param[0] = NAME(obj_class);
	PGresult *res = PQexecPrepared(wm_db, _can_see, 1, param, NULL, NULL, 0);

	if (PQresultStatus(res) != PGRES_COMMAND_OK)
		db_error(res);

	int rows = PQntuples(res);

	PQclear(res);

	return rows > 0 ? true : false;
}

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
	ros::init(argc, argv, "tr_planner_node");
	ros::NodeHandle nh;
	std::unique_ptr<Planner> planner = std::make_unique<Planner>(nh);
	ros::spin();
	ros::shutdown();

	return 0;
}
