#include <unistd.h>
#include <stdarg.h>
#include <stdio.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <string.h>

#define Sleep(msec) usleep(( msec) * 1000)

#define L_MOTOR_PORT      OUTPUT_B
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_C
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define S_MOTOR_PORT      OUTPUT_D
#define S_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define E_MOTOR_PORT      OUTPUT_A
#define E_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define IR_CHANNEL        0

#define SPEED_LINEAR      75  /* Motor speed for linear motion, in percent */
#define SPEED_CIRCULAR    50  /* ... for circular motion */

#define DEGREE_TO_COUNT(d) ((d) * 260 / 90)


static int max_drive_speed, max_small_motor_speed, max_extra_motor_speed;  /* Maximum motor speed */

enum {L, R, S, E};

uint8_t touch_l, touch_r, us, gyro; 				/* Sequence numbers of sensors */
uint8_t motor[3] = {DESC_LIMIT, DESC_LIMIT, DESC_LIMIT}; 	/* Sequence numbers of motors */


// The library version seems to be using an old way of accessing the ports
// in /sys/class/tacho-motor/motor*/address.
// In this version we search for the string ev3-ports:out

#define PORTS_PREFIX_LEN 13

void fixed_ev3_parse_out_port_name( char *name, uint8_t *port,
                                    uint8_t *extport, uint8_t *addr )
{
  *port = EV3_PORT__NONE_;
  *extport = EXT_PORT__NONE_;
  *addr = 0;

  if(strncmp(name, "ev3-ports:out", PORTS_PREFIX_LEN) == 0) {
    /* "out" */
    name += PORTS_PREFIX_LEN;
    *port = *name++;
    if ( *name != ':' ) return;
    ++name;
    
    return;
  }										
  return;
}


size_t fixed_get_tacho_desc( uint8_t sn, EV3_TACHO *desc )
{
        uint8_t addr;
        char buf[ 32 ];

        desc->type_inx = get_tacho_type_inx( sn );
        if ( desc->type_inx == TACHO_TYPE__NONE_ ) return ( 0 );

        if ( !get_tacho_address( sn, buf, sizeof( buf ))) return ( 0 );

        fixed_ev3_parse_out_port_name( buf, &desc->port, &desc->extport, &addr );
        
        return ( sizeof( EV3_TACHO ));
}


int fixed_ev3_tacho_init( void )
{
	char list[ 256 ];
	char *p;
	uint32_t sn;
	int cnt = 0;

        // initialize the ev3_tacho data structure
        memset(ev3_tacho, 0, sizeof(ev3_tacho));

	if ( !ev3_listdir( "/sys/class/tacho-motor", list, sizeof( list )))
          return ( -1 );

	p = strtok( list, " " );
	while ( p ) {
		if (( ev3_string_suffix( "motor", &p, &sn ) == 1 ) &&
                    ( sn < TACHO_DESC__LIMIT_)) {
                        fixed_get_tacho_desc(sn, ev3_tacho + sn );

			++cnt;
		}
		p = strtok( NULL, " " );
	}
	return ( cnt );
}


/************************************************************************/
/*			     EV3 Initialisation      			*/
/************************************************************************/

extern bool p_ev3_init()
{
	char s[256];

	fprintf(stderr, "Waiting the EV3 brick online...\n" );
	if (ev3_init() < 1) return false;

	fprintf(stderr, "*** (EV3) Hello! ***\n");
	ev3_sensor_init();
        
	while (fixed_ev3_tacho_init() < 1) { fprintf(stderr, "sleep\n");Sleep(1000);}

	if (ev3_search_tacho_plugged_in(L_MOTOR_PORT, L_MOTOR_EXT_PORT, motor + L, 0))
	{
		fprintf(stderr, "Found left motor (%s).\n", ev3_port_name(L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s));
		get_tacho_max_speed(motor[L], &max_drive_speed);
		fprintf(stderr, "max_drive_speed = %d\n", max_drive_speed);
		set_tacho_command_inx(motor[L], TACHO_RESET);	// reset motor
	} else {
		fprintf(stderr, "L motor not found %d\n", L_MOTOR_PORT);
		if (ev3_search_tacho(LEGO_EV3_L_MOTOR, motor+L, 0)) {
			fprintf(stderr, "L found  %d %d\n",motor[L], ev3_tacho_desc_port( motor[L]));
		}
	}
	if (ev3_search_tacho_plugged_in(R_MOTOR_PORT, R_MOTOR_EXT_PORT, motor + R, 0))
	{
		fprintf(stderr, "Found right motor (%s).\n", ev3_port_name(R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s));
		get_tacho_max_speed(motor[R], &max_drive_speed);
		fprintf(stderr, "max_drive_speed = %d\n", max_drive_speed);
		set_tacho_command_inx(motor[R], TACHO_RESET);	// reset motor
	} else {
		fprintf(stderr, "R motor not found %d\n", R_MOTOR_PORT);
		if (ev3_search_tacho(LEGO_EV3_L_MOTOR, motor+R, motor[L]+1)) {
			fprintf(stderr, "R found  %d %d\n",motor[R], ev3_tacho_desc_port( motor[R]));
		}
	}
	if (ev3_search_tacho_plugged_in(S_MOTOR_PORT, S_MOTOR_EXT_PORT, motor + S, 0))
	{
		fprintf(stderr, "Found small motor (%s).\n", ev3_port_name(S_MOTOR_PORT, S_MOTOR_EXT_PORT, 0, s));
		get_tacho_max_speed(motor[S], &max_small_motor_speed);
		fprintf(stderr, "max_small_motor_speed = %d\n", max_drive_speed);
		set_tacho_command_inx(motor[S], TACHO_RESET);	// reset motor
	}
	if (ev3_search_tacho_plugged_in(E_MOTOR_PORT, E_MOTOR_EXT_PORT, motor + E, 0))
	{
		fprintf(stderr, "Found extra motor (%s).\n", ev3_port_name(E_MOTOR_PORT, E_MOTOR_EXT_PORT, 0, s));
		get_tacho_max_speed(motor[E], &max_extra_motor_speed);
		fprintf(stderr, "max_extra_motor_speed = %d\n", max_drive_speed);
		set_tacho_command_inx(motor[E], TACHO_RESET);	// reset motor
	}

	if (ev3_search_sensor(LEGO_EV3_TOUCH, &touch_l, 0))
	{
		fprintf(stderr, "Found left touch sensor\n");
	}

	if (ev3_search_sensor(LEGO_EV3_TOUCH, &touch_r, 1))
	{
		fprintf(stderr, "Found right touch sensor\n");
	}

	if (ev3_search_sensor(LEGO_EV3_US, &us, 0))
	{
		fprintf(stderr, "Found ultrasonic sensor\n");
	}

	if (ev3_search_sensor(LEGO_EV3_GYRO, &gyro, 0))
	{
		fprintf(stderr, "Found gyro\n");
	}

	return true;
}


extern bool p_ev3_finish()
{
	ev3_uninit();
	fprintf(stderr, "*** (EV3) Bye! ***\n" );
	return true;
}

/************************************************************************/
/*				Motor Commands      			*/
/************************************************************************/

extern void p_ev3_run_forever(int l_speed, int r_speed)
{
  //fprintf(stderr, "run_forever(%d, %d)\n", l_speed, r_speed );
  set_tacho_speed_sp(motor[L], (l_speed * max_drive_speed)/100);
  set_tacho_speed_sp(motor[R], (r_speed * max_drive_speed)/100);
  //	multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);
  set_tacho_command_inx(motor[L], TACHO_RUN_FOREVER);
  set_tacho_command_inx(motor[R], TACHO_RUN_FOREVER);
  //fprintf(stderr, "end_run_forever(%d, %d)\n", l_speed, r_speed );
}

extern void p_ev3_run_to_rel_pos(int l_speed, int l_pos, int r_speed, int r_pos)
{
  set_tacho_speed_sp(motor[L], (l_speed * max_drive_speed)/100);
  set_tacho_speed_sp(motor[R], (r_speed * max_drive_speed)/100);
  set_tacho_position_sp(motor[L], l_pos);
  set_tacho_position_sp(motor[R], r_pos);
  //	multi_set_tacho_command_inx(motor, TACHO_RUN_TO_REL_POS);
  set_tacho_command_inx(motor[L], TACHO_RUN_TO_REL_POS);
  set_tacho_command_inx(motor[R], TACHO_RUN_TO_REL_POS);
}

extern void p_ev3_run_timed(int l_speed, int r_speed, int ms)
{
  set_tacho_speed_sp(motor[L], (l_speed * max_drive_speed)/100);
  set_tacho_speed_sp(motor[R], (r_speed * max_drive_speed)/100);
  multi_set_tacho_time_sp(motor, ms);
  //	multi_set_tacho_command_inx(motor, TACHO_RUN_TIMED);
  set_tacho_command_inx(motor[L], TACHO_RUN_TIMED);
  set_tacho_command_inx(motor[R], TACHO_RUN_TIMED);
}

extern void p_ev3_set_position(int pos)
{
  //	fprintf(stderr, "Set to absolute position (%d)\n", pos);
  set_tacho_speed_sp(motor[S], max_small_motor_speed / 2);
  set_tacho_ramp_up_sp(motor[S], 500);
  set_tacho_ramp_down_sp(motor[S], 500);
  set_tacho_position_sp(motor[S], pos);
  
  set_tacho_command_inx(motor[S], TACHO_RUN_TO_ABS_POS);
  Sleep(500);
}

extern int p_ev3_is_running(void)
{
  FLAGS_T state = TACHO_STATE__NONE_;
  
  get_tacho_state_flags(motor[L], &state);
  if (state != TACHO_STATE__NONE_) return 1;
  get_tacho_state_flags(motor[R], &state);
  if (state != TACHO_STATE__NONE_) return 1;
  
  return 0;
}

extern void p_ev3_get_tacho_speed(int *l_speed, int *r_speed)
{
  int *l_speed_sp, *r_speed_sp;
  get_tacho_speed_sp(motor[L], l_speed_sp);
  get_tacho_speed_sp(motor[R], r_speed_sp);
  *l_speed = (*l_speed_sp * 100) / max_drive_speed;
  *r_speed = (*r_speed_sp * 100) / max_drive_speed;
}

extern void p_ev3_stop(void)
{
  //fprintf(stderr, "start stop\n");
  multi_set_tacho_command_inx(motor, TACHO_STOP);
  //fprintf(stderr, "end stop\n");
}


/************************************************************************/
/*				Sensor Queries      			*/
/************************************************************************/


extern int p_ev3_touch(int sensor)
{
	int val;

	//fprintf(stderr, "start touch\n");
	if (! get_sensor_value(0, sensor, &val))
		fprintf(stderr, "Could not read touch sensor: %d", sensor);

	//fprintf(stderr, "end touch(%d, %d)\n", sensor, val);
	return val;
}


extern int p_ev3_ultrasonic()
{
	int val;

	if (! get_sensor_value(0, us, &val))
		fprintf(stderr, "Could not read ultrasonic");

	return val;
}


extern int p_ev3_gyro()
{
	int val;

	if (! get_sensor_value(0, gyro, &val))
		fprintf(stderr, "Could not read gyro");

	return val;
}
