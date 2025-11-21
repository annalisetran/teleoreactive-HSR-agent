//#include <stdio.h>
#include "QuProlog.h"
extern "C" void p_ev3_init(void);
extern "C" void p_ev3_finish(void);
extern "C" void p_ev3_run_forever(long, long);
extern "C" void p_ev3_run_to_rel_pos(long, long, long, long);
extern "C" void p_ev3_run_timed(long, long, long);
extern "C" void p_ev3_set_position(long);
extern "C" long p_ev3_is_running(void);
extern "C" void p_ev3_get_tacho_speed(long*, long*);
extern "C" void p_ev3_stop(void);
extern "C" long p_ev3_touch(long);
extern "C" long p_ev3_ultrasonic(void);
extern "C" long p_ev3_gyro(void);

extern "C" bool
p_ev3_init_interface(ForeignInterface* fi)
{
 bool result = true;



 p_ev3_init();

 return(result);
}

extern "C" bool
p_ev3_finish_interface(ForeignInterface* fi)
{
 bool result = true;



 p_ev3_finish();

 return(result);
}

extern "C" bool
p_ev3_run_forever_interface(ForeignInterface* fi)
{
//fprintf(stderr, "start run forever interface\n");
 bool result = true;
 long integer0;
 long integer1;
 Object* object0;
 Object* object1;

 object0 = fi->getXReg(0);
 if (!object0->isInteger())
 {
  return(false);
 }
 integer0 = object0->getInteger();
 object1 = fi->getXReg(1);
 if (!object1->isInteger())
 {
  return(false);
 }
 integer1 = object1->getInteger();


 p_ev3_run_forever(integer0, integer1);

//fprintf(stderr, "end run forever interface\n");
 return(result);
}

extern "C" bool
p_ev3_run_to_rel_pos_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 long integer1;
 long integer2;
 long integer3;
 Object* object0;
 Object* object1;
 Object* object2;
 Object* object3;

 object0 = fi->getXReg(0);
 if (!object0->isInteger())
 {
  return(false);
 }
 integer0 = object0->getInteger();
 object1 = fi->getXReg(1);
 if (!object1->isInteger())
 {
  return(false);
 }
 integer1 = object1->getInteger();
 object2 = fi->getXReg(2);
 if (!object2->isInteger())
 {
  return(false);
 }
 integer2 = object2->getInteger();
 object3 = fi->getXReg(3);
 if (!object3->isInteger())
 {
  return(false);
 }
 integer3 = object3->getInteger();


 p_ev3_run_to_rel_pos(integer0, integer1, integer2, integer3);

 return(result);
}

extern "C" bool
p_ev3_run_timed_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 long integer1;
 long integer2;
 Object* object0;
 Object* object1;
 Object* object2;

 object0 = fi->getXReg(0);
 if (!object0->isInteger())
 {
  return(false);
 }
 integer0 = object0->getInteger();
 object1 = fi->getXReg(1);
 if (!object1->isInteger())
 {
  return(false);
 }
 integer1 = object1->getInteger();
 object2 = fi->getXReg(2);
 if (!object2->isInteger())
 {
  return(false);
 }
 integer2 = object2->getInteger();


 p_ev3_run_timed(integer0, integer1, integer2);

 return(result);
}

extern "C" bool
p_ev3_set_position_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 Object* object0;

 object0 = fi->getXReg(0);
 if (!object0->isInteger())
 {
  return(false);
 }
 integer0 = object0->getInteger();


 p_ev3_set_position(integer0);

 return(result);
}

extern "C" bool
p_ev3_is_running_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 Object* object0;
 Object* outarg0;


 object0 = fi->getXReg(0);

 integer0 = p_ev3_is_running();
 outarg0 = fi->makeInteger(integer0);
 result = result && fi->unify(object0, outarg0);

 return(result);
}

extern "C" bool
p_ev3_get_tacho_speed_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 long integer1;
 Object* object0;
 Object* object1;
 Object* outarg0;
 Object* outarg1;

 object0 = fi->getXReg(0);
 object1 = fi->getXReg(1);


 p_ev3_get_tacho_speed(&integer0, &integer1);
 outarg0 = fi->makeInteger(integer0);
 result = result && fi->unify(object0, outarg0);
 outarg1 = fi->makeInteger(integer1);
 result = result && fi->unify(object1, outarg1);

 return(result);
}

extern "C" bool
p_ev3_stop_interface(ForeignInterface* fi)
{
 bool result = true;

//fprintf(stderr, "start stop interface\n");


 p_ev3_stop();
//fprintf(stderr, "stop stop interface\n");

 return(result);
}

extern "C" bool
p_ev3_touch_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 long integer1;
 Object* object0;
 Object* object1;
 Object* outarg1;

//fprintf(stderr, "start touch interface\n");
 object0 = fi->getXReg(0);
 if (!object0->isInteger())
 {
  return(false);
 }
 integer0 = object0->getInteger();

 object1 = fi->getXReg(1);

 integer1 = p_ev3_touch(integer0);
 outarg1 = fi->makeInteger(integer1);
 result = result && fi->unify(object1, outarg1);
//fprintf(stderr, "stop touch interface\n");

 return(result);
}

extern "C" bool
p_ev3_ultrasonic_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 Object* object0;
 Object* outarg0;


 object0 = fi->getXReg(0);

 integer0 = p_ev3_ultrasonic();
 outarg0 = fi->makeInteger(integer0);
 result = result && fi->unify(object0, outarg0);

 return(result);
}

extern "C" bool
p_ev3_gyro_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 Object* object0;
 Object* outarg0;


 object0 = fi->getXReg(0);

 integer0 = p_ev3_gyro();
 outarg0 = fi->makeInteger(integer0);
 result = result && fi->unify(object0, outarg0);

 return(result);
}
