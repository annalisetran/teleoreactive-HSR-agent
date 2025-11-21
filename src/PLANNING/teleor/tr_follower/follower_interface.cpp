#include "QuProlog.h"

// Normally we would construct QP terms representing the percepts but this
// is reasonably inefficient as we repeatedly construct the same sort of
// compound terms with the only thing different from call to call is
// the distance arguments of each structure.
// What we do instead is pass in the terms that are the percept terms and
// we simply update the arguments.
// The slight twist is that if near start is true we simply cons  that term
// on to the list of scan terms.

// List to return as part of percepts
Object* scan_list;
// Add to above list if near to become full percept list
Object* near_start_term;
// The individual scan terms - the argument is to be modified based on
// the result of the last scan
Structure* front_term;           //front(num)       scan_data index = 0
Structure* front_left_term;      //front_left(num)  scan_data index = 1
Structure* left_front_term;      //left_front(num)  scan_data index = 2
Structure* front_right_term;     //front_right(num)  scan_data index = 11

// used only when node is closed
Object* previous_percepts;

extern "C" void *start_nodes(void);
extern "C" void drive(double linear, double angular);
extern "C" bool get_percepts(bool *near, double *front, double *front_left,
			     double *left_front, double *front_right);


// The interface between the QP level and the C-level code for
// creating the nodes
extern "C" bool
start_nodes_interface(ForeignInterface* fi)
{
  Object* tmp;
  Object* in_scan_list;
  // The list of scan terms in the specific order as given above
  // (and in the QP code)
  in_scan_list = fi->getXReg(0);
  // The near start term
  Object* in_near_start_term;
  in_near_start_term = fi->getXReg(1);
  
  scan_list = in_scan_list;
  previous_percepts = scan_list;
  near_start_term = in_near_start_term;

  // Error checking is added just in case.
  if (!in_scan_list->isCons())
    return false;
  tmp  = reinterpret_cast<Cons*>(in_scan_list)->getHead();
  if (!tmp->isStructure())
    return false;
  front_term = reinterpret_cast<Structure*>(tmp);
  in_scan_list = reinterpret_cast<Cons*>(in_scan_list)->getTail();
  if (!in_scan_list->isCons())
    return false;
  tmp = reinterpret_cast<Cons*>(in_scan_list)->getHead();
  if (!tmp->isStructure())
    return false;

  // Extract direction terms from in_scan_list
  front_left_term = reinterpret_cast<Structure*>(tmp);
  in_scan_list = reinterpret_cast<Cons*>(in_scan_list)->getTail();
  if (!in_scan_list->isCons())
    return false;
  tmp = reinterpret_cast<Cons*>(in_scan_list)->getHead();
  if (!tmp->isStructure())
    return false;
  left_front_term = reinterpret_cast<Structure*>(tmp);
  in_scan_list = reinterpret_cast<Cons*>(in_scan_list)->getTail();
  if (!in_scan_list->isCons())
    return false;
  tmp = reinterpret_cast<Cons*>(in_scan_list)->getHead();
  if (!tmp->isStructure())
    return false;
  front_right_term = reinterpret_cast<Structure*>(tmp);

  // Start up the Percept and Action nodes
  start_nodes();
  return true;
  
}

extern "C" bool
get_percepts_interface(ForeignInterface* fi)
{
  Object* object0;
  object0 = fi->getXReg(0);
  double front, front_left, left_front, front_right;
  bool near;

  // get the sensor data
  bool call_ok =
    get_percepts(&near, &front, &front_left, &left_front, &front_right);

  if (call_ok) {
    // update the arguments of the scan terms
    front_term->setArgument(1, fi->makeDouble(front));
    front_left_term->setArgument(1, fi->makeDouble(front_left));
    left_front_term->setArgument(1, fi->makeDouble(left_front));
    front_right_term->setArgument(1, fi->makeDouble(front_right));
    
    Object* pterm;
    if (near) {
      //cons on near_start_term so it becomes a percept
      pterm = fi->makeCons(near_start_term, scan_list);
    } else {
      // near is not true so we ignore
      pterm = scan_list;
    }
    return fi->unify(object0, pterm);
  }
  // call failed (because node exited) - return a default
  return fi->unify(object0, previous_percepts);
  
}

extern "C" bool drive_interface(ForeignInterface* fi)
{
  Object* object0;
  object0 = fi->getXReg(0);
  Object* object1;
  object1 = fi->getXReg(1);
  double linear;
  double angular;
  if (object0->isDouble())
    linear = object0->getDouble();
  else if (object0->isShort() || object0->isLong())
    linear = object0->getInteger();
  else
    return false;
  if (object1->isDouble())
    angular = object1->getDouble();
  else if (object1->isShort() || object1->isLong())
    angular = object1->getInteger();
  else
    return false;
  
  drive(linear, angular);
  return true;
}
 
