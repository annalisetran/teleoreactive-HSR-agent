
/*
  Copyright (C) 2006, 2007, 2008 Peter Robinson
  Email: pjr@itee.uq.edu.au

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#ifdef WIN32

#include <windows.h>
#define u_int UINT

#endif

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include "machine.h"
#include "atoms.h"

extern void logit(char* fmt, ...);

typedef struct
{ gboolean is_int;      
  union {
    long  i;                  
    double d;                  
  } value;
} number;

#define IS_ZERO(x) ((x.is_int) && (x.value.i == 0))

#define IS_INT(x) (x.is_int)
#define BOTH_INTS(x,y) ((x.is_int) && (y.is_int))
#define GET_INT_VAL(x) (x.value.i)
#define GET_DOUBLE_VAL(x) ((x.is_int) ? (double)(x.value.i) : x.value.d)
#define MAKE_INT(x, v) {x.is_int = TRUE; x.value.i = v;}
#define MAKE_DOUBLE(x, v) {x.is_int = FALSE; x.value.d = v;}

double round(double number)
{
  return number < 0.0 ? ceil(number-0.5) : floor(number+0.5);
}


number zero = {TRUE, 0};

number arithEvaluate(ObjectPtr t, gboolean* ok)
{
  t = dereference(t);
  if (IS_INTEGER(t)) {
    number y;
    MAKE_INT(y, GET_INTEGER(t));
    return y;
  }
  else if (IS_DOUBLE(t)) {
    number y;
    MAKE_DOUBLE(y, GET_DOUBLE(t));
    return y;
  }
  else if (IS_ATOM(t)) { 
    if (t == a_pi) {
      number y;
      MAKE_DOUBLE(y, G_PI); // PI
      return y;
    }
    if (t == a_e) {
      number y;
      MAKE_DOUBLE(y, G_E); // E
      return y;
    }
    else {
      *ok = FALSE;
      return zero;
    }
  }      
  else if (IS_COMPOUND(t)) {
    number res1, res2;
 
    size_t arity = GET_TAG_INFO(t);
      
    //
    // calcualte the values of the arguments
    //
    if (arity == 1) {
      res1 = arithEvaluate(GET_ARG(t, 1), ok);
	  
      if (!*ok) {
	return zero;
      }
    }
    else if (arity == 2) {
      res1 = arithEvaluate(GET_ARG(t, 1), ok);
      if (!*ok) {
	return zero;
      }
      res2 = arithEvaluate(GET_ARG(t, 2), ok);
      if (!*ok) {
	return zero;
      }

    }
    else {         // illegal number of arguments
      *ok = FALSE;
      return zero;
    }
      
    //
    // evaluate the expression according to the operator
    //
    ObjectPtr op = GET_ARG(t, 0);
      
    if (op == a_plus) {
      if (arity != 2) {
	*ok = FALSE;
	return zero;
      }
      number res;
      if BOTH_INTS(res1, res2) {
	number res;
	MAKE_INT(res, GET_INT_VAL(res1) + GET_INT_VAL(res2));
	return res;
      }
      else {
	MAKE_DOUBLE(res, GET_DOUBLE_VAL(res1) + GET_DOUBLE_VAL(res2));
	return res;
      }
    }
    else if (op == a_minus) {
      if (arity == 1) {
	if IS_INT(res1) {
	  number res;
	  MAKE_INT(res, - GET_INT_VAL(res1));
	  return res;
	}
	else {
	  number res;
	  MAKE_DOUBLE(res, - GET_DOUBLE_VAL(res1));
	  return res;
	}
      }
      else if (arity == 2) {
	if BOTH_INTS(res1, res2) {
	  number res;
	  MAKE_INT(res, GET_INT_VAL(res1) - GET_INT_VAL(res2));
	  return res;
	}
	else {
	  number res;
	  MAKE_DOUBLE(res, GET_DOUBLE_VAL(res1) - GET_DOUBLE_VAL(res2));
	  return res;
	}
      }
      else {
	*ok = FALSE;
	return zero;
      }
    }
    else if (op == a_multiply) {
      if (arity != 2) {
	*ok  = FALSE;
	return zero;
      }
      if BOTH_INTS(res1, res2) {
	number res;
	MAKE_INT(res, GET_INT_VAL(res1) * GET_INT_VAL(res2));
	return res;
      }
      else {
	number res;
	errno = 0;
	MAKE_DOUBLE(res, GET_DOUBLE_VAL(res1) * GET_DOUBLE_VAL(res2));
	if (errno != 0) {
	  *ok = FALSE;
	  return zero;
	}
	return res;
      }
    }
    else if (op == a_abs) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      if IS_INT(res1) {
	number res;
	MAKE_INT(res, abs(GET_INT_VAL(res1)));
	return res;
      }
      else {
	number res;
	MAKE_DOUBLE(res, fabs(GET_DOUBLE_VAL(res1)));
	return res;
      }
    }
    else if (op == a_round) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      if IS_INT(res1) {
	return res1;
      }
      else {
	number res;
	double r = round(GET_DOUBLE_VAL(res1));
	MAKE_INT(res, (long)r);
	return res;
      }
    }
    else if (op == a_floor) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      if IS_INT(res1) {
	return res1;
      }
      else {
	number res;
	MAKE_INT(res, (long)floor(GET_DOUBLE_VAL(res1)));
	return res;
      }
    }
    else if (op == a_ceiling) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      if IS_INT(res1) {
	return res1;
      }
      else {
	number res;
	MAKE_INT(res, (long)ceil(GET_DOUBLE_VAL(res1)));
	return res;
      }
    }
    else if (op == a_sqrt) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      number res;
      MAKE_DOUBLE(res, sqrt(GET_DOUBLE_VAL(res1)));
      return res;
    }
    else if (op == a_sin) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      number res;
      MAKE_DOUBLE(res, sin(GET_DOUBLE_VAL(res1)));
      return res;
    }
    else if (op == a_cos) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      number res;
      MAKE_DOUBLE(res, cos(GET_DOUBLE_VAL(res1)));
      return res;
    }
    else if (op == a_tan) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      number res;
      errno = 0;
      MAKE_DOUBLE(res, tan(GET_DOUBLE_VAL(res1)));
      if (errno != 0) {
	*ok = FALSE;
	return zero;
      }
      return res;
    }
    else if (op == a_asin) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      number res;
      errno = 0;
      MAKE_DOUBLE(res, asin(GET_DOUBLE_VAL(res1)));
      if (errno != 0) {
	*ok = FALSE;
	return zero;
      }
      return res;
    }
    else if (op == a_acos) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      number res;
      errno = 0;
      MAKE_DOUBLE(res, acos(GET_DOUBLE_VAL(res1)));
      if (errno != 0) {
	*ok = FALSE;
	return zero;
      }
      return res;
    }
    else if (op == a_atan) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      number res;
      errno = 0;
      MAKE_DOUBLE(res, atan(GET_DOUBLE_VAL(res1)));
      if (errno != 0) {
	*ok = FALSE;
	return zero;
      }
      return res;
    }
    else if (op == a_log) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      number res;
      errno = 0;
      MAKE_DOUBLE(res, log(GET_DOUBLE_VAL(res1)));
      if (errno != 0) {
	*ok = FALSE;
	return zero;
      }
      return res;
    }
    else if (op == a_divide) {
      if (arity != 2) {
	*ok = FALSE;
	return zero;
      }
      if IS_ZERO(res2) {
	*ok = FALSE;
	return zero;
      }
      number res;
      errno = 0;
      MAKE_DOUBLE(res, (double)GET_DOUBLE_VAL(res1) 
		  / (double)GET_DOUBLE_VAL(res2));
      if (errno != 0) {
	*ok = FALSE;
	return zero;
      }
      return res;
    }
    else if (op == a_intdivide) {
      if (arity != 2) {
	*ok = FALSE;
	return zero;
      }
      if IS_ZERO(res2) {
	*ok = FALSE;
	return zero;
      }
      if BOTH_INTS(res1, res2) {
	number res;
	MAKE_INT(res, GET_INT_VAL(res1) / GET_INT_VAL(res2));
	return res;
      }
      else {
	*ok = FALSE;
	return zero;
      }
    }
    else if (op == a_mod){
      if (arity != 2) {
	*ok = FALSE;
	return zero;
      }	      
      if IS_ZERO(res2) {
	*ok = FALSE;
	return zero;
      }

      if BOTH_INTS(res1, res2) {
	number res;
	MAKE_INT(res, GET_INT_VAL(res1) % GET_INT_VAL(res2));
	return res;
      }
      else {
	*ok = FALSE;
	return zero;
      }
    }
    else if (op == a_power) {
      if (arity != 2) {
	*ok = FALSE;
	return zero;
      }
      if (BOTH_INTS(res1, res2) && (GET_INT_VAL(res2) >= 0)) {
	number res;
	MAKE_INT(res, pow(GET_INT_VAL(res1), GET_INT_VAL(res2)));
	return res;
      }
      else {
	number res;
	double x = GET_DOUBLE_VAL(res1);
	double y = GET_DOUBLE_VAL(res2);
	errno = 0;
	double z = pow(x, y);
	if (errno != 0) {
	  *ok = FALSE;
	  return zero;
	}
	  
	MAKE_DOUBLE(res, z);
	return res;
      }
    }
    else if (op == a_bitwiseand) {
      if (arity != 2) {
	*ok = FALSE;
	return zero;
      }
      if BOTH_INTS(res1, res2) {
	number res;
	MAKE_INT(res, GET_INT_VAL(res1) & GET_INT_VAL(res2));
	return res;
      }
      else {
	*ok = FALSE;
	return zero;
      }
    }
    else if (op == a_bitwiseor) {
      if (arity != 2) {
	*ok = FALSE;
	return zero;
      }
      if BOTH_INTS(res1, res2) {
	number res;
	MAKE_INT(res, GET_INT_VAL(res1) | GET_INT_VAL(res2));
	return res;
      }
      else {
	*ok = FALSE;
	return zero;
      }
    }
    else if (op == a_shiftl) {
      if (arity != 2) {
	*ok ;
	return zero;
      }
      if BOTH_INTS(res1, res2) {
	number res;
	MAKE_INT(res, GET_INT_VAL(res1) << GET_INT_VAL(res2));
	return res;
      }
      else {
	*ok = FALSE;
	return zero;
      }
    }
    else if (op == a_shiftr) {
      if (arity != 2) {
	*ok = FALSE;
	return zero;
      }
      if BOTH_INTS(res1, res2) {
	number res;
	MAKE_INT(res, GET_INT_VAL(res1) >> GET_INT_VAL(res2));
	return res;
      }
      else {
	*ok = FALSE;
	return zero;
      }
    }
    else if (op == a_bitneg) {
      if (arity != 1) {
	*ok = FALSE;
	return zero;
      }
      if IS_INT(res1) {
	number res;
	MAKE_INT(res, ~ GET_INT_VAL(res1));
	return res;
      }
      else {
	*ok = FALSE;
	return zero;
      }
    }
    else {
      *ok = FALSE;
      return zero;
    }
  }
  else if (IS_VAR(t)) {
    *ok = FALSE;
    return zero;
  }
  else {
    *ok = FALSE;
    return zero;
  }

  return zero;
}


gboolean arith_simplify(ObjectPtr in, ObjectPtr* out, Heap* heap)
{
  gboolean ok = TRUE;
  number result = arithEvaluate(in, &ok);
  if (!ok) return FALSE;
  if (result.is_int) {
    *out = pushInteger(heap, result.value.i);
  }
  else {
    *out = pushDouble(heap, result.value.d);
  }
  return TRUE;
}


void bind_to(ObjectPtr o1, ObjectPtr o2, Trail* t)
{
  assert(IS_VAR(o1));
  *o1 = (heapobject)o2;
  trail_push(t, o1);
}

gboolean is_proper_list(ObjectPtr term)
{
  ObjectPtr t = dereference(term);

  while (IS_LIST(t)) {
    t = dereference(GET_TAIL(t));
  }
  return (t == a_nil);
}

#define M_FAILED do { stack_set_top(stk, stop); return FALSE; } while(0)

#define UNIFIED                                      \
  do {                                               \
    if (stop == STACK_GET_TOP(stk))                  \
      return TRUE;                                   \
    o1 = dereference(stack_pop(stk));                \
    o2 = dereference(stack_pop(stk));                \
    goto start;                                      \
  } while(0)

#define BIND                                         \
  do {                                               \
    if (octop == STACK_GET_TOP(stk))                 \
      { bind_to(var, savedterm, t);  UNIFIED; }	     \
    octerm = stack_pop(stk);                         \
    goto occurs;                                     \
  } while (0)

/*
 * unify uses a stack to store unification subproblems
 */
gboolean unify(ObjectPtr o1, ObjectPtr o2, Trail* t, Stack* stk)
{
  stack_reset(stk);
  ObjectPtr* stop = STACK_GET_TOP(stk);
  o1 = dereference(o1);
  o2 = dereference(o2);
  int type1;
  int type2;
  int arity;
  ObjectPtr var;
  ObjectPtr savedterm;
  ObjectPtr octerm;
  ObjectPtr ocstruct;
  ObjectPtr* octop;

 start:

  if (o1 == o2) UNIFIED;

  type1 = GET_TYPE(o1);

  if (type1 == TYPEVAR && !IS_FROZEN_VAR(o1)) { 
      var = o1;
      octerm = o2;
      savedterm = o2;
      octop = STACK_GET_TOP(stk);
      goto occurs;
    }

  type2 = GET_TYPE(o2);

  if (type2 == TYPEVAR && !IS_FROZEN_VAR(o2)) {
      var = o2;
      octerm = o1;
      savedterm = o1;
      octop = STACK_GET_TOP(stk);
      goto occurs;
    }

  if (type1 != type2) M_FAILED;
  /* trying to bind frozen var */
  if (IS_FROZEN_VAR(o1) || IS_FROZEN_VAR(o2)) M_FAILED;

  switch (type1) {
  case TYPEINTEGER: 
    {
      if (GET_INTEGER(o1) == GET_INTEGER(o2)) UNIFIED;
      M_FAILED;
      break;
    }
  case TYPEDOUBLE:
    {
      if (GET_DOUBLE(o1) == GET_DOUBLE(o2)) UNIFIED;
      M_FAILED;
      break;
    }
  case TYPEATOM:
    {
      M_FAILED;
      break;
    }
  case TYPEDYNATOM:
    {
      char* s1 = GET_DYNATOM_NAME(o1);
      char* s2 = GET_DYNATOM_NAME(o2);
      if (strcmp(s1, s2) == 0) UNIFIED;
      M_FAILED;
      break;
    }
  case TYPESTRING:
    {
      if (strcmp(GET_STRING(o1), GET_STRING(o2)) == 0) UNIFIED;
      M_FAILED;
      break;
    }
  case TYPELIST:
    {
      stack_push(stk, GET_ARG(o1, 1));
      stack_push(stk, GET_ARG(o2, 1));
      o1 = dereference(GET_ARG(o1, 0));
      o2 = dereference(GET_ARG(o2, 0));
      goto start;
    }
    break;
  case TYPECOMPOUND:
    {
      arity = GET_TAG_INFO(o1);
      if (arity != GET_TAG_INFO(o2)) M_FAILED;
      int i;
      for (i = 1; i <= arity; i++) {
        stack_push(stk, GET_ARG(o1, i));
        stack_push(stk, GET_ARG(o2, i));
      }
      o1 = dereference(GET_ARG(o1, 0));
      o2 = dereference(GET_ARG(o2, 0));
      goto start;
      break;
    }
  default:
    assert(FALSE);
    return FALSE;
  }

 occurs:
  switch (GET_TYPE(octerm)) {
  case TYPEVAR:
    {
      if (var == octerm) M_FAILED;
      else BIND;
      break;
    }
  case TYPEINTEGER:
  case TYPEDOUBLE:
  case TYPEATOM:
  case TYPEDYNATOM:
  case TYPESTRING:
    {
      BIND;
      break;
    }
  case TYPELIST:
    {
      stack_push(stk, dereference(GET_ARG(octerm, 1)));
	octerm = dereference(GET_ARG(octerm, 0));
	goto occurs;
    }
    break;
  case TYPECOMPOUND:
    {
      arity = GET_TAG_INFO(octerm);
      int i;
      for (i = 1; i <= arity; i++) {
        stack_push(stk, dereference(GET_ARG(octerm, i)));
      }
      octerm = dereference(GET_ARG(octerm, 0));
      goto occurs;
      break;
    }

  default:
    assert(FALSE);
  }
}

/*
 * check to see if optr is a valid subscription
 */

gboolean check(ObjectPtr optr, Stack* stk, Trail* trail)
{
  //ObjectPtr* trail_top = TRAIL_GET_TOP(trail);

  if (!IS_COMPOUND(optr))
    return FALSE;
  assert(GET_ARG(optr, 0) ==  a_subscribe); 

  /* check if the head is compound */
  ObjectPtr head = GET_ARG(optr, 1);
  if (IS_ATOM(head) || IS_VAR(head))
    return TRUE;
  if (!IS_COMPOUND(head))
    return FALSE;

  ObjectPtr t = GET_ARG(optr, 2);
  stack_reset(stk);

  stack_push(stk, NULL);
  
  /* t should be a valid test */
 check_start:
  if (t == NULL) {
    //backtrack(trail, trail_top);
    return TRUE;
  }

  switch (GET_TYPE(t)) {
  case TYPEVAR:
  case TYPEINTEGER:
  case TYPEDOUBLE:
  case TYPESTRING:
    return FALSE;
    break;

  case TYPEATOM:
    if ((t == a_true) || (t == a_false)) {
      t = stack_pop(stk);
      goto check_start;	
    }
    else
      return FALSE;
    break;

  case TYPECOMPOUND: {
    if (GET_TAG_INFO(t) == 0) return FALSE;

    ObjectPtr functor = GET_ARG(t, 0);
    if (!IS_ATOM(functor))
      return FALSE;

    if (functor == a_and) {
      if (GET_TAG_INFO(t) != 2) return FALSE;
      stack_push(stk, GET_ARG(t, 2));
      t = GET_ARG(t, 1);
      goto check_start;
    }
    else if (functor == a_or) {
      if (GET_TAG_INFO(t) != 2) return FALSE;
      ObjectPtr arg1 = GET_ARG(t, 1);
      if (IS_COMPOUND(arg1) && (GET_TAG_INFO(arg1) == 2)
	  && (GET_ARG(arg1, 0) == a_arrow)) {
	PUT_ARG(t, 0, a_if);
	stack_push(stk, GET_ARG(arg1, 2));
	stack_push(stk, GET_ARG(arg1, 1));
	t = GET_ARG(t, 2);
	goto check_start;
      }
      else {
	stack_push(stk, GET_ARG(t, 2));
	t = GET_ARG(t, 1);
	goto check_start;
      }
    }
    else if (functor == a_not) {
      if (GET_TAG_INFO(t) != 1) return FALSE;
      t = GET_ARG(t, 1);
      goto check_start;
    }
    else if (functor == a_once) { 
      if (GET_TAG_INFO(t) != 1) return FALSE;
      t = GET_ARG(t, 1);
      goto check_start;
    }
    else if ((functor == a_eq) ||
	     (functor == a_is) ||
	     (functor == a_lt) ||
	     (functor == a_gt) ||
	     (functor == a_le) ||
	     (functor == a_ge) ||
	     (functor == a_member)) {
      if (GET_TAG_INFO(t) != 2) return FALSE;
      
      t = stack_pop(stk);
      goto check_start;
    }
    else if ((functor == a_split) ||
	     (functor == a_splitstring)) {
      if (GET_TAG_INFO(t) != 3) return FALSE;
      t = stack_pop(stk);
      goto check_start;
    }
    else if ((functor == a_number) ||
	     (functor == a_atom) ||
	     (functor == a_string) ||
             (functor == a_list)) {
      if (GET_TAG_INFO(t) != 1) return FALSE;
    }

    else
      return FALSE;
    break;
  }
  default:
    return FALSE;
  }	
}

gboolean isOr(ObjectPtr t)
{
  return (IS_COMPOUND(t) && (GET_ARG(t, 0) == a_or));
}
gboolean isIf(ObjectPtr t)
{
  return (IS_COMPOUND(t) && (GET_ARG(t, 0) == a_if));
}

int
compile(ObjectPtr optr, int index, ObjectPtr type, int* alt, int* next, 
	gboolean* addJump,   
	gboolean atEnd, CodeBuffer* cb, Trail* trail)
{
  ObjectPtr t = optr;

  int tmp1 = 0;
  int tmp2 = 0;
  
  switch (GET_TYPE(t)) {
  case TYPEATOM:
    {
      if ((t == a_true)) {
	return index;
      }
      assert(t == a_false);
      setCodeWord(cb, index, FAIL);
      return index+1;
      break;
    }


  case TYPECOMPOUND:
    {
      assert(GET_TAG_INFO(t) != 0);
      ObjectPtr functor = GET_ARG(t, 0);
      assert(IS_ATOM(functor));
      int newindex = index;
      if (functor == a_and) {
	newindex =
	  compile(GET_ARG(t, 1), newindex, a_and, &tmp1, &tmp2, 
		  addJump, TRUE, cb, trail);
	newindex =
	  compile(GET_ARG(t, 2), newindex, a_and, &tmp1, &tmp2, 
		  addJump, TRUE, cb, trail);
	return newindex;
      }
      else if (functor == a_or) {
	// Arg 1
	if (type != a_or) {
	  setCodeWord(cb, newindex, TRY);
	  newindex++;
	  *alt = newindex;
	  setCodeWord(cb, newindex, 0);
	  newindex++;
	}
	newindex = 
	  compile(GET_ARG(t, 1), newindex, a_or, alt, next, 
		  addJump, FALSE, cb, trail);
	if (addJump) {
	  setCodeWord(cb, newindex, JUMP);
	  newindex++;
	  setCodeWord(cb, newindex, *next);
	  *next = newindex;
	  newindex++;
	  *addJump = TRUE;
	}
	// Arg 2
	if (atEnd && !isOr(GET_ARG(t, 2))) {
	  setCodeWord(cb, *alt, newindex);
	  setCodeWord(cb, newindex, TRUST); 
	  newindex++;    
	  addJump = FALSE;
	}
	else {
	  setCodeWord(cb, newindex, RETRY);
	  setCodeWord(cb, *alt, newindex);
	  newindex++;
	  setCodeWord(cb, newindex, 0);
	  *alt = newindex;
	  newindex++;
	}
	newindex = 
	  compile(GET_ARG(t, 2), newindex, a_or, alt, next, 
		  addJump,  TRUE, cb, trail);
	if (addJump) {
	  setCodeWord(cb, newindex, JUMP);
	  newindex++;
	  setCodeWord(cb, newindex, *next);
	  *next = newindex;
	  newindex++;
	  addJump = FALSE;
	}
	    
	if (type != a_or)
	  while (*next != 0) {             // set up all next indexes
	    int tmp = GET_CODE_WORD(cb, *next);
	    setCodeWord(cb, *next, newindex);
	    *next = tmp;
	  }
	return newindex;
      }
      else if (functor == a_if) {
	ObjectPtr arg1 = GET_ARG(GET_ARG(t, 1), 1);
	ObjectPtr arg2 = GET_ARG(GET_ARG(t, 1), 2);
	ObjectPtr arg3 = GET_ARG(t, 2);
	    
	if (type != a_if) {
	  setCodeWord(cb, newindex, IFTRY);
	  newindex++;
	  *alt = newindex;
	  setCodeWord(cb, newindex, 0);
	  newindex++;
	}
	    
	newindex = 
	  compile(arg1, newindex, a_and, &tmp1, &tmp2, addJump,   
		  TRUE, cb, trail);
	setCodeWord(cb, newindex, CUT);
	newindex++;
	newindex = 
	  compile(arg2, newindex, a_and, &tmp1, &tmp2, addJump, 
		  TRUE, cb, trail);
	setCodeWord(cb, newindex, JUMP);
	newindex++;
	setCodeWord(cb, newindex, *next);
	*next = newindex;
	newindex++;
	setCodeWord(cb, *alt, newindex);
	if (isIf(arg3)) {
	  setCodeWord(cb, newindex, RETRY);
	  setCodeWord(cb, *alt, newindex);
	  newindex++;
	  setCodeWord(cb, newindex, 0);
	  *alt = newindex;
	  newindex++;
	  newindex = 
	    compile(arg3, newindex, a_if, alt, next, addJump,  
		    TRUE, cb, trail);
	}
	else {
	  setCodeWord(cb, newindex, TRUST);
	  setCodeWord(cb, *alt, newindex);
	  newindex++;
	  newindex =
	    compile(arg3, newindex, a_if, alt, next, addJump,  
		    TRUE, cb, trail);
	  while (*next != 0) {            // set up all next indexes
	    int tmp = GET_CODE_WORD(cb, *next);
	    setCodeWord(cb, *next, newindex);
	    *next = tmp;
	  }
	}
	return newindex;
      }
      else if (functor == a_not) {
	setCodeWord(cb, newindex, NOTTRY);
	newindex++;
	int notalt = newindex;
	int alt = 0;
	setCodeWord(cb, newindex, 0);
	newindex++;
	newindex = 
	  compile(GET_ARG(t, 1), newindex, a_not, &alt, next,
		  addJump, TRUE, cb, trail);
	setCodeWord(cb, newindex, CUT);
	newindex++;
	setCodeWord(cb, newindex, FAIL);
	newindex++;
	setCodeWord(cb, notalt, newindex);
	return newindex;
      }
      else if (functor == a_once) {
	setCodeWord(cb, newindex, ONCE);
	newindex++;
	newindex = 
	  compile(GET_ARG(t, 1), newindex, a_once, alt, next, 
		  addJump, TRUE, cb, trail);
	setCodeWord(cb, newindex, CUT);
	newindex++;
	return newindex;
      }
      else if (functor == a_member) {
	assert(GET_TAG_INFO(t) == 2);
	setCodeWord(cb, newindex, MEMBERTRY);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	setCodeWord(cb, newindex, MEMBER);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, 0);
	newindex++;
	    
	return newindex;
      }
      else if (functor == a_eq) {
	assert(GET_TAG_INFO(t) == 2);
	setCodeWord(cb, newindex, UNIFY);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	return newindex;
      }
      else if (functor == a_is) {
	assert(GET_TAG_INFO(t) == 2);
	setCodeWord(cb, newindex, IS);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	return newindex;
      }
      else if (functor == a_le) {
	assert(GET_TAG_INFO(t) == 2);
	setCodeWord(cb, newindex, LE);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	return newindex;
      }
      else if (functor == a_ge) {
	assert(GET_TAG_INFO(t) == 2);
	setCodeWord(cb, newindex, GE);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	return newindex;
      }
      else if (functor == a_lt) {
	assert(GET_TAG_INFO(t) == 2);
	setCodeWord(cb, newindex, LT);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	return newindex;
      }
      else if (functor == a_gt) {
	assert(GET_TAG_INFO(t) == 2);
	setCodeWord(cb, newindex, GT);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	return newindex;
      }
      else if (functor == a_number) {
	assert(GET_TAG_INFO(t) == 1);
	if (IS_NUMBER(GET_ARG(t, 1))) {
	  setCodeWord(cb, newindex, TRUE);
	  newindex++;
	}
	else if (IS_VAR(GET_ARG(t, 1))) {
	  setCodeWord(cb, newindex, NUMBER);
	  newindex++;
	  setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	  newindex++;
	  return newindex;
	}
	else {
	  setCodeWord(cb, newindex, FAIL);
	  newindex++;
	}

      }      
      else if (functor == a_atom) {
	assert(GET_TAG_INFO(t) == 1);
	if (IS_ATOM(GET_ARG(t, 1))) {
	  setCodeWord(cb, newindex, TRUE);
	  newindex++;
	}
	else if (IS_VAR(GET_ARG(t, 1))) {
	  setCodeWord(cb, newindex, ATOM);
	  newindex++;
	  setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	  newindex++;
	  return newindex;
	}
	else {
	  setCodeWord(cb, newindex, FAIL);
	  newindex++;
	}
      }
      else if (functor == a_string) {
	assert(GET_TAG_INFO(t) == 1);
	if (IS_STRING(GET_ARG(t, 1))) {
	  setCodeWord(cb, newindex, TRUE);
	  newindex++;
	}
	else if (IS_VAR(GET_ARG(t, 1))) {
	  setCodeWord(cb, newindex, STRING);
	  newindex++;
	  setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	  newindex++;
	  return newindex;
	}
	else {
	  setCodeWord(cb, newindex, FAIL);
	  newindex++;
	}
      }
      else if (functor == a_list) {
	assert(GET_TAG_INFO(t) == 1);
	if (IS_LIST(GET_ARG(t, 1)) || (GET_ARG(t, 1) == a_nil)) {
	  setCodeWord(cb, newindex, TRUE);
	  newindex++;
	}
	else if (IS_VAR(GET_ARG(t, 1))) {
	  setCodeWord(cb, newindex, LIST);
	  newindex++;
	  setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	  newindex++;
	  return newindex;
	}
	else {
	  setCodeWord(cb, newindex, FAIL);
	  newindex++;
	}
      }
      else if (functor == a_split) {
	assert(GET_TAG_INFO(t) == 3);
	setCodeWord(cb, newindex, SPLITTRY);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, SPLIT);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 3));
	newindex++;
	setCodeWord(cb, newindex, 0);
	newindex++;
	setCodeWord(cb, newindex, 0);
	newindex++;
	setCodeWord(cb, newindex, 0);
	newindex++;
	    
	return newindex;
      }
      else if (functor == a_splitstring) {
	assert(GET_TAG_INFO(t) == 3);
	setCodeWord(cb, newindex, SPLITSTRINGTRY);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 1));
	newindex++;
	setCodeWord(cb, newindex, SPLITSTRING);
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 2));
	newindex++;
	setCodeWord(cb, newindex, (guint64)GET_ARG(t, 3));
	newindex++;
	setCodeWord(cb, newindex, 0);
	newindex++;
	setCodeWord(cb, newindex, 0);
	newindex++;
	    
	return newindex;
      }
      else {
	assert(FALSE);
	return 0;
      }
      break;
    }
  default:
    {
      assert(FALSE);
      return 0;
    }
  }	
}

/*
 * freeze all teh variables in optr
 */
void
freeze_vars(ObjectPtr optr)
{
  ObjectPtr t = optr;
  switch (GET_TYPE(t)) {
  case TYPEVAR:
    FREEZE_VAR(optr);
    break;
  case TYPEINTEGER:
  case TYPEDOUBLE:
  case TYPESTRING:
  case TYPEATOM:
    break;
  case TYPELIST:
    {
      freeze_vars(GET_HEAD(t));
      freeze_vars(GET_TAIL(t));
    }
    break;
  case TYPECOMPOUND:
    {
      int arity = GET_TAG_INFO(t);
      int i;
      for (i = 0; i <= arity; i++)
        freeze_vars(GET_ARG(t, i));
    }
    break;
  default:
    assert(FALSE); 
    break;
  }
}

/*
 * copy the term optr to the heap cpheap
 */
ObjectPtr 
copyTerm(ObjectPtr optr, Heap* cpheap, Trail* trail)
{
  ObjectPtr t = optr;
  switch (GET_TYPE(t)) {
  case TYPEVAR:
    {
      ObjectPtr deref = dereference(t);
      if (t == deref) {
	ObjectPtr var = pushVariable(cpheap);
	bind_to(t, var, trail);
	return var;
      }
      else
	return deref;
      break;
    }
  case TYPEINTEGER:
    return pushInteger(cpheap, GET_INTEGER(t));
    break;
  case TYPEDOUBLE:
    return pushDouble(cpheap, GET_DOUBLE(t));
    break;
  case TYPESTRING:
    return pushString(cpheap, GET_STRING(t));
    break;
  case TYPEATOM:
    return t;
    break;
  case TYPELIST:
    {
	ObjectPtr term = pushList(cpheap);
	PUT_HEAD(term, copyTerm(GET_HEAD(t), cpheap, trail));
	PUT_TAIL(term, copyTerm(GET_TAIL(t), cpheap, trail));
	return term;
        break;
    }
  case TYPECOMPOUND:
    {
      int arity = GET_TAG_INFO(t);
      
      ObjectPtr term = pushCompound(cpheap, arity);
      int i;
      for (i = 0; i <= arity; i++)
        PUT_ARG(term, i, copyTerm(GET_ARG(t, i), cpheap, trail));
      return term;
      
    }
    break;
  default:
    assert(FALSE);
    return NULL;
  }
}

#define BACKTRACK                              \
  do {                   	               \
    if (CPSTACK_EMPTY(cpstack))                \
      {                                        \
        TRAIL_RESET(trail);	               \
        return FALSE;                          \
      }                                        \
    ChoicePoint* cp = cpstack_peek(cpstack);   \
    backtrack(trail, cp->trailtop);            \
    heap->top =  cp->heaptop;                  \
    pc = cp->alt;                              \
    cutPoint = cp->cutPoint;                   \
  } while (0)

#define ERROR_EXIT                        \
  do {                                    \
    TRAIL_RESET(trail);                   \
    return FALSE;                         \
  } while (0)

/*
 * Unify the subscription "head" with term and if that succeeds
 * execute the subscription code (i.e. the test)
 */ 
gboolean execute(Subscription* sub, ObjectPtr term, Heap* heap, Trail* trail,
		 ChoicePointStack* cpstack, Stack* stack)
{
  guint64* code = sub->code;
  CPSTACK_RESET(cpstack);
  ChoicePoint* cutPoint = cpstack_getTop(cpstack);
  ObjectPtr* savedtop = TRAIL_GET_TOP(trail);
  //logit("trail_top = %x\n", (guint64)savedtop);

  if (!unify(term, sub->head, trail, stack)) {
    backtrack(trail, savedtop);
    return FALSE;
  }
  guint64* pc = code;
  while (TRUE) {
    u_int instr = *pc;
    pc++;
    switch(instr) {
    case FAIL:
      BACKTRACK;
      break;
    case TTRUE:
      assert(FALSE);
      break;
    case TRY:
      {
	cpstack_push(cpstack, (guint64*)(*pc), heap->top, 
		     TRAIL_GET_TOP(trail), cutPoint);
	pc++;
	break;
      }
    case RETRY:
      {
	cpstack_peek(cpstack)->alt = (guint64*)(*pc);
	pc++;
	break;
      }
    case NOTTRY:
      {
	ChoicePoint* oldtop = cpstack_getTop(cpstack);
	cpstack_push(cpstack, (guint64*)(*pc), heap->top, 
		     TRAIL_GET_TOP(trail), cutPoint);
	cutPoint = oldtop;
	pc++;
	break;
      }
    case IFTRY:
      {
	ChoicePoint* oldtop = cpstack_getTop(cpstack);
	cpstack_push(cpstack, (guint64*)(*pc), heap->top, 
		     TRAIL_GET_TOP(trail), cutPoint);
	cutPoint = oldtop;
	pc++;
	break;
      }
      
    case TRUST:
      {
	cpstack_pop(cpstack);
	break;
      }
    case ONCE:
      {
	cutPoint = cpstack_getTop(cpstack);
	break;
      }
    case EXIT:
      TRAIL_RESET(trail);
      return TRUE;
      break;
    case JUMP:
      pc = (guint64*)(*pc);
      break;
    case CUT:
      {
	cpstack_setTop(cpstack, cutPoint);
	break;
      }
    case NOOP:
      { 
	assert(FALSE);
	break;
      }
    case UNIFY:
      { 
	ObjectPtr t1 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr t2 = (ObjectPtr)(*pc);
	pc++;
	if (!unify(t1, t2, trail, stack))
	  BACKTRACK;
	break;
      }
    case IS:
      {
	ObjectPtr t1 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr t2 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr res1;
	ObjectPtr res2;
	if (IS_VAR(t1)) {
	  if (!arith_simplify(t2, &res2, heap))
	    ERROR_EXIT;
	  if (!unify(t1, res2, trail, stack))
	    BACKTRACK;
	}
	else {
	  if (!arith_simplify(t1, &res1, heap) 
	      || !arith_simplify(t2, &res2, heap))
	    ERROR_EXIT;
	  if (!unify(res1, res2, trail, stack))
	    BACKTRACK;
	}
	break;
      }
      
    case LT:
      {
	ObjectPtr t1 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr t2 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr res1;
	ObjectPtr res2;
	if (!arith_simplify(t1, &res1, heap) 
	    || !arith_simplify(t2, &res2, heap))
	  ERROR_EXIT;
	if (getNumber(res1) >= getNumber(res2))
	  BACKTRACK;
	break;
      }
    case GT:
      {
	ObjectPtr t1 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr t2 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr res1;
	ObjectPtr res2;
	if (!arith_simplify(t1, &res1, heap) 
	    || !arith_simplify(t2, &res2, heap))
	  ERROR_EXIT;
	if (getNumber(res1) <= getNumber(res2))
	  BACKTRACK;
	break;
      }
    case LE:
      {
	ObjectPtr t1 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr t2 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr res1;
	ObjectPtr res2;
	if (!arith_simplify(t1, &res1, heap) 
	    || !arith_simplify(t2, &res2, heap))
	  ERROR_EXIT;
	if (getNumber(res1) > getNumber(res2))
	  BACKTRACK;
	break;
      }
      
    case GE:
      {
	ObjectPtr t1 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr t2 = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr res1;
	ObjectPtr res2;
	if (!arith_simplify(t1, &res1, heap) 
	    || !arith_simplify(t2, &res2, heap))
	  ERROR_EXIT;
	if (getNumber(res1) < getNumber(res2))
	  BACKTRACK;
	break;
      }
      
    case MEMBERTRY:
      {
	ObjectPtr list = (ObjectPtr)(*pc);
	list = dereference(list);
	pc++;
        if (!IS_LIST(list) || (list == a_nil))
          BACKTRACK;
	*(pc + 2) = (guint64)list;
	cpstack_push(cpstack, pc, heap->top, 
		     TRAIL_GET_TOP(trail), cutPoint);
      }
      break;
    case MEMBER:
      {
	ObjectPtr x = (ObjectPtr)(*pc);
	pc++;
	ObjectPtr list = (ObjectPtr)(*pc);
	pc++;
	list = dereference(list);
        if (!IS_LIST(list) || (list == a_nil)) {
	  cpstack_pop(cpstack);
          BACKTRACK;
	}

	ObjectPtr h = GET_HEAD(list);
	ObjectPtr t = GET_TAIL(list);
	*(pc-1) = (guint64)(dereference(t));
	if (!unify(x, h, trail, stack))
	  BACKTRACK;
	break;
      }
    case SPLITTRY:
      {
	ObjectPtr list = (ObjectPtr)(*pc);
	list = dereference(list);
	pc++;  // Now pointing at SPLIT instr

        ObjectPtr l1 = (ObjectPtr)(*(pc+1));
        ObjectPtr l2 = (ObjectPtr)(*(pc+2));
        if (list == a_nil) {
          if (!unify(l1, a_nil, trail, stack) ||
	      !unify(l2, a_nil, trail, stack))
	    BACKTRACK;
          else {    // skip over SPLIT
            pc += 6;
          }
	} else if (!IS_LIST(list)){
          BACKTRACK;
	} else {
          *(pc+3) = (guint64)a_nil;
          *(pc+4) = (guint64)(pc+3);
          *(pc+5) = (guint64)(list);
          cpstack_push(cpstack, pc, heap->top, 
                       TRAIL_GET_TOP(trail), cutPoint);
          // try l1 = [] and l2 = list
          if (!unify(l1, a_nil, trail, stack) ||
	      !unify(l2, list, trail, stack))
	    BACKTRACK;
          else
            pc += 6;
        }
      }
      break;
    case SPLIT:
      {
        guint64 *cell1ptr = pc;  // argument 2 of split   
        pc++;
        guint64 *cell2ptr = pc;  // argument 3 of split
        pc++;
        guint64 *cell3ptr = pc;  // to unify against argument 2 of split
        pc++;
        guint64 *cell4ptr = pc;  // tail ptr of cell3 list
        pc++;
        guint64 *cell5ptr = pc;  // to unify against argument 3 of split
        pc++;
        // PC now pointing at following instruction

	guint64 targ1 = *cell3ptr;
	guint64 targ2 = *cell4ptr;
	guint64 targ3 = *cell5ptr;
	
	if (!IS_LIST((ObjectPtr)targ3) || ((ObjectPtr)targ3 == a_nil)) {
          // Run out of choices
	  cpstack_pop(cpstack);
	  BACKTRACK;
	} else {
          ObjectPtr list = (ObjectPtr)targ3;
          ObjectPtr head = dereference(GET_HEAD(list));
          ObjectPtr tail = dereference(GET_TAIL(list));
          ObjectPtr newlist = pushList(heap);
          PUT_HEAD(newlist, head);
          PUT_TAIL(newlist, a_nil);
          ObjectPtr* tailptr = (ObjectPtr*)targ2;
          *tailptr = newlist;
          *(cell4ptr) = (guint64)GET_TAILPTR(newlist);
          *(cell5ptr) = (guint64)tail;
          
          ChoicePoint* cp = cpstack_peek(cpstack);
          cp->heaptop = heap->top;
          
          ObjectPtr l1 = (ObjectPtr)(*cell3ptr);
          ObjectPtr l2 = (ObjectPtr)(*cell5ptr);
          ObjectPtr a1 = (ObjectPtr)(*cell1ptr);
          ObjectPtr a2 = (ObjectPtr)(*cell2ptr);
        
          if (!unify(l1, a1, trail, stack) ||
              !unify(l2, a2, trail, stack))
            BACKTRACK;
        }
        break;
      }
    case SPLITSTRINGTRY:
      {
	ObjectPtr str = (ObjectPtr)(*pc);
	str = dereference(str);
	if (!IS_STRING(str)) {
	  ERROR_EXIT;
        }
	pc++;    // Now pointing at SPLITSTRING instr
        // make str1 and str2 big enough to hold entire string
        char* strchars = GET_STRING(str);
	ObjectPtr str1 = pushString(heap, strchars);
	ObjectPtr str2 = pushString(heap, strchars);
	GET_STRING(str1)[0] = '\0';
        ObjectPtr s1 = (ObjectPtr)(*(pc+1));
        ObjectPtr s2 = (ObjectPtr)(*(pc+2));
        
        if (strchars[0] == '\0') {
          if (!unify(s1, str1, trail, stack) ||
	      !unify(s2, str2, trail, stack))
	    BACKTRACK;
          else {    // skip over SPLITSTRING
            pc += 5;
          }
	} else {
          *(pc+3) = (guint64)str1;
          *(pc+4) = (guint64)str2;
          
          cpstack_push(cpstack, pc, heap->top, 
                       TRAIL_GET_TOP(trail), cutPoint);
          // try s1 = str1 and s2 = str2
          if (!unify(s1, str1, trail, stack) ||
	      !unify(s2, str2, trail, stack))
	    BACKTRACK;
          else
            pc += 5;
        }
        break;
      }
    case SPLITSTRING:
      {
        guint64 *cell1ptr = pc;  // argument 2 of splitstring   
        pc++;
        guint64 *cell2ptr = pc;  // argument 3 of splitstring
        pc++;
        guint64 *cell3ptr = pc;  // to unify against argument 2 of splitstring
        pc++;
        guint64 *cell4ptr = pc;  // to unify against argument 3 of splitstring
        pc++;
        // PC now pointing at following instruction

	guint64 targ1 = *cell3ptr;
	guint64 targ2 = *cell4ptr;
	
	ObjectPtr str1 = (ObjectPtr)targ1;
	ObjectPtr str2 = (ObjectPtr)targ2;

	if (GET_STRING(str2)[0] == '\0') {
	  cpstack_pop(cpstack);
	  BACKTRACK;
	}
	else {
          char* str2chars = GET_STRING(str2);
          char c = str2chars[0];
          char* ptr = GET_STRING(str1);
          while (*ptr != '\0') 
            ptr++;
          *ptr = c;
          *(ptr+1) = '\0';
          ptr = str2chars;
          while (*ptr != '\0') {
            *ptr = *(ptr+1);
            ptr++;
          }
	  ObjectPtr a1 = (ObjectPtr)(*cell1ptr);
	  ObjectPtr a2 = (ObjectPtr)(*cell2ptr);
	  if (!unify(str1, a1, trail, stack) ||
	      !unify(str2, a2, trail, stack))
	    BACKTRACK;
	}
	break;
      }
    case NUMBER:
      {
	ObjectPtr x = (ObjectPtr)(*pc);
	x = dereference(x);
	pc++;
	if (!IS_NUMBER(x))
	  BACKTRACK;
	break;
      }
    case ATOM:
      {
	ObjectPtr x = (ObjectPtr)(*pc);
	x = dereference(x);
	pc++;
	if (!IS_ANY_ATOM(x))
	  BACKTRACK;
	break;
      }
    case STRING:
      {
	ObjectPtr x = (ObjectPtr)(*pc);
	x = dereference(x);
	pc++;
	if (!IS_STRING(x))
	  BACKTRACK;
	break;
      }
    case LIST:
      {
	ObjectPtr x = (ObjectPtr)(*pc);
	x = dereference(x);
	pc++;
	if (!IS_LIST(x) && (x != a_nil))
	  BACKTRACK;
	break;
      }
    default:
      logit("Bad instruction: %u\n", instr);
      assert(FALSE);
      break;

    }
  }
}


