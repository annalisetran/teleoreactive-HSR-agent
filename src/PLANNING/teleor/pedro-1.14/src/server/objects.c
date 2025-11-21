
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
#include "objects.h"
#include "atoms.h"

#ifndef NDEBUG
#include <stdio.h>
#endif

extern void logit(char* fmt, ...);

    
size_t getSize(ObjectPtr optr)
{
  switch (GET_TYPE(optr)){
  case TYPEVAR: 
    return SIZEOFVAR; break;
  case TYPEATOM: 
  case TYPEDYNATOM: 
    return GET_TAG_INFO(optr); break;
  case TYPEINTEGER: 
    return SIZEOFINTEGER; break;
  case TYPEDOUBLE: 
    return SIZEOFDOUBLE; break;
  case TYPELIST:
    return SIZEOFLIST; break;
  case TYPECOMPOUND: 
    return (SIZEOFCOMPOUNDBASE + GET_TAG_INFO(optr));
    break;
  case TYPESTRING: 
    return (SIZEOFSTRINGBASE + 
	    ((GET_TAG_INFO(optr)== 0) ? 1 : GET_TAG_INFO(optr))); 
    break;
  default: assert(0); return 0; break;
  }
}
  

gboolean isCompoundTerm(ObjectPtr optr, int arity, ObjectPtr functor)
{ 
  return IS_COMPOUND(optr) && (GET_TAG_INFO(optr) == arity) 
    && (GET_ARG(optr, 0) == functor);
}


double getNumber(ObjectPtr optr) 
{
  if (GET_TYPE(optr) == TYPEDOUBLE) return GET_DOUBLE(optr);
  else return (double)GET_INTEGER(optr);
}


ObjectPtr dereference(ObjectPtr optr)
{
  ObjectPtr curr = optr;
  while (IS_VAR(curr) && ((*curr & ~FROZEN) != (heapobject)curr)) {
    curr = (ObjectPtr)(*curr);
  }
  return curr;
}

guint64 makeKey(ObjectPtr optr)
{
  if (IS_VAR(optr)) 
    return 0;
  if (IS_ATOM(optr)) 
    return (guint64)optr << 2;
  if (GET_TAG_INFO(optr) == 0) 
    return (((guint64)a_dot << 2) | 2);
  return ((optr)[1] << 2) | ((optr)[0] >> 4);
}

void negateInteger(ObjectPtr optr)
{
  *(optr+1) *= -1;
}

void negateDouble(ObjectPtr optr)
{
  double d = -1*GET_DOUBLE(optr);
  memcpy((optr+1), &d, sizeof(double));
}


#ifndef NDEBUG
void printMe(ObjectPtr optr)
{
  ObjectPtr o1 = dereference(optr);
  heapobject type1 = GET_TYPE(o1);
  switch (type1) {
  case TYPEVAR:
    {
      if (IS_FROZEN_VAR(o1))
        logit("X%ud\n", (guint64)o1);
      else
        logit("X%ud\n", (guint64)o1);
      break;
    }
  case  TYPEINTEGER: 
    {
      logit("%d\n", GET_INTEGER(o1));
      break;
    }
  case  TYPEDOUBLE:
    {
      logit("%lf\n", GET_DOUBLE(o1));
      break;
      }
  case  TYPEATOM:
    { 
      logit("%s\n", GET_ATOM_NAME(o1));
      break;
    }
  case  TYPEDYNATOM:
    { 
      logit("D:%s\n", GET_DYNATOM_NAME(o1));
      break;
    }
  case  TYPESTRING:
    {
      logit("\"%s\"\n", GET_STRING(o1));
      break;
    }
    
  case  TYPELIST:
    {
      logit(".(\n");
      printMe(GET_HEAD(o1));
      logit(", \n");
      printMe(GET_TAIL(o1));
      logit(")\n");
      break;
    }
  case TYPECOMPOUND:
    {
      guint64 arity = GET_TAG_INFO(o1);
      ObjectPtr o2 = GET_ARG(o1, 0);
      printMe(o2); 
      logit("(\n");
      printMe(GET_ARG(o1, 1));
      guint i;
      for (i = 2; i <= arity; i++) {
        logit(", \n");
        printMe(GET_ARG(o1, i));
      }
      logit(")\n");
      break;
    }
  default:
    { 
      assert(0);
    }
  }
}

#endif


