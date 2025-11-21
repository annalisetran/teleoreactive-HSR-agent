
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
//
// objects.h - Terms
//

#ifndef OBJECTS_H
#define OBJECTS_H

#include <config.h>
#include <sys/types.h>
#include <assert.h>
#include <string.h>
#include <glib.h>

G_BEGIN_DECLS

typedef guint64 heapobject;
typedef heapobject* ObjectPtr;

#define SECONDBITSMASK   0x00000002UL
#define LOW2BITSMASK     0x00000003UL
#define TYPEMASK         0x0000000fUL

#define TYPEVAR          0x00000000UL 
#define TYPEINTEGER      0x00000002UL  
#define TYPEDOUBLE       0x00000003UL
#define TYPEATOM         0x00000006UL
#define TYPEDYNATOM      0x00000007UL
#define TYPESTRING       0x0000000aUL
#define TYPECOMPOUND     0x0000000bUL
#define TYPELIST         0x0000000eUL  

#define FROZEN           0x00000001UL
// Lists are compounds with arity 0

#define SIZEOFVAR            1
#define SIZEOFATOM           3
#define SIZEOFINTEGER        2
#define SIZEOFDOUBLE         3
#define SIZEOFCOMPOUNDBASE   2
#define SIZEOFSTRINGBASE     1
#define SIZEOFLIST           3



#define GET_TAG_INFO(optr)  (size_t)((optr)[0] >> 4)

#define GET_TYPE(optr) (((SECONDBITSMASK  & (optr)[0]) == 0) ? TYPEVAR : TYPEMASK & (optr)[0])

#define IS_ATOM(optr) ((TYPEMASK & (optr)[0]) == TYPEATOM)

#define IS_DYN_ATOM(optr) ((TYPEMASK & (optr)[0]) == TYPEDYNATOM)

#define IS_ANY_ATOM(optr) (((TYPEMASK & (optr)[0]) == TYPEATOM) || ((TYPEMASK & (optr)[0]) == TYPEDYNATOM))
 
#define IS_INTEGER(optr) ((TYPEMASK & (optr)[0]) == TYPEINTEGER)
#define IS_DOUBLE(optr) ((TYPEMASK & (optr)[0]) == TYPEDOUBLE)
#define IS_NUMBER(optr) (((TYPEMASK & (optr)[0]) == TYPEINTEGER) || ((TYPEMASK & (optr)[0]) == TYPEDOUBLE))

#define IS_VAR(optr) ((SECONDBITSMASK & (optr)[0]) == 0)
#define IS_FROZEN_VAR(optr) ((LOW2BITSMASK & (optr)[0]) == 1)

#define IS_COMPOUND(optr) ((TYPEMASK & (optr)[0]) == TYPECOMPOUND)

#define IS_LIST(optr) ((optr)[0] == TYPELIST)

#define IS_STRING(optr) ((TYPEMASK &(optr)[0]) == TYPESTRING)


#define GET_INTEGER(optr) ((gint64)((optr)[1]))
#define GET_DOUBLE(optr) (*((double*)((optr)+1)))

#define GET_ARG(optr, i) ((ObjectPtr)((optr)[(i)+1]))

#define PUT_ARG(optr, i, a)  (optr)[(i)+1] = (heapobject)(a)

#define PUT_HEAD(optr, a) (optr)[1] = (heapobject)(a)
#define PUT_TAIL(optr, a) (optr)[2] = (heapobject)(a)

#define GET_HEAD(optr) ((ObjectPtr)((optr)[1]))
#define GET_TAIL(optr) ((ObjectPtr)((optr)[2]))
#define GET_TAILPTR(optr) ((ObjectPtr*)((optr)+2))

#define RESET(optr) (optr)[0] = (heapobject)(optr)

#define GET_STRING(optr) ((char*)((optr)+1))

#define GET_DYNATOM_NAME(optr) ((char*)((optr)+2))
#define GET_ATOM_NAME(optr) ((char*)((optr)+2))

#define FREEZE_VAR(optr) (optr)[0] = (heapobject)(optr) | FROZEN

guint64 makeKey(ObjectPtr optr);

size_t getSize(ObjectPtr optr);

gboolean isCompoundTerm(ObjectPtr optr, int arity, ObjectPtr functor);

double getNumber(ObjectPtr optr);


char* getDynAtomString(ObjectPtr optr);
char* getAtomName(ObjectPtr optr);

gboolean occursIn(ObjectPtr optr, ObjectPtr o2);

ObjectPtr dereference(ObjectPtr optr);

void negateInteger(ObjectPtr optr);
void negateDouble(ObjectPtr optr);

#ifndef NDEBUG
void printMe(ObjectPtr optr);
#endif



#define YYSTYPE ObjectPtr

G_END_DECLS

#endif 
