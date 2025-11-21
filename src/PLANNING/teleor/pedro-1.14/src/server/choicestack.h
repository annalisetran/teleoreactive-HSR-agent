
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

#ifndef CHOICESTACK_H
#define CHOICESTACK_H

#include <glib.h>
#include "objects.h"

G_BEGIN_DECLS

typedef struct _ChoicePoint ChoicePoint;

struct _ChoicePoint
{
  guint64* alt;
  ObjectPtr heaptop;
  ObjectPtr* trailtop;
  ChoicePoint* cutPoint;
};

typedef struct _ChoicePointStack ChoicePointStack;

struct _ChoicePointStack
{
  ChoicePoint* data;
  ChoicePoint* top;
  ChoicePoint* high;
  size_t size;
};


ChoicePointStack* cpstack_new(size_t s);

void cpstack_destroy(ChoicePointStack* cp);

void cpstack_push(ChoicePointStack* cpstack, guint64* alt, ObjectPtr heaptop,
		  ObjectPtr* trailtop, ChoicePoint* t);

ChoicePoint* cpstack_pop(ChoicePointStack* cpstack);
ChoicePoint* cpstack_peek(ChoicePointStack* cpstack);

void cpstack_setTop(ChoicePointStack* cpstack, ChoicePoint* t);
ChoicePoint* cpstack_getTop(ChoicePointStack* cpstack);


#define CPSTACK_EMPTY(cpstack) (cpstack->top == cpstack->data)
#define CPSTACK_RESET(cpstack) cpstack->top = cpstack->data

G_END_DECLS

#endif
