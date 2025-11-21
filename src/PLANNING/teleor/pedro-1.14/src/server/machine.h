
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
#ifndef MACHINE_H
#define MACHINE_H

#include "config.h"
#include "objects.h"
#include "ptrail.h"
#include "pstack.h"
#include "pheap.h"
#include "codeBuffer.h"
#include "choicestack.h"
#include "subscription.h"

G_BEGIN_DECLS

void bind_to(ObjectPtr o1, ObjectPtr o2, Trail* t);

int compile(ObjectPtr optr, int index, ObjectPtr type, int* alt, int* next, 
	    gboolean* isTrust,
	    gboolean atEnd, CodeBuffer* cb, Trail* trail);

gboolean check(ObjectPtr optr, Stack* stack, Trail* trail);

ObjectPtr copyTerm(ObjectPtr optr, Heap* cpheap, Trail* trail);

gboolean unify(ObjectPtr o1, ObjectPtr o2, Trail* t, Stack* s);

gboolean arith_simplify(ObjectPtr in, ObjectPtr* out, Heap* heap);

gboolean execute(Subscription* sub, ObjectPtr term, Heap* heap, Trail* trail,
	     ChoicePointStack* cpstack, Stack* stack);

void freeze_vars(ObjectPtr o);

G_END_DECLS

#endif
