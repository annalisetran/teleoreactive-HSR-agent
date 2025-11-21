
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
#include <stdio.h>
#include <stdlib.h>
#include <glib.h>

#include "objects.h"
#include "choicestack.h"

extern void logit(char* fmt, ...);


/*
 * A Prolog-like choice point stack
 */

ChoicePointStack* cpstack_new(size_t s)
{
  ChoicePointStack* stk =  g_new(ChoicePointStack, 1);
  stk->data = g_new(ChoicePoint, s);
  stk->top = stk->data;
  stk->size = s;
  stk->high = stk->data + s;
  return stk; 
}

void cpstack_destroy(ChoicePointStack* cp)
{
  g_free(cp->data);
  g_free(cp);
}

void cpstack_push(ChoicePointStack* cpstack, guint64* alt, ObjectPtr heaptop, 
		  ObjectPtr* trailtop, ChoicePoint* t)
{
  {
    if (cpstack->top >= cpstack->high)
      {
	logit("Out of space in choice point stack\n");
	exit(1);
      }
    cpstack->top->alt = alt;
    cpstack->top->heaptop = heaptop;
    cpstack->top->trailtop = trailtop;
    cpstack->top->cutPoint = t;
    (cpstack->top)++;
  }
}

void cpstack_setTop(ChoicePointStack* cpstack, ChoicePoint* t)
{
  assert(t <= cpstack->top);
  cpstack->top = t;
}

ChoicePoint* cpstack_getTop(ChoicePointStack* cpstack)
{
  return  cpstack->top;
}

ChoicePoint* cpstack_pop(ChoicePointStack* cpstack)
{
  assert(!CPSTACK_EMPTY(cpstack));
  cpstack->top--;
  return (cpstack->top);
}

ChoicePoint* cpstack_peek(ChoicePointStack* cpstack)
{
  return ((cpstack->top) - 1);
}
