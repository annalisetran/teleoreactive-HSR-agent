
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
#include "pstack.h"

/*
 * The stack is used to store yet-to-be-processed terms - e.g. for unification
 */

Stack* stack_new(size_t s)
{
  Stack* stk =  g_new(Stack, 1);
  stk->data = g_new(ObjectPtr, s);
  stk->top = stk->data;
  stk->size = s;
  stk->high = stk->data + s;
  return stk; 
}

void stack_destroy(Stack* s)
{
  g_free(s->data);
  g_free(s);
}

void stack_push(Stack* stk, ObjectPtr ho) 
{ 
  assert(stk->top < stk->high - 1); 
  *(stk->top) = ho; 
  stk->top++; 
}

ObjectPtr stack_pop(Stack* stk) 
{ 
  assert(stk->top > stk->data); 
  stk->top--; 
  return *(stk->top); 
}

// look back n elements into the stack - last at position 1
ObjectPtr stack_peek(Stack* stk, int n)
{
  assert(n > 0);
  assert(stk->top-n >= stk->data);
  return (*(stk->top-n));
}

void stack_init(Stack* stk) { stk->top = stk->data; stack_push(stk, NULL); }

void stack_reset(Stack* stk) { stk->top = stk->data; }

void stack_set_top(Stack* stk, ObjectPtr* newtop) { stk->top = newtop; }

