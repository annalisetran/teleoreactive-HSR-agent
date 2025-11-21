
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
#ifndef STACK_H
#define STACK_H

#include <glib.h>
#include "objects.h"

G_BEGIN_DECLS

typedef struct _Stack Stack;

struct _Stack
{
  ObjectPtr* data;
  ObjectPtr* top;
  ObjectPtr* high;
  size_t size;
};



Stack* stack_new(size_t s);

void stack_destroy(Stack* stk);

void stack_push(Stack* stk, ObjectPtr ho);

ObjectPtr stack_pop(Stack* stk);

// look back n elements into the stack - last at position 1
ObjectPtr stack_peek(Stack* stk, int n);

void stack_init(Stack* stk);
void stack_reset(Stack* stk);
void stack_set_top(Stack* stk, ObjectPtr* newtop);

#define STACK_GET_TOP(stk) stk->top
#define STACK_GET_BASE(stk) stk->data

G_END_DECLS

#endif // STACK_H
