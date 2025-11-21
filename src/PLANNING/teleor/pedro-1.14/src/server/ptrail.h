
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
#ifndef TRAIL_H
#define TRAIL_H

#include <glib.h>

#include "objects.h"

struct Heap;

typedef struct _Trail Trail;
struct _Trail
{
  ObjectPtr* data;
  ObjectPtr* top;
  ObjectPtr* high;
  size_t size;
};

G_BEGIN_DECLS

Trail* trail_new(size_t s);

void trail_destroy(Trail* t);

void trail_push(Trail* t, ObjectPtr o);

#define TRAIL_GET_TOP(t) t->top

void backtrack(Trail* t, ObjectPtr* savedtop);


#define TRAIL_RESET(t) { backtrack(t, t->data); }

G_END_DECLS

#endif // TRAIL_H
