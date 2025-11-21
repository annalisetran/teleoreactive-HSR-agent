
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
// pheap.h - Storage of terms
//

#ifndef HEAP_H
#define HEAP_H

#include <glib.h>

#include "objects.h"

G_BEGIN_DECLS

typedef struct _Heap Heap;

struct _Heap
{
  ObjectPtr data;
  ObjectPtr top;
  ObjectPtr high;
  size_t size;
  char *heapname;
};


Heap* heap_new(size_t s, char* name);

void heap_destroy(Heap* h);

ObjectPtr allocateSpace(Heap* h, size_t size);


#define HEAP_RESET(heap) heap->top = heap->data;

ObjectPtr pushInteger(Heap* heap, gint64 i);
ObjectPtr pushDouble(Heap* heap, double d);
ObjectPtr pushVariable(Heap* heap);
ObjectPtr pushCompound(Heap* heap, size_t arity);
ObjectPtr pushList(Heap* heap);
ObjectPtr pushNewString(Heap* heap, const char* str);
ObjectPtr pushString(Heap* heap, const char* str);

G_END_DECLS


#endif // HEAP_H
