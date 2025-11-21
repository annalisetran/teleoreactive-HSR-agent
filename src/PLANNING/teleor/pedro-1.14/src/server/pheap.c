
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
#ifdef WIN32
        #include <windows.h>
        #define u_int UINT
#endif


#include "pheap.h"

extern void logit(char* fmt, ...);

/*
 * A Prolog heap for storing terms
 */

Heap* heap_new(size_t s, char* name)
{
  Heap* heap =  g_new(Heap, 1);
  heap->data = g_new(heapobject, s);
  heap->top = heap->data;
  heap->size = s;
  heap->high = heap->data + s;
  heap->heapname = name;
  return heap; 
}

void heap_destroy(Heap* h)
{
  g_free(h->data);
  g_free(h);
}


ObjectPtr allocateSpace(Heap* h, size_t size)
{
  assert(size > 0);
  if (h->top + size > h->high) {
    logit("Out of %s space\n" , h->heapname);
    exit(1);
  }
  ObjectPtr ret = h->top;
  h->top += size;
  return ret;
}


ObjectPtr pushInteger(Heap* heap, gint64 i)
{
  ObjectPtr pos = allocateSpace(heap, SIZEOFINTEGER);
  *pos = TYPEINTEGER;
  *(pos+1) = (heapobject)i;
  return pos;
}

ObjectPtr pushDouble(Heap* heap, double d)
{
  ObjectPtr pos = allocateSpace(heap, SIZEOFDOUBLE);
  *pos = TYPEDOUBLE;
  pos++;
  memcpy(pos, &d, sizeof(double));
  pos--;
  return pos;
}

ObjectPtr pushVariable(Heap* heap)
{
  ObjectPtr pos = allocateSpace(heap, SIZEOFVAR);
  *pos = (heapobject)pos;
  return pos;
}


ObjectPtr pushCompound(Heap* heap, size_t arity)
{
  ObjectPtr pos = allocateSpace(heap, SIZEOFCOMPOUNDBASE + arity);
  *pos = (heapobject)((u_int)(TYPECOMPOUND) | (arity << 4));
  return pos;
}

ObjectPtr pushList(Heap* heap)
{
  ObjectPtr pos = allocateSpace(heap, SIZEOFLIST );
  *pos = (heapobject)((u_int)(TYPELIST));
  return pos;
}

/*
 * The tokenizer matches a string (including the quotes
 * This version strinp off the quotes when storing the string
 */
ObjectPtr pushNewString(Heap* heap, const char* str)
{
  int size = strlen(str) - 2;
  int wordSize = 1 + (size >> 2);
  ObjectPtr pos = allocateSpace(heap, SIZEOFSTRINGBASE + wordSize);
  *pos = TYPESTRING | (wordSize << 4);
  char* cptr = (char*)(pos+1);
  strncpy(cptr, str+1, size);
  cptr[size] = '\0';
  return pos;
}

/*
 * This version is used when copying a string term (where the quotes
 * have already been removed
 */
ObjectPtr pushString(Heap* heap, const char* str)
{
  int size = strlen(str);
  int wordSize = 1 + (size >> 2);
  ObjectPtr pos = allocateSpace(heap, SIZEOFSTRINGBASE + wordSize);
  *pos = TYPESTRING | (wordSize << 4);
  char* cptr = (char*)(pos+1);
  strcpy(cptr, str);
  cptr[size] = '\0';
  
  return pos;
}

