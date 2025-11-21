
#include <stdio.h>
#include <stdlib.h>

#include "ptrail.h"
#include "pheap.h"

extern void logit(char* fmt, ...);

/*
 * A Prolog trail
 */

Trail* trail_new(size_t s)
{
  Trail* trail =  g_new(Trail, 1);
  trail->data = g_new(ObjectPtr, s);
  trail->top = trail->data;
  trail->size = s;
  trail->high = trail->data + s;
  return trail; 
}

void trail_destroy(Trail* t)
{
  g_free(t->data);
  g_free(t);
}

void trail_push(Trail* t, ObjectPtr optr)
{
  if (t->top + 1 >= t->high) {
    logit("Out of trail space\n");
    exit(1);
  }
  *(t->top) = optr;
  t->top++;
}

void backtrack(Trail* t, ObjectPtr* savedtop)
{
  while (t->top > savedtop) {
    t->top--;
    RESET(*(t->top));
  }
}
