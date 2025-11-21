
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

#include "manager.h"
#include "subscription.h"
#include "pstack.h"
#include "varMap.h"
#include "parser.h"
#include "atoms.h"

extern void logit(char* fmt, ...);
extern gboolean check(ObjectPtr optr, Stack* stk, Trail* trail);
/*
 * The manager is used to bundle up all the global data areas
 */

extern gint sizes;

Manager* manager_new()
{
  Manager* mgr = g_new(Manager, 1);
  mgr->heap = heap_new(sizes, "the heap");
  mgr->atom_table = atom_table_new();
  mgr->trail = trail_new(sizes);
  mgr->stack = stack_new(sizes);
  mgr->cb =  codebuffer_new(sizes);
  mgr->cpstack = cpstack_new(sizes);
  mgr->sub_table = subscription_table_new();
  mgr->p2p = p2p_new();
  return mgr;
}

void  manager_destroy(Manager* mgr)
{
  subscription_table_destroy(mgr->sub_table);
  cpstack_destroy(mgr->cpstack);
  codebuffer_destroy(mgr->cb);
  stack_destroy(mgr->stack);
  trail_destroy(mgr->trail);
  atom_table_destroy(mgr->atom_table);
  heap_destroy(mgr->heap);
  p2p_destroy(mgr->p2p);
  g_free(mgr);
}

Subscription* makeSubscription(Manager* manager, Connection* conn, 
			       int id, ObjectPtr term)
{
  assert(IS_COMPOUND(term) && (GET_ARG(term, 0) ==  a_subscribe));
  if (!check(term, manager->stack, manager->trail))
    {
      logit("SUBSCRIBE ERROR\n");
      return NULL;
    }
  int size = manager->heap->top - manager->heap->data;
#ifndef NDEBUG
  printMe(term);
  logit("size = %d\n", size);
#endif
  Subscription* subscription =
    subscription_new(conn, id, size, term, manager->cb, manager->trail);

  subscription_table_add(manager->sub_table, makeKey(subscription->head), 
			 subscription);

  return subscription;
}




ObjectPtr buff2term(Manager* manager, char* buff, guint size)
{
  VarMap* varmap = varmap_new(manager->heap);
  HEAP_RESET(manager->heap);
  ObjectPtr term = parse(buff, size, manager->heap, manager->atom_table,
			 varmap, manager->stack);
/*   if (term != NULL) printMe(term); */
  varmap_destroy(varmap);
  return term;

}
