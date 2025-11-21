
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

#include "subscription.h"
#include "connection.h"
#include "atoms.h"
#include "machine.h"
#include "manager.h"

extern Manager* manager;
extern Connection* admin_connection;
extern void logit(char* fmt, ...);

/*
 * Create a new subscription:
 * 1. The subscription is copied into a new heap of the appropriate size
 * 2. The "body" od the subscription is compiled into cb
 * 3. The code is optimized
 * 4. The code is copied into a new code area
 */
Subscription* subscription_new(Connection* conn, guint id, int size, 
			       ObjectPtr sub_term, CodeBuffer* cb, 
			       Trail* trail)
{
  Subscription* sub =  g_new(Subscription, 1);
  sub->subobjects = heap_new(size, "Subscription heap");
  ObjectPtr copy_sub_term = copyTerm(sub_term, sub->subobjects, trail);
  TRAIL_RESET(trail);
  sub->head = GET_ARG(copy_sub_term, 1);
  sub->rock =  GET_INTEGER(GET_ARG(sub_term, 3));
  CODE_BUFFER_RESET(cb);
  int alt = 0;
  int next = 0;
  gboolean addJump = TRUE;
  int index = 
    compile(GET_ARG(copy_sub_term, 2), 0, a_and, &alt, &next,  &addJump, 
	    TRUE, cb, trail);
  setCodeWord(cb, index, EXIT);
  // XXX
  //optimize(cb);
  sub->codesize = index+1;
  sub->code = g_new(guint64, sub->codesize);
  sub->connection = conn;
  sub->id = id;
  copyCodeblock(cb, sub->code, sub->codesize);
#ifndef NDEBUG
  dump(sub);
#endif
  assert(sub->subobjects != NULL);
  return sub;
}

void subscription_destroy(Subscription* sub)
{
  assert(sub != NULL);
  assert(sub->subobjects != NULL);
  g_free(sub->code);
  heap_destroy(sub->subobjects);
  g_free(sub);
}

#ifndef NDEBUG
void dump(Subscription* sub) 
{
  guint64* code = sub->code;
  int i;
  for (i = 0; i < sub->codesize; ) {
    logit("%u : \n", (guint64)(code+i));
    switch(code[i]) {
    case FAIL :
      logit("FAIL\n");
      i++;
      break;
    case TRY:
      logit("TRY\n");
      i++;
      logit("%u : %u\n", (guint64)(code+i), code[i]);
      i++;
      break;
    case RETRY:
      logit("RETRY\n");
      i++;
      logit("%u : %u\n", (guint64)(code+i), code[i]);
      i++;
      break;
    case TRUST :
      logit("TRUST\n");
      i++;
      break;
    case NOTTRY:
      logit("NOTTRY\n");
      i++;
      logit("%u : %u\n", (guint64)(code+i), code[i]);
      i++;
      break;
    case ONCE :
      logit("ONCE\n");
      i++;
      break;
    case IS :
      logit("IS\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case UNIFY :
      logit("UNIFY\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
	  
    case LT :
      logit("LT\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case GT :
      logit("GT\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case LE :
      logit("LE\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case GE :
      logit("GE\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case TTRUE :
      logit("TRUE\n");
      i++;
      break;
    case MEMBERTRY :
      logit("MEMBERTRY\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case MEMBER:	 
      logit("MEMBER\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case SPLITTRY :
      logit("SPLITTRY\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case SPLIT:	 
      logit("SPLIT\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case SPLITSTRINGTRY :
      logit("SPLITSTRINGTRY\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case SPLITSTRING:	 
      logit("SPLITSTRING\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case EXIT :
      logit("EXIT\n");
      i++;
      break;
    case JUMP:
      logit("JUMP\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case CUT :
      logit("CUT\n");
      i++;
      break;
    case IFTRY:
      logit("IFTRY\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case NOOP :
      logit("NOOP\n");
      i++;
      break;
    case NUMBER :
      logit("NUMBER\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case ATOM :
      logit("ATOM\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case STRING :
      logit("STRING\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
    case LIST :
      logit("LIST\n");
      i++;
      logit("%u\n", code[i]);
      i++;
      break;
	   
    default:
      assert(FALSE);
    }
  }
}
#endif

/*
 * A subscription table entry stores a list of subscriptions with the
 * same hash value
 */
SubscriptionTableEntry* subscription_table_entry_new()
{
  SubscriptionTableEntry* e = g_new(SubscriptionTableEntry, 1);
  e->subs = NULL;
  return e;
}
 
/*
 * Only used when hash table is destroyed
 */
void subscription_table_entry_destroy(SubscriptionTableEntry* e)
{
  if (e->subs != NULL)
    {
      GSList* lst;
      for (lst = e->subs; lst != NULL; lst = g_slist_next(lst))
	{
	  assert(((Subscription*)(lst->data))->subobjects != NULL);
	  subscription_destroy((Subscription*)(lst->data));
	}
      g_slist_free(e->subs);
    }
  g_free(e);
}

void subscription_table_entry_add(SubscriptionTableEntry* e, Subscription* sub)
{
  e->subs = g_slist_prepend(e->subs, sub);
}

Subscription* subscription_table_entry_remove(SubscriptionTableEntry* e,
					      Connection* c, 
					      guint id)
{
  GSList* lst;
  for (lst = e->subs; lst != NULL; lst = g_slist_next(lst)) {
    Subscription* sub = (Subscription*)(lst->data);
    if ((id == sub->id) && (c == sub->connection)) {
      assert(sub->subobjects != NULL);
      e->subs = g_slist_delete_link(e->subs, lst);
      assert(sub->subobjects != NULL);
      return sub;
    }
  }
  return NULL;
}

/*
 * Look through all the subscriptions in the hash table entry (that matches
 * hash value) and execute test - sending notification if successful
 */
void
sub_table_entry_match(SubscriptionTableEntry* e, 
		      ObjectPtr term, Connection* conn,
		      Heap* heap, Trail* trail, 
		      ChoicePointStack* cpstack,
		      Stack* stk)
{
  GSList* lst;
  for (lst = e->subs; lst != NULL; lst = g_slist_next(lst)) {
    Subscription *subptr = (Subscription*)(lst->data);
    if (execute(subptr, term, heap, trail, cpstack, stk)) {
      GIOStatus status = 
	connection_send_notification(subptr->connection, conn, subptr->rock);
      if (status != G_IO_STATUS_NORMAL) {
	initDestroy(subptr->connection->chan);
      }
    }
  }
}

void delete_int(gpointer key)
{
  g_free(key);
}

void delete_subscription_entry(gpointer sub)
{
  subscription_table_entry_destroy((SubscriptionTableEntry*)sub);
}

SubscriptionTable* subscription_table_new()
{
  SubscriptionTable* st = g_new(SubscriptionTable, 1);
  st->theHashTable = 
    g_hash_table_new_full(g_int_hash, g_int_equal,
			  delete_int, delete_subscription_entry);
  return st;
}


void subscription_table_destroy(SubscriptionTable* st)
{
  g_hash_table_destroy(st->theHashTable);
  g_free(st);
}

void subscription_table_add(SubscriptionTable* st, 
			    int key,  Subscription* sub)
{
  SubscriptionTableEntry* result = 
    (SubscriptionTableEntry*)g_hash_table_lookup(st->theHashTable, &key);
  if (result == NULL)
    {
      result = subscription_table_entry_new();
      int* keyptr = g_new(int, 1);
      *keyptr = key;
      g_hash_table_insert(st->theHashTable, keyptr, result);
    }
  subscription_table_entry_add(result, sub);
}

/*
 * remove sub from table:
 * 1. remove from entry
 * 2. if entry becomes empty remove from hash table
 * 3. dec ref count on all atoms in sub
 */
Subscription* subscription_table_remove(SubscriptionTable* st, Connection* c, 
			       ObjectPtr head, guint id)
{
  guint64 key = makeKey(head);
  
  SubscriptionTableEntry* result = 
    (SubscriptionTableEntry*)g_hash_table_lookup(st->theHashTable, &key);

  if (result == NULL)
    {
      return NULL;
    }

  Subscription* sub = subscription_table_entry_remove(result, c, id);
  if (sub != NULL) {
    dec_refcount_all_atoms(manager->atom_table, sub->subobjects->data);
    if (result->subs == NULL)
      g_hash_table_remove(st->theHashTable, &key);
  }
  return sub;
}

void 
subscription_table_match(SubscriptionTable* st, ObjectPtr term, 
		Connection* conn, Heap* heap, Trail* trail, 
		ChoicePointStack* cpstack, Stack* stk)
{
  if (admin_connection != NULL) {
    GIOStatus status = 
      connection_send_notification(admin_connection, conn, 0);
    if (status != G_IO_STATUS_NORMAL) {
      initDestroy(admin_connection->chan);
    }
  }
  guint64 key = makeKey(term);
  SubscriptionTableEntry* result = 
    (SubscriptionTableEntry*)g_hash_table_lookup(st->theHashTable, &key);
  if (result != NULL)
    sub_table_entry_match(result, term, conn, heap, trail, cpstack, stk);
  /* now try match against VAR terms */
  key = 0;
  SubscriptionTableEntry* varresult = 
    (SubscriptionTableEntry*)g_hash_table_lookup(st->theHashTable, &key);
  if (varresult != NULL)
    sub_table_entry_match(varresult, term, conn, heap, trail, cpstack, stk);
}


