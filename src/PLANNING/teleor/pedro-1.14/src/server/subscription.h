
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

#ifndef SUBSCRIPTIONS_H
#define  SUBSCRIPTIONS_H

#include <glib.h>

#include "objects.h"
#include "atomTable.h"
#include "ptrail.h"
#include "pstack.h"
#include "codeBuffer.h"
#include "choicestack.h"
#include "connection.h"

G_BEGIN_DECLS


typedef struct _Subscription Subscription;

struct _Subscription
{
  Connection* connection;
  guint id;
  guint64 rock;
  Heap* subobjects;
  ObjectPtr head;
  guint64* code;
  int codesize;
};


Subscription* subscription_new(Connection* c, guint id, int size, 
			       ObjectPtr sub_term, CodeBuffer* cb, 
			       Trail* trail);

void subscription_destroy(Subscription* sub);


#define SUB_GET_HEAD(sub) sub->head
#define SUB_GET_ID(sub) sub->id
#define SUB_IS_MATCH(sub, mc, mid)  ((mc == sub->connection) && (sub->id == mid))

#ifndef NDEBUG
void dump(Subscription* sub);
#endif

typedef struct _SubscriptionTableEntry SubscriptionTableEntry;

struct _SubscriptionTableEntry
{
  GSList* subs;
};

SubscriptionTableEntry* subscription_table_entry_new();

void subscription_table_entry_destroy(SubscriptionTableEntry* e);

void subscription_table_entry_add(SubscriptionTableEntry* e,
				  Subscription* sub);
Subscription*  subscription_table_entry_remove(SubscriptionTableEntry* e,
					       Connection* c,
					       guint id);


void
sub_table_entry_match(SubscriptionTableEntry* e,
		      ObjectPtr term, Connection* conn,
		      Heap* heap, Trail* trail,
		      ChoicePointStack* cpstack,
		      Stack* stk);


void delete_int(gpointer key);
void delete_subscription_entry(gpointer sub);

typedef struct _SubscriptionTable SubscriptionTable;

struct _SubscriptionTable
{
  GHashTable* theHashTable;
};

SubscriptionTable* subscription_table_new();

void subscription_table_destroy(SubscriptionTable* st);


void subscription_table_add(SubscriptionTable* st,
			    int key,  Subscription* sub);

Subscription* subscription_table_remove(SubscriptionTable* st, Connection* c,
			       ObjectPtr head, guint id);

void
subscription_table_match(SubscriptionTable* st, ObjectPtr term,
		Connection* conn, Heap* heap, Trail* trail,
		ChoicePointStack* cpstack, Stack* stk);



G_END_DECLS


#endif
