
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
#ifndef MANAGER_H
#define MANAGER_H

#include <glib.h>

#include "objects.h"
#include "subscription.h"
#include "connection.h"
#include "pstack.h"
#include "choicestack.h"
#include "p2p.h"

G_BEGIN_DECLS

typedef struct _Manager Manager;

struct _Manager
{
  Heap* heap;
  AtomTable* atom_table;
  Trail* trail;
  Stack* stack;
  CodeBuffer* cb;
  ChoicePointStack* cpstack;
  SubscriptionTable* sub_table;
  P2P* p2p;
};

Manager* manager_new();

void manager_destroy(Manager* m);



ObjectPtr buff2term(Manager* manager,char* buff, guint pos);
Subscription* makeSubscription(Manager* manager, Connection* conn, 
			       int id, ObjectPtr term);

G_END_DECLS

#endif
