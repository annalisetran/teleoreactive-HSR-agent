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
#ifndef P2P_H
#define P2P_H

#include <glib.h>
#include <assert.h>
#include <string.h>

#include "objects.h"
#include "connection.h"

G_BEGIN_DECLS


typedef struct _P2P  P2P;

guint64 name2ip(char* name);

gboolean valid_p2p_notification(ObjectPtr term);

gboolean correct_p2p_sender_address(P2P* p, ObjectPtr term, Connection* conn);

P2P* p2p_new();

void p2p_destroy(P2P* p);

gboolean p2p_register(P2P* p, ObjectPtr name, guint ip, Connection* conn);

gboolean p2p_deregister(P2P* p, ObjectPtr name, guint ip, Connection* conn);

void send_p2p_message(P2P* p, ObjectPtr term, Connection* conn);

G_END_DECLS


#endif
