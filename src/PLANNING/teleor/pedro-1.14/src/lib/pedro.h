
/*
  Copyright (C) 2006, 2007, 2008 Peter Robinson
  Email: pjr@itee.uq.edu.au

  This library is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef CLIENT_H
#define CLIENT_H

#include <sys/types.h>
#include <glib.h>
#include "clientConnection.h"

G_BEGIN_DECLS

typedef struct _Client Client;


Client* client_new(void (*cb) (char*, gpointer, gpointer), gpointer user_data);
Client* client_new_full(int port, char* hostname,
		   void (*cb) (char*, gpointer, gpointer), gpointer user_data);

void client_destroy(Client* client);

int subscribe(Client* client, char* term, char* goal, gpointer rock);
gboolean unsubscribe(Client* client, gulong id);
gboolean notify(Client* client, char* term);
gboolean register_name(Client* client, char* name);
gboolean deregister_name(Client* client);
gboolean p2p(Client* client, char* toaddr, char* msg);

G_END_DECLS

#endif
