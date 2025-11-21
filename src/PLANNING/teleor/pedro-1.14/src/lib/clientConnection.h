
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
#ifndef CLIENT_CONNECTION_H
#define CLIENT_CONNECTION_H

#include <glib.h>


#include "channel.h"

G_BEGIN_DECLS

typedef struct _ClientConnection ClientConnection;
struct _ClientConnection
{
  int buff_size;

  char* inbuff;
  char* outbuff;
  char* outbuffptr;
  gsize chars2send;
  char* msg;
  GString* in;
  Channel* chan;
  void (*callback) (char*, gpointer, gpointer);
  gpointer user_data;
};

ClientConnection* 
client_connection_new(int f, int buffsize,
		      void (*cb) (char*, gpointer, gpointer), 
		      gpointer user_data);

gboolean client_connection_destroy(gpointer gconn);

GIOStatus client_connection_recv(gpointer gconn);
GIOStatus client_connection_send(gpointer gconn);
void send_it(ClientConnection* conn, GString* msg);


G_END_DECLS

#endif

