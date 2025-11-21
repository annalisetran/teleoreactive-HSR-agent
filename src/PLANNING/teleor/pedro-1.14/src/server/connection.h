
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
#ifndef CONNECTION_H
#define CONNECTION_H

#include <glib.h>

#ifndef WIN32

#include <sys/ioctl.h>

#endif

#include <unistd.h>
#include <fcntl.h>

#include "objects.h"
#include "../lib/pedro_defs.h"
#include "../lib/channel.h"


G_BEGIN_DECLS


typedef struct _Connection Connection;

typedef struct _DelayNotify DelayNotify;

struct _DelayNotify
{
  guint rock;               /* the rock of the subscription */
  Connection* notify_conn;  /* the connection sending me a notification */
};


struct _Connection
{
  ObjectPtr name;           /* registered name of this client */
  guint ip;                 /* IP address of client */
  char* inbuff;             /* raw input buffer  (for use in read) */
  char* parsebuff;          /* buffer for parsing */
  char numberbuff[16];      /* buffer for storing numbers for writing */
  char* notifybuff;         /* buffer containing current notification */
  GString* in;              /* collect inbuff for later processing */
  GSList*  subscriptions;   /* the list of subscriptions made by this client */
  int next_id;              /* id for next subscription for this client */
  int ref_count;            /* number of reference to this connection */
  int w_ref_count;          /* the number of connections write-blocked on */
                            /* my notification */
  GIOChannel* ack_channel;  /* this connection's ack channel */
  Channel* chan;            /* this connection's channel */
  GQueue* delays;           /* delayed notifications */
  gsize notifybuff_size;    /* the number of characters in notifybuff */
  guint notify_offset;      /* the offset into notify_conn's notifybuff */
  guint number_offset;      /* the current offset into numberbuff */
  guint number_len;         /* the length of the number in numberbuff */
  Connection* notify_conn;  /* the connection sending me a notification */
  guint timeout_id;
  gboolean has_incref;      /* true if this conn has alread called */
                            /*     inc_w_refcount(conn->notify_conn) */
  gboolean truncating;      /* true if input is to be truncated until nl */

};

Connection* connection_new(GIOChannel* chan, guint ip);

void connection_add_data_channel(Connection* conn, int f);

GIOStatus connection_send_notification(Connection* conn, Connection* sub_conn,
				       guint rock);

GIOStatus connection_send(gpointer gconn); 
gboolean connection_close(gpointer gconn);
GIOStatus connection_recv(gpointer gconn);

gboolean 
connection_remove_subscription(Connection* conn, guint id);

void 
connection_remove_all_subscription(Connection* conn);

G_END_DECLS

#endif
