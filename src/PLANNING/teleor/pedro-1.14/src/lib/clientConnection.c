
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

#ifdef WIN32

#include <io.h>
#define _WINSOCKAPI_
#include <windows.h>
#include <winsock2.h>
typedef int socklen_t;

#else

#include <sys/types.h>
#include <sys/socket.h>

#endif

#include <string.h>
#include <stdio.h>

#include <assert.h>
#include "clientConnection.h"


ClientConnection*
client_connection_new(int f, int buffsize,
		      void (*cb) (char*, gpointer, gpointer), 
		      gpointer user_data)
{
  ClientConnection* conn =  g_new(ClientConnection, 1);
  conn->buff_size = buffsize;
  conn->inbuff = (char*)g_malloc(buffsize+2);
  conn->outbuff = (char*)g_malloc(buffsize+2);
  conn->msg = (char*)g_malloc(buffsize+2);
  conn->outbuffptr = conn->outbuff;
  conn->chars2send = 0;
  conn->in = g_string_new("");

  conn->chan = channel_new(f, client_connection_recv,
                           client_connection_send, 
			   client_connection_destroy, conn);
  conn->callback = cb;
  conn->user_data = user_data;

  return conn;
}

gboolean client_connection_destroy(gpointer gconn)
{
  ClientConnection* conn = (ClientConnection*)gconn;
  channel_destroy(conn->chan);
  g_free(conn->outbuff);
  g_free(conn->inbuff);
  g_free(conn->msg);
  g_string_free(conn->in, TRUE);

  g_free(conn);

  return FALSE;
}


GIOStatus client_connection_recv(gpointer gconn)
{
  ClientConnection* conn = (ClientConnection*)gconn;

  GIOStatus status 
    = readbuff(conn->chan->channel, conn->inbuff, conn->buff_size);
  if (status != G_IO_STATUS_NORMAL)
    return status;

  g_string_append(conn->in, conn->inbuff);
  char* nlptr = strchr(conn->in->str, '\n');

  while (nlptr != NULL) {
    /* complete message read */
    int pos = nlptr - conn->in->str;
    if (pos > conn->buff_size) {
      g_string_erase(conn->in, 0, pos+1);
      setInPoll(conn->chan);
      return  G_IO_STATUS_NORMAL;
    }
    // At this point we have a message we can process
    unsetInPoll(conn->chan);
    strncpy(conn->msg, conn->in->str, pos + 1);
    conn->msg[pos+1] = '\0';
    char* spptr = strchr(conn->msg, ' ');
    g_string_erase(conn->in, 0, pos+1);
    assert(spptr != NULL);

    // all messages should be int_ followed by a term
    guint64 id;
    sscanf(conn->msg, "%lu", &id);

    conn->callback(spptr+1, (gpointer)id, conn->user_data);

    nlptr = strchr(conn->in->str, '\n');
  }
  setInPoll(conn->chan);
  return  G_IO_STATUS_NORMAL;
}


GIOStatus client_connection_send(gpointer gconn)
{
  ClientConnection* conn = (ClientConnection*)gconn;

  assert(conn->chars2send != 0);
  gsize num_written;
  GIOStatus status = writebuff(conn->chan->channel, conn->outbuffptr, &num_written);
  conn->outbuffptr += num_written;

  if (status != G_IO_STATUS_NORMAL) 
    return status;

  if (num_written == conn->chars2send)
    {
      unsetOutPoll(conn->chan);
      conn->outbuffptr = conn->outbuff;
      conn->chars2send = 0;
      return status;
    }
  setOutPoll(conn->chan);
  return status;
}


void send_it(ClientConnection* conn, GString* msg)
{
  conn->chars2send = strlen(msg->str);
  strcpy(conn->outbuff, msg->str);
  conn->outbuffptr = conn->outbuff;
  client_connection_send(conn);
}
