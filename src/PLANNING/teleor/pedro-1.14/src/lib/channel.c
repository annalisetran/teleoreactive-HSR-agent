
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
	#include <unistd.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <netinet/tcp.h>

#endif
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "channel.h"
#include "channel_watch.h"


/* don't actually destroy the channel immediately - delay 200 ns */
void initDestroy(gpointer data)
{
  Channel* chan = (Channel*)data;
  if (!chan->do_destroy) {
    unsetInPoll(chan);
    unsetOutPoll(chan);
    chan->do_destroy = TRUE;
    g_timeout_add(200, chan->destroy, chan->data);
  }
}


void incallback(gpointer data)
{
  assert(data != NULL);
  Channel* chan = (Channel*)data;
  if (!chan->do_destroy && chan->recv(chan->data) != G_IO_STATUS_NORMAL) 
    initDestroy(chan);
}

void outcallback(gpointer data)
{
  assert(data != NULL);
  Channel* chan = (Channel*)data;
  if (!chan->do_destroy) {
    GIOStatus status = chan->send(chan->data);
    if (status != G_IO_STATUS_NORMAL) {
      initDestroy(chan);
    }
  }
}

Channel* channel_new(int f, GIOStatus (*recv) (gpointer), 
		     GIOStatus (*send) (gpointer), gboolean (*destroy) (gpointer),
		     gpointer data)
{
  Channel* chan =  g_new(Channel, 1);
#ifdef WIN32
  chan->channel = g_io_channel_win32_new_socket(f);
#else
  chan->channel = g_io_channel_unix_new(f);
#endif
  chan->fd = f;
  chan->send = send;
  chan->recv = recv;
  chan->destroy = destroy;
  chan->data = data;
  chan->do_destroy = FALSE;

#ifndef WIN32
  /* set socket to non-blocking */
  fcntl(f, F_SETFL, fcntl(f, F_GETFL) | O_NONBLOCK);
#else
  unsigned long flags = 1;
  ioctlsocket(f, FIONBIO, &flags);
  //fcntl(f, F_SETFL, O_NONBLOCK);
#endif
  /* send immediately on socket - don't delay */
  int flag=1;
  setsockopt(f, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));
  GError *err = NULL;
  if (g_io_channel_set_encoding(chan->channel, NULL, &err) 
      != G_IO_STATUS_NORMAL)
    {
      g_error ("Error while setting encoding: %s\n", err->message);
    }
  g_clear_error(&err);
  g_io_channel_set_buffered(chan->channel, FALSE);
  chan->watch = channel_watch_new(chan);

  return chan;
}

void channel_destroy(Channel* chan)
{
  channel_watch_destroy(chan->watch);
  GError *err = NULL;
  g_io_channel_shutdown(chan->channel, TRUE, &err);
  g_io_channel_unref(chan->channel);
  g_clear_error(&err);
  g_free(chan);
}


void setInPoll(Channel* chan)
{
  if (!chan->do_destroy)
    channel_watch_setInPoll(chan->watch);
}

void setOutPoll(Channel* chan)
{
  if (!chan->do_destroy)
    channel_watch_setOutPoll(chan->watch);
}

void unsetInPoll(Channel* chan)
{
  channel_watch_unsetInPoll(chan->watch);
}

void unsetOutPoll(Channel* chan)
{
  channel_watch_unsetOutPoll(chan->watch);
}

GIOStatus readbuff(GIOChannel* channel, char* buff, int buff_size)
{
  gsize num_read;
  GError *err = NULL;
  GIOStatus ret = g_io_channel_read_chars(channel, 
					  buff, buff_size, 
					  &num_read, &err);
  if (ret == G_IO_STATUS_EOF)
    {
      g_clear_error(&err);
      return ret;
    }
  if (ret == G_IO_STATUS_AGAIN)
    {
      g_clear_error(&err);
      return G_IO_STATUS_NORMAL;
    }
  if (ret == G_IO_STATUS_ERROR)
    {
      g_clear_error(&err);
      return ret;
    }
  g_clear_error(&err);
  buff[num_read] = '\0';
  return G_IO_STATUS_NORMAL;
}

GIOStatus writebuff(GIOChannel* chan, const char* buff, gsize* num_written)
{
  GError *err = NULL;
  GIOStatus ret
    = g_io_channel_write_chars(chan, buff, -1, num_written, &err);
  switch(ret) {
  case G_IO_STATUS_ERROR:
    g_clear_error(&err);
    break;
  case G_IO_STATUS_NORMAL:
    ret = g_io_channel_flush(chan, &err);
    //if (ret != G_IO_STATUS_NORMAL) logit("FAIL\n");
    break;
  case G_IO_STATUS_EOF:
    break;
  case G_IO_STATUS_AGAIN:
    //logit("G_IO_STATUS_AGAIN\n");
    *num_written = 0;
    ret = G_IO_STATUS_NORMAL;
    break;
  default:
    assert(FALSE);
    break;
  }
  return ret;
}
