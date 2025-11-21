
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
#ifndef CHANNEL_H
#define CHANNEL_H

#include <glib.h>
#include "channel_watch.h"

G_BEGIN_DECLS

typedef struct _Channel Channel;

struct _Channel 
{
  GIOChannel* channel;
  int fd;
  GIOStatus (*recv) (gpointer);
  GIOStatus (*send) (gpointer);
  gboolean (*destroy) (gpointer);
  gpointer data;
  ChannelWatch* watch;
  gboolean do_destroy;
};

Channel* channel_new(int f, GIOStatus (*recv) (gpointer), 
		     GIOStatus (*send) (gpointer), gboolean (*destroy) (gpointer),
		     gpointer data);

void incallback(gpointer data);
void outcallback(gpointer data);

void initDestroy(gpointer data);

void channel_destroy(Channel* chan);

void setInPoll(Channel* chan);
void setOutPoll(Channel* chan);
void unsetInPoll(Channel* chan);
void unsetOutPoll(Channel* chan);

GIOStatus readbuff(GIOChannel* chan, char* buff, int buff_size);
GIOStatus writebuff(GIOChannel* chan, const char* buff, gsize* num_written);

#define IN_POLLING(chan) (chan->watch->events & G_IO_IN)

G_END_DECLS

#endif
