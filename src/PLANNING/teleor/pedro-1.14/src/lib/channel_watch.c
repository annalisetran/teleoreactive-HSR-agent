
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
#include <glib.h>
#include <stdio.h>

#include "channel.h"

/* Management of sockets - sets and unsets watches on GIOChannels */

/* The callback for g_io_add_watch */
gboolean watch_callback(GIOChannel *source, GIOCondition condition, 
	                gpointer data)
{
  Channel* chan = (Channel *)data;
  if (condition & G_IO_HUP) {
    initDestroy(chan);
    return TRUE;
  }
  if (condition & G_IO_ERR) {
    initDestroy(chan);
    return TRUE;
  }

  if  (condition & chan->watch->events & G_IO_IN) {
    incallback(chan);
  }
  if (condition & chan->watch->events & G_IO_OUT) {
    outcallback(chan);
  }
  return TRUE;
}

/* Create a new channel watch */
ChannelWatch* channel_watch_new(gpointer chan)
{
  ChannelWatch* watch = g_new(ChannelWatch, 1);
  watch->chanptr = chan;
  watch->events = G_IO_IN | G_IO_HUP | G_IO_ERR;
  watch->sourceid = g_io_add_watch(((Channel *)chan)->channel, watch->events, 
                                   watch_callback,chan);
  return watch;
}

/* Destroy the channel watch */
void channel_watch_destroy(ChannelWatch *chanwatch)
{
  g_source_remove(chanwatch->sourceid);
  g_free(chanwatch);
}

/* Allow polling on input - i.e. add G_IO_IN to events */
void channel_watch_setInPoll(ChannelWatch *watch)
{
  if (watch->events & G_IO_IN) return;
  
  watch->events |= G_IO_IN;
  g_source_remove(watch->sourceid);
  Channel * chan = (Channel *)watch->chanptr;
  watch->sourceid = g_io_add_watch(chan->channel, watch->events, 
                                   watch_callback,chan);
}

/* Remove polling on input */
void channel_watch_unsetInPoll(ChannelWatch *watch)
{
  if (watch->events & G_IO_IN) {
    watch->events &= ~G_IO_IN;
    g_source_remove(watch->sourceid);
    Channel * chan = (Channel *)watch->chanptr;
    watch->sourceid = g_io_add_watch(chan->channel, watch->events,
                                     watch_callback,chan); 
  }
}

/* Allow polling on output - i.e. add G_IO_OUT to events */
void channel_watch_setOutPoll(ChannelWatch *watch)
{
  if (watch->events & G_IO_OUT) return;
  
  watch->events |= G_IO_OUT;
  g_source_remove(watch->sourceid);
  Channel * chan = (Channel *)watch->chanptr;
  watch->sourceid = g_io_add_watch(chan->channel, watch->events, 
                                   watch_callback,chan);
  
}

/* Remove polling on output */
void channel_watch_unsetOutPoll(ChannelWatch *watch)
{
  if (watch->events & G_IO_OUT) {
    watch->events &= ~G_IO_OUT;
    g_source_remove(watch->sourceid);
    Channel * chan = (Channel *)watch->chanptr;
    watch->sourceid = g_io_add_watch(chan->channel, watch->events,
                                     watch_callback,chan); 
    
  }
}
