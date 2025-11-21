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
#ifndef CHANNEL_WATCH_H
#define CHANNEL_WATCH_H

G_BEGIN_DECLS

typedef struct _ChannelWatch ChannelWatch;

struct _ChannelWatch {
  gpointer chanptr;     
  guint sourceid;
  gint events;
};

/* Create a new channel watch */
ChannelWatch* channel_watch_new(gpointer chan);

/* Destroy the channel watch */
void channel_watch_destroy(ChannelWatch *chanwatch);

/* Allow polling on input - i.e. add G_IO_IN to events */
void channel_watch_setInPoll(ChannelWatch *chanwatch);

/* Remove polling on input */
void channel_watch_unsetInPoll(ChannelWatch *chanwatch);

/* Allow polling on output - i.e. add G_IO_OUT to events */
void channel_watch_setOutPoll(ChannelWatch *chanwatch);

/* Remove polling on output */
void channel_watch_unsetOutPoll(ChannelWatch *chanwatch);


G_END_DECLS

#endif
