
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
#include <glib.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#include "../lib/pedro.h"
#include "../lib/pedro_defs.h"

/* 
 * A simple example program - a notification producer
 * reads in terms and send as notifications
 */

void callback(char* c, gpointer p1, gpointer p2) {}

static gint server_port = DEFAULTPORT;
static gchar* host = "localhost";
static gchar* notification = NULL;

static GOptionEntry entries[] = 
{
  { "port", 'p', 0, G_OPTION_ARG_INT, &server_port, "Use port p for connecting", "p" },
  { "host", 'h', 0, G_OPTION_ARG_STRING, &host, "Host of Pedro", NULL },
  { "notification", 'n', 0, G_OPTION_ARG_STRING, &notification, "Notification string", NULL },
  { NULL }
};

int main(int argc, char** argv)
{
  char term[BUFFSIZE];

  GError *error = NULL;

  GOptionContext* context = g_option_context_new ("");
  g_option_context_add_main_entries (context, entries, NULL);
  g_option_context_parse (context, &argc, &argv, &error);

  if (notification == NULL) {
    fprintf(stderr, "Must supply notification\n");
    exit(1);
  }

  Client* producer 
    = client_new_full(server_port, host, callback, 0);

  notify(producer, notification);
  client_destroy(producer);
}
