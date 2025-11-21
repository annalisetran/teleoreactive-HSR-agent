
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
#include "../lib/pedro.h"
#include "../lib/pedro_defs.h"

/* 
 * A simple example program - a notification producer
 * reads in terms and send as notifications
 */

void callback(char* c, gpointer p1, gpointer p2) {}


int main()
{
  char term[BUFFSIZE];
  Client* producer = client_new(callback, 0);

  while (TRUE) {
    int size = read(0, term, BUFFSIZE-1);
    term[size-1] = '\0';
    
    if (size == 0) break;
    if (!notify(producer, term)) fprintf(stdout, "Bad notification\n");
  }
  client_destroy(producer);
}
