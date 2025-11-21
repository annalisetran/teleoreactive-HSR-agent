
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

/*
 * A simple example program for consuming notifications.
 * Read in a subscription - the program will then print out
 * any matching notifications
 * */

#include "../lib/pedro.h"
#include "../lib/pedro_defs.h"
#include <stdio.h>
 
void callback(char* msg, gpointer rock, gpointer data)
{
  fprintf(stderr, "%s\n", msg);
}


int main()
{
  Client* consumer =  client_new(callback, 0);
 
  fprintf(stdout, "Term: ");
  char term[BUFFSIZE];
  scanf("%s", term);
  fprintf(stdout, "Goal: ");
  char goal[BUFFSIZE];
  scanf("%s", goal);
  int id = subscribe(consumer, term, goal, 0);
  fprintf(stdout, "Subscription ID = %d\n", id);
  g_main_context_iteration(NULL, TRUE);

  GMainLoop*  loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(loop);
}

