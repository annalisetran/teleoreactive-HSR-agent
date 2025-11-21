
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "client_api.h"

int main()
{
  // connect to pedro
  PedroConnection* consumer = pedro_default_connect();
    if (consumer == NULL) {
    fprintf(stderr, "Failed to connect\n");
    exit(1);
  }
  // read in subscription term and goal
  fprintf(stdout, "Term: ");
  fflush(stdout);
  char term[PEDRO_BUFFER_SIZE];
  int size = read(0, term, PEDRO_BUFFER_SIZE-1);
  // exit on blank line
  if (size == 1) return 1;
  term[size-1] = '\0';
  fprintf(stdout, "Goal: ");
  fflush(stdout);
  char goal[PEDRO_BUFFER_SIZE];
  size = read(0, goal, PEDRO_BUFFER_SIZE-1);
  // exit on blank line
  if (size == 1) return 1;
  goal[size-1] = '\0';
  // subscribe with term and goal
  int id = pedro_subscribe(consumer, term, goal, 0);
  fprintf(stdout, "Subscription ID = %d\n", id);
  char msg[PEDRO_BUFFER_SIZE];
  while(1) {
    // wait for message
    pedro_wait_message(consumer);
    // get message
    int rock;
    int msg_size =  pedro_get_message(consumer, msg, &rock);       
    printf("Notification: |%s|\n", msg);
    if (strstr(msg, "quit") != NULL) break;
  }  
}

