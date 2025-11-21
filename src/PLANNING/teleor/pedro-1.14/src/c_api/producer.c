
/*
  Copyright (C) 2011 Peter Robinson
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

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "client_api.h"

/* 
 * A simple example program - a notification producer
 * reads in terms and send as notifications
 */


int main(int argc, char** argv)
{
  char term[PEDRO_BUFFER_SIZE];
  // connect to pedro
  PedroConnection* producer = pedro_default_connect();
  if (producer == NULL) {
    fprintf(stderr, "Failed to connect\n");
    return 1;
  }

  
  while (1) {
    // read in notification
    printf("Notification? ");
    fflush(stdout);
    int size = read(0, term, PEDRO_BUFFER_SIZE-1);
    // exit on blank line
    if (size == 1) break;
    term[size-1] = '\0';
    // send notification
    if (pedro_notify(producer, term) == 0) 
      fprintf(stdout, "Bad notification\n");
    // a notification containing the string "quit" causes termination
    if (strstr(term, "quit") != NULL) break;
  }
}
