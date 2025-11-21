
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

/*
 * This program is the partner for pong.
 * It registers the name "ping" with Pedro and then repeatedly
 * gets a message from the user, sends it to pong as a peer-to-peer message,
 * and waits for the response from pong.
 * Exits when the user enters "quit" 
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "client_api.h"

int main()
{
  // Create a connection to pedro
  PedroConnection* ping = pedro_default_connect();
  // Register the name "ping" with pedro
  if (pedro_register(ping, "ping") == 0) {
    fprintf(stderr, "Unable to register\n");
    exit(1);
  }
  char msg[PEDRO_BUFFER_SIZE];
  while(1) {
    // read message from user
    printf("Message? ");
    fflush(stdout);
    int size = read(0, msg, PEDRO_BUFFER_SIZE-1);
    // ignore empty messages (just a newline)
    if (size == 1) continue;
    msg[size-1] = '\0';
    // send the peer-to-peer message 
    if (pedro_p2p(ping, "pong@localhost", msg) == 0) {
      fprintf(stderr, "Unable to send message\n");
      exit(1);
    }
    // exit if quit
    if (strcmp(msg, "quit") == 0) break;
    // wait for a message
    pedro_wait_message(ping);
    // get message
    int rock;
    int msg_size =  pedro_get_message(ping, msg, &rock);
    // display message
    printf("|%s|\n", msg);
   }  
}

