
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
 * This program is the partner for ping.
 * It registers "pong" as it's name and waits for a peer-to-peer message
 * from ping and echos the message back to ping.
 * Exits when the message is the atom 'quit'
 */

#include <stdio.h>
#include <stdlib.h>
#include "client_api.h"

int main()
{
  // Create a connection to pedro
  PedroConnection* pong = pedro_default_connect();
  // Register the name "pong" with pedro
  if (pedro_register(pong, "pong") == 0) {
    fprintf(stderr, "Unable to register\n");
    exit(1);
  }
  // Create term buffers for storing Prolog terms
  TermBuffer *tb = termbuff_new();
  TermBuffer *out_tb = termbuff_new();
  // Create a write buffer to write a prolog term to a string
  WriteBuffer *wb = writebuff_new();
  char msg[PEDRO_BUFFER_SIZE];
  while(1) {
    // Wait for a message
    pedro_wait_message(pong);
    // Get the next message
    int rock;
    int msg_size =  pedro_get_message(pong, msg, &rock);
    // Display the message
    printf("|%s|\n", msg);
    // Reset tb ready for parsing the message
    termbuff_reset(tb);
    // parse the message into tb
    // msg_term points at the parsed Prolog term
    termptr msg_term = pedro_parse(msg, tb);
    // Check the message is a peer-to-peer message
    if ((get_type(msg_term) == PEDRO_STRUCTURE) &&
	(get_arity(msg_term) == 3) &&
	(atom_eq(get_functor(msg_term), "p2pmsg"))) {
      // exit if message is the atom quit
      if (atom_eq(get_arg(msg_term, 3), "quit")) break;
      // reset out_tb ready to build the response message
      termbuff_reset(out_tb);
      // make the "echo" atom
      termptr echo = push_atom("echo", out_tb);
      // make a structured term with 1 arg
      termptr echo_msg = push_struct(1, out_tb);
      // set the functor
      set_arg(0, echo, echo_msg);
      // set the argument to the message term from ping
      set_arg(1, get_arg(msg_term, 3), echo_msg);
      // reset wb ready to convert echo_msg into a string
      writebuff_reset(wb);
      // write echo_msg to the buffer
      writebuff_addterm(wb, echo_msg);
      // send peer-to-peer message
      if (pedro_p2p(pong, "ping@localhost", writebuff_getbuff(wb)) == 0) {
	fprintf(stderr, "Unable to send message\n");
	exit(1);
      }
    }  
  }
}
