
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
 * A Pedro Client API
 */
 
#ifndef CLIENT_API_H
#define CLIENT_API_H
#include <sys/types.h>

// The size of the pedro buffer - this should match the buffer size for the
// Pedro server (default 1024).
#define PEDRO_BUFFER_SIZE 1024

// Types for Prolog terms
#define PEDRO_VAR        0
#define PEDRO_ATOM       1
#define PEDRO_INT        2
#define PEDRO_DOUBLE     3
#define PEDRO_STRING     4
#define PEDRO_STRUCTURE  5
#define PEDRO_LIST       6

#ifdef WIN32
typedef unsigned long u_long;
#endif

// Used to store Prolog terms
typedef unsigned long         	word;
typedef word*                   termptr;

// A buffer to store Prolog terms
typedef struct _TermBuffer
{
  word buffer[PEDRO_BUFFER_SIZE]; // the buffer
  termptr top;                    // pointer to first unused word in buffer
} TermBuffer;
  
/*
 * Prolog terms are stored in a block of words in TermBuffers as follows:
 * VAR       :  PEDRO_VAR       | block of words storing the name of the var
 * ATOM      :  PEDRO_ATOM      | block of words storing the name of the var
 *              - if the atom is quoted then quotes are part of name
 * INT       :  PEDRO_INT       | value
 * DOUBLE    :  PEDRO_DOUBLE    | block of words containing double
 * STRING    :  PEDRO_STRING    | block of words storing the string 
                   (including quotes)
 * STRUCTURE :  PEDRO_STRUCTURE | arity | functor_ptr | arg1_ptr | ...
 * LIST      :  PEDRO_LIST      | head_ptr | tail_ptr
 */

// Used for converting prolog terms into a string
typedef struct _WriteBuffer
{
  char buff[PEDRO_BUFFER_SIZE];   // buffer
  int offset;                     // offset into buffer where chars are 
                                  // to be placed
} WriteBuffer;

// Information for a Pedro connection
typedef struct _PedroConnection
{
  int data_fd;                      // the data socket FD
  int ack_fd;                       // the ack socket FD
  int data_offset;                  // offset to start of next message
  char host[128];                   // name of clients host
  char name[128];                   // registered name
  char data_buffer[PEDRO_BUFFER_SIZE]; // buffer top store message
  
} PedroConnection;

// Create Prolog terms in the TermBuffer
termptr push_integer(long i, TermBuffer *buffer);
termptr push_double(double i, TermBuffer *buffer);
termptr push_atom(const char* i, TermBuffer *buffer);
termptr push_string(const char* i, TermBuffer *buffer);
termptr push_var(const char* i, TermBuffer *buffer);
termptr push_struct(int i, TermBuffer *buffer);
termptr push_list(TermBuffer *buffer);
// Fill in a Prolog list term with head and tail
void set_head(termptr lst, termptr head);
void set_tail(termptr lst, termptr tail);
// Set an argument of a Prolog structured term (arg 0 is the functor)
void set_arg(int arg, termptr val, termptr str);
// Extract information from a Prolog term
long get_type(termptr str);
long get_int(termptr str);
double get_double(termptr str);
char* get_var(termptr str);
char* get_atom(termptr str);
char* get_string(termptr str);
termptr get_head(termptr str);
termptr get_tail(termptr str);
long get_arity(termptr str);
termptr get_functor(termptr str);
termptr get_arg(termptr str, int arg);

// test if a is an atom whose name is str (1 for true; 0 for false)
int atom_eq(termptr a, const char* str);

// for debugging
// void debug_print(termptr term, int indent);

// Create a new WriteBuffer (using malloc)
WriteBuffer* writebuff_new(void);
// Delete (free) WriteBuffer
void writebuff_delete(WriteBuffer* wb);
// Reset the WriteBuffer offset
void writebuff_reset(WriteBuffer* wb);
// Add chars to WriteBuffer
void writebuff_addchars(WriteBuffer* buff, const char* chars);
// Add string representing Prolog  term to WriteBuffer
void writebuff_addterm(WriteBuffer* buff, termptr term);
// Get the string stored in the WriteBuffer
char* writebuff_getbuff(WriteBuffer* buff);

// Create a TermBuffer (using malloc)
TermBuffer* termbuff_new(void);
// Delete (free) TermBuffer
void termbuff_delete(TermBuffer* tb);
// Reset buffer top
void termbuff_reset(TermBuffer* tb);

// Parse the string representing a Prolgo term in buff into the TermBuffer.
// Returns a pointer to the term
termptr pedro_parse(char* buff, TermBuffer* tbuff);

// Get the IP address for the given hostname
u_long host_ip_address(char* hostname);
// Connect to Pedro given port and hostname
PedroConnection* pedro_connect(int pedro_port, char* hostname);
// Default connection (port = 4550, hostname = localhost)
PedroConnection* pedro_default_connect();
// Close sockets and free memory
void pedro_destroy(PedroConnection* connection);
// Get the ack for a message sent to server.
int pedro_ack(PedroConnection* connection);
// get the data FD - for use, e.g. in select
int pedro_get_data_fd(PedroConnection* connection);
// get the hostname of client 
char*  pedro_get_host(PedroConnection* connection);
// Get the message and rock from server (message length returned)
int pedro_get_message(PedroConnection* connection, char* msg, int* rock);
// Block waiting for message from server
void pedro_wait_message(PedroConnection* connection);
// Send a notification to server (0 returned means failure)
int pedro_notify(PedroConnection* connection, const char* term);
// Subscribe with term, goal, and rock - returns ID
// (0 returned means failure)
int pedro_subscribe(PedroConnection* connection, const char* term,
		    const char* goal, int rock);
// Remove subscription with given ID (0 returned means failure)
int pedro_unsubscribe(PedroConnection* connection, int id);
// Register name with server (0 returned means failure)
int pedro_register(PedroConnection* connection, const char* name);
// Deregister name with server (0 returned means failure)
int pedro_deregister(PedroConnection* connection, const char* name);
// Send peer-to-peer message to given address (0 returned means failure)
int pedro_p2p(PedroConnection* connection, const char* toaddr, const char* term);


#endif
