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



#ifdef WIN32
        #include <winsock2.h>
        #define _WINSOCKAPI_
        #include <windows.h>
        typedef int socklen_t;

        //We also need to initialise the winsock tcp crud
        WSADATA wsaData;
        WORD wVersionRequested = MAKEWORD( 2, 2 );
        int err = WSAStartup( wVersionRequested, &wsaData );

#else
        #include <sys/types.h>
        #include <netdb.h>
        #include <sys/socket.h>
        #include <sys/file.h>
        #include <sys/ioctl.h>
        #include <sys/un.h>
        #include <net/if.h>
        #include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include "client_api.h"
#include "pedro_token.h"

/*
 * Get the IP address of the machine with given name
 */
u_long host_ip_address(char* hostname)
{
  if (strcmp(hostname, "localhost") == 0) {
    char namebuff[1000];
    strcpy(namebuff, "localhost");
    hostname = namebuff;
    if (gethostname(hostname, 1000) != 0) {
      return 0;
    }
  }
  struct hostent *hp = gethostbyname(hostname);
  if (hp != NULL) {
    return(*(int *)hp->h_addr_list[0]);
  }
  else {
    return 0;
  }
}

// Make a connection for given socket, port and ip address
// Return 1 if successful (0 otherwise)
int do_connection(int sockfd, int port, u_long ip_address)
{
  struct sockaddr_in add;
  memset((char *)&add, 0, sizeof(add));
  add.sin_family = AF_INET;
  add.sin_port = port;
  add.sin_addr.s_addr = ip_address;
  const int ret = connect(sockfd, (struct sockaddr *)&add, sizeof(add));
  if (ret == 0) return 0;

#ifdef WIN32
  else if ((errno = WSAGetLastError() == WSAEINPROGRESS) && errno == WSAEWOULDBLOCK)
#else
  else if (errno == EINPROGRESS)
#endif
    {
      fd_set fds;
      
      FD_ZERO(&fds);
      FD_SET((unsigned int)sockfd, &fds);
      select(sockfd + 1, (fd_set *) NULL, &fds, (fd_set *) NULL, NULL);
      return 0;
    }
  else
    {
      return 1;
    }
}

// Get the ack on given socket
int get_ack(int ack_fd)
{
  char buff[32];
  int size;
  int offset = 0;
  while (1) {
    size = recv(ack_fd, buff + offset, 30 - offset, 0);
    offset += size;
    if (offset > 25) {
      fprintf(stderr, "Can't get ack\n");
      exit(1);
    }
    if (buff[offset-1] == '\n') {
      buff[offset] = '\0';
      break;
    }
  }
  return atoi(buff);
}

// Replace (the first occurrence) of substring orig in str
// with repl putting the modified string in result
// result is copy of str if no occurrence found.
void str_replace(const char *str, const char *orig, 
                 const char *repl, char* result)
{
  char *ptr;
  if(!(ptr = strstr((char *)str, (char *)orig))) {
    strcpy(result, str);
    return;
  }
  strncpy(result, str, ptr-str);
  result[ptr-str] = '\0';
  sprintf(result+(ptr-str), "%s%s", repl, ptr+strlen(orig));
}

// Connect to pedro - return NULL if connection isn't made.
// malloc is used
PedroConnection* pedro_connect(int pedro_port, char* hostname)
{
  
  // Create a socket to get info
  int info_fd = socket(AF_INET, SOCK_STREAM, 0); 
  u_short port = ntohs(pedro_port);
  u_long ip_address = host_ip_address(hostname);

  // connect socket
  if (do_connection(info_fd, port, ip_address) != 0) {
    close(info_fd);
    return NULL;
  }
  // read info
  char infobuff[1024];
  int isize;
  int ioffset = 0;
  // Read information from server
  while (1) {
    isize = recv(info_fd, infobuff + ioffset, 1000 - ioffset, 0);
    ioffset += isize;
    if (ioffset > 1000) {
      fprintf(stderr, "Can't get info\n");
      close(info_fd);
      exit(1);
    }
    if (infobuff[ioffset-1] == '\n') {
      infobuff[ioffset] = '\0';
      break;
    }
  }
  // Info socket no longer needed
  close(info_fd);
  int ack_port, data_port;
  char ipstr[20];
  // Get the IP address of server and the ports to be used for ack and 
  // data connection
  if (sscanf(infobuff, "%s %d %d", ipstr, &ack_port, &data_port) != 3) {
    fprintf(stderr, "Can't read ip and ports\n");
    exit(1);
  }
  ack_port = htons((unsigned short)ack_port);
  data_port = htons((unsigned short)data_port);
  unsigned long ipaddr = inet_addr(ipstr);

  // Create a socket connection for ack
  int ack_fd = socket(AF_INET, SOCK_STREAM, 0); 
  if (do_connection(ack_fd, ack_port, ipaddr) != 0) {
    close(ack_fd);
    return NULL;
  }


  // get the client ID from server on ack socket
  u_int id = (u_int)get_ack(ack_fd);

  // create data socket
  int data_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (do_connection(data_fd, data_port, ipaddr) != 0) {
    close(ack_fd);
    close(data_fd);
    return NULL;
  }


  // send the client ID on data socket
  char buff[32];
  sprintf(buff, "%d\n", id);
  int len = strlen(buff);
  int num_written = send(data_fd, buff, len, 0);
  if (num_written != len) {
    fprintf(stderr, "Pedro Connect: Can't send ID\n");
    exit(1);
  }
  // read flag on data socket
  // was handshake successful?
  int size;
  int offset = 0;
  while (1) {
    size = recv(data_fd, buff + offset, 30 - offset, 0);
    offset += size;
    if (offset > 25) {
      fprintf(stderr, "Can't get flag\n");
      exit(1);
    }
    if (buff[offset-1] == '\n') {
      buff[offset] = '\0';
      break;
    }
  }

  // figure out my IP address
  static char host[256];
  struct sockaddr_in add;
  memset(&add, 0, sizeof(add));
  socklen_t addr_len = sizeof(add);
  
  getsockname(ack_fd, (struct sockaddr *)&add, &addr_len);
  strcpy(ipstr, inet_ntoa(add.sin_addr));
  /*
  struct in_addr in = add.sin_addr;
  struct hostent *hp = gethostbyaddr((char *) &in, sizeof(in), AF_INET);
  if (hp == NULL) 
    {
      // we can't look up name given address so just use dotted IP
      strcpy(host, ipstr);
    } 
  else 
    {
      // check if we can look up the same IP from hostname 
      struct hostent *hp2 = gethostbyname(hp->h_name);
      if ((hp2 == NULL) || (strcmp(hp->h_name, hp2->h_name) != 0))
        {
          // no - so use IP address
          strcpy(host, ipstr);
        }
      else
        {
          strcpy(host, hp->h_name);
        }
    }
  */
  strcpy(host, ipstr);
  // buff contains flag - test if "ok\n"
  if (strcmp(buff, "ok\n") == 0) {
    // handshake was successful - so create Pedro connection
    PedroConnection* client_connection = 
      (PedroConnection*)malloc(sizeof(PedroConnection));
    sprintf(client_connection->host, "'%s'", host);
    client_connection->data_fd = data_fd;
    client_connection->ack_fd = ack_fd;
    client_connection->data_buffer[0] = '\0';
    client_connection->data_offset = 0;
    client_connection->name[0] = '\0';
    return client_connection;
  }
  else
    return NULL;
}

// The default data connection
PedroConnection* pedro_default_connect()
{
  return pedro_connect(4550, (char*)("localhost"));
}

// destroy the connection, close sockets and free memory
void pedro_destroy(PedroConnection* connection)
{
  close(connection->data_fd);  
  close(connection->ack_fd);
  free(connection);
}


// Send the message to pedro
// return the ack
int pedro_send(PedroConnection* connection, char* msg)
{
  int len = strlen(msg);
  int num_written = send(connection->data_fd, msg, len, 0);
  if (num_written == len) 
    return 0;
  else {
    fprintf(stderr, "Message:\"%s\" not correctly sent\n", msg);
    return 1;
  }
}

// get the ack from server
int pedro_ack(PedroConnection* connection)
{
  return get_ack(connection->ack_fd);
}

// get the data FD - for use, e.g. in select
int pedro_get_data_fd(PedroConnection* connection)
{
  return connection->data_fd;
}

// get the hostname of client
char*  pedro_get_host(PedroConnection* connection)
{
  return connection->host;
}

// return 1 iff message is available
int data_ready(PedroConnection* connection)
{
  struct timeval tv = {0, 0};
  int data_fd = connection->data_fd;
   fd_set fds;
    FD_ZERO(&fds);
    FD_SET(data_fd, &fds);
    int result = select(data_fd + 1, &fds, (fd_set *) NULL, 
                        (fd_set *) NULL, &tv);
    return result;
}
  
// Get the message and rock from server (message length returned)
int pedro_get_message(PedroConnection* connection, char* msg, int* rock)
{
  int size;
  int offset = 0;
  int data_fd = connection->data_fd;
   fd_set fds;
  *msg = '\0';
  // Read from data socket until newline (end of message)
  // if necessary wait until newline
  while (1) {
    // keep reading until a newline
    FD_ZERO(&fds);
    FD_SET(data_fd, &fds);
    int result = select(data_fd + 1, &fds, (fd_set *) NULL, 
                        (fd_set *) NULL, NULL);
    if (!result || !FD_ISSET(data_fd, &fds))
      return 0;
    size = recv(data_fd, connection->data_buffer + connection->data_offset, 
                1023 - connection->data_offset, 0);
    if (size == 0) {
      fprintf(stderr, "Pedro disconnected client\n");
      exit(1);
    }
    connection->data_offset += size;
    connection->data_buffer[connection->data_offset] = '\0';
    char *p = strchr(connection->data_buffer, '\n');
    if (p != NULL) {
      int len = p - connection->data_buffer + 1;
      char* sp_ptr = connection->data_buffer;
      while (isdigit(*sp_ptr) && (sp_ptr < connection->data_buffer + len-1)) {
	sp_ptr++;
      }
      
      if (*sp_ptr != ' ') {
	fprintf(stderr, "Error: no rock or no following space\n");
	exit(1);
      }
      len = len - (connection->data_buffer - sp_ptr + 1);
      sscanf(connection->data_buffer, "%d", rock);
      strncpy(msg, sp_ptr+1, len);
      msg[len] = '\0';
      char *ptr = connection->data_buffer;
      p++;
      // slide string along
      while (*p != '\0') {
        *ptr = *p;
        p++;
        ptr++;
      }
      connection->data_offset -= len;
      return len;
    }
  }

}

// Block waiting for message from server
void pedro_wait_message(PedroConnection* connection)
{
  char *p = strchr(connection->data_buffer, '\n');
  if (p == NULL) {
    // no message yet - so wait
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(connection->data_fd, &fds);
    if (select(connection->data_fd + 1, &fds, (fd_set *) NULL, 
               (fd_set *) NULL, NULL) == -1) {
      fprintf(stderr, "Error: select failed\n");
      exit(1);
    }
  }
}

// Send a notification to server (0 returned means failure)
int pedro_notify(PedroConnection* connection, const char* term)
{
  char command[PEDRO_BUFFER_SIZE];
  sprintf(command, "%s\n", term);
  if (pedro_send(connection, command) != 0) return 0;
  return pedro_ack(connection);
}

// Subscribe with term, goal, and rock - returns ID
// (0 returned means failure)
int pedro_subscribe(PedroConnection* connection, const char* term,
                         const char* goal, int rock)
{
  char command[PEDRO_BUFFER_SIZE];
  sprintf(command, "subscribe(%s, (%s), %d)\n", term, goal, rock);
  if (pedro_send(connection, command) != 0) return 0;
  return pedro_ack(connection);
}

// Remove subscription with given ID (0 returned means failure)
int pedro_unsubscribe(PedroConnection* connection, int id)
{
  char command[PEDRO_BUFFER_SIZE];
  sprintf(command, "unsubscribe(%d)\n", id);
  if (pedro_send(connection, command) != 0) return 0;
  return pedro_ack(connection);
}

// Register name with server (0 returned means failure)
int pedro_register(PedroConnection* connection, const char* name)
{
  char command[PEDRO_BUFFER_SIZE];
  sprintf(command, "register(%s)\n", name);
  if (pedro_send(connection, command) != 0) return 0;
  strcpy(connection->name, name);
  return pedro_ack(connection);
}

// Deregister name with server (0 returned means failure)
int pedro_deregister(PedroConnection* connection, const char* name)
{
  if (connection->name[0] == '\0') return 0;
  char command[PEDRO_BUFFER_SIZE];
  sprintf(command, "deregister(%s)\n", connection->name);
  if (pedro_send(connection, command) != 0) return 0;
  return pedro_ack(connection);
}


// Send peer-to-peer message to given address (0 returned means failure)
int pedro_p2p(PedroConnection* connection, const char* toaddr, const char* term)
{
  if (connection->name[0] == '\0') return 0;
  char command[PEDRO_BUFFER_SIZE];
  char full_toaddr[256];
  char local[20];
  // replace "localhost" by clients "real" hostname
  str_replace(toaddr, "localhost", connection->host, full_toaddr);
  sprintf(command, "p2pmsg(%s, %s@%s, %s)\n", full_toaddr,
          connection->name, connection->host, term);
  if (pedro_send(connection, command) != 0) return 0;
  return pedro_ack(connection);
}
///////////////////////////////////////////////////////////////
//
// Create Prolog terms on TermBuffer and return pointer to term
//
termptr push_integer(long i, TermBuffer *buffer)
{
  if ((buffer->top - buffer->buffer) > PEDRO_BUFFER_SIZE - 2) {
    fprintf(stderr, "Error: Term buffer overflow\n");
    exit(1);
  }
  termptr  ptr = buffer->top;
  *(buffer->top) = (word)(PEDRO_INT);
  (buffer->top)++;
  *(buffer->top) = (word)(i);
  buffer->top += 1;
  return ptr;
}

termptr push_double(double i, TermBuffer *buffer)
{
  int size = sizeof(double)/sizeof(word);
  if ((buffer->top - buffer->buffer) > PEDRO_BUFFER_SIZE - 1 - size) {
    fprintf(stderr, "Error: Term buffer overflow\n");
    exit(1);
  }
  termptr  ptr = buffer->top;
  *(buffer->top) = (word)(PEDRO_DOUBLE);
  (buffer->top)++;
  *((double*)(buffer->top)) = i;
  buffer->top += size;
  return ptr;
}

termptr push_atom(const char* i, TermBuffer *buffer)
{
  int str_size = strlen(i);
  int size = 1 + str_size/sizeof(word);
  if ((buffer->top - buffer->buffer) > PEDRO_BUFFER_SIZE - 2) {
    fprintf(stderr, "Error: Term buffer overflow\n");
    exit(1);
  }
  termptr  ptr = buffer->top;
  *(buffer->top) = (word)(PEDRO_ATOM);
  (buffer->top)++;
  strcpy((char*)(buffer->top), i);
  buffer->top += size;
  return ptr;
}

termptr push_string(const char* i, TermBuffer *buffer)
{
  int str_size = strlen(i);
  int size = 1 + str_size/sizeof(word);
  if ((buffer->top - buffer->buffer) > PEDRO_BUFFER_SIZE - 2) {
    fprintf(stderr, "Error: Term buffer overflow\n");
    exit(1);
  }
  termptr  ptr = buffer->top;
  *buffer->top = (word)(PEDRO_STRING);
  (buffer->top)++;
  strcpy((char*)(buffer->top), i);
  buffer->top += size;
  return ptr;
}

termptr push_var(const char* i, TermBuffer *buffer)
{
  int str_size = strlen(i);
  int size = 1 + str_size/sizeof(word);
  if ((buffer->top - buffer->buffer) > PEDRO_BUFFER_SIZE - 2) {
    fprintf(stderr, "Error: Term buffer overflow\n");
    exit(1);
  }
  termptr  ptr = buffer->top;
  *buffer->top = (word)(PEDRO_VAR);
  buffer->top++;
  strcpy((char*)(buffer->top), i);
  buffer->top += size;
  return ptr;
}

termptr push_struct(int i, TermBuffer *buffer)
{
  if ((buffer->top - buffer->buffer) > PEDRO_BUFFER_SIZE - i - 2) {
    fprintf(stderr, "Error: Term buffer overflow\n");
    exit(1);
  }
  termptr  ptr = buffer->top;
  *buffer->top = (word)(PEDRO_STRUCTURE);
  buffer->top++;
  *buffer->top = (word)(i);
  buffer->top += i+2;
  return ptr;
}
termptr push_list(TermBuffer *buffer)
{
  if ((buffer->top - buffer->buffer) > PEDRO_BUFFER_SIZE - 4) {
    fprintf(stderr, "Error: Term buffer overflow\n");
    exit(1);
  }
  termptr  ptr = buffer->top;
  *buffer->top = (word)(PEDRO_LIST);
  buffer->top += 3;
  return ptr;
}

// set the head value of a list term
void set_head(termptr  lst, termptr  head)
{
  *(lst+1) = (word)head;
}

// set the tail value of a list term
void set_tail(termptr  lst, termptr  tail)
{
  *(lst+2) = (word)tail;
}

// Set an argument of a Prolog structured term (arg 0 is the functor)
void set_arg(int arg, termptr  val, termptr  str)
{
  *(str+2+arg) = (word)(val);
}

///////////////////////////////////////////////////////
//
// Extract information from a Prolog term
//
long get_type(termptr  str)
{
  return *((long*)(str));
}

long get_int(termptr  str)
{
  return *((long*)(str+1));
}

double get_double(termptr  str)
{
  return *((double*)(str+1));
}

char* get_var(termptr  str)
{
  return (char*)(str+1);
}

char* get_atom(termptr  str)
{
  return (char*)(str+1);
}

char* get_string(termptr  str)
{
  return (char*)(str+1);
}


termptr  get_head(termptr  str)
{
  return (termptr )(*(str+1));
}

termptr  get_tail(termptr  str)
{
  return (termptr )(*(str+2));
}
long get_arity(termptr  str)
{
  return (long)(*(str+1));
}

termptr  get_functor(termptr  str)
{
  return (termptr )(*(str+2));
}

termptr  get_arg(termptr  str, int arg)
{
  return (termptr )(*(str+2+arg));
}

// test if a is an atom whose name is str (1 for true; 0 for false)
int atom_eq(termptr a, const char* str)
{
  return (get_type(a) == PEDRO_ATOM) && (strcmp(str, get_atom(a)) == 0);
}

// For debugging
/*
void debug_print(termptr  term, int indent)
{
  long ttype = get_type(term);
  switch (ttype)
    {
    case PEDRO_INT:
      printf("%*sint      : %ld\n", indent*4, " ", get_int(term));
      break;
    case PEDRO_DOUBLE:
      printf("%*sdouble   : %f\n", indent*4, " ", get_double(term));
      break;
     case PEDRO_ATOM:
      printf("%*satom     : %s\n", indent*4, " ", get_atom(term));
      break;
     case PEDRO_VAR:
      printf("%*svar      : %s\n", indent*4, " ", get_var(term));
      break;
    case PEDRO_STRING:
      printf("%*sstring   : %s\n", indent*4, " ", get_string(term));
      break;
    case PEDRO_STRUCTURE:
      {
        int arity = get_arity(term);
        printf("%*sstruct : \n", indent*4, " ");
        debug_print(get_functor(term), indent+1);
        int i;
        for (i = 1; i <= arity; i++)
          debug_print(get_arg(term, i), indent+1);
        break;
      }
    case PEDRO_LIST:
      {
        printf("%*slist   :\n", indent*4, " ");
        debug_print(get_head(term), indent+1);
        debug_print(get_tail(term), indent+1);
        break;
      }
    }
}
*/

// Create a new WriteBuffer (using malloc)
WriteBuffer* writebuff_new(void) 
{
  WriteBuffer* wb = (WriteBuffer*)(malloc(sizeof(WriteBuffer)));
  wb->offset = 0;
}
  
// Delete (free) WriteBuffer
void writebuff_delete(WriteBuffer* wb)
{
  free(wb);
}

// Reset the WriteBuffer offset
void writebuff_reset(WriteBuffer* wb)
{
  wb->offset = 0;
}

// Add chars to WriteBuffer
void writebuff_addchars(WriteBuffer* buff, const char* chars)
{
  int size = strlen(chars);
  if (buff->offset + size + 2 > PEDRO_BUFFER_SIZE) {
    fprintf(stderr, "Error: write buffer overflow\n");
    exit(1);
  }
  strcpy(buff->buff + buff->offset, chars);
  buff->offset += size;
}


void writebuff_addterm_aux( WriteBuffer* buff, termptr term, int prec);

// Add string representing Prolog  term to WriteBuffer
void writebuff_addterm(WriteBuffer* buff, termptr term)
{
  writebuff_addterm_aux(buff, term, 999);
}

// Get the string stored in the WriteBuffer
char* writebuff_getbuff(WriteBuffer* buff) {
  return buff->buff;
}

// Create a TermBuffer (using malloc)
TermBuffer* termbuff_new(void) 
{
  TermBuffer* tb = (TermBuffer*)(malloc(sizeof(TermBuffer)));
  tb->top = tb->buffer;
}

// Delete (free) TermBuffer  
void termbuff_delete(TermBuffer* tb)
{
  free(tb);
}

// Reset buffer top
void termbuff_reset(TermBuffer* tb)
{
  tb->top = tb->buffer;
}


// write prefix operator term to WriteBuff based on precedences
// - determine if surrounding brackets are needed
void write_prefix(WriteBuffer* buff, termptr functor, termptr arg, 
		  int prec, int argprec)
{
  if (argprec < prec) {
    writebuff_addterm_aux(buff, functor, 999);
    writebuff_addchars(buff, " ");
    writebuff_addterm_aux(buff, arg, argprec);
  } else {
    writebuff_addchars(buff, "(");
    writebuff_addterm_aux(buff, functor, 999);
    writebuff_addchars(buff, " ");
    writebuff_addterm_aux(buff, arg, argprec);
    writebuff_addchars(buff, ")");
  }
}

// get precedence info for given functor name
void get_infix_prec(char* functor, int *funprec, int *lprec, int *rprec)
{
  if (strcmp(functor, "**") == 0)
    {
      *funprec = 200;
      *lprec = 199;
      *rprec = 199;
    }
  else if ((strcmp(functor, "*") == 0) ||
	   (strcmp(functor, "/") == 0) ||
	   (strcmp(functor, "//") == 0) ||
	   (strcmp(functor, "mod") == 0) ||
	   (strcmp(functor, "<<") == 0) ||
	   (strcmp(functor, ">>") == 0))
    {
      *funprec = 400;
      *lprec = 400;
      *rprec = 399;
    }
  else if ((strcmp(functor, "+") == 0) ||
	   (strcmp(functor, "-") == 0) ||
	   (strcmp(functor, "/\\") == 0) ||
	   (strcmp(functor, "\\/") == 0))
    {
      *funprec = 500;
      *lprec = 500;
      *rprec = 499;
    }
  else if ((strcmp(functor, "=") == 0) ||
	   (strcmp(functor, "<") == 0) ||
	   (strcmp(functor, ">") == 0) ||
	   (strcmp(functor, "=<") == 0) ||
	   (strcmp(functor, "is") == 0) ||
	   (strcmp(functor, ">=") == 0))
    {
      *funprec = 700;
      *lprec = 699;
      *rprec = 699;
    }
  else if (strcmp(functor, ",") == 0)
    {
      *funprec = 1000;
      *lprec = 999;
      *rprec = 1000;
    }
  else if (strcmp(functor, "->") == 0)
    {
      *funprec = 1050;
      *lprec = 1049;
      *rprec = 1050;
    }
  else if (strcmp(functor, ";") == 0)
    {
      *funprec = 1100;
      *lprec = 1099;
      *rprec = 1100;
    }
  else if (strcmp(functor, ":") == 0)
    {
      *funprec = 50;
      *lprec = 49;
      *rprec = 49;
    }
  else if (strcmp(functor, "@") == 0)
    {
      *funprec = 200;
      *lprec = 199;
      *rprec = 199;
    }
}

// write infix term to buff based on given precedences
void write_infix(WriteBuffer* buff, termptr functor, termptr larg, termptr rarg,
		 int prec, int funprec, int leftprec, int rightprec)
{
  if (funprec <= prec)
    {
      writebuff_addterm_aux(buff, larg, leftprec);
      writebuff_addchars(buff, " ");
      writebuff_addterm_aux(buff, functor, 999);
      writebuff_addchars(buff, " ");
      writebuff_addterm_aux(buff, rarg, rightprec);
    }
  else
    {
      // extra brackets needed
      writebuff_addchars(buff, "(");
      writebuff_addterm_aux(buff, larg, leftprec);
      writebuff_addchars(buff, " ");
      writebuff_addterm_aux(buff, functor, 999);
      writebuff_addchars(buff, " ");
      writebuff_addterm_aux(buff, rarg, rightprec);
      writebuff_addchars(buff, ")");
    }
}

// write infix term when functor is ',' to buff
void write_infix_comma(WriteBuffer* buff, termptr larg, termptr rarg, int prec)
{
  if (1000 <= prec)
    {
      writebuff_addterm_aux(buff, larg, 999);
      writebuff_addchars(buff, " , ");
      writebuff_addterm_aux(buff, rarg, 1000);
    }
  else
    {
      // extra brackets needed
      writebuff_addchars(buff, "(");
      writebuff_addterm_aux(buff, larg, 999);
      writebuff_addchars(buff, " , ");
      writebuff_addterm_aux(buff, rarg, 1000);
      writebuff_addchars(buff, ")");
    }
}

// write structure term to buff
void write_structure( WriteBuffer* buff, termptr str, int arity)
{
  writebuff_addterm_aux(buff, get_functor(str), 999);
  writebuff_addchars(buff, "(");
  int i;
  for (i = 1; i <= arity; i++) {
    writebuff_addterm_aux(buff, get_arg(str, i), 999);
    writebuff_addchars(buff, ", ");
  }
  buff->offset -= 2;
  writebuff_addchars(buff, ")");
}

// write term to buff with given precedence
void writebuff_addterm_aux(WriteBuffer* buff, termptr term, int prec)
{
  long ttype = get_type(term);
  switch (ttype)
    {
    case PEDRO_INT:
      {
        char tmp[100];
        sprintf(tmp, "%ld", get_int(term));
        writebuff_addchars(buff, tmp);
        break;
      }
    case PEDRO_DOUBLE:
      {
        char tmp[100];
        sprintf(tmp, "%f", get_double(term));
        writebuff_addchars(buff, tmp);
        break;
      }
     case PEDRO_ATOM:
       writebuff_addchars(buff, get_atom(term));
       break;
     case PEDRO_VAR:
       writebuff_addchars(buff, get_var(term));
       break;
    case PEDRO_STRING:
      writebuff_addchars(buff, get_string(term));
      break;
    case PEDRO_STRUCTURE:
      {
        int arity = get_arity(term);
        if (arity == 0) {
            writebuff_addterm_aux(buff, get_functor(term), 999);
            writebuff_addchars(buff, "()");
        } else if (arity > 2) {
	  // default structure write
	  write_structure(buff, term, arity);
	} else if (arity == 1) {
	  int argprec = -1;
	  termptr functor = get_functor(term);
	  if (atom_eq(functor, "-"))
	    {
	      argprec = 200;
	    }
	  if (argprec < 0)
	      {
		// default structure write
		write_structure(buff, term, 1);
	      }
	    else
	      {
		// write as prefix with '-' as functor
		termptr arg = get_arg(term, 1);
		write_prefix(buff, functor, arg, prec, argprec);
	      }
	} else {
	  // arity == 2
	  termptr functor = get_functor(term);
	  if (strcmp(get_atom(functor), ",") == 0) {
	    // functor is ','
	    write_infix_comma(buff, get_arg(term,1), get_arg(term, 2), prec);
	  } else {
	    int funprec = -1;
	    int leftprec = -1;
	    int rightprec = -1;
	    get_infix_prec(get_atom(functor), &funprec, &leftprec, &rightprec);
	    if (funprec < 0) {
	      // default structure write
	      write_structure(buff, term, 2);
	    } else {
	      // write as infix term
	      write_infix(buff, functor,  get_arg(term,1), get_arg(term, 2), 
			  prec, funprec, leftprec, rightprec);
	    }
	    
	  }
	}
	break;
      }
    case PEDRO_LIST:
      {
        writebuff_addchars(buff, "[");
        termptr next = term;
        while (get_type(next) == PEDRO_LIST) {
          writebuff_addterm_aux(buff, get_head(next), 999);
          writebuff_addchars(buff, ", ");
          next = get_tail(next);
        }
        buff->offset -= 2;
        if (atom_eq(next, "[]")) {
          writebuff_addchars(buff, "]");
          return;
        }
        writebuff_addchars(buff, "|");
        writebuff_addterm_aux(buff, next, 999);
        writebuff_addchars(buff, "]");
        return;
        break;
      }
    }
}

// assigned in next_token and used in parser
int current_token_type;   
termptr  current_token;

// stack used for parsing arguments to structure
termptr  args_stack[PEDRO_BUFFER_SIZE];
int args_stack_offset;

// build a term from next token obtained from tokenizer
// ("special" tokens have NULL as term)
int next_token(TermBuffer* tbuff)
{
  current_token_type = yylex();
  switch (current_token_type)
    {
    case  ERROR_TOKEN:
      fprintf(stderr, "Error: parse error\n");
      exit(1);
      break;
    case  NEWLINE_TOKEN:
    case  OBRA_TOKEN:
    case  CBRA_TOKEN:
    case  OSBRA_TOKEN:
    case  CSBRA_TOKEN:
    case  VBAR_TOKEN:
      current_token = NULL;
      break;
    case  COMMA_TOKEN:
      current_token = push_atom(yytext, tbuff);
      break;
    case  INT_TOKEN:
      current_token = push_integer(atoi(yytext), tbuff);
      break;
    case  DOUBLE_TOKEN:
      current_token = push_double(atof(yytext), tbuff);
      break;
    case  ATOM_TOKEN:
      current_token = push_atom(yytext, tbuff);
      break;
    case  STRING_TOKEN:
      current_token = push_string(yytext, tbuff);
      break;
    case  VAR_TOKEN:
      current_token = push_var(yytext, tbuff);
      break;
    }
}

termptr  parse_prec700(TermBuffer* tbuff);
termptr  parse_prec1100(TermBuffer* tbuff);

// parse arguments of structure - pushing onto args_stack
int parseargs(TermBuffer* tbuff)
{
  int num = 0;
  termptr  t = parse_prec700(tbuff);
  args_stack[args_stack_offset++] = t;
  num++;
  while (current_token_type == COMMA_TOKEN) {
    next_token(tbuff);
    t = parse_prec700(tbuff);
    if (args_stack_offset > PEDRO_BUFFER_SIZE - 2) {
      fprintf(stderr, "Error: parse error\n");
      exit(1);
    }
    args_stack[args_stack_offset++] = t;
    num++;
  }
  return num; 
}

// parse basic terms
termptr  parse_basic(TermBuffer* tbuff)
{
  switch (current_token_type)
    {
    case  ERROR_TOKEN:
    case  NEWLINE_TOKEN:
      fprintf(stderr, "Error: parse error\n");
      exit(1);
      break;
    case  OBRA_TOKEN:
      {
        next_token(tbuff);
        termptr  t1 = parse_prec1100(tbuff);
        if (current_token_type == CBRA_TOKEN) {
          next_token(tbuff);
          return t1;
            }
        fprintf(stderr, "Error: parse error\n");
        exit(1);
        break;
      }
    case  CBRA_TOKEN:
      fprintf(stderr, "Error: parse error\n");
      exit(1);
      break;
    case  OSBRA_TOKEN:
      {
        next_token(tbuff);
        if (current_token_type == CSBRA_TOKEN) {
          return push_atom("[]", tbuff);
        }
        termptr  t = parse_prec700(tbuff);
        termptr  lst = push_list(tbuff);
        set_head(lst, t);
        termptr  lst_tmp = lst;
        while (current_token_type == COMMA_TOKEN) {
          next_token(tbuff);
          t = parse_prec700(tbuff);
          termptr  tmp =  push_list(tbuff);
          set_head(tmp, t);
          set_tail(lst_tmp, tmp);
          lst_tmp = tmp;
        }
        if (current_token_type == VBAR_TOKEN) {
          next_token(tbuff);
          t = parse_prec700(tbuff);
          set_tail(lst_tmp, t);
        }
        else {
          set_tail(lst_tmp, push_atom("[]", tbuff));
        }
        if (current_token_type != CSBRA_TOKEN) {
          fprintf(stderr, "Error: parse error\n");
          exit(1);
        }
	next_token(tbuff);
        return lst;
        break;
      }     
    case  CSBRA_TOKEN:
      fprintf(stderr, "Error: parse error\n");
      exit(1);
      break;
    case  VBAR_TOKEN:
      fprintf(stderr, "Error: parse error\n");
      exit(1);
      break;
    case  INT_TOKEN:
    case  DOUBLE_TOKEN:
    case  STRING_TOKEN:
    case  VAR_TOKEN:
      {
        termptr  t1 = current_token;
        next_token(tbuff);
        return t1;
        break;
      }
    default:
      {
        // atom token
        termptr  t1 = current_token;
        next_token(tbuff);
        if (current_token_type != OBRA_TOKEN) {
          return t1;
        }
        next_token(tbuff);
        int current_offset = args_stack_offset;
        int size;
        if (current_token_type == CBRA_TOKEN) {
          size = 0;
        } else {
          size = parseargs(tbuff);
          if (current_token_type != CBRA_TOKEN) {
            fprintf(stderr, "Error: parse error\n");
            exit(1);
          }
        }
        next_token(tbuff);
        termptr  sterm = push_struct(size, tbuff);
        int i;
	// atom is functor - build structure from args_stack
        for (i = size; i > 0 ; i--) {
          set_arg(i, args_stack[current_offset+i-1], sterm);
        }
        args_stack_offset = current_offset;
        set_arg(0, t1, sterm);
        return sterm;
      }
    }
}

// parse with precedence 50 
termptr  parse_prec50(TermBuffer* tbuff)
{
  termptr  t1 = parse_basic(tbuff);
  if ((current_token_type == ATOM_TOKEN) && 
      (strcmp(":", get_atom(current_token)) == 0)) {
    termptr  op = current_token;
    next_token(tbuff);
    termptr  t2 =  parse_basic(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    return sterm;
  } 
  else {
    return t1;
  }
}

// parse with precedence 100
termptr  parse_prec100(TermBuffer* tbuff)
{
  termptr  t1 = parse_prec50(tbuff);
  if ((current_token_type == ATOM_TOKEN) && 
      (strcmp("@", get_atom(current_token)) == 0)) {
    termptr  op = current_token;
    next_token(tbuff);
    termptr  t2 =  parse_prec50(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    return sterm;
  } 
  else {
    return t1;
  }
}

// parse with precedence 200 
termptr  parse_prec200(TermBuffer* tbuff)
{
  if (current_token_type == NEWLINE_TOKEN) {
    fprintf(stderr, "Error: parse error\n");
    exit(1);
  }
  termptr  op = current_token;
  if ((current_token_type == ATOM_TOKEN) && 
      (strcmp("-", get_atom(current_token)) == 0)) {
    next_token(tbuff);
    termptr  t2 =  parse_prec100(tbuff);
    termptr  sterm = push_struct(1, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t2, sterm);
    return sterm;
  }
  termptr  t1 = parse_prec100(tbuff);
  if ((current_token_type == ATOM_TOKEN) && 
      (strcmp("**", get_atom(current_token)) == 0)) {
    termptr  t2 =  parse_prec100(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    t1 = sterm;
  }
  return t1;
}

// parse with precedence 400 
termptr  parse_prec400(TermBuffer* tbuff)
{
  termptr  t1 = parse_prec200(tbuff);
  while ((current_token_type == ATOM_TOKEN) && 
         ((strcmp("*", get_atom(current_token)) == 0) ||
          (strcmp("/", get_atom(current_token)) == 0) ||
          (strcmp("//", get_atom(current_token)) == 0) ||
          (strcmp("mod", get_atom(current_token)) == 0) ||
          (strcmp(">>", get_atom(current_token)) == 0) ||
          (strcmp("<<", get_atom(current_token)) == 0))
         ) {
    termptr  op = current_token;
    next_token(tbuff);
    termptr  t2 =  parse_prec200(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    t1 = sterm;
  } 
  return t1;
}

// parse with precedence 500 
termptr  parse_prec500(TermBuffer* tbuff)
{
  termptr  t1 = parse_prec400(tbuff);
  while ((current_token_type == ATOM_TOKEN) && 
         ((strcmp("+", get_atom(current_token)) == 0) ||
          (strcmp("-", get_atom(current_token)) == 0) ||
          (strcmp("\\/", get_atom(current_token)) == 0) ||
          (strcmp("/\\", get_atom(current_token)) == 0))
         ) {
    termptr  op = current_token;
    next_token(tbuff);
    termptr  t2 =  parse_prec400(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    t1 = sterm;
  } 
  return t1;
}

// parse with precedence 700 
termptr  parse_prec700(TermBuffer* tbuff)
{
  termptr  t1 = parse_prec500(tbuff);
  if ((current_token_type == ATOM_TOKEN) && 
      ((strcmp("=", get_atom(current_token)) == 0) ||
       (strcmp("is", get_atom(current_token)) == 0) ||
       (strcmp("<", get_atom(current_token)) == 0) ||
       (strcmp(">", get_atom(current_token)) == 0) ||
       (strcmp("=<", get_atom(current_token)) == 0) ||
       (strcmp(">=", get_atom(current_token)) == 0))
       ) {
    termptr  op = current_token;
    next_token(tbuff);
    termptr  t2 =  parse_prec500(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    return sterm;
  } 
  else {
    return t1;
  }
}

// parse with precedence 1000 : for ','
termptr  parse_prec1000(TermBuffer* tbuff)
{
  termptr  t1 = parse_prec700(tbuff);
  if (current_token_type == COMMA_TOKEN) {
    termptr  op = current_token;
    next_token(tbuff);
    termptr  t2 =  parse_prec1000(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    return sterm;
  } 
  else {
    return t1;
  }
}

// parse with precedence 1050 : for '->'
termptr  parse_prec1050(TermBuffer* tbuff)
{
  termptr  t1 = parse_prec1000(tbuff);
  if ((current_token_type == ATOM_TOKEN) && 
      (strcmp("->", get_atom(current_token)) == 0)) {
    termptr  op = current_token;
    next_token(tbuff);
    termptr  t2 =  parse_prec1050(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    return sterm;
  } 
  else {
    return t1;
  }
}

// parse with precedence 1100:  for ';'
termptr  parse_prec1100(TermBuffer* tbuff)
{
  termptr  t1 = parse_prec1050(tbuff);
  if ((current_token_type == ATOM_TOKEN) && 
      (strcmp(";", get_atom(current_token)) == 0)) {
    termptr  op = current_token;
    next_token(tbuff);
    termptr  t2 =  parse_prec1100(tbuff);
    termptr  sterm = push_struct(2, tbuff);
    set_arg(0, op, sterm);
    set_arg(1, t1, sterm);
    set_arg(2, t2, sterm);
    return sterm;

  } 
  else {
    return t1;
  }
}

// Parse the string representing a Prolgo term in buff into the TermBuffer.
// Returns a pointer to the term
termptr  pedro_parse(char* buff, TermBuffer* tbuff)
{
  //set up for tokenizer buffer
  int size = strlen(buff);
  buff[size+1] = '\0';
  set_buffstate(buff, size+2);
  args_stack_offset = 0;
  // initialize with first token
  next_token(tbuff);
  // parse term
  termptr  term = parse_prec1100(tbuff);
  // delete tokenizer buffer
  delete_buffstate();
  return term;
}
