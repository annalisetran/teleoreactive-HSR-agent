
/*
  Copyright (C) 2006-2009 Peter Robinson
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

#include <sys/types.h>

#ifdef WIN32

#include <io.h>
#define _WINSOCKAPI_
#include <windows.h>
#include <winsock2.h>
typedef int socklen_t;
#define uint UINT

#else

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <arpa/inet.h>
#endif

#include <unistd.h>
#include <glib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#include "pedro.h"
#include "pedro_defs.h"
#include "clientConnection.h"


struct _Client
{
  GIOChannel* ack_channel;
  ClientConnection* connection;
  GString* name;
  GString* host;
};

Client* client_new_aux(int port, u_long ip, 
		   void (*cb) (char*, gpointer, gpointer), gpointer ud);

gboolean client_connect_prepare(GSource *source, gint *timeout)
{
  *timeout = -1;
  return FALSE;
}

gboolean client_connect_check(GSource *source)
{
  return TRUE;
}

gboolean client_connect_dispatch(GSource *source, GSourceFunc callback, 
				 gpointer  user_data)
{
  return TRUE;
}


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

Client* client_new(void (*cb) (char*, gpointer, gpointer), gpointer ud)
{
  client_new_full(DEFAULTPORT, "localhost", cb, ud);
}

int read_from_channel(GIOChannel* channel, char* buff)
{
  int size;
  int offset = 0;
  GIOStatus status;
  while (1) {
    status = readbuff(channel, buff + offset, 30 - offset);
    if (status != G_IO_STATUS_NORMAL) return status;
    offset = strlen(buff);
    if (offset > 25) {
      return G_IO_STATUS_ERROR;
    }
    if (buff[offset-1] == '\n') {
      buff[offset] = '\0';
      break;
    }
  }
  return G_IO_STATUS_NORMAL;
}

Client* client_new_full(int port, char* hostname,
		   void (*cb) (char*, gpointer, gpointer), gpointer user_data)
{
  u_long ip = host_ip_address(hostname);
  if (ip == 0) {
    fprintf(stderr, "Invalid hostname: %s\n", hostname);
    exit(1);
  }
  client_new_aux(port, ip,  cb, user_data);
}

Client* client_new_aux(int port, u_long ip, 
		   void (*cb) (char*, gpointer, gpointer), gpointer ud)
{
  int info_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

  struct sockaddr_in add;
  memset((char *)&add, 0, sizeof(add));
  add.sin_family = AF_INET;
  add.sin_port =  htons((unsigned short)port);
  add.sin_addr.s_addr = ip;

  if(connect(info_fd, (struct sockaddr *)&add, sizeof(add)))
    {
      close(info_fd);
      fprintf(stderr, "Can't connect to info\n");
      exit(1);
    }
  // read info
  char infobuff[1024];
  int isize;
  int ioffset = 0;
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
  close(info_fd);
  int ack_port, data_port;
  char ipstr[20];
  if (sscanf(infobuff, "%s %d %d", ipstr, &ack_port, &data_port) != 3) {
    fprintf(stderr, "Can't read ip and ports\n");
    exit(1);
  }
  unsigned long ipaddr = inet_addr(ipstr);
  //inet_aton(ipstr, &ipaddr);

  int ack_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

  memset((char *)&add, 0, sizeof(add));
  add.sin_family = AF_INET;
  add.sin_port =  htons((unsigned short)ack_port);
  add.sin_addr.s_addr = ipaddr;

  if(connect(ack_fd, (struct sockaddr *)&add, sizeof(add)))
    {
      close(ack_fd);
      fprintf(stderr, "Can't connect to ack\n");
      exit(1);
    }

  Client* client =  g_new(Client, 1);

  memset(&add, 0, sizeof(add));
  socklen_t addr_len = sizeof(add);
  
  getsockname(ack_fd, (struct sockaddr *)&add, &addr_len);
  char hostname[1024];
  strcpy(hostname, inet_ntoa(add.sin_addr));
  client->host = g_string_new(hostname);

#ifdef WIN32
  client->ack_channel = g_io_channel_win32_new_socket(ack_fd);
#else
  client->ack_channel = g_io_channel_unix_new(ack_fd);
#endif

  GIOStatus status;
  GError *err = NULL;
  if (g_io_channel_set_encoding(client->ack_channel, NULL, &err) 
      != G_IO_STATUS_NORMAL)
    {
      g_error ("Error while setting encoding: %s\n", err->message);
    }
  g_clear_error(&err);
  g_io_channel_set_buffered(client->ack_channel, FALSE);

  char buff[32];
  status = read_from_channel(client->ack_channel, buff);

  if (status != G_IO_STATUS_NORMAL){
    fprintf(stderr, "Can't complete connection\n");
    close(ack_fd);
    exit(1);
  }
  uint id;
  sscanf(buff, "%u", &id);
  
  /* Now connect to the data socket */
  int data_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

  memset((char *)&add, 0, sizeof(add));
  add.sin_family = AF_INET;
  add.sin_port =  htons((unsigned short)data_port);
  add.sin_addr.s_addr = ipaddr;
  if(connect(data_fd, (struct sockaddr *)&add, sizeof(add)))
    {
      close(ack_fd);
      close(data_fd);
      fprintf(stderr, "Can't connect to data\n");
      exit(1);
    }

  client->connection = 
    client_connection_new(data_fd, BUFFSIZE, cb, ud);
  client->name = NULL;

  /* Send client ID on data socket and get back status */
  sprintf(buff, "%u\n",  id);
  int size = strlen(buff);
  gsize num_written;
  status = writebuff(client->connection->chan->channel, 
                     buff, &num_written);
  if (status != G_IO_STATUS_NORMAL || num_written != size) {
    fprintf(stderr, "Can't complete connection\n");
    close(ack_fd);
    close(data_fd);
    exit(1);
  }
  buff[0] = '\0';
  status = read_from_channel(client->connection->chan->channel, buff);
  if (status != G_IO_STATUS_NORMAL || strcmp(buff, "ok\n") != 0) {
    fprintf(stderr, "Can't complete connection\n");
    close(ack_fd);
    close(data_fd);
    exit(1);
  }

  client->name = NULL;
  return client;
}

void client_destroy(Client* client)
{
  client_connection_destroy((gpointer)(client->connection));
  GError *err = NULL;
  g_io_channel_shutdown(client->ack_channel, TRUE, &err);
  g_io_channel_unref(client->ack_channel);
  g_clear_error(&err);
  if (client->name != NULL) {
    g_string_free(client->name, TRUE);
    g_string_free(client->host, TRUE);
  }
  g_free(client);
}

int get_ack(Client* client)
{
  char buff[20];
  GIOStatus ret = readbuff(client->ack_channel, buff, 19);
  if (ret != G_IO_STATUS_NORMAL) return 0;
  while (strchr(buff, '\n') == NULL) {
    int len = strlen(buff);
    ret = readbuff(client->ack_channel, buff + len, 19 - len);
    if (ret != G_IO_STATUS_NORMAL) return 0;
  }
  return atoi(buff);
}

int subscribe(Client* client, char* term, char* goal, gpointer rock)
{
  GString* gstr = g_string_new("");
  g_string_printf(gstr, "subscribe(%s, %s, %lu)\n", term, goal, (gulong)rock);
  send_it(client->connection, gstr);
  g_string_free(gstr, TRUE);
  return get_ack(client);
}


gboolean
unsubscribe(Client* client, gulong id)
{
  GString* gstr = g_string_new("");
  g_string_printf(gstr, "unsubscribe(%lu)\n", id);
  send_it(client->connection, gstr);
  g_string_free(gstr, TRUE);
  return (get_ack(client) != 0);
}


gboolean
notify(Client* client, char* term)
{
  GString* gstr = g_string_new("");
  g_string_printf(gstr, "%s\n", term);
  send_it(client->connection, gstr);
  g_string_free(gstr, TRUE);
  return (get_ack(client) != 0);
}

gboolean
register_name(Client* client, char* name)
{
  if (client->name != NULL) return FALSE;
  GString* gstr = g_string_new("");
  g_string_printf(gstr, "register(%s)\n", name);
  send_it(client->connection, gstr);
  g_string_free(gstr, TRUE);
 
  if (get_ack(client) == 0) return FALSE;

  client->name = g_string_new(name);
  return TRUE;
}

gboolean
deregister_name(Client* client)
{
  if (client->name == NULL) return FALSE;
  GString* gstr = g_string_new("");
  g_string_printf(gstr, "deregister(%s)\n", client->name->str);
  send_it(client->connection, gstr);
  g_string_free(gstr, TRUE);
  
  if (get_ack(client) == 0) return FALSE;
  g_string_free(client->name, TRUE);
  g_string_free(client->host, TRUE);
  client->name = NULL;
  return TRUE;
}

gboolean p2p(Client* client, char* toaddr, char* msg)
{
  if (client->name == NULL) return FALSE;

  GString* gstr = g_string_new("");
  g_string_printf(gstr, "p2pmsg(%s, %s@'%s', %s)\n", toaddr, 
		  client->name->str, client->host->str, msg);
  send_it(client->connection, gstr);
  g_string_free(gstr, TRUE);
  return (get_ack(client) != 0);
}


