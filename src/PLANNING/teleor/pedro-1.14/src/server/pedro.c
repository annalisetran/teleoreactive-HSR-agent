
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
#ifdef WIN32

#include <io.h>
#define _WINSOCKAPI_
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
typedef int socklen_t;
#define SIGPIPE SIGABRT

#else

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <netdb.h>
#include <sys/un.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#endif

#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/file.h>
#include <errno.h>
#include <glib.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <signal.h>
#include <time.h>

#include "config.h"
#include "manager.h"
#include "p2p.h"

static FILE *logf = NULL;
static int server_ack_socket;
static int server_data_socket;
static int server_info_socket;
GIOChannel* ack_chan;
GIOChannel* data_chan;
GIOChannel* info_chan;
char hostname[1024];
int ack_port;
int data_port;

static void getMachineAddress(char* ip);

// Improved logging thanks to Duncan White, Imperial College
void logit(char* fmt, ...)
{
  time_t now = time(NULL);
  struct tm *n = localtime( &now );
  fprintf( logf, "%04d/%02d/%02d %02d:%02d:%02d ",
	 n->tm_year+1900, n->tm_mon+1, n->tm_mday,
	 n->tm_hour, n->tm_min, n->tm_sec );
  va_list args;
  va_start(args,fmt);
  vfprintf(logf, fmt, args);
  va_end(args);
  fflush(logf);
}


pid_t ignorepid = -1;

void going_down()
{
  pid_t p = getpid();
  if (logf != NULL && p != ignorepid )
    logit("Process %u going down\n", p );
  if (server_ack_socket >= 0) close(server_ack_socket);
  if (server_data_socket >= 0) close(server_data_socket);
}


extern void set_buffstate(char* buff, int size);
extern void delete_buffstate();

Manager* manager;

GPollFD* server_ack_FD;
GPollFD* server_data_FD;

struct sockaddr_in addr;
socklen_t length = sizeof(struct sockaddr_in);

/* The following hash table is used to manage partial connections
 * i.e. where a client has connected on the ack socket but not on the 
 * data socket.
 * The hash and value are both the pointer to the connection structure
 */
GHashTable* connecting_table;
/*
 * This has table is for all the current connections and is similar to the 
 * above table.
 */
GHashTable* connected_table;

/* Data for a channel used during the initial client handshake
 * Used to read the client ID
 */
struct _data_connection_data
{
  char buff[32];
  GString* in;
  Channel* chan;
};

typedef struct  _data_connection_data  data_connection_data;

GIOStatus data_connection_send(gpointer gconn)
{
  return G_IO_STATUS_NORMAL;
}

gboolean data_connection_close(gpointer gp)
{
  data_connection_data* conn_data = (data_connection_data*)gp;
  channel_destroy(conn_data->chan);
  g_free(conn_data);
  return FALSE;
}

/* something sent on data socket from partially connected client */
GIOStatus data_connection_recv(gpointer gp)
{
  data_connection_data* conn_data = (data_connection_data*)gp;
  GIOStatus status = readbuff(conn_data->chan->channel, conn_data->buff, 30);

  if (status != G_IO_STATUS_NORMAL)
    return status;
  
  g_string_append(conn_data->in, conn_data->buff);
  char* nlptr = strchr(conn_data->in->str, '\n');
  if (nlptr != NULL) {
    /* complete client ID sent */
    guint id;
    sscanf(conn_data->in->str, "%u", &id);
    logit("ID = %u\n", id);
    
    gpointer val = g_hash_table_lookup(connecting_table, &id);
    if (val == NULL) {
      logit("Invalid client ID: %u\n", id);
      char buff[32];
      sprintf(buff, "failed\n");
      gsize size = (gsize)strlen(buff);
      gsize num;
      GIOStatus status = writebuff(conn_data->chan->channel, buff, &num);
      if (status != G_IO_STATUS_NORMAL || num != size) {
        logit("Can't write data connection ack\n");
        exit(1);
      }
      channel_destroy(conn_data->chan);
    }
    else {
      /* client ID matches partially connected client */
      /*  kill the timer and reset the data channel for normal data */
      Connection* conn = (Connection*)(val);
      g_source_remove(conn->timeout_id);
      conn->timeout_id = -1;
      conn->chan = conn_data->chan;
      conn->chan->data = conn;
      conn->chan->send = connection_send;
      conn->chan->recv = connection_recv;
      conn->chan->destroy = connection_close;
      
      char buff[32];
      sprintf(buff, "ok\n");
      gsize size = (gsize)strlen(buff);
      gsize num;
      GIOStatus status = writebuff(conn_data->chan->channel, buff, &num);
      if (status != G_IO_STATUS_NORMAL || num != size) {
        logit("Can't write data connection ack\n");
        exit(1);
      }
      guint key = GPOINTER_TO_UINT(conn);
      g_hash_table_remove(connecting_table, &key);

      guint *connkey = g_new(int, 1);
      *connkey = GPOINTER_TO_UINT(conn);
      g_hash_table_insert(connected_table, connkey, conn);

      logit("Connection for client %u complete\n", connkey);
    }
    g_free(conn_data);
  }
  
  return G_IO_STATUS_NORMAL;
}


gboolean connection_timeout_callback(gpointer gconn)
{
  Connection* conn  = (Connection*)gconn;
  logit("Connection attempt timed out from client %ul\n", (guint64)(gconn));
  GError *err = NULL;
  //g_io_channel_shutdown(conn->ack_channel, TRUE, &err);
  //g_io_channel_unref(conn->ack_channel);
  //g_clear_error(&err);


  connection_close(conn);
  guint key = GPOINTER_TO_UINT(conn);
  g_hash_table_remove(connecting_table, &key);
  return FALSE;
}

gboolean server_ack_prepare(GSource *source, gint *timeout)
{
  *timeout = -1;
  return FALSE;
}

gboolean server_ack_check(GSource *source)
{
  return (server_ack_FD->revents != 0);
}

/*
 * Function for removing connections from connected_table
 */
gboolean remove_connection_entry_test(gpointer key, gpointer value,
				      gpointer data)
{
  Connection* conn = (Connection*)value;
  connection_close(conn);
  return TRUE;
}
/*
 * A client connects on the info socket
 */
gboolean info_fun(GIOChannel* chan, GIOCondition cond, gpointer data)
{
  char newhostname[1024];
  strcpy(newhostname, "127.0.0.1");
  getMachineAddress(newhostname);
  if (strcmp(hostname, newhostname) != 0) {
    logit("Machine name changed - now %s\n", newhostname);
    strcpy(hostname, newhostname);
    g_hash_table_foreach_remove(connected_table,  remove_connection_entry_test,
				NULL);
  }
    

  char info[1024];
  logit("Attempted info connection\n");
  const int infosockfd =
    (int)(accept(server_info_socket, (struct sockaddr *) &addr, &length));
  if (infosockfd < 0)
    {
      close(server_info_socket);
      logit("Can't accept\n");
      exit(1);
    }
  sprintf(info, "%s %d %d\n", hostname, ack_port, data_port);
  send(infosockfd, info, strlen(info), 0);
  close(infosockfd);
  return TRUE;
}

/*
 * A client connects on the ack socket
 */
gboolean ack_fun(GIOChannel* chan, GIOCondition cond, gpointer data)
{
  logit("Attempted ack connection\n");
  const int acksockfd =
    (int)(accept(server_ack_socket, (struct sockaddr *) &addr, &length));
  if (acksockfd < 0)
    {
      close(server_ack_socket);
      logit("Can't accept\n");
      exit(1);
    }
  guint64 ip_address = addr.sin_addr.s_addr;

#ifdef WIN32
  GIOChannel* achan = g_io_channel_win32_new_socket(acksockfd);
#else
  GIOChannel* achan = g_io_channel_unix_new(acksockfd);
#endif
  Connection* conn = connection_new(achan, ntohl(ip_address));

  logit("Ack connection accepted for client %u  fd %d\n", 
        GPOINTER_TO_UINT(conn), acksockfd);

  int flag=1;
  setsockopt(acksockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));

  GError *err = NULL;
  if (g_io_channel_set_encoding(achan, NULL, &err)
      != G_IO_STATUS_NORMAL)
    {
      g_error ("Error while setting encoding: %s\n", err->message);
    }

  char buff[32];
  sprintf(buff, "%u\n",  GPOINTER_TO_UINT(conn));
  gsize size = (gsize)strlen(buff);
  gsize num_written;
  GIOStatus status = writebuff(achan, buff, &num_written);
  if (status != G_IO_STATUS_NORMAL || size != num_written) {
    logit("Can't write client ID\n");
    exit(1);
  }
  guint *key = g_new(int, 1);
  *key = GPOINTER_TO_UINT(conn);
  conn->timeout_id = g_timeout_add(TIMEOUT, connection_timeout_callback, conn);
  g_hash_table_insert(connecting_table, key, conn);

  return TRUE;


}




gboolean server_data_prepare(GSource *source, gint *timeout)
{
  *timeout = -1;
  return FALSE;
}

gboolean server_data_check(GSource *source)
{
  return (server_data_FD->revents != 0);
}

/*
 * A client connects on the data socket
 */
gboolean data_fun(GIOChannel* chan, GIOCondition cond, gpointer data)
{
  logit("Attempted data connection\n");
  const int datasockfd = 
    (int)(accept(server_data_socket, (struct sockaddr *) &addr, &length));
  if (datasockfd < 0)
    { 
      close(server_data_FD->fd);
      logit("Can't accept\n");
      exit(1);
    }
  logit("Data connection accepted fd %d\n", datasockfd);
  
  int flag=1;
  setsockopt(datasockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));
  data_connection_data* dcd = g_new(data_connection_data, 1);
  dcd->in = g_string_new("");
  dcd->chan = channel_new(datasockfd, data_connection_recv, 
                          data_connection_send, data_connection_close, dcd);
  
  return TRUE;
}




static gboolean show_version = FALSE;
static gint server_port = DEFAULTPORT;
gint sizes = BUFFSIZE;
static gchar* logfile = NULL;
static gchar* admin_machine = NULL;
static GOptionEntry entries[] = 
{
  { "version", 0, 0, G_OPTION_ARG_NONE, &show_version, "Show version", NULL },
  { "Port", 'P', 0, G_OPTION_ARG_INT, &server_port, "Use port p for connecting", "p" },
  { "Size", 'S', 0, G_OPTION_ARG_INT, &sizes, "The size for various areas", NULL },
  { "Logfile", 'L', 0, G_OPTION_ARG_FILENAME, &logfile, "Use logfile for logging", "logfile" },
  { "Admin", 'A', 0, G_OPTION_ARG_FILENAME, &admin_machine, "Use admin_machine for receiving all messages", "admin_machine" },
  { NULL }
};

guint64 admin_ip;
Connection* admin_connection;

guint conn_uint_hash(gconstpointer key)
{
  return *(guint*)(key);
}

gboolean conn_uint_equal(gconstpointer a, gconstpointer b)
{
  return *(guint*)a == *(guint*)b;
}

void conn_key_destroy(gpointer key)
{
  g_free(key);
}


void getMachineAddress(char* ip)
{
#ifdef WIN32
  SOCKET sd = WSASocket(AF_INET, SOCK_DGRAM, 0, 0, 0, 0);
  if (sd == SOCKET_ERROR) {
    fprintf(stderr, "Failed to get a socket. Error %d\n",WSAGetLastError()); 
    return;
  }
  
  INTERFACE_INFO InterfaceList[20];
  unsigned long nBytesReturned;
  if (WSAIoctl(sd, SIO_GET_INTERFACE_LIST, 0, 0, &InterfaceList,
               sizeof(InterfaceList), &nBytesReturned, 0, 0) == SOCKET_ERROR) {
    fprintf(stderr, "Failed calling WSAIoctl: error %d\n",WSAGetLastError());
    return;
  }
  
  int nNumInterfaces = nBytesReturned / sizeof(INTERFACE_INFO);
  int i;
  for (i = 0; i < nNumInterfaces; ++i) {
    u_long nFlags = InterfaceList[i].iiFlags;
    if ((nFlags & IFF_UP) && !(nFlags & IFF_LOOPBACK)) {
      struct sockaddr_in *pAddress;
      pAddress = (struct sockaddr_in *) & (InterfaceList[i].iiAddress);
      fprintf(stderr, "%s\n", inet_ntoa(pAddress->sin_addr));
      strcpy(ip, inet_ntoa(pAddress->sin_addr));
      return;
    }                                              
  }    
#else
  struct ifaddrs *ifaddr, *ifa;
  int family, s;
  char host[NI_MAXHOST];
  
  if (getifaddrs(&ifaddr) == -1) {
    perror("getifaddrs");
    exit(EXIT_FAILURE);
  }
  for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == NULL) continue;
    family = ifa->ifa_addr->sa_family;
    if ((family == AF_INET) 
        && (ifa->ifa_flags & IFF_UP) 
        && !(ifa->ifa_flags & IFF_LOOPBACK )) {
      s = getnameinfo(ifa->ifa_addr,
                      (family == AF_INET) ? sizeof(struct sockaddr_in) :
                      sizeof(struct sockaddr_in6),
                      host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      if (s != 0) {
        printf("getnameinfo() failed: %s\n", gai_strerror(s));
        exit(EXIT_FAILURE);
      }
      logit("getMachineAddress: %s\n", host);
      strcpy(ip, host);
      freeifaddrs(ifaddr);
      return;
    }
  }
  freeifaddrs(ifaddr);
#endif
}

int
main(int argc, char** argv)
{
#ifdef WIN32
WSADATA wsaData;
WORD wVersionRequested = MAKEWORD(2,2);
int err = WSAStartup(wVersionRequested, &wsaData);
#endif
  server_ack_socket = -1;
  server_data_socket = -1;
  server_info_socket = -1;
  atexit(going_down);
  GError *error = NULL;

  GOptionContext* context = 
    g_option_context_new ("");
  g_option_context_add_main_entries (context, entries, NULL);
  g_option_context_parse (context, &argc, &argv, &error);

  if (show_version) {
    printf("Pedro %s\n", VERSION);
    exit(0);
  }  

  GMainLoop*  loop = g_main_loop_new(NULL, FALSE);
  connecting_table = g_hash_table_new_full(conn_uint_hash, conn_uint_equal, 
					   conn_key_destroy, NULL);
  connected_table = g_hash_table_new_full(conn_uint_hash, conn_uint_equal, 
					  conn_key_destroy, NULL);

  /* ignore SIGPIPE - otherwise writing to a client that has exited
     causes the server to exit */
  signal(SIGPIPE, SIG_IGN);

  if (admin_machine == NULL) {
    admin_ip = 0;
  } else {
    admin_ip = name2ip(admin_machine);
  }

  admin_connection = NULL;

  if (logfile == NULL) {
#ifdef WIN32    
    logf = fopen("nul", "w");
#else
    logf = fopen("/dev/null", "w");
#endif

    if (logf == NULL) {
      fprintf(stderr, "Can't open log file\n");
      exit(1);
    }
  }
  else if (strcmp(logfile, "stdout") == 0) {
    logf = stderr;
  }
  else {
    logf = fopen(logfile , "w");
    if (logf == NULL) {
      fprintf(stderr, "Can't open log file\n");
      exit(1);
    }
  }
  /* find an IP address for server */
  strcpy(hostname, "127.0.0.1");
  getMachineAddress(hostname);
  logit("Hostname: %s\n", hostname);

  manager = manager_new();

  /* Set up server for client connections 
     server_ack_socket  - socket for initial connection
     server_data_socket - server for connecting for data socket */
  server_ack_socket = socket(AF_INET, SOCK_STREAM, 0);
  server_data_socket = socket(AF_INET, SOCK_STREAM, 0);
  server_info_socket = socket(AF_INET, SOCK_STREAM, 0);

#ifdef WIN32
  ack_chan = g_io_channel_win32_new_socket(server_ack_socket);
  data_chan = g_io_channel_win32_new_socket(server_data_socket);
  info_chan = g_io_channel_win32_new_socket(server_info_socket);
#else
  ack_chan = g_io_channel_unix_new(server_ack_socket);
  data_chan = g_io_channel_unix_new(server_data_socket);
  info_chan = g_io_channel_unix_new(server_info_socket);
#endif
  if (server_ack_socket < 0)
    {
      logit("Can't open server ack socket\n");
      exit(1);
    }
  if (server_data_socket < 0)
    {
      logit("Can't open server data socket\n");
      exit(1);
    }  
  if (server_info_socket < 0)
    {
      logit("Can't open server info socket\n");
      exit(1);
    }
  struct sockaddr_in server_ack;
  struct sockaddr_in server_data;
  struct sockaddr_in server_info;

  server_info.sin_family = AF_INET;
  server_info.sin_addr.s_addr = ((u_long)INADDR_ANY);
  server_info.sin_port = htons((unsigned short)(server_port));

  if (bind(server_info_socket, (struct sockaddr *) &server_info, 
           sizeof(server_info)) != 0)
    { 
      close(server_info_socket);
      logit("Can't bind info socket to port %d\n", server_port);
      exit(1);
    }

  listen(server_info_socket, 5);

  server_ack.sin_family = AF_INET;
  server_ack.sin_addr.s_addr = ((u_long)INADDR_ANY);
  server_ack.sin_port = 0;

  if (bind(server_ack_socket, (struct sockaddr *) &server_ack, 
           sizeof(server_ack)) != 0)
    { 
      close(server_ack_socket);
      logit("Can't bind ack socket\n");
      exit(1);
    }
  memset(&addr, 0, sizeof(addr));
  socklen_t addr_len = sizeof(addr);
  
  getsockname(server_ack_socket, (struct sockaddr *)&addr, &addr_len);
  
  ack_port = ntohs((unsigned short)(addr.sin_port));

  logit("Ack port = %d\n", ack_port);

  listen(server_ack_socket, 5);

  server_data.sin_family = AF_INET;
  server_data.sin_addr.s_addr = ((u_long)INADDR_ANY);
  server_data.sin_port = 0;

  if (bind(server_data_socket, (struct sockaddr *) &server_data, 
           sizeof(server_data)) != 0)
    { 
      close(server_data_socket);
      logit("Can't bind data socket");
      exit(1);
    }
  memset(&addr, 0, sizeof(addr));
  addr_len = sizeof(addr);
  
  getsockname(server_data_socket, (struct sockaddr *)&addr, &addr_len);
  
  data_port = ntohs((unsigned short)(addr.sin_port));

  logit("Data port = %d\n", data_port);

 
  listen(server_data_socket, 5);



  /* Make it a daemon (unless log to stdout) */
  if (logf != stderr) {
#ifndef WIN32
    pid_t ppid = getpid();
    pid_t pid = fork();
    
    if(pid<0) {
      logit("Can't create child process\n");
      exit(1);
    }
    else if (pid!=0)
    {
      ignorepid = ppid;
      exit(0);                    /* terminate parent */
    } else{
      /* We are the child */
      /* Close stdIO */
      fclose(stdin);
      fclose(stdout);
      fclose(stderr);
      
      setsid();
      if (chdir("/") == -1) {
	logit("Can't chdir\n");
	exit(1);
      }

      umask(0);  
    }
#endif
  }
  logit( "server operational on port %d\n", server_port );

  /* set up polling for initial connections on info socket */

  guint info_watch = g_io_add_watch(info_chan, G_IO_IN | G_IO_HUP | G_IO_ERR,
	info_fun, NULL);

  /* set up polling for initial connections on ack socket */

  guint ack_watch = g_io_add_watch(ack_chan, G_IO_IN | G_IO_HUP | G_IO_ERR,
	ack_fun, NULL);

  /* set up polling for initial connection on data socket */

  guint data_watch = g_io_add_watch(data_chan, G_IO_IN | G_IO_HUP | G_IO_ERR,
	data_fun, NULL);
  g_main_loop_run(loop);
  logit("Exit\n");

}
