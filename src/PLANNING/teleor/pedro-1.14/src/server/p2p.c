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
typedef int socklen_t;

#else

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#endif

#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include "atoms.h"
#include "manager.h"
#include "p2p.h"

extern Manager* manager;
extern Connection* admin_connection;
extern guint64 admin_ip;

extern void logit(char* fmt, ...);

GString* remove_quotes(char* name)
{
  GString* machine_name = g_string_new(name);
  if (name[0] == '\'') {
    int len = machine_name->len;
    machine_name = g_string_truncate(machine_name, len-1);
    machine_name = g_string_erase(machine_name, 0, 1);
  }
  return machine_name;
}

guint64 name2ip(char* hostname)
{
/*   if (strcmp(hostname, "localhost") == 0)  */
/*     { */
/*       logit("localhost error \n"); */
/*       return 0; */
/*     } */
  struct hostent *hp = gethostbyname(hostname);
#ifdef WIN32
  if (hp == NULL)
    {
      struct in_addr in;
      in.s_addr = inet_addr(hostname);
      hp = gethostbyaddr((char *) &in, sizeof(in), AF_INET);
    }
#endif
  if (hp == NULL)
    {	  
      logit("host error \n");
      return 0;
    }
  return ntohl(*(int*)(hp->h_addr));

}


gboolean valid_p2p_address(ObjectPtr term, gboolean vars_allowed)
{
  if (IS_VAR(term)) 
    return vars_allowed;
  if (GET_TYPE(term) != TYPECOMPOUND) return FALSE;
  if (GET_TAG_INFO(term) != 2) return FALSE;
  
  ObjectPtr name = GET_ARG(term, 1);
  ObjectPtr ip_obj = GET_ARG(term, 2);
  if (GET_TYPE(name) == TYPECOMPOUND) {
    if (GET_ARG(name, 0) != a_colon) return FALSE;
    name = GET_ARG(name, 2);
  }
  if (IS_VAR(name) && IS_VAR(ip_obj))
    return vars_allowed;

  if (IS_VAR(name)) {
    return vars_allowed && IS_ANY_ATOM(ip_obj);
  }
  if (IS_VAR(ip_obj)) {
    return vars_allowed && IS_ANY_ATOM(name);
  }
  return (IS_ANY_ATOM(name) && IS_ANY_ATOM(ip_obj));
}

gboolean valid_p2p_notification(ObjectPtr term)
{
  assert(GET_TYPE(term) == TYPECOMPOUND);
  assert(GET_ARG(term, 0) == a_p2pmsg);
  if (GET_TAG_INFO(term) == 3) {
  return
    (valid_p2p_address(GET_ARG(term, 1), TRUE) &&
     valid_p2p_address(GET_ARG(term, 2), FALSE));
  }
  else
    return FALSE;
}

struct _P2P_address
{
  ObjectPtr name;                /* process name */
  guint ip;                      /* ip address */
};

typedef struct _P2P_address P2P_address;

typedef struct _TableEntry TableEntry;

struct _TableEntry
{
  GSList* lst;
};

struct _P2P
{
  GHashTable* ip_table;     /* table that associates name lists with ip's */
  GHashTable* name_table;   /* table that associates ip lists with names */
  GHashTable* name_ip_table; /* associates connections with (name, ip) pairs */
  GHashTable* ip_cache;     /* cache mapping between machine names and IPs */
};

P2P_address* p2p_address_new(ObjectPtr n, guint ip)
{
  P2P_address* pa = g_new(P2P_address, 1);
  pa->name = n;
  pa->ip = ip;
  return pa;
}

void p2p_address_destroy(gpointer p)
{
  g_free(p);
}

void ip_string_destroy(gpointer p)
{
  g_free(p);
}


guint name_ip_hash(gconstpointer key)
{
  P2P_address* keyval = (P2P_address*)key;
  return ((GPOINTER_TO_UINT(keyval->name) & 0x0000ffff) 
	  | (GPOINTER_TO_UINT(keyval->ip) << 16));
}


gboolean name_ip_eq(gconstpointer a, gconstpointer b)
{
  P2P_address* aval = (P2P_address*)a;
  P2P_address* bval = (P2P_address*)b;
  return ((aval->name == bval->name) && (aval->ip == bval->ip));
}

void delete_table_entry(gpointer e)
{
  g_free(e);
}

P2P* p2p_new()
{
  P2P* p = g_new(P2P, 1);
  p->ip_table = g_hash_table_new_full(NULL, NULL, NULL, delete_table_entry);
  p->name_table = g_hash_table_new_full(NULL, NULL, NULL, delete_table_entry);
  p->name_ip_table = g_hash_table_new_full(name_ip_hash, name_ip_eq, 
					   p2p_address_destroy, NULL);
  p->ip_cache = g_hash_table_new_full(g_str_hash, g_str_equal, 
                                      ip_string_destroy, NULL);
  return p;
}

void p2p_destroy(P2P* p)
{
  g_hash_table_destroy(p->ip_table);
  g_hash_table_destroy(p->name_table);
  g_hash_table_destroy(p->name_ip_table);
  g_hash_table_destroy(p->ip_cache);
  g_free(p);
}

void add_hash_table_list_entry(GHashTable* ht, gpointer key, gpointer val)
{
  TableEntry* entry = g_hash_table_lookup(ht, key);
  if (entry == NULL) {
    entry = g_new(TableEntry, 1);
    entry->lst = NULL;
    entry->lst = g_slist_prepend(entry->lst, val);
    g_hash_table_insert(ht, key, entry);
  }
  else {
    entry->lst = g_slist_prepend(entry->lst, val);
  }
}

gboolean remove_hash_table_list_entry(GHashTable* ht, gpointer key, gpointer val)
{
  TableEntry* entry = g_hash_table_lookup(ht, key);
  gboolean deleted = FALSE;
  assert(entry != NULL);
  GSList* lst;
  for (lst = entry->lst; lst != NULL; lst = g_slist_next(lst)) {
    if (lst->data == val) {
      entry->lst = g_slist_delete_link(entry->lst, lst);
      if (entry->lst == NULL) {
	g_hash_table_remove(ht, key);
	deleted = TRUE;
      }
      return deleted;
    }
  }
  assert(FALSE);
}

gboolean remove_cache_entry_test(gpointer key, gpointer value,
			    gpointer data)
{
  logit("remove %s\n", (gchar*)key);
  return value == data;
}

void remove_cache_entry(GHashTable* ip_cache, guint ip)
{
  logit("remove cache ip: %d.%d.%d.%d \n", 
        (ip>>24)&255, (ip>>16)&255, (ip>>8)&255, ip&255);
  g_hash_table_foreach_remove(ip_cache, remove_cache_entry_test, 
			      GUINT_TO_POINTER(ip));
}

gboolean p2p_register(P2P* p, ObjectPtr name, guint ip, Connection* conn)
{

  if ((ip == admin_ip) && (name == a_admin_name)) {
    if (admin_connection == NULL) {
      admin_connection = conn;
      return TRUE;
    } else {
      return FALSE;
    }
  }

  logit("  register: %s  ip: %d.%d.%d.%d\n", GET_ATOM_NAME(name), 
        (ip>>24)&255, (ip>>16)&255, (ip>>8)&255, ip&255);
  P2P_address* p2p_addr = p2p_address_new(name, ip);
  if (g_hash_table_lookup(p->name_ip_table, p2p_addr) == NULL) {
    g_hash_table_insert(p->name_ip_table, p2p_addr, conn);
    add_hash_table_list_entry(p->name_table, name, GUINT_TO_POINTER(ip));
    add_hash_table_list_entry(p->ip_table, GUINT_TO_POINTER(ip), name);
    return TRUE;
  }
  p2p_address_destroy(p2p_addr);
  return FALSE;
}

gboolean p2p_deregister(P2P* p, ObjectPtr name, guint ip, Connection* conn)
{
  P2P_address* p2p_addr = p2p_address_new(name, ip);
  Connection* lookup_conn 
    = (Connection*)g_hash_table_lookup(p->name_ip_table, p2p_addr);

  if (lookup_conn == NULL) {
    p2p_address_destroy(p2p_addr);
    logit("Deregister: Can't find hash entry\n");
    return FALSE;
  } else if (lookup_conn != conn) {
    p2p_address_destroy(p2p_addr);
    logit("Deregister: Name does not match connection\n");
    return FALSE;
  } else {
    logit("Deregister: remove %s\n", GET_ATOM_NAME(name));
    g_hash_table_remove(p->name_ip_table, p2p_addr);
    remove_hash_table_list_entry(p->name_table, name, GUINT_TO_POINTER(ip));
    if (remove_hash_table_list_entry(p->ip_table, GUINT_TO_POINTER(ip), name)){
      remove_cache_entry(p->ip_cache, ip);
    }
    dec_atom_refcount(manager->atom_table, name);
    return TRUE;
  }
}

guint64 name_to_IP(P2P* p, char* name)
{
  GString* machine_name = remove_quotes(name);
  guint64 ip 
    = (guint64)g_hash_table_lookup(p->ip_cache, machine_name->str);
  if (ip != 0) {
    g_string_free(machine_name, TRUE);
    return ip;
  }
  ip = name2ip(machine_name->str);
  if (ip != 0) {
    logit("cache %s %d.%d.%d.%d\n", machine_name->str, 
          (ip>>24)&255, (ip>>16)&255, (ip>>8)&255, ip&255);
    g_hash_table_insert(p->ip_cache, g_strdup(machine_name->str), 
                        GUINT_TO_POINTER(ip));
    g_string_free(machine_name, TRUE);
  }
  return ip;
}

void p2p_send_to_connection(Connection* conn, Connection* toconn)
{
  if (toconn != NULL) {
    strcpy(conn->notifybuff, conn->parsebuff);
    GIOStatus status = 
      connection_send_notification(toconn, conn, 0);
    if (status != G_IO_STATUS_NORMAL) {
      initDestroy(toconn->chan);
    }
  }
}

void send_p2p_to_name_ip(P2P* p, ObjectPtr name, guint ip, Connection* conn)
{
  P2P_address* p2p_addr = p2p_address_new(name, ip);
  Connection* toconn 
    = (Connection*)g_hash_table_lookup(p->name_ip_table, p2p_addr);
  p2p_address_destroy(p2p_addr);
  p2p_send_to_connection(conn, toconn);
}

void send_p2p_to_all_names(P2P* p, guint ip, Connection* conn)
{
  TableEntry* names 
    = (TableEntry*)g_hash_table_lookup(p->ip_table, GUINT_TO_POINTER(ip));
  if (names == NULL) return;
  GSList* lst;
  for (lst = names->lst; lst != NULL; lst = g_slist_next(lst)) {
    send_p2p_to_name_ip(p, lst->data, ip, conn);
  }
}

void send_p2p_to_all_ips(P2P* p, ObjectPtr name, Connection* conn)
{
  TableEntry* ips 
    = (TableEntry*)g_hash_table_lookup(p->name_table, name);
  if (ips == NULL) return;
  GSList* lst;
  for (lst = ips->lst; lst != NULL; lst = g_slist_next(lst)) {
    send_p2p_to_name_ip(p, name, GPOINTER_TO_UINT(lst->data), conn);
  }
}

void send_to_one(gpointer key, gpointer value, gpointer user_data)
{
  Connection* toconn = (Connection*)value;
  Connection* conn = (Connection*)user_data;
  p2p_send_to_connection(conn, toconn);
}

void send_p2p_to_all(P2P* p, Connection* conn)
{
  g_hash_table_foreach(p->name_ip_table, send_to_one, conn);
}
 
void send_p2p_message(P2P* p, ObjectPtr term, Connection* conn)
{
  if (admin_connection != NULL) {
    p2p_send_to_connection(conn, admin_connection);
  }

  ObjectPtr addr = GET_ARG(term, 1);
  if (IS_VAR(addr)) {
    send_p2p_to_all(p, conn);
    return;
  }
  ObjectPtr name = GET_ARG(addr, 1);
  ObjectPtr ip_obj = GET_ARG(addr, 2);
  if (GET_TYPE(name) == TYPECOMPOUND) {
    /* name is thread:name */
    name = GET_ARG(name, 2);
  }
  if (IS_VAR(name) && IS_VAR(ip_obj)) {
    send_p2p_to_all(p, conn);
    return;
  }
  if (IS_VAR(name)) {
    send_p2p_to_all_names(p, name_to_IP(p, GET_ATOM_NAME(ip_obj)), 
			  conn);
    return;
  }
  if (IS_VAR(ip_obj)) {
    send_p2p_to_all_ips(p, name, conn);
    return;
  }
  guint ip = name_to_IP(p, GET_ATOM_NAME(ip_obj));
  send_p2p_to_name_ip(p, name, ip, conn);
}



gboolean correct_p2p_sender_address(P2P* p, ObjectPtr term, Connection* conn)
{
  ObjectPtr addr = GET_ARG(term, 2);
  ObjectPtr name = GET_ARG(addr, 1);
  ObjectPtr ip_obj = GET_ARG(addr, 2);
  if (GET_TYPE(name) == TYPECOMPOUND) {
    /* name is thread:name */
    name = GET_ARG(name, 2);
  }
  guint ip = name_to_IP(p, GET_ATOM_NAME(ip_obj));
  P2P_address* p2p_addr = p2p_address_new(name, ip);
  Connection* fromconn 
    = (Connection*)g_hash_table_lookup(p->name_ip_table, p2p_addr);
  p2p_address_destroy(p2p_addr);
  return (conn == fromconn);
}

  
