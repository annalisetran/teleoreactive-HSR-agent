
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
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "connection.h"

#include "manager.h"
#include "atoms.h"

extern int sizes;
extern void logit(char* fmt, ...);
extern void freeze_vars(ObjectPtr optr);

// Improved logging thanks to Duncan White, Imperial College
static void logip( char *msg, guint ip )
{
  logit("%s %d.%d.%d.%d\n", msg, (ip>>24)&255, (ip>>16)&255, (ip>>8)&255, ip&255 );
}

extern Manager* manager;
extern GHashTable* connected_table;

gboolean connection_close(gpointer gconn);

void connection_destroy(gpointer gconn)
{
  Connection* conn = (Connection*)gconn;
  if (conn->chan != NULL) close(conn->chan->fd);
  g_free(gconn);
}

/* inc_refcount and dec_refcount manages the number of references
 * to the connection
 */
void inc_refcount(Connection* conn)
{
  assert(conn != 0);
  conn->ref_count++;
}

void dec_refcount(Connection* conn)
{
  assert(conn != 0);
  assert(conn->ref_count > 0);
  conn->ref_count--;
  if (conn->ref_count == 0) {
    logit("Destroying connection from client %u\n", GPOINTER_TO_UINT(conn));
    connection_destroy(conn);
  }
}


/*
 * inc_w_refcount and dec_w_refcount manages the number of clients who are
 * blocked and so can't get my notification. I can read iff there are no 
 * clients who can't get my notification yet
 */

void inc_w_refcount(Connection* conn)
{
  assert(conn != 0);
  if (conn->w_ref_count == 0) {
    unsetInPoll(conn->chan);
  }
  conn->w_ref_count++;
}

void dec_w_refcount(Connection* conn)
{
  assert(conn != 0);
  assert(conn->w_ref_count > 0);
  assert(!IN_POLLING(conn->chan));
  conn->w_ref_count--;
  if (conn->w_ref_count == 0) {
    setInPoll(conn->chan);
  }
}

DelayNotify* delay_notify_new(guint rock, Connection* notify_conn)
{
  logit("notify_new\n");
  DelayNotify* delay =  g_new(DelayNotify, 1);
  delay->rock = rock;
  delay->notify_conn = notify_conn;
  inc_refcount(notify_conn);
  return delay;
}

void delay_notify_destroy(DelayNotify* delay)
{
  logit("notify_destroy\n");
  dec_refcount(delay->notify_conn);
  g_free(delay);
}

/*
 * Set up my numberbuff with a number and optional nl ready for writing
 */

void connection_put_number(Connection* conn, guint num)
{
  sprintf(conn->numberbuff, "%u ", num);
  conn->number_offset = 0;
  conn->number_len = strlen(conn->numberbuff);
}

void connection_send_ack(Connection* conn, int ack)
{
  char buff[32];
  sprintf(buff, "%d\n", ack);
  gsize size = (gsize)strlen(buff);
  gsize num_written;
  GIOStatus status = writebuff(conn->ack_channel, buff, &num_written);
  if (num_written != size || status != G_IO_STATUS_NORMAL) {
    logit("Can't send ack %s \n", strerror(errno));
    connection_close(conn);
  }
}

GIOStatus connection_send(gpointer gconn);


gboolean timeout_callback(gpointer gconn)
{
  connection_close((Connection*)gconn);
  return FALSE;
}

/*
 * There is something to read - process the input buffer
 */
void processBuff(Connection* conn)
{
  /* attach the new chars in inbuff to the unprocessed chars in in */
  /* and search for terminating newline */
  g_string_append(conn->in, conn->inbuff);
  char* nlptr = strchr(conn->in->str, '\n');
  if (nlptr == NULL) {
    if (conn->in->len > (sizes - 2)) {
      logit("Too long 1: %s\n", conn->in->str);
      /* newline not found but max size exceeded: start throwing away chars */
      if (!conn->truncating) {
	connection_send_ack(conn, 0);
      }
      g_string_truncate(conn->in, 0);
      conn->truncating = TRUE;
      if (conn->w_ref_count == 0) setInPoll(conn->chan);
    }
    return;
  }
  else {
    if (conn->truncating) {
      logit("Truncating long msg: %s\n", conn->in->str);
      int pos = nlptr - conn->in->str;
      g_string_erase(conn->in, 0, pos+1);
      conn->truncating = FALSE;
      nlptr = strchr(conn->in->str, '\n');
      if (nlptr == NULL) {
        if (conn->w_ref_count == 0) setInPoll(conn->chan);
        return;
      }
    }
  }

  while ((conn->w_ref_count == 0) && (nlptr != NULL)) {
    /* we have reached a newline */
    int len = nlptr - conn->in->str + 1;
    unsetInPoll(conn->chan);
    if (len > (sizes-1)) {
      logit("Too long 2: %s\n", conn->in->str);
      connection_send_ack(conn, 0);
      g_string_erase(conn->in, 0, len);
      /* see if another message follows too long message */
      nlptr = strchr(conn->in->str, '\n');
      if (nlptr == NULL) {
        if (conn->w_ref_count == 0) setInPoll(conn->chan);
        return;
      }
      len = nlptr - conn->in->str + 1;
    }
    strncpy(conn->parsebuff, conn->in->str, len);
    g_string_erase(conn->in, 0, len);
    conn->notifybuff_size = len;
    conn->parsebuff[len] = '\0';

    /*logit("msg :%s:\n", conn->parsebuff);*/
    ObjectPtr term = buff2term(manager, conn->parsebuff, len);

    nlptr = strchr(conn->in->str, '\n');
    
    if ((term == NULL) || 
	!((GET_TYPE(term) == TYPECOMPOUND) || (IS_ANY_ATOM(term)))) {
      logit("term NULL %s\n", conn->parsebuff);
      dec_all_atom_ref(manager->atom_table);
      connection_send_ack(conn, 0);
      if (conn->w_ref_count == 0) setInPoll(conn->chan);
      return;
    }
    gboolean sent_ack = FALSE;
    if (GET_ARG(term, 0) == a_subscribe) {
      if (GET_TAG_INFO(term) != 3) {
	dec_all_atom_ref(manager->atom_table);
	connection_send_ack(conn, 0);
	if (conn->w_ref_count == 0) setInPoll(conn->chan);
	return;
      }
      int id = conn->next_id++;
      Subscription* sub = makeSubscription(manager, conn, id, term);
      if (sub == NULL) {
	dec_all_atom_ref(manager->atom_table);
	connection_send_ack(conn, 0);
	if (conn->w_ref_count == 0) setInPoll(conn->chan);
	logit("subscribe failed %s\n", conn->parsebuff);
	return;
      }
      conn->subscriptions = g_slist_prepend(conn->subscriptions, sub);
      connection_send_ack(conn, id);
      sent_ack = TRUE;
      logit("subscribe\n");
      if (conn->w_ref_count == 0) setInPoll(conn->chan);
      return;
    }
    else if (GET_ARG(term, 0) == a_unsubscribe) {
      if (GET_TAG_INFO(term) != 1) {
	connection_send_ack(conn, 0);
	if (conn->w_ref_count == 0) setInPoll(conn->chan);
	return;
      }
      ObjectPtr subid = GET_ARG(term, 1);
      guint id = (guint)GET_INTEGER(subid);
      if (!IS_INTEGER(subid) || 
	  !connection_remove_subscription(conn, id)) {
	assert(conn->w_ref_count == 0);
	setInPoll(conn->chan);
	connection_send_ack(conn, 0);
	logit("unsubscribe failed %s\n", conn->parsebuff);
	return;
      }
      sent_ack = TRUE;
      logit("unsubscribe\n");
      connection_send_ack(conn, id);
      if (conn->w_ref_count == 0) setInPoll(conn->chan);
      return;
    }
    else if (GET_ARG(term, 0) == a_register) {
      if ((conn->name != NULL) || (GET_TAG_INFO(term) != 1) 
	  || !IS_ATOM(GET_ARG(term, 1))) {
	logit("register failed\n");
	connection_send_ack(conn, 0);
	if (conn->w_ref_count == 0) setInPoll(conn->chan);
	return;
      }
      ObjectPtr name = GET_ARG(term, 1);
      if (!p2p_register(manager->p2p, name, conn->ip, conn)) {
	logit("register failed\n");
	connection_send_ack(conn, 0);
	if (conn->w_ref_count == 0) setInPoll(conn->chan);
	return;
      }
      connection_send_ack(conn, 1);
      sent_ack = TRUE;
      conn->name = name;
      logit("register\n");
      if (conn->w_ref_count == 0) setInPoll(conn->chan);
      return;
    }
    else if (GET_ARG(term, 0) == a_deregister) {
      if ((conn->name == NULL) || (GET_TAG_INFO(term) != 1) 
	  || !IS_ATOM(GET_ARG(term, 1))) {
	connection_send_ack(conn, 0);
	if (conn->w_ref_count == 0) setInPoll(conn->chan);
	return;
      }
      ObjectPtr name = GET_ARG(term, 1);
      if (!p2p_deregister(manager->p2p, name, conn->ip, conn)) {
	connection_send_ack(conn, 0);
	if (conn->w_ref_count == 0) setInPoll(conn->chan);
	return;
      }
      connection_send_ack(conn, 1);
      logit("deregister\n");
      conn->name = NULL;
      if (conn->w_ref_count == 0) setInPoll(conn->chan);
      return;
    }
    else if (GET_ARG(term, 0) == a_p2pmsg) {
      if (!valid_p2p_notification(term) || 
          !correct_p2p_sender_address(manager->p2p, term, conn)) {
	logit("Invalid p2p notification : %s\n", conn->parsebuff);
	connection_send_ack(conn, 0);
	if (conn->w_ref_count == 0) setInPoll(conn->chan);
	return;
      }
      //      ObjectPtr name = GET_ARG(term, 1);
      connection_send_ack(conn, 1);
      send_p2p_message(manager->p2p, term, conn);
      if (conn->w_ref_count == 0) setInPoll(conn->chan);
      continue;
    }
    freeze_vars(term);
    strcpy(conn->notifybuff, conn->parsebuff);
    subscription_table_match(manager->sub_table, term, conn, manager->heap, 
			     manager->trail, manager->cpstack, manager->stack);

    if (!sent_ack) connection_send_ack(conn, 1);
    if (conn->w_ref_count == 0) {
      setInPoll(conn->chan);
    }
  }
}



GIOStatus connection_recv(gpointer gconn)
{
  Connection* conn = (Connection*)gconn;
  GIOStatus status = readbuff(conn->chan->channel, conn->inbuff, sizes-2);

  if (status != G_IO_STATUS_NORMAL)
    return status;

  processBuff(conn);
  return G_IO_STATUS_NORMAL;
}

GIOStatus send_delays(Connection* conn)
{
  assert(conn != 0);
  if (g_queue_is_empty(conn->delays))
    {
      if (conn->has_incref) {
	conn->has_incref = FALSE;
	dec_w_refcount(conn->notify_conn);
      }
      return G_IO_STATUS_NORMAL;
    }
  else {
    dec_w_refcount(conn->notify_conn);
    logit("wake delay %x\n", conn->notify_conn);
    DelayNotify* delay = (DelayNotify*)g_queue_pop_head(conn->delays);
    guint rock = delay->rock;
    Connection*  notify_conn =  delay->notify_conn;
    delay_notify_destroy(delay);
    connection_put_number(conn, rock);
    conn->notify_conn = notify_conn;
    return connection_send(conn);
  }
}

/*
 * Send info on connection
 */
GIOStatus connection_send(gpointer gconn)
{
  assert(gconn != 0);
  Connection* conn = (Connection*)gconn;
  if (conn->timeout_id != (guint)-1) {
    g_source_remove(conn->timeout_id);
    conn->timeout_id = (guint)-1;
  }
  unsetOutPoll(conn->chan);
  GIOStatus status;
  gsize num_written;
  
  status = writebuff(conn->chan->channel, conn->numberbuff + conn->number_offset, 
		     &num_written);
  if (status != G_IO_STATUS_NORMAL) {
    setOutPoll(conn->chan);
    return status;
  }
  conn->number_offset += num_written;
  if (conn->number_offset < conn->number_len) {
    /* If not all of number written then set timeout and
     * notifier should block */
    setOutPoll(conn->chan);
    conn->timeout_id = g_timeout_add(TIMEOUT, timeout_callback, conn);
    if (!conn->has_incref) {
      conn->has_incref = TRUE;
      (void)inc_w_refcount(conn->notify_conn);
    }
    return G_IO_STATUS_NORMAL;
  }
  
  assert(conn->notify_conn != NULL);
  gsize size = conn->notify_conn->notifybuff_size;
  assert(size-1 > conn->notify_offset);
  
  status = writebuff(conn->chan->channel, 
		     conn->notify_conn->notifybuff + conn->notify_offset, 
		     &num_written);
  if (status != G_IO_STATUS_NORMAL) {
    return status;
  }
  
  conn->notify_offset += num_written;
  if (size-1 > conn->notify_offset) {
    /* If not all of notification written then set timeout and
     * notifier should block */
    setOutPoll(conn->chan);
    conn->timeout_id = g_timeout_add(TIMEOUT, timeout_callback, conn);
    if (!conn->has_incref) {
      conn->has_incref = TRUE;
      (void)inc_w_refcount(conn->notify_conn);
    }
    return G_IO_STATUS_NORMAL;
  }
  
  conn->notify_offset = 0;
  return send_delays(conn);
}

/* 
 * When the connection is closed timeout and subscriptions must be
 * removed and delays processed 
 */
gboolean connection_close(gpointer gconn)
{
  Connection* conn = (Connection*)gconn;
  logit("Closing connection from client %u\n", GPOINTER_TO_UINT(conn));
  if (conn->timeout_id != (guint)-1) {
    g_source_remove(conn->timeout_id);
    conn->timeout_id = (guint)-1;
  }
  connection_remove_all_subscription(conn);

  if (conn->has_incref) {
    dec_w_refcount(conn->notify_conn);
  }
  GQueue* delays = conn->delays;
  while (!g_queue_is_empty(delays)) {
    DelayNotify* delay = (DelayNotify*)g_queue_pop_head(conn->delays);
    dec_w_refcount(delay->notify_conn);
    delay_notify_destroy(delay);
  }
  if (conn->name != NULL) {
    p2p_deregister(manager->p2p, conn->name, conn->ip, conn);
  }
  GError *err = NULL;
  g_io_channel_shutdown(conn->ack_channel, TRUE, &err);
  close(g_io_channel_unix_get_fd(conn->ack_channel));
  g_io_channel_unref(conn->ack_channel);
  g_clear_error(&err);

  g_queue_free(delays);
  if (conn->chan != NULL) {
    guint key = GPOINTER_TO_UINT(conn);
    logit("Remove %u from connection table\n", key);
    if (!g_hash_table_remove(connected_table, &key))
      logit("Error in removing from connection table\n");
    channel_destroy(conn->chan);
  }
  g_string_free(conn->in, TRUE);
  g_free(conn->inbuff);
  g_free(conn->parsebuff);
  g_free(conn->notifybuff);
  dec_refcount(conn);
  return FALSE;
}


Connection* connection_new(GIOChannel* chan, guint ip)
{
  Connection* conn =  g_new(Connection, 1);
  conn->name = NULL;
  conn->ip = ip;
  conn->inbuff = g_malloc(sizes);
  conn->parsebuff = g_malloc(sizes);
  conn->notifybuff = g_malloc(sizes);
  conn->in = g_string_new("");
  conn->subscriptions = NULL;
  conn->next_id = 1;
  conn->ref_count = 1;
  conn->w_ref_count = 0;
  conn->ack_channel = chan;
  conn->chan = NULL;
  conn->delays = g_queue_new();
  conn->notifybuff_size = 0;
  conn->notify_offset = 0;
  conn->number_offset = 0;
  conn->number_len = 0;
  conn->notify_conn = NULL;
  conn->timeout_id = (guint)-1;
  conn->has_incref = FALSE;
  conn->truncating = FALSE;

  logip( "New connection from", ip );

  return conn;
}

void connection_add_data_channel(Connection* conn, int f)
{
  conn->chan = channel_new(f, connection_recv, connection_send, 
			   connection_close, conn);
}

  
GIOStatus connection_send_notification(Connection* conn, 
				       Connection* notify_conn, 
				       guint rock)
{
  if (conn->has_incref) {
    inc_w_refcount(notify_conn);
    logit("make delay %x\n", notify_conn);
    DelayNotify* delay = delay_notify_new(rock, notify_conn);
    g_queue_push_head(conn->delays, delay);
    return G_IO_STATUS_NORMAL;
  } else {
    connection_put_number(conn, rock);
    conn->notify_conn = notify_conn;
    return connection_send(conn);
  }
}



/*
 * Remove subscription with id from connection - this must remove
 * sub from connection and from hash table
 */
gboolean 
connection_remove_subscription(Connection* conn, guint id)
{
  GSList* lst;
  for (lst = conn->subscriptions; lst != NULL; lst = g_slist_next(lst)) {
    Subscription* sub = (Subscription*)(lst->data);
    if (sub->id == id) {
      conn->subscriptions = g_slist_delete_link(conn->subscriptions, lst);
#ifndef NDEBUG
      Subscription* debug_sub =
#endif
	subscription_table_remove(manager->sub_table, conn,  
				  SUB_GET_HEAD(sub), SUB_GET_ID(sub));
      assert(sub == debug_sub);
      subscription_destroy(sub);
      return TRUE;
    }
  }
  return FALSE;
}

/*
 * connection is about to be destroyed - remove all subscriptions from
 * connection and hash table
 */
void 
connection_remove_all_subscription(Connection* conn)
{
  GSList* lst;
  for (lst = conn->subscriptions; lst != NULL; lst = g_slist_next(lst)) {
    Subscription* sub = (Subscription*)(lst->data);
#ifndef NDEBUG
    Subscription* debug_sub =
#endif
      subscription_table_remove(manager->sub_table, conn,  
				SUB_GET_HEAD(sub), SUB_GET_ID(sub));
    assert(sub == debug_sub);
    subscription_destroy(sub);
  }
  g_slist_free(conn->subscriptions);
}






