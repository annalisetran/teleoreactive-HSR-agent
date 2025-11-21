// Copyright 2021 Keith Clark, Peter Robinson
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "MQTTAsync.h"
#include <unistd.h>

#include "client_api.h"

// MQTT support
MQTTAsync the_client;
int connected = 0;
int subscribed = 0;
int message_sent = 0;
char saved_ip[50];
long saved_port;

// Pedro support
PedroConnection* pedro_client;
char msg[PEDRO_BUFFER_SIZE];
pthread_t tid;


// The Pedro address of the percepts thread of the agent named 'agent'
// running on the same machine
static const char * agent_address = "percepts:agent@localhost";

void connlost(void *context, char *cause)
{
  MQTTAsync client = (MQTTAsync)context;
  MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
  int rc;
  
  printf("\nConnection lost\n");
  printf("     cause: %s\n", cause);
  
  printf("Reconnecting\n");
  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;
  if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
      printf("Failed to start connect, return code %d\n", rc);
      connected = 0;
    }
}

// Here we are assuming another MQTT node publishes the list of percepts 
// and this node simply sends this string as a peer-to-peer message
// to the percepts thread of the agent.
// In a real application this callback could be used for converting
// (or collating) low-level percepts into high-level percepts to
// be sent to the agent.
// If the updates interface is being used it is probably the responsibility of
// this node to remember the current state so that it can correctly send the
// appropriate r_ f_ u_ terms
int msgarrvd(void *context, char *topicName, int topicLen,
             MQTTAsync_message *message)
{
    char msg[PEDRO_BUFFER_SIZE];
    sprintf(msg, "%s", (char*)(message->payload));
    size_t size = strlen(msg);
    // remove trailing newlines if there
    while ((size > 0) && msg[size-1] == '\n') {
      msg[size-1] = '\0';
      size--;
    }
    printf("sending percepts: %s\n", msg);
    // send ROS message as a peer-to-peer message (percepts)
    if (pedro_p2p(pedro_client, agent_address, msg) == 0) {
      fprintf(stderr, "Unable to send message |%s|\n", msg);
      exit(1);
    }
    return 1;
}

void onSubscribe(void* context, MQTTAsync_successData* response)
{
#if !NDEBUG
  printf("onSubscribe");
#endif  
  subscribed = 1;
}

void onSubscribeFailure(void* context, MQTTAsync_failureData* response)
{
#if !NDEBUG
  printf("onSubscribeFailure");
#endif
  subscribed = 0;
}

void onConnectFailure(void* context, MQTTAsync_failureData* response)
{
  printf("Connect failed, rc %d\n", response ? response->code : 0);
  connected = 0;
}


void onConnect(void* context, MQTTAsync_successData* response)
{
#if !NDEBUG
  printf("onConnect");
#endif
  connected = 1;
}

void onSend(void* context, MQTTAsync_successData* response)
{
  message_sent = 1;
}


int mqtt_connect_client(char* ip, long port, char* clientid) {
  char address[100];
  sprintf(address, "tcp://%s:%ld", ip, port);
   MQTTAsync client;
  MQTTAsync_create(&client, address, clientid, 
                   MQTTCLIENT_PERSISTENCE_NONE, NULL);
  the_client = client;
  MQTTAsync_setCallbacks(client, NULL, connlost, msgarrvd, NULL);
  MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
  conn_opts.keepAliveInterval = 600;
  conn_opts.cleansession = 1;
  conn_opts.onSuccess = onConnect;
  conn_opts.onFailure = onConnectFailure;
  int rc;
  connected = 0;
  if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
      return -1;	
    }
  while (connected == 0) {
    usleep(200);
  }
  return 0;
}

int mqtt_notify(char* topic, char* payload, long qos) {
  MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
  MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
  int rc;
  opts.onSuccess = onSend;
  opts.context = the_client;
  pubmsg.payload = payload;
  pubmsg.payloadlen = strlen(payload);
  pubmsg.qos = qos;
  pubmsg.retained = 0;
  if ((rc = MQTTAsync_sendMessage(the_client, topic, &pubmsg, &opts)) != MQTTASYNC_SUCCESS)
    {
      return(-1);	
    }
  while (message_sent == 0){
    usleep(10);
  }
  message_sent = 0;
  printf("finished notify %s %s\n", topic, payload);
  return 0;
}

int mqtt_subscribe(char* topic, long qos) {
  int rc;
  MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
  opts.onSuccess = onSubscribe;
  opts.onFailure = onSubscribeFailure;
  opts.context = the_client;
  printf("subscribing %s\n", topic);
  if ((rc = MQTTAsync_subscribe(the_client, topic, qos, &opts)) != 
      MQTTASYNC_SUCCESS)
    {
      printf("XXX  %d\n", rc);
      return -1;	
    }
  while (subscribed == 0) {
    usleep(20);
  }
  subscribed = 0;
  return 0;
}


void *pedro_loop(void){
  int rock;
  // buffer for parsing messages into Prolog terms
  TermBuffer* termbuff = termbuff_new();

  // buffer for writing terms
  WriteBuffer *wb = writebuff_new();
  while (1) {
    // block until message arrives
    pedro_get_message(pedro_client, msg, &rock);
    // prepare for parsing message
    termbuff_reset(termbuff);
    // parse the message into termbuff
    // msg_term points at the parsed Prolog term
    fprintf(stderr,"msg:%s\n", msg);
    termptr msg_term = pedro_parse(msg, termbuff);
    // in this example we are assuming this is a peer-to-peer message
    // so we need to extract the actual message
    // check the message is indeed a peer-to-peer message and ignore if not
    if ((get_type(msg_term) == PEDRO_STRUCTURE) &&
	(get_arity(msg_term) == 3) &&
	(atom_eq(get_functor(msg_term), "p2pmsg"))) {
      // exit if message is the atom quit
      if (atom_eq(get_arg(msg_term, 3), "initialise_")) {
        // should send complete state (percepts) to the agent
        // for this example we simply send back the empty list of percepts
        // The ROS loop is responsible for managing the state and so
        // a lock is probably needed here in a real application
        if (pedro_p2p(pedro_client, agent_address, "[]") == 0) {
          fprintf(stderr, "Unable to send message\n");
          exit(1);
        }
      } else {
        // the message should be of the form actions(task_name, list_of_actions)
        termptr term = get_arg(msg_term, 3);
        if ((get_type(term) == PEDRO_STRUCTURE) &&
            (get_arity(term) == 2) &&
            (atom_eq(get_functor(term), "actions"))) {
          // for this example we simply extract the elements of the list
          // and publish them
          // In a real application the list elements would be decoded into
          // controls that are perhaps published.
          // if you need the task name then
          // termptr task_name = get_arg(term, 1);
          termptr actions = get_arg(term, 2);
          // we assme here the actions form a list
          while (get_type(actions) == PEDRO_LIST) {
            // turn head element into string for publishing
            writebuff_reset(wb);
            writebuff_addterm(wb, get_head(actions));
            
            mqtt_notify("controls",  writebuff_getbuff(wb), 0);
            
            // process the tail
            actions = get_tail(actions);
          }
        } else {
          fprintf(stderr, "Message is not an action term\n");
          exit(1);
        }
        
      }
    }
    
  }
}

int main()
{
  // Create a connection to pedro
  pedro_client = pedro_default_connect();
  // Register the name "robo" with pedro
  if (pedro_register(pedro_client, "robo") == 0) {
    fprintf(stderr, "Unable to register\n");
    exit(1);
  }

  mqtt_connect_client("localhost", 1883, "example");
  mqtt_subscribe("percepts", 0);

  pedro_loop();

}
