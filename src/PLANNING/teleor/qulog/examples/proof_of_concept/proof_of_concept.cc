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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include "client_api.h"
ros::NodeHandle* node_ptr;
ros::Rate* rate;
ros::Publisher controls;
ros::Subscriber percepts;

PedroConnection* pedro_client;
char msg[PEDRO_BUFFER_SIZE];
pthread_t tid;

// buffer for parsing messages into Prolog terms
TermBuffer* termbuff = termbuff_new();
// buffer for writing terms
WriteBuffer *wb = writebuff_new();


// The Pedro address of the percepts thread of the agent named 'agent'
// running on the same machine
static const char * agent_address = "percepts:agent@localhost";

// Here we are assuming another ROS node publishes the list of percepts 
// and this node simply sends this string as a peer-to-peer message
// to the percepts thread of the agent.
// In a real application this callback could be used for converting
// (or collating) low-level percepts into high-level percepts to
// be sent to the agent.
// If the updates interface is being used it is probably the responsibility of
// this node to remember the current state so that it can correctly send the
// appropriate r_ f_ u_ terms
void sub_callback(const std_msgs::String::ConstPtr& msg){
#if !NDEBUG
    ROS_INFO("I heard: [%s]", msg->data.c_str());
#endif
    char s[PEDRO_BUFFER_SIZE];
    strcpy(s, msg->data.c_str());
    size_t size = strlen(s);
    // remove trailing newline if there
    if ((size > 0) && s[size-1] == '\n') {
      s[size-1] = '\0';
    }
    printf("sending percepts: %s\n", s);
    // send ROS message as a peer-to-peer message (percepts)
    if (pedro_p2p(pedro_client, agent_address, s) == 0) {
      fprintf(stderr, "Unable to send message\n");
      exit(1);
    }
  };

void *pedro_loop(void *args){
  int rock;
  int msg_size;
  while (1) {
    // block until message arrives
    msg_size =  pedro_get_message(pedro_client, msg, &rock);
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
            std_msgs::String str;
            str.data = writebuff_getbuff(wb);
            // publish message to ROS 
            controls.publish(str);
            
            // process the tail
            actions = get_tail(actions);
          }
        } else {
          fprintf(stderr, "Message is not a list\n");
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
  // Initialize ROS

  int argc = 0;
  ros::init(argc, NULL, "ros_pedro");
  node_ptr = new ros::NodeHandle;
  rate = new ros::Rate(10);
  controls = node_ptr->advertise<std_msgs::String>("controls",10);
  percepts = node_ptr->subscribe("percepts", 10, sub_callback);

  // Pedro messages being sent to this node are processed in this thread
  pthread_create(&tid, NULL, pedro_loop, NULL);

  ros::spin();
  

}
