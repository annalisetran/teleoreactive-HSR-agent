
MQTT_SOURCES = \
	mqtt_pedro.c \
	client_api.c \
	pedro_token.c \
	MQTTAsync.c \
	StackTrace.c \
	Log.c \
	Messages.c \
	Thread.c \
	LinkedList.c \
	Heap.c \
	Tree.c \
	MQTTProtocolClient.c \
	Socket.c \
	SocketBuffer.c \
	MQTTPacket.c \
	MQTTTime.c \
	MQTTPersistence.c \
	MQTTPersistenceDefault.c \
	MQTTPacketOut.c \
	MQTTProperties.c \
	Clients.c \
	utf-8.c \
	SHA1.c \
	Base64.c \
	MQTTProtocolOut.c \
	SSLSocket.c \
	WebSocket.c

MQTT_BASES 	= $(basename $(MQTT_SOURCES))
MQTT_OBJECTS 	=  $(addsuffix .o,$(MQTT_BASES))

.PHONY:	all
all: mqtt_pedro

mqtt_pedro:  $(MQTT_OBJECTS)
	gcc -DNDEBUG  -o mqtt_pedro $(MQTT_OBJECTS) -lpthread




.c.o:
	gcc -DNDEBUG -Wall -fPIC -D_GNU_SOURCE=1 -D_THREAD_SAFE=1 -D_REENTRANT=1 -Wno-uninitialized -c -g  $< 



