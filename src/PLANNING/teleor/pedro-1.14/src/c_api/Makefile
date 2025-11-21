
export SOURCES = \
	pedro_token.c \
	client_api.c 

export BASES = $(basename $(SOURCES))
export OBJECTS = $(addsuffix .o, $(BASES))

.PHONY: all

all: producer consumer ping pong

producer: producer.o $(OBJECTS)
	g++ -o producer producer.o $(OBJECTS)

consumer: consumer.o $(OBJECTS)
	g++ -o consumer consumer.o $(OBJECTS)

ping: ping.o $(OBJECTS)
	g++ -o ping ping.o $(OBJECTS)

pong: pong.o $(OBJECTS)
	g++ -o pong pong.o $(OBJECTS)


producer.o: producer.c
	g++ -c -o producer.o producer.c

consumer.o: consumer.c
	g++ -c -o consumer.o consumer.c

ping.o: ping.c
	g++ -c -o ping.o ping.c

pong.o: pong.c
	g++ -c -o pong.o pong.c

client_api.o : client_api.c
	g++ -c -o client_api.o client_api.c 

pedro_token.o : pedro_token.c
	g++ -c -o pedro_token.o pedro_token.c 

pedro_token.c: pedro_token.l
	flex -opedro_token.c pedro_token.l
