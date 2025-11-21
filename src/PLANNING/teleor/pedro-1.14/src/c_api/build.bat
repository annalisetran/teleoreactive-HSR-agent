g++ -c client_api.c
g++ -c pedro_token.c
g++ -c producer.c
g++ -c consumer.c
g++ -c ping.c
g++ -c pong.c

g++ -o producer.exe producer.o client_api.o pedro_token.o -lws2_32
g++ -o consumer.exe consumer.o client_api.o pedro_token.o -lws2_32
g++ -o ping.exe ping.o client_api.o pedro_token.o -lws2_32
g++ -o pong.exe pong.o client_api.o pedro_token.o -lws2_32

