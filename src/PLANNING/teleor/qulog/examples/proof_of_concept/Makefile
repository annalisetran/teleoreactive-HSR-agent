.PHONY:	all
all: ros_pedro

ros_pedro: ros_pedro.o client_api.o pedro_token.o
	g++ -DNDEBUG  -o ros_pedro  ros_pedro.o client_api.o pedro_token.o -L/opt/ros/noetic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib -fPIC



client_api.o : client_api.c
	g++ -c -o client_api.o client_api.c 

pedro_token.o : pedro_token.c
	g++ -c -o pedro_token.o pedro_token.c 



ros_pedro.o: ros_pedro.cc 
	g++ -DNDEBUG  -c ros_pedro.cc -I/opt/ros/noetic/include -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib -fPIC
