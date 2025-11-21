.PHONY:	all
all: follower

follower: follower_foreign.o follower_interface.o

follower_foreign.o: follower_foreign.cpp
	g++ -DNDEBUG  -c follower_foreign.cpp -I/home/rsa/research/qp/src -I/opt/ros/foxy/include -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib -fPIC

follower_interface.o: follower_interface.cpp
	qcc -DNDEBUG  follower_interface.cpp -I/opt/ros/foxy/include  -I/home/rsa/research/qp/src -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lboost_system -lboost_thread -pthread -lactionlib -fPIC

clean:
	rm follower_foreign.o follower_interface.o
