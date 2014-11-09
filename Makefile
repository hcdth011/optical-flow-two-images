all:opflow.cpp
	g++ -o opflow opflow.cpp -I/usr/local/include/opencv `pkg-config --libs opencv` 
		
