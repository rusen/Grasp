COMMON=-o -O0 -g3 -I./include -I./include/controller -I./include/planner -L./bin -std=c++11 -lstdc++ -mavx

all:
	g++ $(COMMON) ./sample/basicGrasp.cpp ./sample/controller/*.cpp ./sample/planner/*.cpp    -lmujoco150 -lglfw.3 -o ./bin/basicGrasp
#	g++ $(COMMON) ./sample/simulate.cpp   -lmujoco150 -lglfw.3 -o ./bin/simulate
#	g++ $(COMMON) ./sample/graspSimulate.cpp   -lmujoco150 -lglfw.3 -o ./bin/graspSimulate
#	g++ $(COMMON) ./sample/grasp.cpp      -lmujoco150 -lglfw.3 -o ./bin/grasp
#	g++ $(COMMON) ./sample/test.cpp       -lmujoco150          -o ./bin/test
#	g++ $(COMMON) ./sample/compile.cpp    -lmujoco150          -o ./bin/compile
#	g++ $(COMMON) ./sample/derivative.cpp -lmujoco150          -o ./bin/derivative
#	g++ $(COMMON) ./sample/record.cpp     -lmujoco150 -lglfw.3 -o ./bin/record
