CXX = g++ 
FLAGS = -std=c++17 -lgflags -O3



BUILD = build
BIN = ${BUILD}/bin


ALGO = algo
GRAPH = graph 
UTILS = utils
THIRD = thirdpart

third_include = -I./${THIRD}/glog/include -I./${THIRD}/gtest -I./${THIRD}/


all: dir main
dir: mkdir -p ${BIN}

${BIN}/main : ${ALGO}/main.cpp ${ALGO}/*.h ${UTILS}/*.h ${GRAPH}/*.h  ${THIRD}/CLI11.hpp
	${CXX} ${FLAGS}  -I${ALGO} ${third_include}  $^ -o $@


clean : 
	rm -rf ${BIN}/*


