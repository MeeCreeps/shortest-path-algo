CXX = g++ 
FLAGS = -std=c++17  -L./build/lib -lglog  -lgflags -O3



BUILD = build
BIN = ${BUILD}/bin


ALGO = algo
TOOLS = toolkits
GRAPH = graph
UTILS = utils
THIRD = thirdpart

third_include = -I./${THIRD}/glog/include -I./${THIRD}/gflag/include -I./${THIRD}/gtest -I./${THIRD}/


all: dir ${BIN}/main ${BIN}/generate ${BIN}/benchmark
dir: 
	mkdir -p ${BIN} ${BUILD}/lib

${BIN}/main : ${TOOLS}/main.cpp $(wildcard ${ALGO}/*.h ${UTILS}/*.h ${GRAPH}/*.h ) ${THIRD}/CLI11.hpp
	${CXX} ${FLAGS}  ${third_include}  $^ -o $@


${BIN}/generate : ${TOOLS}/generate.cpp $(wildcard ${UTILS}/*.h ${GRAPH}/*.h ) ${THIRD}/CLI11.hpp
	${CXX} ${FLAGS}  ${third_include}  $^ -o $@

${BIN}/benchmark :  ${TOOLS}/benchmark.cpp  $(wildcard ${ALGO}/*.h ${UTILS}/*.h ${GRAPH}/*.h )  ${THIRD}/CLI11.hpp
	${CXX} ${FLAGS}   ${third_include}  $^ -o $@



clean : 
	rm -rf ${BIN}/*


