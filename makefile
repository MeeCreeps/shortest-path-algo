CXX = g++ 
FLAGS = -std=c++17 -lglog -O3



BUILD = build
BIN = ${BUILD}/bin


ALGO = algo
TOOLS = toolkits
GRAPH = graph 
UTILS = utils
THIRD = thirdpart

third_include = -I./${THIRD}/glog/include -I./${THIRD}/gtest -I./${THIRD}/


all: dir ${BIN}/main ${BIN}/generate ${BIN}/benchmark
dir: mkdir -p ${BIN}

${BIN}/main : ${TOOLS}/main.cpp ${ALGO}/*.h ${UTILS}/*.h ${GRAPH}/*.h  ${THIRD}/CLI11.hpp
	${CXX} ${FLAGS}  ${third_include}  $^ -o $@


${BIN}/generate : ${TOOLS}/generate.cpp  ${UTILS}/*.h ${GRAPH}/*.h  ${THIRD}/CLI11.hpp
	${CXX} ${FLAGS}  ${third_include}  $^ -o $@

${BIN}/benchmark :  ${TOOLS}/benchmark.cpp ${ALGO}/*.h ${UTILS}/*.h ${GRAPH}/*.h  ${THIRD}/CLI11.hpp
	${CXX} ${FLAGS}   ${third_include}  $^ -o $@



clean : 
	rm -rf ${BIN}/*


