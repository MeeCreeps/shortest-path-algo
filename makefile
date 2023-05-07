CC = g++ 
FLAGS = -std=c++17
LIBS = -lgflags


BUILD = build
OBJ = ${BUILD}/obj

ALGO = algo
GRAPH = graph 
UTILS = utils




all: ${BUILD}/algo


${BUILD}/algo : ${OBJ}/main.o ${OBJ}/ch.o 
	${CC} ${FLAGS} ${LIBS} ${OBJ}/main.o ${OBJ}/ch.o \
	-o ${BUILD}/algo 

${OBJ}/main.o : ${ALGO}/main.cpp
	${CC} -c ${FLAGS} ${LIBS} ${ALGO}/main.cpp -o ${OBJ}/main.o

${OBJ}/ch.o : ${ALGO}/ch.cpp
	${CC} -c ${FLAGS} ${LIBS} ${ALGO}/main.cpp -o ${OBJ}/ch.o


clean : 
	rm -rf ${BUILD}/obj/* ${BUILD}/algo/*


