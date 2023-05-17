GRAPH_PREFIX=./data/
EXP=./exp/

QUERY_PREFIX=""
QUERY=""
GRAPH=""
ORDER=""
INDEX=""
COMMAND=""
EXEC_MAIN="./build/bin/main"
EXEC_GENERATE="./build/bin/main"
EXEC_BENCHMARK="./build/bin/benchmark"

function create_structure(){
    INDEX=$EXP/$2/$3/index.txt
    ORDER=$EXP/$2/$3/order.txt
    GRAPH="$GRAPH_PREFIX"+"$3"+".txt"
    QUERY_PREFIX=$EXP/query/$3

    mkdir -p ${GRAPH_PREFIX} ${QUERY_PREFIX} ${EXP}/$2/$3
}

function process(){

    if [ "$#" -ne 3 ]; then
        echo "Invalid number of parameters. Expected 3 for process."
        exit 1
    fi

    case $2 in 
        "ch")
            COMMAND="${EXEC_MAIN} -i ${INDEX} -a 0 -o 0 -g ${GRAPH} --or ${ORDER}"
            ;; 
        "phl")
            COMMAND="${EXEC_MAIN} -i ${INDEX} -a 1 -o 0 -g ${GRAPH}"
            ;; 
        "h2h")
            COMMAND="${EXEC_MAIN} -i ${INDEX} -a 2 -o 0 -g ${GRAPH}  --or ${ORDER}"
            ;;
    esac
    echo "command : ${COMMAND}"
    ${COMMAND}
    echo "finish building index"
}

function query(){
    # query ch usa 1 10000
    # 1 indicates the query type , 10000 indicates the query size

    if [ "$#" -ne 5 ]; then
        echo "Invalid number of parameters. Expected 5 for query."
        exit 1
    fi

    QUERY=${QUERY_PREFIX}/$4+"_"+$5+".txt"

    if ! [ -f "$QUERY" ]; then
        generate $4 $5 $6
    fi

    #TODO : add query type
    case $2 in 
        "ch")
            COMMAND="${EXEC_MAIN} -i ${INDEX} -a 0 -o 0 -g ${GRAPH}  --or ${ORDER} -q ${QUERY}"
            ;; 
        "phl")
            COMMAND="${EXEC_MAIN} -i ${INDEX} -a 1 -o 0 -g ${GRAPH} -q ${QUERY}"
            ;; 
        "h2h")
            COMMAND="${EXEC_MAIN} -i ${INDEX} -a 2 -o 0 -g ${GRAPH}  --or ${ORDER} -q ${QUERY}"
            ;;
    esac
    echo "command : ${COMMAND}"
    ${COMMAND}
    echo "query finish"
}

function generate(){

    if [ "$#" -ne 3 ]; then
        echo "Invalid number of parameters. Expected 3 for generate."
        exit 1
    fi

    mkdir -p ${QUERY_PREFIX}

    GRAPH="$GRAPH_PREFIX"+"$1"+".txt"
    QUERY_PREFIX=$EXP/query/$1
    QUERY=${QUERY_PREFIX}/$2+"_"+$3+".txt"

    COMMAND="${EXEC_GENERATE} -g ${GRAPH} -q ${QUERY}  -s $3"

    echo "command : ${COMMAND}"
    ${COMMAND}
}

if [ x$1 != x ]; then
    if [ $1 = 'process' ]; then
      create_structure "$@"
      # process ch usa 
      process "$@"; exit 0
    elif [ $1 = 'query' ]; then
      create_structure "$@"
      # query ch usa 1 10000
      query  "$@"; exit 0
    elif [ $1 = 'generate' ]; then
      #generate [graph] [query type] [size] 
      generate $2 $3 $4; exit 0   
    else
      show_help; exit 1
    fi
fi
