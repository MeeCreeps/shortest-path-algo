#include "graph.h"


void graph::init_from_file(std::string file_name){
    std::ifstream fs(file_name);
    if(!fs){
        LOG(INFO)<<"file is not exist";
        exit(-1);
    }
    fs>>v_size_>>e_size_;

    neighbors_.resize(v_size_);
    
    vid_t src,dst,weight;
    for(vid_t i=0;i<e_size_;++i){
        fs>>src>>dst>>weight;   
        neighbors_[src].push_back({dst,weight});
        neighbors_[dst].push_back({src,weight});
    }



}

void graph::remove_edge(vid_t u,vid_t v){

}
