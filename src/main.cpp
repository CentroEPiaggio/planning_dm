#include <iostream>
#include <graph_creator.h>

int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
    lemon::SmartDigraph graph;
    graphCreator temp(graph);
    Object a;
    temp.create_graph(a);
    
    return 0;
}
