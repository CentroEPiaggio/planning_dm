#ifndef GRAPH_H
#define GRAPH_H

#include "arc.h"
#include "node.h"
#include <vector>
#include <fstream>
#include <ostream>

class graph
{
  public:

  std::vector<node> nodes;
  std::vector<arc> arcs;


     graph();
     virtual ~graph();

  
    friend std::ostream &operator<<( std::ostream &out, const graph &g );
    void readGraph(std::ifstream* arg1);

    std::string tolgf();
    
};

#endif // GRAPH_H
