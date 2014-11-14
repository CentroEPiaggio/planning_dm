#ifndef NODE_H
#define NODE_H

#include <string>
#include <fstream>
#include <ostream>
class node
{
  
std::string label;
    void readGraphics( std::ifstream* f );
    void readTextGraphics(std::fstream* arg1);

public:
  int x,y;
  std::string name;
    node();
    virtual ~node();
  
    
    friend std::ostream &operator<<( std::ostream &out, const node &n );
    void readNode( std::ifstream* f );
    
};

#endif // NODE_H
