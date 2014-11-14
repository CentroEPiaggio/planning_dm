#ifndef GMLREADER_HPP
#define GMLREADER_HPP
#include "graph.h"
#include <fstream>
class gmlreader
{
  
  graph g;

public:
  gmlreader()
  {
    
  };

  graph read(std::ifstream* f)
  {
      std::string tmp;
            char s[256];

      while (!f->eof())
      {
	f->getline(s,256);
	tmp=s;
	if (tmp.compare("graph")==0) //inizia il testo interessante
	{
	   g.readGraph(f);
	}
      
      }
    return g;
  }
  
  
};
      
#endif