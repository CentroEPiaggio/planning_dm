#include <iostream>
#include <fstream>
#include "graph.h"
#include "gmlreader.hpp"
using namespace std;

int main(int argc, char **argv) {
  if (true)
  {
      ifstream f;
	  if (argc<2)
	  {
		cout<< "attenzione, inserire il nome del file da convertire"<<endl;
		return 1;
	  }
      string filename=argv[1];
      cout<<filename<<endl;
      gmlreader reader;
      
      f.open(filename.c_str(),ios::in);
      graph g=reader.read(&f);

      
      f.close();
      ofstream fo;
      fo.open(filename.append(".lgf").c_str(),ios::out);
      fo<<g.tolgf();
      fo.close();
  }
  
  return 0;
}
