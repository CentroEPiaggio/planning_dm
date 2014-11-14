#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <algorithm>
#include <iostream>

using namespace std;

class utils
{
public:
  static void skipSection(ifstream *f)
  {
    char s[256];
    f->getline(s,256);
    string tmp;
    tmp=s;
    tmp.erase(remove(tmp.begin(), tmp.end(), ' '),tmp.end());
    tmp.erase(remove(tmp.begin(), tmp.end(), '\t'),tmp.end());
    while(tmp.empty() || tmp.at(0)!=']') 
	{
	  f->getline(s,256);
	  tmp=s;
	  tmp.erase(remove(tmp.begin(), tmp.end(), ' '),tmp.end());
	  tmp.erase(remove(tmp.begin(), tmp.end(), '\t'),tmp.end());
	  if (!tmp.empty() && tmp.at(0)=='[')
	    skipSection(f);
	}
  }
  
};

#endif // UTILS_H
