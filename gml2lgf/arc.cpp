#include "arc.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <fstream>
#include "utils.h"


arc::arc()
{
  len=1;
  first=0;
  second=0;
}

void arc::readLabelGraphics(ifstream* f)
{
  char s[256];
  f->getline(s,256);
  string tmp;
  tmp=s;
  tmp.erase(remove(tmp.begin(), tmp.end(), ' '),tmp.end());
  tmp.erase(remove(tmp.begin(), tmp.end(), '\t'),tmp.end());
  if (tmp.compare("[")==0) //deve esserci subito una quadra
  {
    while(!f->eof()) 
    {
      f->getline(s,256);
      tmp=s;
      tmp.erase(remove(tmp.begin(), tmp.end(), ' '),tmp.end());
      tmp.erase(remove(tmp.begin(), tmp.end(), '\t'),tmp.end());

      if (tmp.find("text\"")==0)
      {
	stringstream in;
	in<<tmp.substr(5,tmp.length()-5);
	in>>len;
// 	cout<<"lunghezza dell'arco: "<<len<<endl;
	continue;
      }
     
      if (tmp.find("[")==0)
      {
	utils::skipSection(f);
	continue;
      }
      if (tmp.find("]")==0)
      {
	 break;
      }
    }
  }
  else
    cerr<<"attenzione, impossibile trovare la quadra di apertura della grafica dell'arco"<<endl;
}


void arc::readArc(ifstream* f)
{
  char s[256];
  f->getline(s,256);
  string tmp;
  tmp=s;
  tmp.erase(remove(tmp.begin(), tmp.end(), ' '),tmp.end());
  tmp.erase(remove(tmp.begin(), tmp.end(), '\t'),tmp.end());
  if (tmp.compare("[")==0) //deve esserci subito una quadra
  {
    while(!f->eof()) //leggiamo le proprietà del nodo
    {
      f->getline(s,256);
      tmp=s;
      tmp.erase(remove(tmp.begin(), tmp.end(), ' '),tmp.end());
      tmp.erase(remove(tmp.begin(), tmp.end(), '\t'),tmp.end());

      if (tmp.find("source")==0)
      {
	stringstream in;
	in<<tmp.substr(6,tmp.length()-6);
	in>>first;
// 	cout<<"primo nodo dell'arco: "<<first<<endl;
	continue;
      }
      if (tmp.find("target")==0)
      {
	stringstream in;
	in<<tmp.substr(6,tmp.length()-6);
	in>>second;
// 	cout<<"secondo nodo dell'arco: "<<second<<endl;
	continue;
      }
      if (tmp.compare("LabelGraphics")==0)
      {
	readLabelGraphics(f);
	continue;
      }
     
      if (tmp.find("[")==0)
      {
	utils::skipSection(f);
	continue;
      }
      if (tmp.find("]")==0)
      {
	break;
      }
    }
  }
  else
    cerr<<"attenzione, impossibile trovare la quadra di apertura dell'arco"<<endl;
}


ostream &operator<<( ostream &out, const arc &a )
{
  out<<a.first<<"\t"<<a.second<<"\t"<<a.name<<"\t"<<a.len;
  return out;
}



arc::~arc()
{

}


