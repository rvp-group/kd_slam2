#include "json_item.h"
#include <stdexcept>
#include <iostream>
using namespace std;

JsonItemBase::~JsonItemBase() {};

std::string JsonItemBase::asString() {
  throw std::runtime_error("undef");
  return "";
}
double JsonItemBase::asDouble(){
  throw std::runtime_error("undef");
  return 0;
}
int    JsonItemBase::asInt(){
  throw std::runtime_error("undef");
  return 0;
}

JsonMapItem* JsonItemBase::asMap() {
  throw std::runtime_error("undef");
  return nullptr;
}

JsonVectorItem* JsonItemBase::asVector() {
  throw std::runtime_error("undef");
  return nullptr;
}
void JsonItemBase::print(ostream& os, int& ilevel) {
  std::runtime_error("undef");
}

ostream& indent(ostream& os, int l){
  for (int i=0; i<l; ++i)
    os<<"  ";
  return os;
}

JsonMapItem* JsonMapItem::asMap()  {return this;}

void JsonMapItem::print(ostream& os, int& ilevel)   {
  os << "{" << endl;
  ++ ilevel;
  size_t s=size();
  size_t i=0;
  for (auto& [key, value]: *this) {
    indent(os, ilevel);
    os << '"' << key << "\" : ";
    value->print(os, ilevel);
    if (i<(s-1))
      os <<"," << endl;
    ++i;
  }
  --ilevel;
  indent(os, ilevel);
  os << "}";
}

JsonVectorItem* JsonVectorItem::asVector()  {return this;}
void JsonVectorItem::print(ostream& os, int& ilevel)   {
  os << "[" << endl;
  ++ ilevel;
  for (size_t i=0; i<size(); ++i) {
    indent(os, ilevel);
    at(i)->print(os, ilevel);
    if (i<size()-1) {
      os << ",";
    }
    os << endl;
  }
  --ilevel;
  indent(os, ilevel);
  os << "]";
}

std::string JsonSingleItem::asString()  {
  return value;
}

double JsonSingleItem::asDouble()  {
  return atof(value.c_str());
}

int JsonSingleItem::asInt()  {
  return atoi(value.c_str());
}

void JsonSingleItem::print(ostream& os, int& ilevel)  {
  if (type==STRING)
    os << '"';
  os << value;
  if (type==STRING)
    os << '"';
}
