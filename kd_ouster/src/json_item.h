#pragma once
#include <memory>
#include <map>
#include <vector>
#include <string>
#include <iostream>

struct JsonMapItem;
struct JsonVectorItem;

struct JsonItemBase{

  enum JsonItemType {
    STRING=0,       // "keys" or "values" 6
    NUMBER=1,       // 13.762, 7502, -16.68 7 
    BOOLEAN=2,         // 8
    NULLV=3,         // 9
    OBJECT=5,
    VECTOR=6,
    UNKNOWN=-1
  };
  JsonItemType type;
  JsonItemBase(JsonItemType t=UNKNOWN): type(t){}
  virtual ~JsonItemBase()=0;
  virtual std::string asString();
  virtual double asDouble();
  virtual int    asInt();
  virtual JsonMapItem* asMap();
  virtual JsonVectorItem* asVector();
  virtual void print(std::ostream& os, int& ilevel);
};

struct JsonMapItem: public JsonItemBase,
                    public std::map<std::string, std::unique_ptr<JsonItemBase> > {
  JsonMapItem(): JsonItemBase(JsonItemType::OBJECT){}
  JsonMapItem* asMap() override;
  void print(std::ostream& os, int& ilevel)  override;
};

struct JsonVectorItem: public JsonItemBase,
                    public std::vector<std::unique_ptr<JsonItemBase> > {
  JsonVectorItem(): JsonItemBase(JsonItemType::VECTOR){}
  JsonVectorItem* asVector() override;
  void print(std::ostream& os, int& ilevel)  override;
};

struct JsonSingleItem: public JsonItemBase  {
  std::string value;
  JsonSingleItem(JsonItemType t, std::string v):
    JsonItemBase(t){
    type=t;
    value=v;
  }
  std::string asString() override;
  double asDouble() override ;
  int asInt() override ;
  void print(std::ostream& os, int& ilevel) override ;
};
