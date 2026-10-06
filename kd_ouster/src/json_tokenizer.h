#pragma once
#include <string>
#include <vector>

#pragma once

struct Token {
  enum  TokenType {
    STRING=0,       // "keys" or "values" 6
    NUMBER=1,       // 13.762, 7502, -16.68 7 
    BOOLEAN=2,         // 8
    NULLV=3,         // 9
    OBJECT_START=4, // { 0 
    OBJECT_END=5,   // } 1
    ARRAY_START=6,  // [ 2
    ARRAY_END=7,    // ] 3
    COLON=8,        // : 4
    COMMA=9,        // , 5
    UNKNOWN=-1    // 10
  };

  TokenType type;
    std::string value;
};

struct JsonTokenizer {
  static std::vector<Token> tokenize(const std::string& json);
};
