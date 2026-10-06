#include "json_tokenizer.h"
std::vector<Token> JsonTokenizer::tokenize(const std::string& json) {
  std::vector<Token> tokens;
  size_t i = 0;
  size_t len = json.length();

  while (i < len) {
    // Skip whitespace
    if (std::isspace(json[i])) {
      i++;
      continue;
    }

    // Handle Single Structural Characters
    if (json[i] == '{') {
      tokens.push_back({Token::OBJECT_START, "{"});
      i++;
      continue;
    }
    if (json[i] == '}') {
      tokens.push_back({Token::OBJECT_END, "}"});
      i++;
      continue;
    }
    if (json[i] == '[') {
      tokens.push_back({Token::ARRAY_START, "["});
      i++;
      continue;
    }
    if (json[i] == ']') {
      tokens.push_back({Token::ARRAY_END, "]"});
      i++;
      continue;
    }
    if (json[i] == ':') {
      tokens.push_back({Token::COLON, ":"});
      i++;
      continue;
    }
    if (json[i] == ',') {
      tokens.push_back({Token::COMMA, ","});
      i++;
      continue;
    }

    // Handle Strings (Keys and Values inside quotes)
    if (json[i] == '"') {
      std::string strVal;
      i++; // Skip opening quote
      while (i < len && json[i] != '"') {
        // Handle simple backslash escapes if any
        if (json[i] == '\\' && i + 1 < len) {
          i++;
        }
        strVal += json[i];
        i++;
      }
      i++; // Skip closing quote
      tokens.push_back({Token::STRING, strVal});
      continue;
    }

    // Handle Numbers (including negative numbers and decimals)
    if (std::isdigit(json[i]) || json[i] == '-') {
      std::string numVal;
      while (i < len && (std::isdigit(json[i]) || json[i] == '.' || json[i] == '-' || json[i] == 'e' || json[i] == 'E')) {
        numVal += json[i];
        i++;
      }
      tokens.push_back({Token::NUMBER, numVal});
      continue;
    }

    if (std::isalpha(json[i])) {
      std::string tok_val;
      tok_val+=json[i];
      i++;
      while (i < len &&
              std::isalpha(json[i])) {
        tok_val += json[i];
        i++;
      }
      if (tok_val=="true")
        tokens.push_back({Token::BOOLEAN, tok_val});
      else if (tok_val=="false")
        tokens.push_back({Token::BOOLEAN, tok_val});
      else if (tok_val=="null")
        tokens.push_back({Token::NULLV, tok_val});
      else
        tokens.push_back({Token::UNKNOWN, tok_val});
      continue;
    }
    
    // Skip unknown symbols safely to prevent infinite loops
    i++;
  }
  return tokens;
}
