#pragma once
#include "json_tokenizer.h"
#include "json_item.h"

struct JsonParser {

  // Helper to peek at the current token type
  JsonParser(const std::vector<Token>& tokens) : m_tokens(tokens) {}
  // Main entry point
  std::unique_ptr<JsonItemBase> parse();
  static std::unique_ptr<JsonItemBase> parse(const std::string& text);
protected:
  const std::vector<Token>& m_tokens;
  size_t m_index = 0;
  Token::TokenType peekType() const;
  Token consume(Token::TokenType expectedType);
  // Parse an Object: { "key": value, "key2": value2 }
  std::unique_ptr<JsonItemBase> parseObject();
  // Parse an Array: [ value1, value2, value3 ]
  std::unique_ptr<JsonItemBase> parseArray();
};
