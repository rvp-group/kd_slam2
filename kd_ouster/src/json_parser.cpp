#include "json_parser.h"

Token::TokenType JsonParser::peekType() const {
  if (m_index >= m_tokens.size()) return Token::UNKNOWN;
  return m_tokens[m_index].type;
}

Token JsonParser::consume(Token::TokenType expectedType) {
  if (m_index >= m_tokens.size()) {
    throw std::runtime_error("Unexpected end of JSON tokens.");
  }
  if (m_tokens[m_index].type != expectedType) {
    throw std::runtime_error("JSON Syntax Error: Unexpected token '" + m_tokens[m_index].value + "'");
  }
  return m_tokens[m_index++];
}


std::unique_ptr<JsonItemBase> JsonParser::parse(const std::string& text) {
  JsonTokenizer tokenizer;
  auto tokens=tokenizer.tokenize(text); 
  JsonParser parser(tokens);
  return parser.parse();
}


std::unique_ptr<JsonItemBase> JsonParser::parse() {
  Token::TokenType type = peekType();
  switch (type) {
  case Token::OBJECT_START:
    return parseObject();
  case Token::ARRAY_START:
    return parseArray();
  case Token::STRING:
    return std::make_unique<JsonSingleItem>(JsonItemBase::STRING, m_tokens[m_index++].value);
  case Token::NUMBER:
    return std::make_unique<JsonSingleItem>(JsonItemBase::NUMBER, m_tokens[m_index++].value);
  case Token::BOOLEAN:
    return std::make_unique<JsonSingleItem>(JsonItemBase::BOOLEAN, m_tokens[m_index++].value);
  case Token::NULLV:
    return std::make_unique<JsonSingleItem>(JsonItemBase::NULLV, m_tokens[m_index++].value);
  }
  throw std::runtime_error("Invalid JSON root element.");
}

// Parse an Object: { "key": value, "key2": value2 }
std::unique_ptr<JsonItemBase> JsonParser::parseObject() {
  consume(Token::OBJECT_START);
  auto mapItem = std::make_unique<JsonMapItem>();

  if (peekType() == Token::OBJECT_END) {
    consume(Token::OBJECT_END);
    return mapItem;
  }

  while (true) {
    // Objects must start with a string key
    Token keyToken = consume(Token::STRING);
    consume(Token::COLON);
    // Recursively parse the value mapping to this key
    (*mapItem)[keyToken.value] = parse();

    // Check for next item or end of object
    if (peekType() == Token::COMMA) {
      consume(Token::COMMA);
    } else if (peekType() == Token::OBJECT_END) {
      consume(Token::OBJECT_END);
      break;
    } else {
      throw std::runtime_error("Expected ',' or '}' in object configuration.");
    }
  }
  return mapItem;
}

// Parse an Array: [ value1, value2, value3 ]
std::unique_ptr<JsonItemBase> JsonParser::parseArray() {
  consume(Token::ARRAY_START);
  auto vectorItem = std::make_unique<JsonVectorItem>();

  if (peekType() == Token::ARRAY_END) {
    consume(Token::ARRAY_END);
    return vectorItem;
  }

  while (true) {
    // Recursively parse the array element
    vectorItem->push_back(parse());

    // Check for next item or end of array
    if (peekType() == Token::COMMA) {
      consume(Token::COMMA);
    } else if (peekType() == Token::ARRAY_END) {
      consume(Token::ARRAY_END);
      break;
    } else {
      throw std::runtime_error("Expected ',' or ']' in array layout.");
    }
  }
  return vectorItem;
}
