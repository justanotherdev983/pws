#include <cstdint>
#include <fstream>
#include <iostream>
#include <print>
#include <sstream>
#include <string>
#include <vector>

struct ast_node {
    std::string type;
    std::string value;
    std::vector<ast_node> children; // bijv: [+, [2,3]]
};

enum class preprocessor_directives : uint8_t {
    type_include,
    // in een echte compiler hebben we hier veel andere directives zoals define
    // etc
    // ...
};

enum class token_type : uint8_t {
    type_namespace,
    type_identifier,    // namespace
    type_open_paren,    // )
    type_close_paren,   // (
    type_open_bracket,  // {
    type_close_bracket, // }
    type_open_angle,    // [
    type_close_angle,   // ]
    type_string_lit,    // "..."
    type_semicolon,     // ;
    type_double_colon,  // ::
    type_keyword_int,   // int
    type_keyword_void,  // void
};

struct token {
    token_type type;
    std::string identifier;
    std::string
        value; // Voor dit voorbeeld hebben we geen andere soorten values
};

std::vector<token> lexer(const std::string &src_code) {
    std::vector<token> tokens;
    size_t pos = 0;

    while (pos < src_code.size()) {
        // Skip whitespace
        while (pos < src_code.size() && std::isspace(src_code[pos])) {
            pos++;
        }
        if (pos >= src_code.size())
            break;

        char current = src_code[pos];

        // String literals; stukje tekst
        if (current == '"') {
            std::string lit;
            pos++; // skip opening quote; we willen de string hebben
            while (pos < src_code.size() && src_code[pos] != '"') {
                lit += src_code[pos++]; // elke char in string
            }
            pos++; // skip closing quote
            tokens.push_back({token_type::type_string_lit, "", lit});
            continue;
        }

        // Single character tokens
        if (current == '(') {
            tokens.push_back({token_type::type_open_paren, "(", ""});
            pos++;
        } else if (current == ')') {
            tokens.push_back({token_type::type_close_paren, ")", ""});
            pos++;
        } else if (current == '{') {
            tokens.push_back({token_type::type_open_bracket, "{", ""});
            pos++;
        } else if (current == '}') {
            tokens.push_back({token_type::type_close_bracket, "}", ""});
            pos++;
        } else if (current == ';') {
            tokens.push_back({token_type::type_semicolon, ";", ""});
            pos++;
        }
        // Double colon :: // voor std:: scope
        else if (current == ':' && pos + 1 < src_code.size() &&
                 src_code[pos + 1] == ':') {
            tokens.push_back({token_type::type_double_colon, "::", ""});
            pos += 2;
        }
        // Keywords en identifiers; isalpha omdat het een woord is, maar kan ook
        // _ bevatten
        else if (std::isalpha(current) || current == '_') {
            std::string id;
            while (pos < src_code.size() &&
                   (std::isalnum(src_code[pos]) || src_code[pos] == '_')) {
                id += src_code[pos++];
            }

            // Check voor keywords
            if (id == "namespace") {
                tokens.push_back({token_type::type_namespace, id, ""});
            } else if (id == "int") {
                tokens.push_back({token_type::type_keyword_int, id, ""});
            } else {
                tokens.push_back({token_type::type_identifier, id, ""});
            } // in een echte compiler switch je voor keywords en heb je er een
              // stuk meer
        } else {
            pos++; // Skip characters die we niet kennen; in een echte compiler
                   // error je waarschijnlijk
        }
    }

    return tokens;
}

ast_node parse_expression(std::vector<token> tokens, size_t token_index) {
    ast_node node;
    int pos = 0;
    // Simplified: parse functie call; voor 'println'
    if (tokens[pos].type == token_type::type_identifier) {
        node.type = "function_call";
        node.value = tokens[pos].identifier;

        // Skip :: and function name voor std::println; in een echte compiler
        // check je hier of functie in scope is voor de ::
        if (pos + 2 < tokens.size() &&
            tokens[pos + 1].type == token_type::type_double_colon) {
            node.value =
                tokens[pos].identifier + "::" + tokens[pos + 2].identifier;
            pos += 3; // skip 'std', '::', 'println'
        }

        // Parse arguments binnen ()
        if (tokens[pos].type == token_type::type_open_paren) {
            pos++; // skip '('
            while (tokens[pos].type != token_type::type_close_paren) {
                if (tokens[pos].type == token_type::type_string_lit) {
                    ast_node arg;
                    arg.type = "string_literal";
                    arg.value = tokens[pos].value;
                    node.children.push_back(arg);
                }
                pos++;
            }
            pos++; // skip ')'
        }
    }

    return node;
}

std::vector<ast_node> parse_statement(std::vector<token> &token_stream) {
    std::vector<ast_node> program_ast;
    size_t token_index = 0;

    while (token_index < token_stream.size()) {
        token token = token_stream[token_index];
        if (token.type == token_type::type_open_paren) {
            ast_node root_node;
            parse_expression(token_stream, token_index);

            // Look for semicolon
            token = token_stream[token_index];
            if (token.type == token_type::type_semicolon) {
                token_index++;
            } else {
                // in een echte compiler zou je hier een descriptive error geven
                // met vergeten ';'
            }

            program_ast.push_back(std::move(root_node));
        } else {
            // in een echte compiler hebben we hier veel meer statements....
        }
    }

    return program_ast;
}

std::string preprocess(const std::string &src) {
    std::string res;
    std::istringstream stream(
        src); // zodat wij zo std::getline kunnen gebruiken
    std::string line;

    while (std::getline(stream, line)) {
        if (line[0] == '#') { // onze preprocesser directive, dit laten we ook
                              // weg voor lexer
            // in een echte compiler switchen we hier voor de verschillende
            // directives
            continue;
        }
        res.append(line + " "); //
    }

    return res;
}

int main(int argc, char *argv[]) {

    std::println("dit is een voorbeeld");

    std::vector<token> token_stream;

    std::fstream voorbeeld_code("voorbeeld.cpp");

    if (!voorbeeld_code.good()) {
        std::cerr << "We konden de input file niet opene\n";
        return 1; // we returning hier een niet-null exit code, omdat als het
                  // openen faalt we geen code hebben om iets te doen
    }

    std::string line;
    std::string voorbeeld_code_string;

    while (std::getline(voorbeeld_code, line)) {
        voorbeeld_code_string.append(line);
    }

    std::string voorbeld_code_voor_lexer = preprocess(voorbeeld_code_string);

    token_stream = lexer(voorbeld_code_voor_lexer);

    std::vector<ast_node> ast = parse_expression(token_stream);

    return 0;
}
