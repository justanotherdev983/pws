#include <cstdint>
#include <fstream>
#include <iostream>
#include <print>
#include <sstream>
#include <string>
#include <vector>

struct node {};

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

    return tokens;
}

std::string preprocess(const std::string &src) {
    std::string res;
    std::istringstream stream(
        src); // zodat wij zo std::getline kunnen gebruiken
    std::string line;

    while (std::getline(stream, line)) {
        if (line[0] == '#') { // onze preprocesser directive, dit laten we ook
                              // weg voor lexer
            continue;
            switch (line[0.. ' ']) {}
        }
        res.append(line);
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

    return 0;
}
