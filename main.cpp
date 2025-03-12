#include <cctype>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace std;

int g_testCase = 0;

class ExitException : public std::exception {
public:
    [[nodiscard]] const char* what() const noexcept override {
        return "Exit requested";
    }
};

class EOFException : public std::exception {
public:
    [[nodiscard]] const char* what() const noexcept override {
        return "ERROR (no more input) : END-OF-FILE encountered";
    }
};

class RuntimeException : public std::exception {
    string message;
public:
    explicit RuntimeException(string s) : message(std::move(s)) { };

    [[nodiscard]] const char* what() const noexcept override {
        return message.c_str();
    }
};

// Enum class to represent the token type
enum class TokenType {
    LEFT_PAREN, RIGHT_PAREN,
    INT, FLOAT, STRING, DOT,
    NIL, T, QUOTE, SYMBOL,
    EOF_TOKEN
};

// Token class to store the token type, value, line, and column
struct Token {
    TokenType type;
    string value;
    int line, column;
    Token(const TokenType t, string v, const int l, const int c)
            : type(t), value(std::move(v)), line(l), column(c) {}
};

// Debugger class to print items at runtime since CLion's debugger is fucked up
class Debugger {
public:
    static void printTokenType(TokenType type) {
        switch (TokenType (type)) {
            case TokenType::LEFT_PAREN:
                cout << "LEFT_PAREN" << endl;
                break;
            case TokenType::RIGHT_PAREN:
                cout << "RIGHT_PAREN" << endl;
                break;
            case TokenType::INT:
                cout << "INT" << endl;
                break;
            case TokenType::FLOAT:
                cout << "FLOAT" << endl;
                break;
            case TokenType::DOT:
                cout << "DOT" << endl;
                break;
            case TokenType::NIL:
                cout << "NIL" << endl;
                break;
            case TokenType::T:
                cout << "T" << endl;
                break;
            case TokenType::QUOTE:
                cout << "QUOTE" << endl;
                break;
            case TokenType::SYMBOL:
                cout << "SYMBOL" << endl;
                break;
            case TokenType::STRING:
                cout << "STRING" << endl;
                break;
            case TokenType::EOF_TOKEN:
                cout << "EOF_TOKEN" << endl;
                break;
            default:
                cout << "UNKNOWN" << endl;
                break;
        }
    }

    // Should be set to false before submission
    static const bool isDebugging = false;
};

// Abstract class for the AST nodes
class Node {
public:
    virtual ~Node() = default;
    [[nodiscard]] virtual string toString(int indent = 0) const = 0;
};

// AtomNode class to represent the atomic nodes in the AST
class AtomNode final : public Node {
    TokenType type;
    string value;

public:
    AtomNode(const TokenType t, string v) : type(t), value(std::move(v)) {}

    [[nodiscard]] string toString(int indent = 0) const override {
        if (Debugger::isDebugging)
            Debugger::printTokenType(type);

        // Normalize the float output
        if (type == TokenType::FLOAT) {
            double num = stod(value);
            stringstream ss;
            ss << fixed << setprecision(3) << num;
            return ss.str();
        }

        // Normalize the integer output
        if (type == TokenType::INT) {
            return (value[0] == '+') ? value.substr(1) : value;
        }

        return value;
    }

    [[nodiscard]] TokenType getType() const {
        return type;
    }

    [[nodiscard]] string getValue() const {
        return value;
    }
};

// DotNode class to represent the dot nodes in the AST
class DotNode final : public Node {
    shared_ptr<Node> left, right;

public:
    DotNode(shared_ptr<Node> l, shared_ptr<Node> r) : left(std::move(l)), right(std::move(r)) {}

    // Returns the string representation of the dot node
    [[nodiscard]] string toString(int indent = 0) const override {
        string leftStr = left->toString(indent + 2);
        string rightStr = right->toString(indent + 2);

        if (dynamic_pointer_cast<AtomNode>(right) &&
                dynamic_pointer_cast<AtomNode>(right)->getValue() != "nil")
            return "( " + leftStr + " . " + rightStr + " )";

        string result = "( " + leftStr + "\n";
        result += string(indent + 2, ' ') + ". " + right->toString(indent + 2) + "\n";
        result += string(indent, ' ') + ")";

        return result;
    }

    [[nodiscard]] const shared_ptr<Node>& getLeft() const {
        return left;
    }

    [[nodiscard]] const shared_ptr<Node>& getRight() const {
        return right;
    }
};

// QuoteNode class to represent the quote nodes in the AST
class QuoteNode final : public Node {
    shared_ptr<Node> expression;

public:
    explicit QuoteNode(shared_ptr<Node> expr) : expression(std::move(expr)) {}

    [[nodiscard]] string toString(int indent = 0) const override {
        return "'" + expression->toString();
    }

    [[nodiscard]] shared_ptr<Node> getExpression() const {
        return expression;
    }
};

// Parser class to parse the tokens and build the AST
class Parser {
public:
    explicit Parser(vector<Token> t) : tokens(std::move(t)) {}

    size_t current = 0;

    shared_ptr<Node> parse() {
//        if (current >= tokens.size())
//            throw EOFException();

        return parseExpression();
    }

    // Returns true if current token index is less than total number of tokens
    [[nodiscard]] bool hasMore() const {
        return current < tokens.size() - 1;
    }

private:
    vector<Token> tokens;

    // Parse the expression
    shared_ptr<Node> parseExpression() {
        Token& token = tokens[current];
        switch (token.type) {
            case TokenType::LEFT_PAREN:
                return parseSExpression();
            case TokenType::QUOTE:
                current++;
                if (current >= tokens.size())
                    throw RuntimeException(
                            "ERROR (unexpected token) : missing expression after ' at Line " +
                            to_string(token.line) + " Column " + to_string(token.column)
                            );
                return make_shared<QuoteNode>(parseExpression());
            case TokenType::STRING:
                if (token.value[token.value.length()-1] != '"')
                    throw RuntimeException(
                            "ERROR (no closing quote) : END-OF-LINE encountered at Line " +
                            to_string(token.line) + " Column " + to_string(token.column + 1)
                    );
                current++;
                return make_shared<AtomNode>(token.type, token.value);
            case TokenType::INT:
            case TokenType::FLOAT:
            case TokenType::SYMBOL:
            case TokenType::NIL:
            case TokenType::T:
                current++;
                return make_shared<AtomNode>(token.type, token.value);
            default:
                throw RuntimeException(
                        "ERROR (unexpected token) : atom or '(' expected when token at Line " +
                        to_string(token.line) + " Column " + to_string(token.column + 1) +
                        " is >>" + token.value + "<<"
                        );
        }
    }

    // Parse the S-expression
    // <S-exp> ::= <ATOM>
    //    | LEFT-PAREN <S-exp> { <S-exp> } [ DOT <S-exp> ] RIGHT-PAREN
    //    | QUOTE <S-exp>
    shared_ptr<Node> parseSExpression() {
        current++; // skip '('

        // Handle empty list case: ()
        if (current < tokens.size() && tokens[current].type == TokenType::RIGHT_PAREN) {
            current++;
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }

        // Parse the first expression (required)
        auto firstExpr = parseExpression();

        // If we hit the closing paren, it's a single element list
        if (current < tokens.size() && tokens[current].type == TokenType::RIGHT_PAREN) {
            current++; // skip ')'
            // Create a proper list with one element
            return make_shared<DotNode>(firstExpr, make_shared<AtomNode>(TokenType::NIL, "nil"));
        }

        // Parse remaining expressions until DOT or RIGHT_PAREN
        vector<shared_ptr<Node>> restExpressions;
        while (current < tokens.size() &&
               tokens[current].type != TokenType::RIGHT_PAREN &&
               tokens[current].type != TokenType::DOT) {
            restExpressions.push_back(parseExpression());
        }

        // Check if we have a dotted pair
        if (current < tokens.size() && tokens[current].type == TokenType::DOT) {
            current++; // skip the dot

            // Parse the expression after the dot
            auto rightExpr = parseExpression();

            // Ensure dotted pair item
            if (!rightExpr) {
                throw RuntimeException(
                        "ERROR (unexpected token) : atom or '(' expected at Line " +
                        to_string(tokens[current].line) + " Column " + to_string(tokens[current].column)
                );
            }

            // Ensure closing parenthesis
            if (current >= tokens.size() || tokens[current].type != TokenType::RIGHT_PAREN) {
                throw RuntimeException(
                        "ERROR (unexpected token) : ')' expected at Line " +
                        to_string(tokens[current].line) + " Column " + to_string(tokens[current].column)
                        );
            }
            current++; // skip ')'

            // Build the result - first build the left side chain
            shared_ptr<Node> result = rightExpr;
            for (int i = restExpressions.size() - 1; i >= 0; i--) {
                result = make_shared<DotNode>(restExpressions[i], result);
            }
            // Then add the first expression at the beginning
            return make_shared<DotNode>(firstExpr, result);
        }

        // It's a proper list (not a dotted pair)
        else {
            if (current >= tokens.size() || tokens[current].type != TokenType::RIGHT_PAREN) {
                throw runtime_error("ERROR (unexpected token) : ')' expected at Line "
                                    + to_string(tokens[current].line) + " Column "
                                    + to_string(tokens[current].column));
            }
            current++; // skip ')'

            // Build a proper list - all nodes linked with the last pointing to nil
            shared_ptr<Node> result = make_shared<AtomNode>(TokenType::NIL, "nil");
            for (int i = restExpressions.size() - 1; i >= 0; i--) {
                result = make_shared<DotNode>(restExpressions[i], result);
            }

            // Add the first expression
            return make_shared<DotNode>(firstExpr, result);
        }
    }
};

// Scanner class to scan the input and return the tokens
class Scanner {
public:
    int line, column;

    explicit Scanner(string input) : input(std::move(input)) {
        line = 1;
        column = 1;
    }

    // Scan the input and return the tokens
    vector<Token> scanTokens() {
        vector<Token> tokens;
        while (!isAtEnd()) {
            skipWhitespace();
            if (isAtEnd())
                break;
            char c = advance();
            switch (c) {
                case '(':
                    tokens.emplace_back(TokenType::LEFT_PAREN, "(", line, column);
                    column = 0;
                    break;
                case ')':
                    tokens.emplace_back(TokenType::RIGHT_PAREN, ")", line, column);
                    column = 0;
                    break;
                case '"':
                    tokens.push_back(scanString());
                    column = 0;
                    break;
                case '\'':
                    tokens.emplace_back(TokenType::QUOTE, "'", line, column);
                    column = 0;
                    break;
                case ';':
                    while (!isAtEnd() && peek() != '\n')
                        advance();
                    break;
                default:
                    tokens.push_back(scanToken(c));
                    column = 0;
                    break;
            }
        }
        tokens.emplace_back(TokenType::EOF_TOKEN, "", line, column);
        return tokens;
    }

private:
    string input;
    size_t current = 0;

    // Scan the string token
    Token scanString() {
        string value;
        value += '"';
        while (!isAtEnd() && peek() != '"') {
            if (peek() == '\\') {
                advance();
                char escapedChar = advance();
                switch (escapedChar) {
                    case 'n':
                        value += '\n';
                        break;
                    case 't':
                        value += '\t';
                        break;
                    case '"':
                        value += '"';
                        break;
                    case '\\':
                        value += '\\';
                        break;
                    default:
                        value += '\\';
                        value += escapedChar;
                        break;
                }
            } else {
                if (peek() == '\n') {
                    cin.get();
                    return Token(TokenType::STRING, value, line, column);
                }
                value += advance();
            }
        }

        if (isAtEnd())
            throw EOFException();

        value += '"';

        advance();
        return Token(TokenType::STRING, value, line, column);
    }

    // Scan the token
    Token scanToken(char first) {
        string value(1, first);
        while (!isAtEnd() && isSymbolChar(peek()))
            value += advance();

        if (isNumber(value))
            return scanNumber(value);

        else
            return scanSymbol(value);
    }

    // Scan the number token
    [[nodiscard]] Token scanNumber(const string& value) const {
        bool isFloat = value.find('.') != string::npos;
        return isFloat ? Token(TokenType::FLOAT, value, line, column)
                       : Token(TokenType::INT, value, line, column);
    }

    // Scan the symbol token
    [[nodiscard]] Token scanSymbol(const string& value) const {
        if (value == "nil" || value == "#f")
            return Token(TokenType::NIL, "nil", line, column);

        else if (value == "t" || value == "#t")
            return Token(TokenType::T, "#t", line, column);

        else if (value == ".")
            return Token(TokenType::DOT, ".", line, column);

        else
            return Token(TokenType::SYMBOL, value, line, column);
    }

    // Check if the given string is a number
    static bool isNumber(const string& value) {
        bool hasDigit = false;
        bool hasDot = false;
        size_t start = 0;

        // Check for optional leading + or -
        if (value[0] == '+' || value[0] == '-')
            start = 1;

        for (size_t i = start; i < value.length(); ++i) {
            char c = value[i];

            if (isdigit(c))
                hasDigit = true;

            else if (c == '.') {
                if (hasDot)
                    return false;
                hasDot = true;
            }

            else
                return false;
        }

        return hasDigit;
    }

    // Check if the given character is a symbol character
    static bool isSymbolChar(char c) {
        return isalpha(c) || isdigit(c) || string("!?-+*/><=_~#@$%.^&").find(c) != string::npos;
    }

    // Skip the whitespace characters
    void skipWhitespace() {
        while (!isAtEnd()) {
            char c = peek();
            if (isspace(c))
                advance();

            else
                break;
        }
    }

    // Advance the current position and return the character
    char advance() {
        if (peek() == '\n') {
            line++;
            column = 0;
        }

        else
            column++;

        if (isAtEnd())
            return '\0';

        return input[current++];
    }

    // Return the current character without advancing the position
    [[nodiscard]] char peek() const {
        return isAtEnd() ? '\0' : input[current];
    }

    // Check if the current position is at the end
    [[nodiscard]] bool isAtEnd() const {
        return current >= input.length();
    }
};

// Printer class to print the AST nodes
class Printer {
public:
    // The main print function: prints an AST node using proper indentation.
    static string print(const shared_ptr<Node>& node, int indent = 0) {
        // Handle quoted expressions: print a leading single-quote.
        if (auto quote = dynamic_pointer_cast<QuoteNode>(node)) {
            string result = "( quote";
            result += "\n" + string(indent + 2, ' ') + print(quote->getExpression(), indent + 2);
            result += "\n" + string(indent, ' ') + ")";
            return result;
        }
        // If the node is a DotNode, unroll it as a list.
        if (auto dot = dynamic_pointer_cast<DotNode>(node)) {
            vector<shared_ptr<Node>> elems;
            shared_ptr<Node> tail;
            unrollList(node, elems, tail);
            // A proper list has a tail that is the atom "nil".
            bool proper = false;
            if (auto atomTail = dynamic_pointer_cast<AtomNode>(tail)) {
                if (atomTail->getValue() == "nil")
                    proper = true;
            }
            string result;
            result += "( ";
            if (!elems.empty()) {
                // Print the first element inline.
                result += print(elems[0], indent + 2);
                // Print each subsequent element on a new line indented by two spaces.
                for (size_t i = 1; i < elems.size(); i++) {
                    result += "\n" + string(indent + 2, ' ') + print(elems[i], indent + 2);
                }
            }
            // For dotted lists, print the dot on its own line with the same indent,
            // then print the tail on a new line with that same indent.
            if (!proper) {
                result += "\n" + string(indent + 2, ' ') + ".";
                result += "\n" + string(indent + 2, ' ') + print(tail, indent + 2);
            }
            result += "\n" + string(indent, ' ') + ")";
            return result;
        }
        // For all other nodes, use their own toString() implementation.
        return node->toString();
    }

private:
    // Helper to unroll a DotNode chain into its elements and tail.
    static void unrollList(const shared_ptr<Node>& node, vector<shared_ptr<Node>>& elems, shared_ptr<Node>& tail) {
        auto current = node;
        while (auto d = dynamic_pointer_cast<DotNode>(current)) {
            elems.push_back(d->getLeft());
            current = d->getRight();
        }
        tail = current;
    }
};


// Helper function to check whether the S-expression is exactly (exit)
bool isExitExpression(const shared_ptr<Node>& node) {
    // We assume (exit) is represented as a proper list with one element "exit"
    auto dot = dynamic_pointer_cast<DotNode>(node);

    if (!dot)
        return false;

    auto left = dot->getLeft();
    auto right = dot->getRight();
    auto atomLeft = dynamic_pointer_cast<AtomNode>(left);
    auto atomRight = dynamic_pointer_cast<AtomNode>(right);

    if (!atomLeft || !atomRight)
        return false;

    return (atomLeft->getValue() == "exit" && atomRight->getValue() == "nil");
}

int main() {
    try {
        cout << "Welcome to OurScheme!" << endl;
        cin >> g_testCase;

        // Ignore the newline after the test case input
        cin.ignore();

        string buffer;
        while (true) {
            try {
                // Is EOF, print error message and exit the program
                if (cin.peek() == EOF)
                    throw EOFException();

                // Read input line by line
                string line;
                getline(cin, line);

                if (line.empty())
                    continue;

                // Append the line to the buffer
                buffer += line + "\n";

                // Get tokens
                Scanner scanner(buffer);
                vector<Token> tokens = scanner.scanTokens();

                // Handle if expression completed
                int balance = 0;
                for (const auto& token : tokens) {
                    if (token.type == TokenType::LEFT_PAREN)
                        balance++;
                    else if (token.type == TokenType::RIGHT_PAREN)
                        balance--;
                }

                // Expression not complete, keep reading input
                if (balance > 0 ||
                    tokens[0].type == TokenType::EOF_TOKEN ||
                    (tokens[0].type == TokenType::QUOTE && tokens[1].type == TokenType::EOF_TOKEN))
                    continue;

                // Parse the tokens if the expression is complete
                Parser parser(tokens);
                while (parser.hasMore()) {
                    shared_ptr<Node> ast = parser.parse();

                    cout << endl << "> ";

                    // If the parsed expression is "(exit)", throw exception to exit
                    if (isExitExpression(ast))
                        throw ExitException();

                    cout << Printer::print(ast) << endl;
                }

                buffer.clear();

            } catch (const RuntimeException& e) {
                cout << endl << "> " << e.what() << endl;
                buffer.clear();
            }
        }

    } catch (const EOFException& e) {
        cout << endl << "> " << e.what();
    } catch (const ExitException& e) {

    }

    cout << endl << "Thanks for using OurScheme!";
    return 0;
}