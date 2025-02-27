#include <string>
#include <memory>
#include <stdexcept>
#include <vector>
#include <cctype>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <exception>

using namespace std;

int g_testCase = 0;

class ExitException : public std::exception {
public:
    const char* what() const noexcept override {
        return "Exit requested";
    }
};

class EOFException : public std::exception {
public:
    const char* what() const noexcept override {
        return "ERROR (no more input) : END-OF-FILE encountered";
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

// Debugger class to print items at runtime since rider's debugger is fucked up
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
    virtual string toString(int indent = 0) const = 0;
};

// AtomNode class to represent the atomic nodes in the AST
class AtomNode final : public Node {
    TokenType type;
    string value;
public:
    AtomNode(const TokenType t, string v) : type(t), value(std::move(v)) {}

    string toString(int indent = 0) const override {
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

    TokenType getType() const {
        return type;
    }

    string getValue() const {
        return value;
    }
};

class DotNode final : public Node {
    shared_ptr<Node> left;
    shared_ptr<Node> right;
public:
    DotNode(shared_ptr<Node> l, shared_ptr<Node> r)
        : left(std::move(l)), right(std::move(r)) {}

    string toString(int indent = 0) const override {
        string leftStr = left->toString(indent + 2);
        string rightStr = right->toString(indent + 2);
        if (dynamic_pointer_cast<AtomNode>(right) && dynamic_pointer_cast<AtomNode>(right)->getValue() != "nil")
            return "( " + leftStr + " . " + rightStr + " )";
        string result = "( " + leftStr + "\n";
        result += string(indent + 2, ' ') + ". " + right->toString(indent + 2) + "\n";
        result += string(indent, ' ') + ")";
        return result;
    }

    const shared_ptr<Node>& getLeft() const {
        return left;
    }

    const shared_ptr<Node>& getRight() const {
        return right;
    }
};

class QuoteNode final : public Node {
    shared_ptr<Node> expression;
public:
    QuoteNode(shared_ptr<Node> expr) : expression(std::move(expr)) {}

    string toString(int indent = 0) const override {
        return "'" + expression->toString();
    }

    shared_ptr<Node> getExpression() const {
        return expression;
    }
};

class Parser {
    vector<Token> tokens;
    size_t current = 0;

public:
    Parser(vector<Token> t) : tokens(std::move(t)) {}

    shared_ptr<Node> parse() {
        if (current >= tokens.size())
            throw EOFException();
        return parseExpression();
    }

    // returns true if current token index is less than total number of tokens
    bool hasMore() const {
        return current < tokens.size() - 1;
    }

private:
    shared_ptr<Node> parseExpression() {
        Token& token = tokens[current];
        switch (token.type) {
            case TokenType::LEFT_PAREN:
                return parseSExpression();
            case TokenType::QUOTE:
                current++;
                if (current >= tokens.size())
                    throw runtime_error("ERROR (unexpected token) : missing expression after ' at line " + to_string(token.line) + " column " + to_string(token.column));
                return make_shared<QuoteNode>(parseExpression());
            case TokenType::INT:
            case TokenType::FLOAT:
            case TokenType::STRING:
            case TokenType::SYMBOL:
            case TokenType::NIL:
            case TokenType::T:
                current++;
                return make_shared<AtomNode>(token.type, token.value);
            default:
                throw runtime_error("ERROR (unexpected token) : atom expected at line " + to_string(token.line) + " column " + to_string(token.column));
        }
    }

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
        vector<shared_ptr<Node>> restExprs;
        while (current < tokens.size() &&
               tokens[current].type != TokenType::RIGHT_PAREN &&
               tokens[current].type != TokenType::DOT) {
            restExprs.push_back(parseExpression());
        }

        // Check if we have a dotted pair
        if (current < tokens.size() && tokens[current].type == TokenType::DOT) {
            current++; // skip the dot

            // Parse the expression after the dot
            auto rightExpr = parseExpression();

            // Ensure closing parenthesis
            if (current >= tokens.size() || tokens[current].type != TokenType::RIGHT_PAREN) {
                throw runtime_error("ERROR (unexpected token) : ')' expected at line "
                                    + to_string(tokens[current].line) + " column "
                                    + to_string(tokens[current].column));
            }
            current++; // skip ')'

            // Build the result - first build the left side chain
            shared_ptr<Node> result = rightExpr;
            for (int i = restExprs.size() - 1; i >= 0; i--) {
                result = make_shared<DotNode>(restExprs[i], result);
            }
            // Then add the first expression at the beginning
            return make_shared<DotNode>(firstExpr, result);
        }
            // It's a proper list (not a dotted pair)
        else {
            if (current >= tokens.size() || tokens[current].type != TokenType::RIGHT_PAREN) {
                throw runtime_error("ERROR (unexpected token) : ')' expected at line "
                                    + to_string(tokens[current].line) + " column "
                                    + to_string(tokens[current].column));
            }
            current++; // skip ')'

            // Build a proper list - all nodes linked with the last pointing to nil
            shared_ptr<Node> result = make_shared<AtomNode>(TokenType::NIL, "nil");
            for (int i = restExprs.size() - 1; i >= 0; i--) {
                result = make_shared<DotNode>(restExprs[i], result);
            }
            // Add the first expression
            return make_shared<DotNode>(firstExpr, result);
        }
    }
};

class Scanner {
    string input;
    size_t current = 0;
    size_t line = 1;
    size_t column = 1;
    struct Position {
        size_t line;
        size_t column;
        Position(size_t l, size_t c) : line(l), column(c) {}
    };

public:
    Scanner(string input) : input(std::move(input)) {}

    vector<Token> scanTokens() {
        vector<Token> tokens;
        while (!isAtEnd()) {
            skipWhitespace();
            if (isAtEnd())
                break;
            Position start{line, column};
            char c = advance();
            switch (c) {
                case '(':
                    tokens.emplace_back(TokenType::LEFT_PAREN, "(", start.line, start.column);
                    break;
                case ')':
                    tokens.emplace_back(TokenType::RIGHT_PAREN, ")", start.line, start.column);
                    break;
                case '"':
                    tokens.push_back(scanString(start));
                    break;
                case '\'':
                    tokens.emplace_back(TokenType::QUOTE, "'", start.line, start.column);
                    break;
                case ';':
                    while (!isAtEnd() && peek() != '\n')
                        advance();
                    break;
                default:
                    tokens.push_back(scanToken(c, start));
                    break;
            }
        }
        tokens.emplace_back(TokenType::EOF_TOKEN, "", line, column);
        return tokens;
    }

private:
    Token scanString(const Position& start) {
        string value;
        value += '"';
        while (!isAtEnd() && peek() != '"') {
            if (peek() == '\\') {
                advance();
                if (isAtEnd())
                    throw EOFException();
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
                if (peek() == '\n')
                    throw runtime_error(
                            "ERROR (no closing quote) : END-OF-LINE encountered at Line " +
                            to_string(start.line) +
                            " Column " +
                            to_string(start.column)
                    );
                value += advance();
            }
        }

        if (isAtEnd())
            throw EOFException();

        value += '"';

        advance();
        return Token(TokenType::STRING, value, start.line, start.column);
    }

    Token scanToken(char first, const Position& start) {
        string value(1, first);
        while (!isAtEnd() && isSymbolChar(peek()))
            value += advance();

        if (isNumber(value))
            return scanNumber(value, start);
        else
            return scanSymbol(value, start);
    }

    Token scanNumber(const string& value, const Position& start) {
        bool isFloat = value.find('.') != string::npos;
        return isFloat ? Token(TokenType::FLOAT, value, start.line, start.column)
                       : Token(TokenType::INT, value, start.line, start.column);
    }

    Token scanSymbol(const string& value, const Position& start) {
        if (value == "nil")
            return Token(TokenType::NIL, "nil", start.line, start.column);
        if (value == "#t")
            return Token(TokenType::T, "#t", start.line, start.column);
        return Token(TokenType::SYMBOL, value, start.line, start.column);
    }

    static bool isNumber(const string& value) {
        bool hasDigit = false;
        bool hasDot = false;
        size_t start = 0;

        // Check for optional leading + or -
        if (value[0] == '+' || value[0] == '-') {
            start = 1;
        }

        for (size_t i = start; i < value.length(); ++i) {
            char c = value[i];
            if (isdigit(c)) {
                hasDigit = true;
            } else if (c == '.') {
                if (hasDot) {
                    return false;
                }
                hasDot = true;
            } else {
                return false;
            }
        }
        return hasDigit;
    }

    static bool isSymbolChar(char c) {
        return isalpha(c) || isdigit(c) || string("!?-+*/><=_~#@$%.").find(c) != string::npos;
    }

    void skipWhitespace() {
        while (!isAtEnd()) {
            char c = peek();
            if (isspace(c)) {
                if (c == '\n') {
                    line++;
                    column = 1;
                } else {
                    column++;
                }
                advance();
            } else
                break;
        }
    }

    char advance() {
        if (isAtEnd())
            return '\0';
        column++;
        return input[current++];
    }

    char peek() const {
        return isAtEnd() ? '\0' : input[current];
    }

    bool isAtEnd() const {
        return current >= input.length();
    }
};

class Printer {
public:
    static string print(const shared_ptr<Node>& node) {
        if (!node)
            return "";

        if (auto atom = dynamic_pointer_cast<AtomNode>(node))
            return printAtom(atom);

        if (auto quote = dynamic_pointer_cast<QuoteNode>(node))
            return "'" + print(quote->getExpression());

        if (auto dot = dynamic_pointer_cast<DotNode>(node))
            return printList(dot, 0);

        return "";
    }

private:
    static string printAtom(const shared_ptr<AtomNode>& atom) {
        string value = atom->toString();
        if (value == "nil" || value == "()" || value == "#f")
            return "nil\n";
        if (value == "t" || value == "#t")
            return "#t\n";
        return value + "\n";
    }

    static string printList(const shared_ptr<DotNode>& node, int indent) {
        string result = "( ";
        shared_ptr<Node> current = node;
        bool first = true;
        int M = indent + 1;

        while (true) {
            if (auto dot = dynamic_pointer_cast<DotNode>(current)) {
                if (!first) {
                    result += " " + string(M, ' ');
                }
                result += print(dot->getLeft());
                first = false;
                current = dot->getRight();
            } else {
                if (auto atom = dynamic_pointer_cast<AtomNode>(current)) {
                    if (atom->getValue() == "nil")
                        break;
                    else {
                        result += " " + string(M, ' ') + ".\n" + string(M, ' ') + print(current);
                        break;
                    }
                } else {
                    result += " " + string(M, ' ') + print(current);
                    break;
                }
            }
        }
        result += string(indent, ' ') + ")";
        return result;
    }
};

// Helper function to check whether the S-expression is exactly (exit)
bool isExitExpression(const shared_ptr<Node>& node) {
    // We assume (exit) is represented as a proper list with one element "exit"
    auto dot = dynamic_pointer_cast<DotNode>(node);
    if (!dot) return false;
    auto left = dot->getLeft();
    auto right = dot->getRight();
    auto atomLeft = dynamic_pointer_cast<AtomNode>(left);
    auto atomRight = dynamic_pointer_cast<AtomNode>(right);
    if (!atomLeft || !atomRight) return false;
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
                // Check for EOF
                if (cin.peek() == EOF) {
                    // Should not be empty, unless the last line is not a complete expression
                    if (!buffer.empty()) {
                        Scanner sc(buffer);
                        auto tokens = sc.scanTokens();
                    }

                    // Is EOF, print error message and exit the program
                    else
                        throw EOFException();

                    break;
                }

                // Read input line by line
                string line;
                getline(cin, line);

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
                if (balance > 0)
                    continue;

                // Parse the tokens if the expression is complete
                Parser parser(tokens);
                while (parser.hasMore()) {
                    shared_ptr<Node> ast = parser.parse();

                    cout << endl << "> ";
                    // If the parsed expression is "(exit)", throw exception to exit
                    if (isExitExpression(ast))
                        throw ExitException();

                    cout << Printer::print(ast);
                }

                buffer.clear();

            } catch (const runtime_error& e) {
                cout << e.what();
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
