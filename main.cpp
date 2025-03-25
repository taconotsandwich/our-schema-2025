#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

// Scanner class to scan the input and return the tokens
class Scanner {
public:
    int line, column;

    explicit Scanner() {
        line = 1;
        column = 0;
    }

    // Scan the input and return the tokens
    Token scanToken() {
        skipWhitespace();
        char c = advance();
        switch (c) {
            case '(':
                return Token(TokenType::LEFT_PAREN, "(", line, column);
            case ')':
                return Token(TokenType::RIGHT_PAREN, ")", line, column);
            case '"':
                return scanString();
            case '\'':
                return Token(TokenType::QUOTE, "'", line, column);
            case ';':
                while (peek() != '\n')
                    advance();
                advance();
                return scanToken();
            default:
                return scanToken(c);
        }
    }

    Token peekToken() {
        buffer.clear();
        int tLine = line, tColumn = column;
        Token token = scanToken();
        line = tLine, column = tColumn;
        reverse(buffer.begin(), buffer.end());
        for (char c : buffer)
            putback(c);
        return token;
    }

    static void skipIfLineLeftoverEmpty() {
        string line;
        getline(cin, line);
        line = splitStringAtFirstSemicolon(line);
        line += '\n';
        if (isOnlyWhitespace(line))
            return;

        else {
            reverse(line.begin(), line.end());
            for (char c : line)
                putback(c);
        }
    }

private:
    string buffer;

    // Scan the string token
    Token scanString() {
        string value;
        value += '"';
        while (peek() != '"') {
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
                    throw RuntimeException(
                            "ERROR (no closing quote) : END-OF-LINE encountered at Line " +
                            to_string(line) + " Column " + to_string(column + 1)
                    );
                }
                value += advance();
            }
        }

        value += advance();
        return Token(TokenType::STRING, value, line, column);
    }

    // Scan the token
    Token scanToken(char first) {
        string value(1, first);
        while (isSymbolChar(peek()))
            value += advance();

        if (isNumber(value))
            return scanNumber(value);

        else
            return scanSymbol(value);
    }

    // Scan the number token
    [[nodiscard]] Token scanNumber(const string& value) const {
        bool isFloat = value.find('.') != string::npos;
        return isFloat ?
            Token(TokenType::FLOAT, value, line, column) :
                Token(TokenType::INT, value, line, column);
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
        return isalpha(c) || isdigit(c) || string("!?-+*/><=_~#@$%.^&,").find(c) != string::npos;
    }

    // Advance the current position and return the character
    char advance() {
        if (cin.peek() == EOF)
            throw EOFException();

        else if (peek() == '\n') {
            line++;
            column = 0;
        }

        else
            column++;

        buffer += peek();

        return static_cast<char>(cin.get());
    }

    // Return the current character without advancing the position
    [[nodiscard]] static char peek() {
        return static_cast<char>(cin.peek());
    }

    // Skip the whitespace characters
    void skipWhitespace() {
        while (true) {
            char c = peek();
            if (isspace(c))
                advance();

            else
                break;
        }
    }

    static bool isOnlyWhitespace(const string& str) {
        return all_of(str.begin(), str.end(), [](unsigned char c) { return std::isspace(c); });
    }

    static string splitStringAtFirstSemicolon(const string& line) {
        size_t pos = line.find(';');

        if (pos != std::string::npos) {
            string firstPart = line.substr(0, pos), secondPart = line.substr(pos + 1);

            return firstPart;
        }

        return line;
    }

    static void putback(char c) {
        cin.putback(c);
    }
};

// Parser class to parse the tokens and build the AST
class Parser {
public:
    explicit Parser() {
        scanner = Scanner();
    }

    shared_ptr<Node> parse() {
        return parseExpression();
    }

private:
    Scanner scanner;

    // Parse the expression
    shared_ptr<Node> parseExpression() {
        Token token = scanner.scanToken();
        switch (token.type) {
            case TokenType::LEFT_PAREN:
                return parseSExpression();
            case TokenType::QUOTE:
                return make_shared<QuoteNode>(parseExpression());
            case TokenType::INT:
            case TokenType::FLOAT:
            case TokenType::STRING:
            case TokenType::SYMBOL:
            case TokenType::NIL:
            case TokenType::T:
                return make_shared<AtomNode>(token.type, token.value);
            default:
                throw RuntimeException(
                        "ERROR (unexpected token) : atom or '(' expected when token at Line " +
                        to_string(token.line) + " Column " + to_string(token.column) +
                        " is >>" + token.value + "<<"
                        );
        }
    }

    // Parse the S-expression
    // <S-exp> ::= <ATOM>
    //    | LEFT-PAREN <S-exp> { <S-exp> } [ DOT <S-exp> ] RIGHT-PAREN
    //    | QUOTE <S-exp>
    shared_ptr<Node> parseSExpression() {
        // Handle empty list case: ()
        if (scanner.peekToken().type == TokenType::RIGHT_PAREN) {
            scanner.scanToken();
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }

        // Parse the first expression (required)
        auto firstExpr = parseExpression();

        // If we hit the closing paren, it's a single element list
        if (scanner.peekToken().type == TokenType::RIGHT_PAREN) {
            scanner.scanToken(); // skip ')'
            // Create a proper list with one element
            return make_shared<DotNode>(firstExpr, make_shared<AtomNode>(TokenType::NIL, "nil"));
        }

        // Parse remaining expressions until DOT or RIGHT_PAREN
        vector<shared_ptr<Node>> restExpressions;
        while (scanner.peekToken().type != TokenType::RIGHT_PAREN &&
               scanner.peekToken().type != TokenType::DOT)
            restExpressions.push_back(parseExpression());

        // Check if we have a dotted pair
        if (scanner.peekToken().type == TokenType::DOT) {
            scanner.scanToken(); // skip the dot

            // Parse the expression after the dot
            auto rightExpr = parseExpression();

            // Ensure closing parenthesis
            if (scanner.peekToken().type != TokenType::RIGHT_PAREN) {
                Token token = scanner.scanToken();
                throw RuntimeException(
                        "ERROR (unexpected token) : ')' expected when token at Line " +
                        to_string(token.line) + " Column " + to_string(token.column) +
                        " is >>" + token.value + "<<"
                        );
            }
            scanner.scanToken(); // skip ')'

            // Build the result - first build the left side chain
            shared_ptr<Node> result = rightExpr;
            for (int i = restExpressions.size() - 1; i >= 0; i--)
                result = make_shared<DotNode>(restExpressions[i], result);

            // Then add the first expression at the beginning
            return make_shared<DotNode>(firstExpr, result);
        }

        // It's a proper list (not a dotted pair)
        else {
            if (scanner.peekToken().type != TokenType::RIGHT_PAREN) {
                Token token = scanner.scanToken();
                throw RuntimeException(
                        "ERROR (unexpected token) : ')' expected when token at Line " +
                        to_string(token.line) + " Column " + to_string(token.column) +
                        " is >>" + token.value + "<<" );
            }
            scanner.scanToken(); // skip ')'

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

// Global environment for user-defined bindings.
unordered_map<string, shared_ptr<Node>> globalEnv;
// Protected built-in names.
unordered_set<string> builtins = {
        "cons", "car", "cdr", "list", "quote", "define", "if", "cond", "begin",
        "+", "-", "*", "/", "clean-environment", "exit",
        "atom?", "pair?", "list?", "null?", "integer?", "real?", "number?", "string?",
        "boolean?", "symbol?",
        "not", "and", "or",
        ">", ">=", "<", "<=", "=",
        "string-append", "string>?", "string<?", "string=?",
        "eqv?", "equal?"
};

// Unroll a proper list into a vector of nodes.
vector<shared_ptr<Node>> unrollList(const shared_ptr<Node>& list) {
    vector<shared_ptr<Node>> elems;
    auto curr = list;
    while (true) {
        if (auto dot = dynamic_pointer_cast<DotNode>(curr)) {
            elems.push_back(dot->getLeft());
            curr = dot->getRight();
        } else {
            auto atom = dynamic_pointer_cast<AtomNode>(curr);
            if (atom && atom->getValue() == "nil")
                break;
            else
                throw RuntimeException("ERROR (non-list) : " + Printer::print(list));
        }
    }
    return elems;
}

// Extract a number from an evaluated node.
double getNumber(const shared_ptr<Node>& node) {
    auto atom = dynamic_pointer_cast<AtomNode>(node);
    if (!atom || (atom->getType() != TokenType::INT && atom->getType() != TokenType::FLOAT))
        throw RuntimeException("ERROR (incorrect argument type) : " + node->toString());
    return stod(atom->getValue());
}

// Helper function to check if a node represents a proper list.
// A proper list is either the empty list or a chain of DotNodes ending in an AtomNode with value "nil".
bool isProperList(const shared_ptr<Node>& node) {
    auto current = node;
    while (true) {
        if (auto dot = dynamic_pointer_cast<DotNode>(current)) {
            current = dot->getRight();
        } else {
            if (auto atom = dynamic_pointer_cast<AtomNode>(current))
                return (atom->getValue() == "nil");
            return false;
        }
    }
}

// Helper function for deep, structural equality comparison.
bool structuralEqual(const shared_ptr<Node>& n1, const shared_ptr<Node>& n2) {
    // If both are atoms, compare their types and values (with special handling for strings)
    auto atom1 = dynamic_pointer_cast<AtomNode>(n1);
    auto atom2 = dynamic_pointer_cast<AtomNode>(n2);
    if (atom1 && atom2) {
        if (atom1->getType() == TokenType::STRING && atom2->getType() == TokenType::STRING) {
            string s1 = atom1->getValue();
            string s2 = atom2->getValue();
            // Remove surrounding quotes if present.
            if (s1.size() >= 2 && s1.front() == '"' && s1.back() == '"')
                s1 = s1.substr(1, s1.size() - 2);
            if (s2.size() >= 2 && s2.front() == '"' && s2.back() == '"')
                s2 = s2.substr(1, s2.size() - 2);
            return s1 == s2;
        }
        return (atom1->getType() == atom2->getType() && atom1->getValue() == atom2->getValue());
    }
    // If both are pairs (lists), unroll them and compare element by element.
    auto dot1 = dynamic_pointer_cast<DotNode>(n1);
    auto dot2 = dynamic_pointer_cast<DotNode>(n2);
    if (dot1 && dot2) {
        vector<shared_ptr<Node>> list1 = unrollList(n1);
        vector<shared_ptr<Node>> list2 = unrollList(n2);
        if (list1.size() != list2.size())
            return false;
        for (size_t i = 0; i < list1.size(); ++i)
            if (!structuralEqual(list1[i], list2[i]))
                return false;
        return true;
    }
    // Otherwise, they are not structurally equal.
    return false;
}

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

shared_ptr<Node> EvalSExp(bool isGlobalLayer, const shared_ptr<Node>& node) {
    // For quoted expressions using the shorthand: return the inner expression unchanged.
    if (auto quote = dynamic_pointer_cast<QuoteNode>(node))
        return quote->getExpression();

    // For atoms: if a symbol, look it up; otherwise, return the atom itself.
    if (auto atom = dynamic_pointer_cast<AtomNode>(node)) {
        if (atom->getType() == TokenType::SYMBOL) {
            string sym = atom->getValue();
            // If the symbol is bound in the environment, return its binding.
            // If the symbol is bound in the built environment, return a placeholder
            // Otherwise, throw error.
            if (globalEnv.find(sym) != globalEnv.end())
                return globalEnv[sym];

            else if (builtins.find(sym) != builtins.end())
                return make_shared<AtomNode>(TokenType::SYMBOL, "#<procedure " + sym + ">");

            else
                throw RuntimeException("ERROR (unbound symbol) : " + node->toString());
        }
        return node;
    }

    // For list expressions (function applications).
    if (dynamic_pointer_cast<DotNode>(node)) {
        vector<shared_ptr<Node>> elems = unrollList(node);

        if (elems.empty())
            throw RuntimeException("ERROR (attempt to apply non-function) : " + node->toString());

        // Evaluate the operator.
        auto opNode = elems[0];
        string op;
        if (auto atomOp = dynamic_pointer_cast<AtomNode>(opNode))
            op = atomOp->getValue();

        else
            op = dynamic_pointer_cast<AtomNode>(EvalSExp(false, opNode))->getValue();

        // If symbol is not defined in environment, throw error
        if (globalEnv.find(op) == globalEnv.end() && builtins.find(op) == builtins.end())
            throw RuntimeException("ERROR (attempt to apply non-function) : " + op);

        while (globalEnv.find(op) != globalEnv.end() && builtins.find(op) == builtins.end())
            op = globalEnv[op]->toString();

        if (op.find("#<procedure ") != string::npos) {
            const string prefix = "#<procedure ";
            const string suffix = ">";
            op = op.substr(prefix.size(), op.size() - prefix.size() - suffix.size());
        }

        // --- Special Forms ---
        if (op == "define") {
            if (!isGlobalLayer)
                throw RuntimeException("ERROR (level of DEFINE)");

            if (elems.size() != 3)
                throw RuntimeException("ERROR (DEFINE format) : " + Printer::print(node));

            auto symNode = elems[1];

            if (auto atomSym = dynamic_pointer_cast<AtomNode>(symNode)) {
                if (atomSym->getType() != TokenType::SYMBOL)
                    throw RuntimeException("ERROR (DEFINE format) : " + Printer::print(node));
                string symName = atomSym->getValue();
                if (builtins.find(symName) != builtins.end())
                    throw RuntimeException("ERROR (DEFINE format) : " + Printer::print(node));
                globalEnv[symName] = EvalSExp(false, elems[2]);
                return make_shared<AtomNode>(TokenType::SYMBOL, symName + " defined");
            }

            else
                throw RuntimeException("ERROR (DEFINE format) : " + Printer::print(node));
        }

        else if (op == "if") {
            if (elems.size() != 3 && elems.size() != 4)
                throw RuntimeException("ERROR (if format) : ( if ... )");
            auto condVal = EvalSExp(false, elems[1]);
            bool condTrue;
            if (auto atomCond = dynamic_pointer_cast<AtomNode>(condVal))
                condTrue = (atomCond->getValue() != "nil");
            else
                condTrue = true;
            if (condTrue)
                return EvalSExp(false, elems[2]);
            else {
                if (elems.size() == 4)
                    return EvalSExp(false, elems[3]);
                else
                    throw RuntimeException("ERROR (no return value) : ( if nil " + elems[2]->toString() + ")");
            }
        }
        else if (op == "cond") {
            for (size_t i = 1; i < elems.size(); i++) {
                vector<shared_ptr<Node>> clauseElems = unrollList(elems[i]);
                if (clauseElems.empty())
                    throw RuntimeException("ERROR (COND format) : ( cond ... )");

                bool testMatches = false;
                bool isLastClause = (i == elems.size() - 1);
                auto firstElem = clauseElems[0];

                // If this is the last clause and its test is literally the symbol "else",
                // then match unconditionally.
                if (isLastClause &&
                    dynamic_pointer_cast<AtomNode>(firstElem) &&
                    dynamic_pointer_cast<AtomNode>(firstElem)->getValue() == "else") {
                    testMatches = true;
                } else {
                    auto testVal = EvalSExp(false, firstElem);
                    if (auto atomTest = dynamic_pointer_cast<AtomNode>(testVal))
                        testMatches = (atomTest->getValue() != "nil");
                    else
                        testMatches = true;
                }

                if (testMatches) {
                    if (clauseElems.size() == 1)
                        throw RuntimeException("ERROR (no return value) : ( cond ... )");
                    shared_ptr<Node> res;
                    for (size_t j = 1; j < clauseElems.size(); j++)
                        res = EvalSExp(false, clauseElems[j]);
                    return res;
                }
            }
            throw RuntimeException("ERROR (no return value) : ( cond ... )");
        }
        else if (op == "begin") {
            if (elems.size() < 2)
                throw RuntimeException("ERROR (begin format) : ( begin ... )");
            shared_ptr<Node> res;
            for (size_t i = 1; i < elems.size(); i++)
                res = EvalSExp(false, elems[i]);
            return res;
        }
        else if (op == "clean-environment") {
            if (!isGlobalLayer)
                throw RuntimeException("ERROR (level of CLEAN-ENVIRONMENT)");
            if (elems.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : clean-environment");
            globalEnv.clear();
            return make_shared<AtomNode>(TokenType::SYMBOL, "environment cleaned");
        }
        else if (op == "exit") {
            if (elems.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : exit");
            throw ExitException();
        }
        else if (op == "quote") {
            if (elems.size() != 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : quote");
            return elems[1];
        }
        else if (op == "not") {
            if (elems.size() != 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : not");
            auto testVal = EvalSExp(false, elems[1]);
            bool truth;
            if (auto atom = dynamic_pointer_cast<AtomNode>(testVal))
                truth = (atom->getValue() != "nil");
            else
                truth = true;
            return truth ? make_shared<AtomNode>(TokenType::NIL, "nil")
                         : make_shared<AtomNode>(TokenType::T, "#t");
        }
        else if (op == "and") {
            if (elems.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : and");
            shared_ptr<Node> lastEvaluated;
            for (size_t i = 1; i < elems.size(); i++) {
                lastEvaluated = EvalSExp(false, elems[i]);
                bool truth;
                if (auto atom = dynamic_pointer_cast<AtomNode>(lastEvaluated))
                    truth = (atom->getValue() != "nil");
                else
                    truth = true;
                if (!truth)
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
            }
            return lastEvaluated;
        }
        else if (op == "or") {
            if (elems.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : or");
            for (size_t i = 1; i < elems.size(); i++) {
                auto val = EvalSExp(false, elems[i]);
                bool truth;
                if (auto atom = dynamic_pointer_cast<AtomNode>(val))
                    truth = (atom->getValue() != "nil");
                else
                    truth = true;
                if (truth)
                    return val;
            }
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }

        // Evaluate operands for the remaining built-in functions.
        vector<shared_ptr<Node>> args;
        for (size_t i = 1; i < elems.size(); i++)
            args.push_back(EvalSExp(false, elems[i]));

        // --- Arithmetic Operations ---
        if (op == "+") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : +");

            double sum = 0.0;
            bool forceFloat = false;
            for (auto arg : args) {
                double n = getNumber(arg);
                sum += n;
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg))
                    if (atom->getType() == TokenType::FLOAT)
                        forceFloat = true;
            }

            if (forceFloat)
                return make_shared<AtomNode>(TokenType::FLOAT, to_string(sum));
            else
                return make_shared<AtomNode>(TokenType::INT, to_string(static_cast<int>(sum)));
        }
        else if (op == "-") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : -");

            double result = getNumber(args[0]);
            bool forceFloat = false;
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                if (atom->getType() == TokenType::FLOAT)
                    forceFloat = true;

            for (size_t i = 1; i < args.size(); i++) {
                double n = getNumber(args[i]);
                result -= n;
                if (auto atom = dynamic_pointer_cast<AtomNode>(args[i]))
                    if (atom->getType() == TokenType::FLOAT)
                        forceFloat = true;
            }

            if (forceFloat)
                return make_shared<AtomNode>(TokenType::FLOAT, to_string(result));
            else
                return make_shared<AtomNode>(TokenType::INT, to_string(static_cast<int>(result)));
        }
        else if (op == "*") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : *");

            double product = 1.0;
            bool forceFloat = false;
            for (auto arg : args) {
                double n = getNumber(arg);
                product *= n;
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg))
                    if (atom->getType() == TokenType::FLOAT)
                        forceFloat = true;
            }

            if (forceFloat)
                return make_shared<AtomNode>(TokenType::FLOAT, to_string(product));
            else
                return make_shared<AtomNode>(TokenType::INT, to_string(static_cast<int>(product)));
        }
        else if (op == "/") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : /");

            double result = getNumber(args[0]);
            bool forceFloat = false;
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                if (atom->getType() == TokenType::FLOAT)
                    forceFloat = true;

            for (size_t i = 1; i < args.size(); i++) {
                double divisor = getNumber(args[i]);
                if (divisor == 0)
                    throw RuntimeException("ERROR (division by zero) : /");
                result /= divisor;
                if (auto atom = dynamic_pointer_cast<AtomNode>(args[i]))
                    if (atom->getType() == TokenType::FLOAT)
                        forceFloat = true;
            }

            if (forceFloat)
                return make_shared<AtomNode>(TokenType::FLOAT, to_string(result));
            else
                return make_shared<AtomNode>(TokenType::INT, to_string(static_cast<int>(result)));
        }

        else if (op == "cons") {
            if (args.size() != 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : cons");
            return make_shared<DotNode>(args[0], args[1]);
        }
        else if (op == "car") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : car");
            auto pairNode = dynamic_pointer_cast<DotNode>(args[0]);
            if (!pairNode)
                throw RuntimeException("ERROR (car with incorrect argument type) : " + args[0]->toString());
            return pairNode->getLeft();
        }
        else if (op == "cdr") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : cdr");
            auto pairNode = dynamic_pointer_cast<DotNode>(args[0]);
            if (!pairNode)
                throw RuntimeException("ERROR (cdr with incorrect argument type) : " + args[0]->toString());
            return pairNode->getRight();
        }
        else if (op == "list") {
            shared_ptr<Node> list = make_shared<AtomNode>(TokenType::NIL, "nil");
            for (int i = args.size() - 1; i >= 0; i--)
                list = make_shared<DotNode>(args[i], list);
            return list;
        }

        else if (op == "atom?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : atom?");
            return dynamic_pointer_cast<DotNode>(args[0])
                   ? make_shared<AtomNode>(TokenType::NIL, "nil")
                   : make_shared<AtomNode>(TokenType::T, "#t");
        }
        else if (op == "pair?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : pair?");
            return dynamic_pointer_cast<DotNode>(args[0])
                   ? make_shared<AtomNode>(TokenType::T, "#t")
                   : make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "list?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : list?");
            bool proper = false;
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                proper = (atom->getValue() == "nil");
            else
                proper = isProperList(args[0]);
            return proper ? make_shared<AtomNode>(TokenType::T, "#t")
                          : make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "null?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : null?");
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return (atom->getValue() == "nil")
                       ? make_shared<AtomNode>(TokenType::T, "#t")
                       : make_shared<AtomNode>(TokenType::NIL, "nil");
            else
                return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "integer?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : integer?");
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return (atom->getType() == TokenType::INT)
                       ? make_shared<AtomNode>(TokenType::T, "#t")
                       : make_shared<AtomNode>(TokenType::NIL, "nil");
            else
                return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "real?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : real?");
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return ((atom->getType() == TokenType::INT || atom->getType() == TokenType::FLOAT)
                        ? make_shared<AtomNode>(TokenType::T, "#t")
                        : make_shared<AtomNode>(TokenType::NIL, "nil"));
            else
                return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "number?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : number?");
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return ((atom->getType() == TokenType::INT || atom->getType() == TokenType::FLOAT)
                        ? make_shared<AtomNode>(TokenType::T, "#t")
                        : make_shared<AtomNode>(TokenType::NIL, "nil"));
            else
                return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "string?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : string?");
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return (atom->getType() == TokenType::STRING
                        ? make_shared<AtomNode>(TokenType::T, "#t")
                        : make_shared<AtomNode>(TokenType::NIL, "nil"));
            else
                return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "boolean?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : boolean?");
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0])) {
                if (atom->getType() == TokenType::T || atom->getType() == TokenType::NIL)
                    return make_shared<AtomNode>(TokenType::T, "#t");
                else
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
            } else
                return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "symbol?") {
            if (args.size() != 1)
                throw RuntimeException("ERROR (incorrect number of arguments) : symbol?");
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return (atom->getType() == TokenType::SYMBOL
                        ? make_shared<AtomNode>(TokenType::T, "#t")
                        : make_shared<AtomNode>(TokenType::NIL, "nil"));
            else
                return make_shared<AtomNode>(TokenType::NIL, "nil");
        }

        else if (op == ">") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : >");
            double prev = getNumber(args[0]);
            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(args[i]);
                if (!(prev > curr))
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                prev = curr;
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        else if (op == ">=") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : >=");
            double prev = getNumber(args[0]);
            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(args[i]);
                if (!(prev >= curr))
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                prev = curr;
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        else if (op == "<") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : <");
            double prev = getNumber(args[0]);
            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(args[i]);
                if (!(prev < curr))
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                prev = curr;
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        else if (op == "<=") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : <=");
            double prev = getNumber(args[0]);
            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(args[i]);
                if (!(prev <= curr))
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                prev = curr;
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        else if (op == "=") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : =");
            double first = getNumber(args[0]);
            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(args[i]);
                if (first != curr)
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }

        else if (op == "string-append") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : string-append");
            string result;
            for (auto & arg : args) {
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg)) {
                    if (atom->getType() != TokenType::STRING)
                        throw RuntimeException("ERROR (string-append with incorrect argument type) : " + atom->toString());
                    string str = atom->getValue();
                    if (str.length() >= 2 && str.front() == '"' && str.back() == '"')
                        str = str.substr(1, str.length() - 2);
                    result += str;
                } else {
                    throw RuntimeException("ERROR (string-append with incorrect argument type)");
                }
            }
            string finalStr = "\"" + result + "\"";
            return make_shared<AtomNode>(TokenType::STRING, finalStr);
        }
        else if (op == "string>?") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : string>?");
            vector<string> strs;
            for (auto & arg : args) {
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg)) {
                    if (atom->getType() != TokenType::STRING)
                        throw RuntimeException("ERROR (string>? with incorrect argument type) : " + atom->toString());
                    string str = atom->getValue();
                    if (str.length() >= 2 && str.front() == '"' && str.back() == '"')
                        str = str.substr(1, str.length() - 2);
                    strs.push_back(str);
                } else {
                    throw RuntimeException("ERROR (string>? with incorrect argument type)");
                }
            }
            bool resultBool = true;
            for (size_t i = 0; i < strs.size() - 1; i++) {
                if (!(strs[i] > strs[i+1])) {
                    resultBool = false;
                    break;
                }
            }
            return resultBool ? make_shared<AtomNode>(TokenType::T, "#t")
                              : make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "string<?") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : string<?");
            vector<string> strs;
            for (auto & arg : args) {
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg)) {
                    if (atom->getType() != TokenType::STRING)
                        throw RuntimeException("ERROR (string<? with incorrect argument type) : " + atom->toString());
                    string str = atom->getValue();
                    if (str.length() >= 2 && str.front() == '"' && str.back() == '"')
                        str = str.substr(1, str.length() - 2);
                    strs.push_back(str);
                } else {
                    throw RuntimeException("ERROR (string<? with incorrect argument type)");
                }
            }
            bool resultBool = true;
            for (size_t i = 0; i < strs.size() - 1; i++) {
                if (!(strs[i] < strs[i+1])) {
                    resultBool = false;
                    break;
                }
            }
            return resultBool ? make_shared<AtomNode>(TokenType::T, "#t")
                              : make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "string=?") {
            if (args.size() < 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : string=?");
            vector<string> strs;
            for (auto & arg : args) {
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg)) {
                    if (atom->getType() != TokenType::STRING)
                        throw RuntimeException("ERROR (string=? with incorrect argument type) : " + atom->toString());
                    string str = atom->getValue();
                    if (str.length() >= 2 && str.front() == '"' && str.back() == '"')
                        str = str.substr(1, str.length() - 2);
                    strs.push_back(str);
                } else {
                    throw RuntimeException("ERROR (string=? with incorrect argument type)");
                }
            }
            bool allEqual = true;
            for (size_t i = 1; i < strs.size(); i++) {
                if (strs[i] != strs[0]) {
                    allEqual = false;
                    break;
                }
            }
            return allEqual ? make_shared<AtomNode>(TokenType::T, "#t")
                            : make_shared<AtomNode>(TokenType::NIL, "nil");
        }

        else if (op == "eqv?") {
            if (args.size() != 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : eqv?");
            if (args[0] == args[1])
                return make_shared<AtomNode>(TokenType::T, "#t");
            auto atom1 = dynamic_pointer_cast<AtomNode>(args[0]);
            auto atom2 = dynamic_pointer_cast<AtomNode>(args[1]);
            if (atom1 && atom2) {
                if (atom1->getType() == TokenType::STRING || atom2->getType() == TokenType::STRING)
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                if (atom1->getType() == atom2->getType() && atom1->getValue() == atom2->getValue())
                    return make_shared<AtomNode>(TokenType::T, "#t");
                else
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
            }
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        else if (op == "equal?") {
            if (args.size() != 2)
                throw RuntimeException("ERROR (incorrect number of arguments) : equal?");
            auto atom1 = dynamic_pointer_cast<AtomNode>(args[0]);
            auto atom2 = dynamic_pointer_cast<AtomNode>(args[1]);
            if (atom1 && atom2) {
                if (atom1->getType() == TokenType::STRING && atom2->getType() == TokenType::STRING) {
                    string s1 = atom1->getValue();
                    string s2 = atom2->getValue();
                    if (s1.size() >= 2 && s1.front() == '"' && s1.back() == '"')
                        s1 = s1.substr(1, s1.size() - 2);
                    if (s2.size() >= 2 && s2.front() == '"' && s2.back() == '"')
                        s2 = s2.substr(1, s2.size() - 2);
                    return (s1 == s2) ? make_shared<AtomNode>(TokenType::T, "#t")
                                      : make_shared<AtomNode>(TokenType::NIL, "nil");
                }
                else if (atom1->getType() == atom2->getType() && atom1->getValue() == atom2->getValue())
                    return make_shared<AtomNode>(TokenType::T, "#t");
                else
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
            }
            if (structuralEqual(args[0], args[1]))
                return make_shared<AtomNode>(TokenType::T, "#t");
            else
                return make_shared<AtomNode>(TokenType::NIL, "nil");
        }

        else
            throw RuntimeException("ERROR (unbound symbol) : " + op);
    }

    throw RuntimeException("ERROR (unknown expression) : " + node->toString());
}

int main() {
    try {
        cout << "Welcome to OurScheme!" << endl;
        cin >> g_testCase;

        // Ignore the newline after the test case input
        cin.ignore();

        while (true) {
            try {
                // Parse the tokens if the expression is complete
                Parser parser;
                shared_ptr<Node> ast = parser.parse();
                shared_ptr<Node> result = EvalSExp(true, ast);

                cout << endl << "> ";
//                // If the parsed expression is "(exit)", throw exception to exit
//                if (isExitExpression(ast))
//                    throw ExitException();

                cout << Printer::print(result) << endl;

                Scanner::skipIfLineLeftoverEmpty();
            } catch (const RuntimeException& e) {
                string line;
                getline(cin, line);
                cout << endl << "> " << e.what() << endl;
            }
        }

    } catch (const EOFException& e) {
        cout << endl << "> " << e.what();
    } catch (const ExitException& e) {
        cout << endl << "> ";
    }
    cout << endl << "Thanks for using OurScheme!";
    return 0;
}