#include <algorithm>
#include <cctype>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

int g_testCase = 0;
bool g_debugMode = false; // Set to true to enable debug prints, false to disable

class ExitException : public std::exception {
public:
    [[nodiscard]] const char* what() const noexcept override {
        return "Exit requested";
    }
};

class EOFException : public std::exception {
public:
    [[nodiscard]] const char* what() const noexcept override {
        return "EOF encountered";
    }
};

class CompileTimeException : public std::exception {
    string message;
public:
    explicit CompileTimeException(string s) : message(std::move(s)) { };

    [[nodiscard]] const char* what() const noexcept override {
        return message.c_str();
    }
};

class RunTimeException : public std::exception {
    string message;
public:
    explicit RunTimeException(string s) : message(std::move(s)) { };

    [[nodiscard]] const char* what() const noexcept override {
        return message.c_str();
    }
};

// Enum class to represent the token type
enum class TokenType {
    LEFT_PAREN, RIGHT_PAREN,
    INT, FLOAT, STRING, DOT,
    NIL, T, QUOTE, SYMBOL
};

// Token class to store the token type, value, line, and column
struct Token {
    TokenType type;
    string value;
    int line, column;
    Token(const TokenType t, string v, const int l, const int c)
            : type(t), value(std::move(v)), line(l), column(c) {}
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
        // Normalize the float output
        if (type == TokenType::FLOAT) {
            const double num = stod(value);
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
        istream::pos_type startPos = cin.tellg();
        int tLine = line, tColumn = column;
        Token token = scanToken();
        line = tLine, column = tColumn;
        cin.clear();
        cin.seekg(startPos);
        return token;
    }

    static void skipIfLineLeftoverEmpty() {
        istream::pos_type startPos = cin.tellg();

        string line;
        getline(cin, line);
        line = splitStringAtFirstSemicolon(line) + '\n';
        if (!isOnlyWhitespace(line)) {
            cin.clear();
            cin.seekg(startPos);
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
                    throw CompileTimeException(
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
        if (value == "t" || value == "#t")
            return Token(TokenType::T, "#t", line, column);
        if (value == ".")
            return Token(TokenType::DOT, ".", line, column);
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
        if (peek() == '\n') {
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
        if (!cin.good())
            throw EOFException();
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
                throw CompileTimeException(
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
        Token tToken = scanner.peekToken();
        while (tToken.type != TokenType::RIGHT_PAREN && tToken.type != TokenType::DOT) {
            restExpressions.push_back(parseExpression());
            tToken = scanner.peekToken();
        }

        // Check if we have a dotted pair
        if (scanner.peekToken().type == TokenType::DOT) {
            scanner.scanToken(); // skip the dot

            // Parse the expression after the dot
            auto rightExpr = parseExpression();

            // Ensure closing parenthesis
            if (scanner.peekToken().type != TokenType::RIGHT_PAREN) {
                const Token token = scanner.scanToken();
                throw CompileTimeException(
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
        Token token = scanner.scanToken();
        if (token.type != TokenType::RIGHT_PAREN) {
            throw CompileTimeException(
                "ERROR (unexpected token) : ')' expected when token at Line " +
                to_string(token.line) + " Column " + to_string(token.column) +
                " is >>" + token.value + "<<" );
        }

        // Build a proper list - all nodes linked with the last pointing to nil
        shared_ptr<Node> result = make_shared<AtomNode>(TokenType::NIL, "nil");
        for (int i = restExpressions.size() - 1; i >= 0; i--) {
            result = make_shared<DotNode>(restExpressions[i], result);
        }

        // Add the first expression
        return make_shared<DotNode>(firstExpr, result);
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
        while (const auto d = dynamic_pointer_cast<DotNode>(current)) {
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
        "cons", "car", "cdr", "list", "quote", "define", "if", "cond", "begin", "lambda",
        "+", "-", "*", "/", "clean-environment", "exit", "let",
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
        }
        else {
            auto atom = dynamic_pointer_cast<AtomNode>(curr);
            if (atom && atom->getValue() == "nil")
                break;
            throw CompileTimeException("ERROR (non-list) : " + Printer::print(list));
        }
    }
    return elems;
}

// Extract a number from an evaluated node.
double getNumber(const string& op, const shared_ptr<Node>& node) {
    auto atom = dynamic_pointer_cast<AtomNode>(node);
    if (!atom) {
        throw RunTimeException("ERROR (" + op + " with incorrect argument type) : " + Printer::print(node));
    }
    
    if (atom->getType() == TokenType::STRING) {
        string str = atom->getValue();
        // Remove surrounding quotes if present
        if (str.length() >= 2 && str.front() == '"' && str.back() == '"') {
            str = str.substr(1, str.length() - 2);
        }
        try {
            return stod(str);
        } catch (...) {
            return 0.0;  // If string can't be converted to number, return 0
        }
    }
    
    if (atom->getType() != TokenType::INT && atom->getType() != TokenType::FLOAT) {
        throw RunTimeException("ERROR (" + op + " with incorrect argument type) : " + Printer::print(node));
    }
    return stod(atom->getValue());
}

// Helper function to check if a node represents a proper list.
// A proper list is either the empty list (nil atom)
// or a chain of DotNodes ending in the empty list (nil atom).
bool isProperList(const shared_ptr<Node>& node) {
    auto current = node;
    while (true) {
        // Follow the cdr chain if it's a pair
        if (auto dot = dynamic_pointer_cast<DotNode>(current)) {
            current = dot->getRight();
        }
            // If it's not a pair, we've reached the end of the chain
        else {
            // Check if the end is specifically the 'nil' atom
            auto atom = dynamic_pointer_cast<AtomNode>(current);
            // It's a proper list ONLY if it ends on the atom 'nil'
            if (atom && atom->getType() == TokenType::NIL && atom->getValue() == "nil") {
                return true;
            } else {
                // Ended on a non-nil atom or something else (like a procedure)
                return false;
            }
        }
    }
    // The loop should theoretically always terminate via the 'else' branch.
    // Adding a fallback return just in case, though it shouldn't be reached.
    // return false; // Or throw an error if this state is considered impossible
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

string printEnv(const unordered_map<string, shared_ptr<Node>>& env) {
    string s = "{";
    bool first = true;
    for (const auto& pair : env) {
        if (!first) s += ", ";
        s += pair.first + ": " + Printer::print(pair.second); // Use Printer for values
        first = false;
    }
    s += "}";
    return s;
}

shared_ptr<Node> EvalSExp(bool isGlobalLayer, const shared_ptr<Node>& node,
                          unordered_map<string, shared_ptr<Node>>* env);

// Procedure class to represent user-defined functions (closures)
class Procedure final : public Node {
public:
    vector<string> params;                   // formal parameter names
    vector<shared_ptr<Node>> body;           // one or more S-expressions (AST nodes)
    // The closure environment (captures the lexical scope at definition time)
    unordered_map<string, shared_ptr<Node>> captured_env;
    string name; // Optional: store the name if defined using (define (name params) ...)

    // Constructor for lambda and direct define of lambda
    Procedure(const vector<string>& p,
              const vector<shared_ptr<Node>>& b,
              const unordered_map<string, shared_ptr<Node>>& env)
            : params(p), body(b), captured_env(env), name("lambda") {}

    // Constructor for named function defined with sugar syntax
    Procedure(const string& n,
              const vector<string>& p,
              const vector<shared_ptr<Node>>& b,
              const unordered_map<string, shared_ptr<Node>>& env)
            : name(n), params(p), body(b), captured_env(env) {}


    // When printed, show an indicative string.
    string toString(int indent = 0) const override {
        if (name == "lambda") {
            return "#<procedure lambda>";
        } else {
            return "#<procedure " + name + ">";
        }
    }

    // Exec function to execute the procedure using evaluated arguments.
    // It creates a new local environment extending the closure's environment.
    shared_ptr<Node> Exec(const vector<shared_ptr<Node>>& args) {
        if (args.size() != params.size()) {
            // Use the stored name in the error message if available
            throw RunTimeException("ERROR (incorrect number of arguments) : " + (name == "lambda" ? "lambda" : name));
        }

        // Create a new local environment by copying the *captured* environment.
        unordered_map<string, shared_ptr<Node>> localEnv = captured_env;

        // Bind each formal parameter to its corresponding *evaluated* argument.
        for (size_t i = 0; i < args.size(); i++) {
            // This binding shadows any variable with the same name in the captured_env
            localEnv[params[i]] = args[i];
        }

        if (g_debugMode) {
            cout << "[DEBUG Proc::Exec]   Created local env (" << &localEnv << "): " << printEnv(localEnv) << endl;
            // Specifically, check F
            auto it_f_check = localEnv.find("F");
            if (it_f_check != localEnv.end()) {
                cout << "[DEBUG Proc::Exec]   localEnv check: Found F -> " << Printer::print(it_f_check->second) << endl;
            } else {
                cout << "[DEBUG Proc::Exec]   localEnv check: F NOT FOUND LOCALLY!" << endl;
            }
        }

        // Evaluate each expression in the procedure body sequentially within the localEnv.
        shared_ptr<Node> result;
        for (size_t i = 0; i < body.size(); ++i) {
            const auto& expr = body[i];
            if (g_debugMode) cout << "[DEBUG Proc::Exec]   Evaluating body expr " << i << ": " << Printer::print(expr) << " using env " << &localEnv << endl; // Added env addr
            // Ensure we are definitely passing the address of localEnv
            result = EvalSExp(false, expr, &localEnv); // <<< Double-check this line uses &localEnv
            if (g_debugMode) cout << "[DEBUG Proc::Exec]   Body expr " << i << " result: " << Printer::print(result) << endl;
        }

        // The result of the last expression in the body is the return value.
        // Need error handling if the body was empty (should be caught by lambda/define format checks)
        // or if the last expression didn't return a value (handled by EvalSExp called above).
        if (!result) {
            // This case might be tricky to hit if EvalSExp correctly throws errors
            // for missing return values internally. But as a safeguard:
            throw RunTimeException("ERROR (no return value) : from body of " + (name == "lambda" ? "lambda expression" : name));
        }
        return result;
    }
};

shared_ptr<Node> EvalSExp(bool isGlobalLayer,
                          const shared_ptr<Node>& node,
                          unordered_map<string, shared_ptr<Node>>* env = &globalEnv) {
    if (g_debugMode) {
        cout << "[DEBUG EvalSExp] Called for node: " << Printer::print(node)
             << " | Env Addr: " << env
             << (env == &globalEnv ? " (Global)" : " (Local)") << endl;
    }

    // For quoted expressions using the shorthand: return the inner expression unchanged.
    if (auto quote = dynamic_pointer_cast<QuoteNode>(node))
        return quote->getExpression();

    // For atoms: if a symbol, look it up; otherwise, return the atom itself.
    if (auto atom = dynamic_pointer_cast<AtomNode>(node)) {
        if (atom->getType() == TokenType::SYMBOL) {
            string sym = atom->getValue();
            if (g_debugMode)
                cout << "[DEBUG EvalSExp] Looking up SYMBOL: " << sym << " in env " << env << endl;

            // 1. Look in the CURRENT environment ('env' parameter) FIRST.
            auto it = env->find(sym);
            if (it != env->end()) {
                // Found locally (parameter, let-binding, or captured from closure)
                if (g_debugMode)
                    cout << "[DEBUG EvalSExp]   Found '" << sym << "' in current env (" << env << "): " << Printer::print(it->second) << endl;
                return it->second;
            }

            // 2. If NOT found locally AND current env is NOT global, check GLOBAL env.
            if (env != &globalEnv) {
                auto git = globalEnv.find(sym);
                if (git != globalEnv.end()) {
                    // Found in global scope as a fallback
                    if (g_debugMode)
                        cout << "[DEBUG EvalSExp]   Found '" << sym << "' in globalEnv: " << Printer::print(git->second) << endl;
                    return git->second;
                }
            }
            // else: 'env' was already globalEnv, and it wasn't found there in step 1.

            // 3. Check for built-in functions/forms if not found in any defined env.
            if (builtins.find(sym) != builtins.end()) {
                if (g_debugMode) cout << "[DEBUG EvalSExp]   Found '" << sym << "' as BUILTIN" << endl;
                return make_shared<AtomNode>(TokenType::SYMBOL, "#<procedure " + sym + ">");
            }

            // 4. If still not found anywhere, it's an unbound symbol.
            if (g_debugMode) cout << "[DEBUG EvalSExp]   Symbol '" << sym << "' is UNBOUND" << endl;
            throw RunTimeException("ERROR (unbound symbol) : " + node->toString());

        }
        // --- Rest of atom handling ---
        if (g_debugMode) cout << "[DEBUG EvalSExp] Returning Atom: " << Printer::print(node) << endl;
        return node;
    }

    // For list expressions (function applications).
    if (dynamic_pointer_cast<DotNode>(node)) {
        // Unroll the list
        vector<shared_ptr<Node>> elems = unrollList(node);

        if (elems.empty())
            throw RunTimeException("ERROR (attempt to apply non-function) : " + node->toString());

        // Evaluate the operator
        if (g_debugMode) cout << "[DEBUG EvalSExp] Evaluating operator: " << Printer::print(elems[0]) << endl;
        const auto& opNode = EvalSExp(false, elems[0], env);

        // Check if operator is a Procedure (user-defined function)
        if (auto proc = dynamic_pointer_cast<Procedure>(opNode)) {
            if (g_debugMode) cout << "[DEBUG EvalSExp] Applying PROCEDURE: " << proc->toString() << endl;
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++) {
                if (g_debugMode) cout << "[DEBUG EvalSExp]   Evaluating arg " << i << ": " << Printer::print(elems[i]) << endl;
                args.push_back(EvalSExp(false, elems[i], env)); // Args evaluated in current env
                if (g_debugMode) cout << "[DEBUG EvalSExp]   Arg " << i << " evaluated to: " << Printer::print(args.back()) << endl;
            }
            shared_ptr<Node> result = proc->Exec(args);
            if (g_debugMode) cout << "[DEBUG EvalSExp] Procedure call returned: " << Printer::print(result) << endl;
            return result;
        }

        auto atomOp = dynamic_pointer_cast<AtomNode>(opNode);
        if (!atomOp)
            throw RunTimeException("ERROR (attempt to apply non-function) : " + Printer::print(opNode));

        if (atomOp->getType() != TokenType::SYMBOL)
            throw RunTimeException("ERROR (attempt to apply non-function) : " + atomOp->getValue());

        string op = atomOp->getValue();

        // Execute for operator in the environment
        if (op.find("#<procedure ") != string::npos) {
            const string prefix = "#<procedure ";
            const string suffix = ">";
            op = op.substr(prefix.size(), op.size() - prefix.size() - suffix.size());
        }
        if (g_debugMode) cout << "[DEBUG EvalSExp] Applying operator (likely BUILTIN or SPECIAL FORM): " << op << endl;

        if (op == "lambda") {
            // Syntax: (lambda (param1 ...) body1 ...)
            if (elems.size() < 3) // Must have params list and at least one body expr
                throw RunTimeException("ERROR (lambda format) : " + Printer::print(node));

            const auto& paramsNode = elems[1];
            vector<string> paramNames;

            // Validate parameters list: must be a proper list of symbols
            if (!isProperList(paramsNode))
                throw RunTimeException("ERROR (lambda format) : parameters must be a proper list");

            vector<shared_ptr<Node>> paramListElems;
            try {
                paramListElems = unrollList(paramsNode);
            } catch (...) { // Catch potential errors from unrolling malformed list
                throw RunTimeException("ERROR (lambda format) : invalid parameter list structure");
            }

            for (const auto& param : paramListElems) {
                auto paramAtom = dynamic_pointer_cast<AtomNode>(param);
                if (!paramAtom || paramAtom->getType() != TokenType::SYMBOL) {
                    throw RunTimeException("ERROR (lambda format) : parameters must be symbols");
                }
                paramNames.push_back(paramAtom->getValue());
            }

            // Collect body expressions (all elements from index 2 onwards)
            vector<shared_ptr<Node>> bodyExprs;
            for (size_t i = 2; i < elems.size(); ++i) {
                bodyExprs.push_back(elems[i]);
            }

            // Create the Procedure object (closure)
            // Capture the *current* environment 'env'
            return make_shared<Procedure>(paramNames, bodyExprs, *env);
        }
        if (op == "define") {
            if (!isGlobalLayer) // Define only allowed at top level
                throw RunTimeException("ERROR (level of DEFINE)");

            // --- Form 1: (define symbol value) ---
            if (elems.size() == 3 && dynamic_pointer_cast<AtomNode>(elems[1]) && dynamic_pointer_cast<AtomNode>(elems[1])->getType() == TokenType::SYMBOL) {
                auto symAtom = dynamic_pointer_cast<AtomNode>(elems[1]);
                string symName = symAtom->getValue();

                if (builtins.count(symName)) // Cannot redefine builtins
                    throw RunTimeException("ERROR (DEFINE format) : " + Printer::print(node));

                // Evaluate the value expression in the current environment
                shared_ptr<Node> value = EvalSExp(false, elems[2], env); // Evaluate value in current env

                // Bind the symbol to the evaluated value in the *global* environment
                globalEnv[symName] = value; // Use globalEnv directly

                // Decide whether to print output based on verbose flag (Add verbose logic if needed)
                // For now, always return the defined message
                return make_shared<AtomNode>(TokenType::SYMBOL, symName + " defined");

                // --- Form 2: (define (func param...) body...) ---
            }
            else if (elems.size() >= 3 && dynamic_pointer_cast<DotNode>(elems[1])) { // function definition shorthand
                vector<shared_ptr<Node>> headerElems;
                try {
                    headerElems = unrollList(elems[1]); // Unroll the (func param...) part
                } catch (...) {
                    throw RunTimeException("ERROR (DEFINE format) : invalid function header structure");
                }

                if (headerElems.empty())
                    throw RunTimeException("ERROR (DEFINE format) : function header cannot be empty");

                auto funcNameAtom = dynamic_pointer_cast<AtomNode>(headerElems[0]);
                if (!funcNameAtom || funcNameAtom->getType() != TokenType::SYMBOL)
                    throw RunTimeException("ERROR (DEFINE format) : function name must be a symbol");

                string funcName = funcNameAtom->getValue();
                if (builtins.count(funcName))
                    throw RunTimeException("ERROR (DEFINE format) : " + Printer::print(node));

                // Extract parameter names (symbols)
                vector<string> paramNames;
                for (size_t i = 1; i < headerElems.size(); ++i) {
                    auto paramAtom = dynamic_pointer_cast<AtomNode>(headerElems[i]);
                    if (!paramAtom || paramAtom->getType() != TokenType::SYMBOL) {
                        throw RunTimeException("ERROR (DEFINE format) : function parameters must be symbols");
                    }
                    paramNames.push_back(paramAtom->getValue());
                }

                // Collect body expressions
                if (elems.size() < 3) // Need at least one body expression
                    throw RunTimeException("ERROR (DEFINE format) : " + Printer::print(node));

                vector<shared_ptr<Node>> bodyExprs;
                for (size_t i = 2; i < elems.size(); ++i) {
                    bodyExprs.push_back(elems[i]);
                }

                // Create the Procedure (closure) capturing the *current* environment 'env'
                // Store the function name in the Procedure object
                auto procedure = make_shared<Procedure>(funcName, paramNames, bodyExprs, *env);

                // Bind the function name in the *global* environment
                globalEnv[funcName] = procedure; // Use globalEnv directly

                // Decide whether to print output based on verbose flag (Add verbose logic if needed)
                // For now, always return the defined message
                return make_shared<AtomNode>(TokenType::SYMBOL, funcName + " defined");

            }
            else {
                // Neither format matched
                throw RunTimeException("ERROR (DEFINE format) : " + Printer::print(node));
            }
        }
        if (op == "let") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : let");

            // Create a new environment for the let bindings, inheriting from the current environment
            unordered_map<string, shared_ptr<Node>> localEnv = *env;

            // First pass: evaluate all values in the original environment
            vector<pair<string, shared_ptr<Node>>> evaluatedBindings;
            const auto& bindingsList = elems[1];
            if (!isProperList(bindingsList))
                throw RunTimeException("ERROR (let with incorrect format) : " + bindingsList->toString());

            auto bindings = unrollList(bindingsList);
            for (const auto& binding : bindings) {
                if (!isProperList(binding))
                    throw RunTimeException("ERROR (let with incorrect format) : " + binding->toString());

                auto bindingElems = unrollList(binding);
                if (bindingElems.size() != 2)
                    throw RunTimeException("ERROR (let with incorrect format) : " + binding->toString());

                auto symbolNode = dynamic_pointer_cast<AtomNode>(bindingElems[0]);
                if (!symbolNode || symbolNode->getType() != TokenType::SYMBOL)
                    throw RunTimeException("ERROR (let with incorrect format) : " + binding->toString());

                // Evaluate the value in the original environment
                auto value = EvalSExp(false, bindingElems[1], env);
                evaluatedBindings.emplace_back(symbolNode->getValue(), value);
            }

            // Second pass: apply all bindings to the local environment
            for (const auto& binding : evaluatedBindings) {
                localEnv[binding.first] = binding.second;
            }

            // Evaluate the body expressions in the new environment
            shared_ptr<Node> result;
            for (size_t i = 2; i < elems.size(); i++) {
                result = EvalSExp(false, elems[i], &localEnv);
            }

            return result;
        }
        if (op == "clean-environment") {
            if (!isGlobalLayer)
                throw RunTimeException("ERROR (level of CLEAN-ENVIRONMENT)");
            if (elems.size() != 1)
                throw RunTimeException("ERROR (incorrect number of arguments) : clean-environment");
            globalEnv.clear();
            return make_shared<AtomNode>(TokenType::SYMBOL, "environment cleaned");
        }
        if (op == "exit") {
            if (!isGlobalLayer)
                throw RunTimeException("ERROR (level of EXIT)");
            if (elems.size() != 1)
                throw RunTimeException("ERROR (incorrect number of arguments) : exit");
            throw ExitException();
        }
        if (op == "if") {
            if (elems.size() != 3 && elems.size() != 4)
                throw RunTimeException("ERROR (incorrect number of arguments) : " + op);
            auto condVal = EvalSExp(false, elems[1], env);
            bool condTrue;
            if (auto atomCond = dynamic_pointer_cast<AtomNode>(condVal))
                condTrue = (atomCond->getValue() != "nil");
            else
                condTrue = true;
            if (condTrue)
                return EvalSExp(false, elems[2], env);
            if (elems.size() == 4)
                return EvalSExp(false, elems[3], env);
            throw RunTimeException("ERROR (no return value) : " + Printer::print(node));
        }
        if (op == "cond") {
            if (elems.size() < 2)
                throw RunTimeException("ERROR (COND format) : " + Printer::print(node));

            // Pre-check: validate every clause is a proper list and non-empty
            for (size_t i = 1; i < elems.size(); i++) {
                const auto& clause = elems[i];
                if (!isProperList(clause))
                    throw RunTimeException("ERROR (COND format) : " + Printer::print(node));

                vector<shared_ptr<Node>> clauseElems = unrollList(clause);
                if (clauseElems.empty())
                    throw RunTimeException("ERROR (COND format) : " + Printer::print(node));
                if (clauseElems.size() < 2)
                    throw RunTimeException("ERROR (COND format) : " + Printer::print(node));
            }

            // Iterate over each clause
            for (size_t i = 1; i < elems.size(); i++) {
                const auto& clause = elems[i];
                // Each clause must be a proper list.
                if (!isProperList(clause))
                    throw RunTimeException("ERROR (COND format) : " + Printer::print(node));

                // Unroll the clause into its component expressions.
                vector<shared_ptr<Node>> clauseElems = unrollList(clause);
                if (clauseElems.empty())
                    throw RunTimeException("ERROR (COND format) : " + Printer::print(node));

                bool clauseTrue = false;
                shared_ptr<Node> testResult = nullptr;
                // Check if this clause is an "else" clause (only valid as the last clause).
                if (auto testAtom = dynamic_pointer_cast<AtomNode>(clauseElems[0])) {
                    if (testAtom->getValue() == "else" && i == elems.size() - 1) {
                        clauseTrue = true;
                    }
                    else {
                        testResult = EvalSExp(false, clauseElems[0], env);
                        // Consider the test true if its evaluated result is not nil.
                        if (auto atomTest = dynamic_pointer_cast<AtomNode>(testResult))
                            clauseTrue = (atomTest->getValue() != "nil");
                        else
                            clauseTrue = true;
                    }
                }
                else {
                    testResult = EvalSExp(false, clauseElems[0], env);
                    if (auto atomTest = dynamic_pointer_cast<AtomNode>(testResult))
                        clauseTrue = (atomTest->getValue() != "nil");
                    else
                        clauseTrue = true;
                }

                if (clauseTrue) {
                    // If the clause has a body, evaluate it sequentially and return the value of the last expression.
                    if (clauseElems.size() >= 2) {
                        shared_ptr<Node> result;
                        for (size_t j = 1; j < clauseElems.size(); j++) {
                            result = EvalSExp(false, clauseElems[j], env);
                        }
                        return result;
                    }
                    throw RunTimeException("ERROR (COND format) : " + Printer::print(node));
                }
            }
            // If no clause yields a value, signal an error.
            throw RunTimeException("ERROR (no return value) : " + Printer::print(node));
        }
        if (op == "begin") {
            if (elems.size() < 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : begin");
            shared_ptr<Node> res;
            for (size_t i = 1; i < elems.size(); i++)
                res = EvalSExp(false, elems[i], env);
            return res;
        }
        if (op == "quote") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : quote");
            return elems[1];
        }

        vector<shared_ptr<Node>> restArgs;
        for (size_t i = 1; i < elems.size(); i++) {
            if (g_debugMode) cout << "[DEBUG EvalSExp]   Evaluating arg " << i << ": " << Printer::print(elems[i]) << endl;
        }

        if (op == "cons") {
            if (elems.size() != 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : cons");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            return make_shared<DotNode>(args[0], args[1]);
        }
        if (op == "car") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : car");

            // Evaluate the argument
            shared_ptr<Node> argVal = EvalSExp(false, elems[1], env);
            // Check if the evaluated value is a pair (i.e. a DotNode)
            auto pairNode = dynamic_pointer_cast<DotNode>(argVal);
            if (!pairNode)
                throw RunTimeException("ERROR (car with incorrect argument type) : " + argVal->toString());

            return pairNode->getLeft();
        }
        if (op == "cdr") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : cdr");

            // Evaluate the argument
            shared_ptr<Node> argVal = EvalSExp(false, elems[1], env);
            // Check if the evaluated value is a pair (i.e. a DotNode)
            auto pairNode = dynamic_pointer_cast<DotNode>(argVal);
            if (!pairNode)
                throw RunTimeException("ERROR (cdr with incorrect argument type) : " + argVal->toString());

            return pairNode->getRight();
        }
        if (op == "list") {
            shared_ptr<Node> list = make_shared<AtomNode>(TokenType::NIL, "nil");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            for (int i = args.size() - 1; i >= 0; i--)
                list = make_shared<DotNode>(args[i], list);

            return list;
        }
        if (op == "+") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : +");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double sum = 0.0;
            bool forceFloat = false;
            for (const auto& arg : args) {
                double n = getNumber(op, arg);
                sum += n;
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg))
                    if (atom->getType() == TokenType::FLOAT)
                        forceFloat = true;
            }

            if (forceFloat)
                return make_shared<AtomNode>(TokenType::FLOAT, to_string(sum));
            return make_shared<AtomNode>(TokenType::INT, to_string(static_cast<int>(sum)));
        }
        if (op == "-") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : -");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double result = getNumber(op, args[0]);
            bool forceFloat = false;
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                if (atom->getType() == TokenType::FLOAT)
                    forceFloat = true;

            for (size_t i = 1; i < args.size(); i++) {
                double n = getNumber(op, args[i]);
                result -= n;
                if (auto atom = dynamic_pointer_cast<AtomNode>(args[i]))
                    if (atom->getType() == TokenType::FLOAT)
                        forceFloat = true;
            }

            if (forceFloat)
                return make_shared<AtomNode>(TokenType::FLOAT, to_string(result));
            return make_shared<AtomNode>(TokenType::INT, to_string(static_cast<int>(result)));
        }
        if (op == "*") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : *");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double product = 1.0;
            bool forceFloat = false;
            for (const auto& arg : args) {
                double n = getNumber(op, arg);
                product *= n;
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg))
                    if (atom->getType() == TokenType::FLOAT)
                        forceFloat = true;
            }

            if (forceFloat)
                return make_shared<AtomNode>(TokenType::FLOAT, to_string(product));
            return make_shared<AtomNode>(TokenType::INT, to_string(static_cast<int>(product)));
        }
        if (op == "/") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : /");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double result = getNumber(op, args[0]);
            bool forceFloat = false;
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                if (atom->getType() == TokenType::FLOAT)
                    forceFloat = true;

            for (size_t i = 1; i < args.size(); i++) {
                double divisor = getNumber(op, args[i]);
                if (divisor == 0)
                    throw RunTimeException("ERROR (division by zero) : /");
                result /= divisor;
                if (auto atom = dynamic_pointer_cast<AtomNode>(args[i]))
                    if (atom->getType() == TokenType::FLOAT)
                        forceFloat = true;
            }

            if (forceFloat)
                return make_shared<AtomNode>(TokenType::FLOAT, to_string(result));
            return make_shared<AtomNode>(TokenType::INT, to_string(static_cast<int>(result)));
        }
        if (op == ">") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : >");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double prev = getNumber(op, args[0]);
            for (size_t i = 1; i < args.size(); i++)
                getNumber(op, args[i]);

            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(op, args[i]);
                if (!(prev > curr))
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                prev = curr;
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        if (op == ">=") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : >=");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double prev = getNumber(op, args[0]);
            for (size_t i = 1; i < args.size(); i++)
                getNumber(op, args[i]);

            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(op, args[i]);
                if (!(prev >= curr))
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                prev = curr;
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        if (op == "<") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : <");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double prev = getNumber(op, args[0]);
            for (size_t i = 1; i < args.size(); i++)
                getNumber(op, args[i]);

            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(op, args[i]);
                if (!(prev < curr))
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                prev = curr;
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        if (op == "<=") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : <=");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double prev = getNumber(op, args[0]);
            for (size_t i = 1; i < args.size(); i++)
                getNumber(op, args[i]);

            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(op, args[i]);
                if (!(prev <= curr))
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                prev = curr;
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        if (op == "=") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : =");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            double first = getNumber(op, args[0]);
            for (size_t i = 1; i < args.size(); i++)
                getNumber(op, args[i]);

            for (size_t i = 1; i < args.size(); i++) {
                double curr = getNumber(op, args[i]);
                if (first != curr)
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
            }
            return make_shared<AtomNode>(TokenType::T, "#t");
        }
        if (op == "not") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : not");
            auto testVal = EvalSExp(false, elems[1], env);
            bool truth;
            if (auto atom = dynamic_pointer_cast<AtomNode>(testVal))
                truth = (atom->getValue() != "nil");
            else
                truth = true;
            return truth ? make_shared<AtomNode>(TokenType::NIL, "nil")
                       : make_shared<AtomNode>(TokenType::T, "#t");
        }
        if (op == "and") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : and");
            shared_ptr<Node> lastEvaluated;
            for (size_t i = 1; i < elems.size(); i++) {
                lastEvaluated = EvalSExp(false, elems[i], env);
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
        if (op == "or") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : or");
            for (size_t i = 1; i < elems.size(); i++) {
                auto val = EvalSExp(false, elems[i], env);
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
        if (op == "string-append") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : string-append");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            string result;
            for (auto & arg : args) {
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg)) {
                    if (atom->getType() != TokenType::STRING)
                        throw RunTimeException("ERROR (string-append with incorrect argument type) : " + atom->toString());
                    string str = atom->getValue();
                    if (str.length() >= 2 && str.front() == '"' && str.back() == '"')
                        str = str.substr(1, str.length() - 2);
                    result += str;
                } else {
                    throw RunTimeException("ERROR (string-append with incorrect argument type)");
                }
            }
            string finalStr = "\"" + result + "\"";
            return make_shared<AtomNode>(TokenType::STRING, finalStr);
        }
        if (op == "string>?") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : string>?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            vector<string> strs;
            for (auto & arg : args) {
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg)) {
                    if (atom->getType() != TokenType::STRING)
                        throw RunTimeException("ERROR (string>? with incorrect argument type) : " + atom->toString());
                    string str = atom->getValue();
                    if (str.length() >= 2 && str.front() == '"' && str.back() == '"')
                        str = str.substr(1, str.length() - 2);
                    strs.push_back(str);
                } else {
                    throw RunTimeException("ERROR (string>? with incorrect argument type)");
                }
            }
            bool resultBool = true;
            for (size_t i = 0; i < strs.size() - 1; i++) {
                if (strs[i] <= strs[i+1]) {
                    resultBool = false;
                    break;
                }
            }
            return resultBool ? make_shared<AtomNode>(TokenType::T, "#t")
                       : make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "string<?") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : string<?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            vector<string> strs;
            for (auto & arg : args) {
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg)) {
                    if (atom->getType() != TokenType::STRING)
                        throw RunTimeException("ERROR (string<? with incorrect argument type) : " + atom->toString());
                    string str = atom->getValue();
                    if (str.length() >= 2 && str.front() == '"' && str.back() == '"')
                        str = str.substr(1, str.length() - 2);
                    strs.push_back(str);
                } else {
                    throw RunTimeException("ERROR (string<? with incorrect argument type)");
                }
            }
            bool resultBool = true;
            for (size_t i = 0; i < strs.size() - 1; i++) {
                if (strs[i] >= strs[i+1]) {
                    resultBool = false;
                    break;
                }
            }
            return resultBool ? make_shared<AtomNode>(TokenType::T, "#t")
                       : make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "string=?") {
            if (elems.size() < 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : string=?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            vector<string> strs;
            for (auto & arg : args) {
                if (auto atom = dynamic_pointer_cast<AtomNode>(arg)) {
                    if (atom->getType() != TokenType::STRING)
                        throw RunTimeException("ERROR (string=? with incorrect argument type) : " + atom->toString());
                    string str = atom->getValue();
                    if (str.length() >= 2 && str.front() == '"' && str.back() == '"')
                        str = str.substr(1, str.length() - 2);
                    strs.push_back(str);
                } else {
                    throw RunTimeException("ERROR (string=? with incorrect argument type)");
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
        if (op == "eqv?") {
            if (elems.size() != 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : eqv?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            if (args[0] == args[1])
                return make_shared<AtomNode>(TokenType::T, "#t");
            auto atom1 = dynamic_pointer_cast<AtomNode>(args[0]);
            auto atom2 = dynamic_pointer_cast<AtomNode>(args[1]);
            if (atom1 && atom2) {
                if (atom1->getType() == TokenType::STRING || atom2->getType() == TokenType::STRING)
                    return make_shared<AtomNode>(TokenType::NIL, "nil");
                if (atom1->getType() == atom2->getType() && atom1->getValue() == atom2->getValue())
                    return make_shared<AtomNode>(TokenType::T, "#t");
                return make_shared<AtomNode>(TokenType::NIL, "nil");
            }
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "equal?") {
            if (elems.size() != 3)
                throw RunTimeException("ERROR (incorrect number of arguments) : equal?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

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
                if (atom1->getType() == atom2->getType() && atom1->getValue() == atom2->getValue())
                    return make_shared<AtomNode>(TokenType::T, "#t");
                return make_shared<AtomNode>(TokenType::NIL, "nil");
            }
            if (structuralEqual(args[0], args[1]))
                return make_shared<AtomNode>(TokenType::T, "#t");
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "atom?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : atom?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            return dynamic_pointer_cast<DotNode>(args[0])
                       ? make_shared<AtomNode>(TokenType::NIL, "nil")
                       : make_shared<AtomNode>(TokenType::T, "#t");
        }
        if (op == "pair?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : pair?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            return dynamic_pointer_cast<DotNode>(args[0])
                       ? make_shared<AtomNode>(TokenType::T, "#t")
                       : make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "list?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : list?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            bool proper = false;
            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                proper = (atom->getValue() == "nil");
            else
                proper = isProperList(args[0]);
            return proper ? make_shared<AtomNode>(TokenType::T, "#t")
                       : make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "null?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : null?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return (atom->getValue() == "nil")
                           ? make_shared<AtomNode>(TokenType::T, "#t")
                           : make_shared<AtomNode>(TokenType::NIL, "nil");
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "integer?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : integer?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return (atom->getType() == TokenType::INT)
                           ? make_shared<AtomNode>(TokenType::T, "#t")
                           : make_shared<AtomNode>(TokenType::NIL, "nil");
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "real?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : real?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return ((atom->getType() == TokenType::INT || atom->getType() == TokenType::FLOAT)
                            ? make_shared<AtomNode>(TokenType::T, "#t")
                            : make_shared<AtomNode>(TokenType::NIL, "nil"));
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "number?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : number?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return ((atom->getType() == TokenType::INT || atom->getType() == TokenType::FLOAT)
                            ? make_shared<AtomNode>(TokenType::T, "#t")
                            : make_shared<AtomNode>(TokenType::NIL, "nil"));
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "string?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : string?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return (atom->getType() == TokenType::STRING
                            ? make_shared<AtomNode>(TokenType::T, "#t")
                            : make_shared<AtomNode>(TokenType::NIL, "nil"));
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "boolean?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : boolean?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0])) {
                if (atom->getType() == TokenType::T || atom->getType() == TokenType::NIL)
                    return make_shared<AtomNode>(TokenType::T, "#t");
                return make_shared<AtomNode>(TokenType::NIL, "nil");
            }
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        if (op == "symbol?") {
            if (elems.size() != 2)
                throw RunTimeException("ERROR (incorrect number of arguments) : symbol?");

            // Evaluate operands for the remaining built-in functions.
            vector<shared_ptr<Node>> args;
            for (size_t i = 1; i < elems.size(); i++)
                args.push_back(EvalSExp(false, elems[i], env));

            if (auto atom = dynamic_pointer_cast<AtomNode>(args[0]))
                return (atom->getType() == TokenType::SYMBOL
                            ? make_shared<AtomNode>(TokenType::T, "#t")
                            : make_shared<AtomNode>(TokenType::NIL, "nil"));
            return make_shared<AtomNode>(TokenType::NIL, "nil");
        }
        throw RunTimeException("ERROR (unbound symbol) : " + op);
    }

    throw RunTimeException("ERROR (unknown expression) : " + node->toString());
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
                shared_ptr<Node> result = EvalSExp(true, parser.parse());

                cout << endl << "> " << Printer::print(result) << endl;

                Scanner::skipIfLineLeftoverEmpty();
            } catch (const CompileTimeException& e) {
                cout << endl << "> " << e.what() << endl;
                string line;
                getline(cin, line);
            } catch (const RunTimeException& e) {
                cout << endl << "> " << e.what() << endl;
                Scanner::skipIfLineLeftoverEmpty();
            }
        }

    } catch (const EOFException& e) {
        cout << endl << "> ERROR (no more input) : END-OF-FILE encountered";
    } catch (const ExitException& e) {
        cout << endl << "> ";
    }
    cout << endl << "Thanks for using OurScheme!";
    return 0;
}