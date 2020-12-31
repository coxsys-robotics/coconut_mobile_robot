#ifndef protoX_Header
#define protoX_Header
#include <sstream>

const std::string col_red("\033[0;31m");        // set color of text to red.
const std::string col_green("\033[0;32m");      // set color of text to green.
const std::string col_yellow("\033[0;33m");     // set color of text to yellow.
const std::string col_blue("\033[0;34m");       // set color of text to blue.
const std::string col_magenta("\033[0;35m");    // set color of text to magenta.
const std::string col_cyan("\033[0;36m");       // set color of text to cyan.
const std::string col_reset("\033[0m");         // reset color of text(set to white).

const std::string bold_on("\e[1m");             // set text to bold.
const std::string bold_off("\e[0m");            // reset bold text.

//// for detect Ctrl+C KeyboardInterrupt
class InterruptException : public std::exception
{
public:
    InterruptException(int s) : S(s) {}
    int S;
};

#endif