#ifndef STDTOSTRING_H
#define STDTOSTRING_H
#include <string>
#include <sstream>
 
 //T.k at 20180903 for error "to_string is not member of std"
 
using namespace std;
namespace std
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}
#endif