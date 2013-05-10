#include <iostream>
#include <boost/regex.hpp>

bool regular_expression_match(const std::string& s)
{
    static const boost::regex e("(\\d{4}[- ]){3}\\d{4}");
    return regex_match(s, e);
}

bool default_expression_match(const std::string& s)
{
    static const boost::regex e( ".*");
    return regex_match(s, e);
}



int main()
{

    std::string test_string( "s;ldfhasd;ighasd;iglkasjd" );

    std::cout << regular_expression_match( test_string ) << std::endl;
    std::cout << default_expression_match( test_string ) << std::endl;


}
