#include <s11n.net/s11n/exception.hpp>
#include <sstream>

// #include <s11n.net/s11n/s11n_debuggering_macros.hpp> // CERR
// #include <iostream>

namespace s11n {


	s11n_exception::s11n_exception( const std::string & What ) : m_what(What)
	{
	}

	const char * s11n_exception::what() const throw()
	{
		return this->m_what.c_str();
	}

        s11n_exception::s11n_exception( const std::string & what,
					const std::string & file,
					unsigned int line )
        {
                std::ostringstream os;
                os << file << ":"<<line<<": " << what;
                this->m_what = os.str();
        }


} // namespace s11n
