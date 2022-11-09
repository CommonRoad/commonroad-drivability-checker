#ifndef s11n_net_s11n_v1_1_EXCEPTION_HPP_INCLUDED
#define s11n_net_s11n_v1_1_EXCEPTION_HPP_INCLUDED 1

#include <string>
#include <exception>

namespace s11n {


	/**
	   The base-most exception type used by s11n.
	*/
        struct s11n_exception : public std::exception
        {
	public:
		virtual ~s11n_exception() throw() {}
                explicit s11n_exception( const std::string & What );
		/**
		   Creates an error string consisting of:

		   file:linenum: What
		*/
                s11n_exception( const std::string & What, const std::string & file, unsigned int linenum );
                virtual const char * what() const throw();
        private:
                std::string m_what;
        };

	/**
	   An exception type for classloader-related exceptions. These
	   need to be caught separately from s11n_exceptions in some
	   cases because sometimes a classloader can try other
	   alternatives on an error.
	*/
	struct factory_exception : public s11n_exception
	{
	public:
		virtual ~factory_exception() throw() {}
		explicit factory_exception( const std::string & What ) : s11n_exception( What ) {}
                factory_exception( const std::string & What, const std::string & file, unsigned int line ) : s11n_exception( What,file,line ) {}
	};


	/**
	   Really for use by clients, i/o layers, and s11nlite, not by
	   the s11n core.
	*/
	struct io_exception : public s11n_exception
	{
	public:
		virtual ~io_exception() throw() {}
		explicit io_exception( const std::string & What ) : s11n_exception( What ) {}
                io_exception( const std::string & What, const std::string & file, unsigned int line ) : s11n_exception( What,file,line ) {}
	};


} // namespace s11n

/**
   S11N_THROW(WHAT) simply throws s11n_exception(WHAT,__FILE__,__LINE__).
*/
#define S11N_THROW(WHAT) throw ::s11n::s11n_exception(WHAT,__FILE__,__LINE__)

#endif // s11n_net_s11n_v1_1_EXCEPTION_HPP_INCLUDED
