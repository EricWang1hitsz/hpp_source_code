// Copyright (C) 2008, 2009 by Florent Lamiraux, Thomas Moulard, CNRS.
//
// This file is part of the hpp-util.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_UTIL_DEBUG_HH
# define HPP_UTIL_DEBUG_HH
# include <cstdlib>
# include <ostream>
# include <fstream>
# include <sstream>
# include <vector>

# include <hpp/util/config.hh>
# include <hpp/util/indent.hh>

namespace hpp
{
  namespace debug
  {
    class Output;
    class JournalOutput;
    class ConsoleOutput;

    class Channel;
  } // end of namespace debug
} // end of namespace hpp.


namespace hpp
{
  namespace debug
  {
    /// \brief Compute the logging prefix.
    ///
    /// This method returns the <i>logging prefix</i>. It means the
    /// directory where <em>all</em> debugging output must be stored.
    ///
    /// The prefix is <code>$localstatedir/log/$packagename</code>.
    ///
    /// \li <code>localstatedir</code>: <code>[prefix]/var<code> (by
    /// default <code>/usr/local/var</code>)
    /// \li<code>packagename</code>: this function's first argument
    ///
    /// To use this function, first include <code>config.h</code> and
    /// use the <code>PACKAGE_TARNAME</code> provided by Autoheader as
    /// this function's first argument.
    ///
    /// The prefix <code>$localstatedir/log</code> can be overrided by
    /// the environment variable <code>HPP_LOGGINGDIR</code>.
    ///
    ///
    /// \param packageName name of the package, must be a valid Unix
    /// filename (the use of <code>PACKAGE_TARNAME</code> is strongly
    /// encouraged).
    HPP_UTIL_DLLAPI std::string getPrefix (const std::string& packageName);

    /// \brief Compute the path of a file in the logging prefix.
    ///
    /// Apply the same rules than getPrefix and append the filename
    /// to build the whole path.
    ///
    /// \param filename Name of the file (will be appended to the prefix)
    /// \param packageName name of the package, must be a valid Unix
    /// filename (the use of <code>PACKAGE_TARNAME</code> is strongly
    /// encouraged).
    HPP_UTIL_DLLAPI std::string getFilename (const std::string& filename,
					const std::string& packageName);

    /// \brief Debugging output.
    ///
    /// Represents a debugging output, i.e. an output stream
    /// which can subscribe to channels.
    ///
    /// There is two default output:
    /// \li console which points to std::cerr
    /// \li journal which points to the ``journal'' file in the
    /// debugging prefix
    class HPP_UTIL_DLLAPI Output
    {
    public:
      explicit Output ();
      virtual ~Output ();

      virtual void
	write (const Channel& channel,
	       char const* file,
	       int line,
	       char const* function,
	       const std::string& data) = 0;

      virtual void
	write (const Channel& channel,
	       char const* file,
	       int line,
	       char const* function,
	       const std::stringstream& data) = 0;

    protected:
      std::ostream&
	writePrefix (std::ostream& stream,
		     const Channel& channel,
		     char const* file,
		     int line,
		     char const* function);
    };

    /// \brief Receive debugging information.
    ///
    /// Receive debugging information and forward it to its
    /// subscribers.
    ///
    /// By default, there is four channels:
    /// - error for fatal problems
    /// - warning for non-fatal problems
    /// - notice for user information
    /// - info for technical information and debugging output
    class HPP_UTIL_DLLAPI Channel
    {
    public:
      typedef std::vector<Output*> subscribers_t;

      explicit Channel (char const* label,
			const subscribers_t& subscribers);
      virtual ~Channel ();

      void write (char const* file,
		  int line,
		  char const* function,
		  const std::string& data);

      void write (char const* file,
		  int line,
		  char const* function,
                  const std::stringstream& data);

      const char* label () const;
    private:
      const char* label_;
      subscribers_t subscribers_;
    };

    /// \brief Logging in journal file in the logging directory.
    class HPP_UTIL_DLLAPI JournalOutput : public Output
    {
    public:
      explicit JournalOutput (std::string filename);
      ~JournalOutput ();

      void write (const Channel& channel,
		  char const* file,
		  int line,
		  char const* function,
		  const std::string& data);

      void write (const Channel& channel,
		  char const* file,
		  int line,
		  char const* function,
		  const std::stringstream& data);

      std::string getFilename () const;

    private:
      std::string filename;
      std::string lastFunction;
      std::ofstream stream;
    };

    /// \brief Logging in console (std::cerr).
    class HPP_UTIL_DLLAPI ConsoleOutput : public Output
    {
    public:
      explicit ConsoleOutput ();
      ~ConsoleOutput ();
      void write (const Channel& channel,
		  char const* file,
		  int line,
		  char const* function,
		  const std::string& data);
      void write (const Channel& channel,
		  char const* file,
		  int line,
		  char const* function,
		  const std::stringstream& data);
    };

    /// \brief Logging class owns all channels and outputs.
    class HPP_UTIL_DLLAPI Logging
    {
    public:
      explicit Logging ();
      ~Logging ();

      /// \brief Logs to console (i.e. stderr).
      ConsoleOutput console;
      /// \brief Logs to main journal file (i.e. journal.XXX.log).
      JournalOutput journal;
      /// \brief Logs to benchmark journal file (i.e. benchmark.XXX.log).
      JournalOutput benchmarkJournal;

      /// \brief Fatal problems channel.
      Channel error;

      /// \brief Non-fatal problems channel.
      Channel warning;

      /// \brief User-oriented information.
      Channel notice;

      /// \brief Technical information and debugging.
      Channel info;

      /// \brief Benchmark information.
      Channel benchmark;
    };
  } // end of namespace debug
} // end of namespace hpp.


namespace hpp
{
  namespace debug
  {
    /// \brief Benchmark information.
    extern HPP_UTIL_DLLAPI Logging logging;
  } // end of namespace debug
} // end of namespace hpp


# ifdef HPP_DEBUG

/// \addtogroup hpp_util_debugging
/// \{

/// \brief Enable \c statement when HPP_DEBUG is defined.
#  define hppDebug(statement)			\
  do {						\
    using namespace ::hpp::debug;		\
    { statement; }				\
  } while(0)

/// \brief Enable \c statement when HPP_DEBUG is defined.
#  define hppDebugStatement(statement)		\
  statement

/// \}

/// \addtogroup hpp_util_logging Macros for logging
/// \{

/// \brief Write \c data to \c channel when HPP_DEBUG is defined.
/// \param channel one of \em error, \em warning, \em notice, \em info or \em benchmark.
/// \param data a statement that can be \c << to a \c std::stringstream.
#  define hppDout(channel, data)					\
  do {									\
    using namespace hpp;						\
    using namespace ::hpp::debug;					\
    std::stringstream __ss;						\
    __ss << data << iendl;						\
    logging.channel.write (__FILE__, __LINE__, __PRETTY_FUNCTION__,     \
			   __ss);					\
  } while (0)

/// \brief Write \c data to \c channel and exit the program.
/// \param channel one of \em error, \em warning, \em notice, \em info or \em benchmark.
/// \param data a statement that can be \c << to a \c std::stringstream.
#  define hppDoutFatal(channel, data)					\
  do {									\
    using namespace hpp;						\
    using namespace ::hpp::debug;					\
    std::stringstream __ss;						\
    __ss << data << iendl;						\
    logging.channel.write ( __FILE__, __LINE__,	__PRETTY_FUNCTION__,	\
			    __ss);					\
    ::std::exit(EXIT_FAILURE);						\
  } while (1)

/// \}

# else

#  define hppDebug(statement)			\
  do {                                          \
  } while (0)
#  define hppDebugStatement(statement)
#  define hppDout(channel, data)                \
  do {                                          \
  } while (0)
#  define hppDoutFatal(channel, data)		\
  do {						\
    using namespace hpp;			\
    ::std::cerr << data << iendl;		\
    ::std::exit (EXIT_FAILURE);			\
  } while (1)

# endif // HPP_DEBUG

#endif //! HPP_UTIL_DEBUG_HH
