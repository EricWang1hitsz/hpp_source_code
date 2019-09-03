// Copyright (C) 2015 by Joseph Mirabel
//
// This file is part of the hpp-manipulation-corba.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_MANIPULATION_CORBA_CLIENT_HH
# define HPP_MANIPULATION_CORBA_CLIENT_HH

# include <omniORB4/CORBA.h>

# include <hpp/corbaserver/manipulation/robot-idl.hh>
# include <hpp/corbaserver/manipulation/problem-idl.hh>
# include <hpp/corbaserver/manipulation/graph-idl.hh>
# include <hpp/corbaserver/obstacle-idl.hh>

# include <hpp/corbaserver/manipulation/config.hh>

namespace hpp
{
  namespace corbaServer
  {
    namespace manipulation
    {
      class HPP_MANIPULATION_CORBA_DLLAPI Client
      {
        public:
          Client (int argc, char* argv[]);

          ~Client ();

          /// \param iiop address of the namesever
          /// \param context the hpp context name (passed to the server)
          void connect (const char* iiop = "corbaloc:rir:/NameService",
              const char* context = "corbaserver");


          hpp::corbaserver::manipulation::Robot_var& robot () {
            return robot_;
          }

          hpp::corbaserver::manipulation::Problem_var& problem () {
            return problem_;
          }

          hpp::corbaserver::manipulation::Graph_var& graph () {
            return graph_;
          }

        private:
          hpp::corbaserver::manipulation::Robot_var robot_;
          hpp::corbaserver::manipulation::Problem_var problem_;
          hpp::corbaserver::manipulation::Graph_var graph_;

          CORBA::ORB_var orb_;
      };
    } // end of namespace manipulation.
  } // end of namespace corbaServer.
} // end of namespace hpp.

#endif // HPP_MANIPULATION_CORBA_CLIENT_HH
