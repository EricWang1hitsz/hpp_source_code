/** \mainpage

\section hpp_template_corba_sec_intro Introduction

This package is intended to ease construction of CORBA servers by templating
actions that are common to all servers.

\section hpp_template_corba_sec_principle Principle

This package only contains one template class hpp::corba::Server. The parameter
of this template class is the implementation of an idl interface.

\section hpp_template_corba_sec_how_to How to

\subsection hpp_template_corba_subsec_server Implementing the server

Let us assume that you want to build a server implementing the following
interface written in file <c>interface.idl</c>.
\code
// file interface.idl
module hpp {
  interface MyInterface {
    // Compute the sum of two real numbers.
    double sum(in double a, in double b);
  };
};
\endcode

Generate C++ code relative to this interface using omniidl.
\code
omniidl -bcxx interface.idl
\endcode
Two files are created: <c>interface.hh</c> and <c>interfaceSK.cc</c>.

You need to write the implementation of your server as a class deriving
from <c>POA_hpp::MyInterface</c> (defined in <c>interface.hh</c>).

\code
// file my-interface.impl.hh
#ifndef HPP_CORBA_INTERFACE_IMPL_HH
#define HPP_CORBA_INTERFACE_IMPL_HH

#include <omniORB4/CORBA.h>
#include "interface.hh"

class MyImplementation : public virtual POA_hpp::MyInterface
{
  virtual CORBA::Double sum(CORBA::Double a, CORBA::Double b)
  {
    return a+b;
  }
}; // end of class MyImplementation
#endif //HPP_CORBA_INTERFACE_IMPL_HH
\endcode
You can now implement a server as follows:
\code
//file server.cc
#include <stdlib.h>
#include <hpp/corba/template/server.hh>
#include "my-interface.impl.hh"

int main(int argc, char** argv)
{
  hpp::corba::Server<MyImplementation> server (argc, argv, true);

  const std::string contextId("cId");
  const std::string contextKind("cKind");
  const std::string objectId("oId");
  const std::string objectKind("oKind");

  if (server.startCorbaServer(contextId, contextKind,
			      objectId, objectKind) != 0) {
    exit (-1);
  }
  server.processRequest(true);
}

\endcode
To compile the above file, use <c>omniORB</c> specific flags:
\code
  g++ -o server `pkg-config --cflags hpp-template-corba` `pkg-config --cflags omniORB4` `pkg-config --libs omniORB4` interfaceSK.cc server.cc
\endcode
  where <c>includedir</c> is the header installation directory of this package.
You get an executable implementing your CORBA. To run the server, you need
to start a name server. Your server will be referenced in the name server by
  <c>cId.cKind/oId.oKind</c>


\subsection hpp_template_corba_subsec_client Implementing the client

In this section, we implement a python client for the above server.

The first step consists in compiling <c>interface.idl</c> to generate python
stubs:
\code
omniidl -bpython -Wbpackage=hpp_corba interface.idl
\endcode
A new directory <c>hpp_corba</c> is created. The following lines implement a python client,
\code
# File client.py
from omniORB import CORBA
import CosNaming
import sys

orb = CORBA.ORB_init (sys.argv, CORBA.ORB_ID)
obj = orb.resolve_initial_references("NameService")
rootContext = obj._narrow(CosNaming.NamingContext)

name = [CosNaming.NameComponent ("cId", "cKind"), CosNaming.NameComponent ("oId", "oKind")]
obj = rootContext.resolve (name)
from hpp_corba.hpp import *
client = obj._narrow(MyInterface)
\endcode

\subsection hpp_template_corba_subsec_running Running the server and client

You need 3 terminals.

In the first terminal, run the name server
\code
omniNames -start
\endcode
in the second terminal, run the server:
\code
./server
\endcode

in the third terminal, open a python terminal
\code
python
Python 2.6.2 (r262:71600, Jan 25 2010, 18:46:45)
[GCC 4.4.2 20091222 (Red Hat 4.4.2-20)] on linux2
Type "help", "copyright", "credits" or "license" for more information.
>>> from client import client
>>> client.sum(1.5, 2.5)
4.0
>>>
\endcode

*/
