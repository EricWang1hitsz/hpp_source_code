// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of hpp-template-corba
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBA_SERVER_HH
# define HPP_CORBA_SERVER_HH

#include <string>
# include <omniORB4/CORBA.h>

namespace hpp
{
  namespace corba
  {
    /**
       \brief Template CORBA server
       
       For information on how to use see the \ref hpp_template_corba_sec_how_to
       section of the main documentation page
    */

    template <class T> class Server
    {
    public:
      /**
	 \brief Constructor
	 \param argc, argv parameter to feed ORB initialization.
	 \param inMultiThread whether the server may process request using
	 multithread policy.

	 \note It is recommended to configure your Corba implementation through
	 environment variables and to set argc to 1 and argv to any string.
      */
      Server (int argc, const char *argv[], bool multiThread = false,
	      const std::string& poaName = "child");

      /// \brief Shutdown CORBA server
      ~Server ();

      /// \brief Initialize CORBA server to process requests from clients
      /// \param contextId first part of context name
      /// \param contextKind second part of context name
      /// \param objectId first part of CORBA server name
      /// \param objectKind second part of CORBA server name
      /// \return 0 if success, -1 if failure.
      /// 
      /// The CORBA server is referenced in the name server by context and
      /// name: contextId.contextKind/objectId.objectKind.
      /// The context can be seen as a directory and the object as a filename.

      int startCorbaServer (const std::string& contextId,
			    const std::string& contextKind,
			    const std::string& objectId,
			    const std::string& objectKind);

      /// \brief If ORB work is pending, process it
      /// \param loop if true, the function never returns; if false, the function processes pending requests and returns.
      int processRequest (bool loop);

      /// \brief Return a reference to the implementation
      T& implementation();
    private:

      /**
	 \name CORBA server initialization
      */

      /**
	 \brief Initialize ORB and CORBA servers.

	 \param argc, argv parameter to feed ORB initialization.
	 \param inMultiThread whether the server may process request using multithred policy.
      */
      bool initORBandServers (int argc, const char *argv[],
			      bool inMultiThread,
			      const std::string& poaName);

      /**
	 @}
      */
      /// \brief Create and activate the Corba servers.
      bool createAndActivateServers ();

      CORBA::ORB_var orb_;
      PortableServer::POA_var poa_;

      /// \brief Implementation of object
      T* servant_;

      /// \brief It seems that we need to store this object to
      /// deactivate the server.
      PortableServer::ObjectId* servantId_;

      /// \brief Corba context.
      CosNaming::NamingContext_var hppContext_;

      /// \brief Create context.
      bool createHppContext (const std::string& id, const std::string kind);

      /// \brief Store objects in Corba name service.
      bool bindObjectToName (CORBA::Object_ptr objref,
			     CosNaming::Name objectName);


      /// \brief Deactivate and destroy servers
      ///
      /// Destroying active servers raises a Corba exception.
      void deactivateAndDestroyServers ();
    };

  } // end of namespace corba.
} // end of namespace hpp.

#include "hpp/corba/template/server.hxx"
#endif
