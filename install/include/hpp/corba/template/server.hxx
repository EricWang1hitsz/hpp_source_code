// Copyright (C) 2009, 2010 by Florent Lamiraux, Thomas Moulard, JRL.
//
// This file is part of hpp-template-corba
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#ifndef HPP_CORBA_TEMPLATE_SERVER_HXX
#define HPP_CORBA_TEMPLATE_SERVER_HXX

#include <errno.h>
#include <pthread.h>
#include <iostream>

#include <hpp/corba/template/debug.hh>

//FIXME: remove me.
#define HPP_CORBA_CATCH(msg, ret)				\
  catch(CORBA::UserException& exc) {				\
    hppCorbaDout (error, "CORBA::UserException: " << msg << " " \
		  << exc._name ());				\
    return ret;							\
  }								\
  catch(CORBA::SystemException&) {				\
    hppCorbaDout (error, "CORBA::SystemException: " << msg);	\
    return ret;							\
  }								\
  catch(CORBA::Exception&) {					\
    hppCorbaDout (error, "CORBA::Exception: " << msg);		\
    return ret;							\
  }								\
  catch(omniORB::fatalException& fe) {				\
    hppCorbaDout (error, "CORBA::fatalException: " << msg);	\
    return ret;							\
  }								\
  catch(...) {							\
    hppCorbaDout (error, "CORBA: unknown exception: " << msg);	\
    return ret;							\
  }

namespace hpp
{
  namespace corba
  {
    using CORBA::Exception;
    using CORBA::Object_var;
    using CORBA::SystemException;
    using CORBA::ORB_init;
    using CORBA::PolicyList;
    using CORBA::Object_ptr;
    using CORBA::COMM_FAILURE;
    using omniORB::fatalException;

    typedef CORBA::ORB::InvalidName InvalidName;

    namespace
    {
      /// \brief Forward logging messages to hpp logging mechanism.
      /// If debug is disabled, CORBA logging will be disabled too.
      ///
      /// Tracing has to be enabled in your ``omniORB.cfg'' to use this
      /// feature.
      /// See ``omniORB configuration and API'' > ``Tracing options''
      /// section of omniORB manual for more information.
      void logFunction (const char* msg);

      void logFunction (const char* msg)
      {
	hppCorbaDout (info, "omniORB: " << msg);
      }
    } // end of anonymous namespace.

    template <class T>
    Server<T>::Server(int argc, const char* argv[], bool inMultiThread,
		      const std::string& poaName)
    {
      // Register log function.
      omniORB::setLogFunction (&logFunction);
      initORBandServers (argc, argv, inMultiThread, poaName);
    }

    /// \brief Shutdown CORBA server
    template <class T>
    Server<T>::~Server()
    {
      deactivateAndDestroyServers();
      delete servantId_;
    }

    template <class T>
    T& Server<T>::implementation()
    {
      return *servant_;
    }

    /*
      CORBA SERVER INITIALIZATION
    */
    template <class T>
    bool Server<T>::initORBandServers(int argc, const char *argv[],
				      bool inMultiThread,
				      const std::string& poaName)
    {
      Object_var obj;
      PortableServer::ThreadPolicy_var threadPolicy;
      PortableServer::POA_var rootPoa;

      /*
	Fine granularity in exception handling
      */

      /*
	ORB init
      */
      try {
	orb_ = ORB_init (argc, const_cast<char **> (argv));
	if (is_nil(orb_)) {
	  hppCorbaDout (error, "failed to initialize ORB");
	  return false;
	}
      }
      HPP_CORBA_CATCH("failed to initialize ORB", false)

	/*
	  ORB init
	*/

	try {
	  obj = orb_->resolve_initial_references("RootPOA");
	}
      HPP_CORBA_CATCH("failed to resolve initial references", false)

	/*
	  Create thread policy
	*/

	try {
	  //
	  // Make the CORBA object single-threaded to avoid GUI krash
	  //
	  // Create a sigle threaded policy object
	  rootPoa = PortableServer::POA::_narrow(obj);

	  if (inMultiThread) {
	    threadPolicy =
	      rootPoa->create_thread_policy(PortableServer::ORB_CTRL_MODEL);
	  }
	  else {
	    threadPolicy =
	      rootPoa->create_thread_policy(PortableServer::MAIN_THREAD_MODEL);
	  }
	}
      HPP_CORBA_CATCH("failed to create thread policy", false)

	/*
	  Duplicate thread policy
	*/
	PolicyList policyList;
        policyList.length(1);

	try {
	  policyList[0]=PortableServer::ThreadPolicy::_duplicate(threadPolicy);
	}
	HPP_CORBA_CATCH("failed to duplicate thread policy", false)

	try {
	  try {
	    poa_ = rootPoa->find_POA(poaName.c_str(), false);
	  } catch (const CORBA::UserException& exc) {
	    poa_ =
	      rootPoa->create_POA(poaName.c_str (),
				  PortableServer::POAManager::_nil(),
				  policyList);
	  }
	}
	HPP_CORBA_CATCH("failed to find or create POA", false)

	/*
	  Destroy thread policy
	*/

	try {
	  // Destroy policy object
	  threadPolicy->destroy();

	}
      HPP_CORBA_CATCH("failed to destroy thread policy", false)

	return createAndActivateServers();
    }

    template <class T>
    int Server<T>::startCorbaServer(const std::string& contextId,
				    const std::string& contextKind,
				    const std::string& objectId,
				    const std::string& objectKind)
    {
      try {
	// Obtain a reference to objects, and register them in
	// the naming service.
	Object_var object = servant_->_this();

	if (!createHppContext(contextId, contextKind)) {
	  return -1;
	}
	// Bind object with name Robot to the hppContext:
	CosNaming::Name objectName;
	objectName.length(1);
	objectName[0].id   = (const char*) objectId.c_str();
	objectName[0].kind = (const char*) objectKind.c_str();

	if(!bindObjectToName(object, objectName)) {
	  return -1;
	}
	servant_->_remove_ref();

	PortableServer::POAManager_var pman =
	  poa_->the_POAManager();
	pman->activate();
      }
      HPP_CORBA_CATCH("failed to start CORBA server", false)
	return 0;
    }


    /// \brief If CORBA requests are pending, process them
    template <class T>
    int Server<T>::processRequest (bool loop)
    {
      if (loop)
	{
	  hppCorbaDout (info, "start processing CORBA requests for ever.");
	  orb_->run();
	}
      else
	{
	  if (orb_->work_pending())
	    orb_->perform_work();
	}
      return 0;
    }

    template <class T>
    bool Server<T>::createAndActivateServers ()
    {
      try {
	servant_ = new T ();
      }
      HPP_CORBA_CATCH("failed to create implementation of ChppciRobot", false)

	try {

	  servantId_ = poa_->activate_object(servant_);
	}
      HPP_CORBA_CATCH("failed to activate implementation of ChppciRobot",
		      false)

	return true;
    }

    template <class T>
    void Server<T>::deactivateAndDestroyServers()
    {
      if (servant_) {
        try {
          poa_->deactivate_object(*servantId_);
          delete servant_;
        } catch (const CORBA::OBJECT_NOT_EXIST& exc) {
          // Servant was already deactivated and deleted.
        }
      }
    }


    template <class T>
    bool Server<T>::createHppContext (const std::string& id,
				      const std::string kind)
    {
      CosNaming::NamingContext_var rootContext;
      Object_var localObj;
      CosNaming::Name contextName;

      try {
	// Obtain a reference to the root context of the Name service:
	localObj = orb_->resolve_initial_references("NameService");
      }
      HPP_CORBA_CATCH("failed to get the name service", false)

	try {
	  // Narrow the reference returned.
	  rootContext = CosNaming::NamingContext::_narrow(localObj);
	  if( is_nil(rootContext) ) {
	    hppCorbaDout (error, "Failed to narrow the root naming context.");
	    return false;
	  }
	}
	catch(InvalidName& ex) {
	  // This should not happen!
	  hppCorbaDout (error, "Service required is invalid [does not exist].");
	  return false;
	}
      HPP_CORBA_CATCH("failed to narrow the root naming context.", false)

	try {
	  // Bind a context called "hpp" to the root context:

	  contextName.length(1);
	  contextName[0].id   = (const char*) id.c_str();   // string copied
	  contextName[0].kind = (const char*) kind.c_str(); // string copied
	  // Note on kind: The kind field is used to indicate the type
	  // of the object. This is to avoid conventions such as that used
	  // by files (name.type -- e.g. hpp.ps = postscript etc.)

	  try {
	    // Bind the context to root.
	    hppContext_ = rootContext->bind_new_context(contextName);
	  }
	  catch(CosNaming::NamingContext::AlreadyBound& ex) {
	    // If the context already exists, this exception will be raised.
	    // In this case, just resolve the name and assign hppContext
	    // to the object returned:
	    Object_var localObj;
	    localObj = rootContext->resolve(contextName);
	    hppContext_ = CosNaming::NamingContext::_narrow(localObj);
	    if( is_nil(hppContext_) ) {
	      hppCorbaDout (error, "Failed to narrow naming context.");
	      return false;
	    }
	  }
	}
	catch(COMM_FAILURE& ex) {
	  hppCorbaDout (error, "Caught system exception COMM_FAILURE -- unable to contact the "
			<< "naming service.");
	  return false;
	}
	catch(SystemException&) {
	  hppCorbaDout (error, "Caught a SystemException while creating the context.");
	  return false;
	}

      return true;
    }

    template <class T>
    bool Server<T>::bindObjectToName(Object_ptr objref,
				     CosNaming::Name objectName)
    {
      try {
	try {
	  hppContext_->bind(objectName, objref);
	}
	catch(CosNaming::NamingContext::AlreadyBound& ex)
	  {
	    hppContext_->rebind(objectName, objref);
	  }
	// Note: Using rebind() will overwrite any Object previously bound
	//       to /hpp/RobotConfig with localObj.
	//       Alternatively, bind() can be used, which will raise a
	//       CosNaming::NamingContext::AlreadyBound exception if the name
	//       supplied is already bound to an object.

	// Amendment: When using OrbixNames, it is necessary to first try bind
	// and then rebind, as rebind on it's own will throw a NotFoundexception if
	// the Name has not already been bound. [This is incorrect behaviour -
	// it should just bind].
      }
      catch(COMM_FAILURE& ex) {
	hppCorbaDout (error, "Caught system exception COMM_FAILURE -- unable to contact the "
		      << "naming service.");
	return false;
      }
      catch(SystemException&) {
	hppCorbaDout(error, "Caught a SystemException while binding object to name service.");
	return false;
      }

      return true;
    }

  } // end of namespace corba.
} // end of namespace hpp.

#endif //HPP_CORBA_TEMPLATE_SERVER_HXX
