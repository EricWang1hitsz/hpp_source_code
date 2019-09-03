#ifndef hpp_core_idl__paths_hxx__
#define hpp_core_idl__paths_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/paths.idl
//

#include <hpp/core_idl/paths-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::core_idl::Path
//
namespace core_impl {
template <typename _Base, typename _Storage>
PathServant<_Base, _Storage>::PathServant(Server* server, const _Storage& s)
  : ServantBase<hpp::core::Path, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
PathServant<_Base, _Storage>::~PathServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::size_type PathServant<_Base, _Storage>::outputSize ()
{
  try {
    // automatically generated code.
    
    return (getT()->outputSize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::size_type PathServant<_Base, _Storage>::outputDerivativeSize ()
{
  try {
    // automatically generated code.
    
    return (getT()->outputDerivativeSize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::value_type PathServant<_Base, _Storage>::length ()
{
  try {
    // automatically generated code.
    
    return (getT()->length ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* PathServant<_Base, _Storage>::initial ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->initial ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* PathServant<_Base, _Storage>::end ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->end ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
char* PathServant<_Base, _Storage>::str ()
{
  try {
    // automatically generated code.
    std::ostringstream oss; oss << *get();
    std::string res = oss.str();
    return CORBA::string_dup(res.c_str());
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* PathServant<_Base, _Storage>::call (hpp::value_type t, ::CORBA::Boolean& success)
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->operator() (t, success));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* PathServant<_Base, _Storage>::at (hpp::value_type t, ::CORBA::Boolean& success)
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/paths.idl:22
    vector_t res (get()->outputSize());
    success = get()->at (t, res);
    return vectorToFloatSeq (res);

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* PathServant<_Base, _Storage>::derivative (hpp::value_type t, ::CORBA::Short order)
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/paths.idl:22
    vector_t res (get()->outputDerivativeSize());
    get()->derivative (res, t, order);
    return vectorToFloatSeq (res);

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::Path_ptr PathServant<_Base, _Storage>::extract (hpp::value_type tmin, hpp::value_type tmax)
{
  try {
    // automatically generated code.
    
    hpp::core::PathPtr_t __return__ (getT()->extract (tmin, tmax));
    return makeServantDownCast<hpp::corbaServer::core_impl::Path,hpp::corbaServer::core_impl::Path>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::PathVector_ptr PathServant<_Base, _Storage>::asVector ()
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/paths.idl:22
    PathPtr_t p = get();
    PathVectorPtr_t pv =
      core::PathVector::create (p->outputSize(), p->outputDerivativeSize());
    pv->appendPath (p);

    return makeServant<hpp::core_idl::PathVector_ptr>
      (server_, new PathVector (server_, pv));

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl

//
// Implementational code for IDL interface hpp::core_idl::PathVector
//
namespace core_impl {
template <typename _Base, typename _Storage>
PathVectorServant<_Base, _Storage>::PathVectorServant(Server* server, const _Storage& s)
  : hpp::corbaServer::core_impl::PathServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
PathVectorServant<_Base, _Storage>::~PathVectorServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::core_idl::size_t PathVectorServant<_Base, _Storage>::numberPaths ()
{
  try {
    // automatically generated code.
    
    return (getT()->numberPaths ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::Path_ptr PathVectorServant<_Base, _Storage>::pathAtRank (hpp::core_idl::size_t rank)
{
  try {
    // automatically generated code.
    
    hpp::core::PathPtr_t __return__ (getT()->pathAtRank (rank));
    return makeServantDownCast<hpp::corbaServer::core_impl::Path,hpp::corbaServer::core_impl::Path>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void PathVectorServant<_Base, _Storage>::appendPath (hpp::core_idl::Path_ptr p)
{
  try {
    // automatically generated code.
    hpp::core::PathPtr_t _p = reference_to_servant_base<hpp::core::Path>(server_, p)->get();
     (getT()->appendPath (_p));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void PathVectorServant<_Base, _Storage>::concatenate (hpp::core_idl::PathVector_ptr p)
{
  try {
    // automatically generated code.
    hpp::core::PathVectorPtr_t _p = reference_to_servant_base<hpp::core::PathVector>(server_, p)->get();
     (getT()->concatenate (_p));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__paths_hxx__

