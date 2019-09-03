#ifndef hpp_core_idl__path__planners_hxx__
#define hpp_core_idl__path__planners_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/path_planners.idl
//

#include <hpp/core_idl/path_planners-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::core_idl::PathPlanner
//
namespace core_impl {
template <typename _Base, typename _Storage>
PathPlannerServant<_Base, _Storage>::PathPlannerServant(Server* server, const _Storage& s)
  : ServantBase<hpp::core::PathPlanner, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
PathPlannerServant<_Base, _Storage>::~PathPlannerServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::core_idl::PathVector_ptr PathPlannerServant<_Base, _Storage>::solve ()
{
  try {
    // automatically generated code.
    
    hpp::core::PathVectorPtr_t __return__ (getT()->solve ());
    return makeServantDownCast<hpp::corbaServer::core_impl::Path,hpp::corbaServer::core_impl::PathVector>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void PathPlannerServant<_Base, _Storage>::startSolve ()
{
  try {
    // automatically generated code.
    
     (getT()->startSolve ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void PathPlannerServant<_Base, _Storage>::tryConnectInitAndGoals ()
{
  try {
    // automatically generated code.
    
     (getT()->tryConnectInitAndGoals ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void PathPlannerServant<_Base, _Storage>::oneStep ()
{
  try {
    // automatically generated code.
    
     (getT()->oneStep ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::PathVector_ptr PathPlannerServant<_Base, _Storage>::computePath ()
{
  try {
    // automatically generated code.
    
    hpp::core::PathVectorPtr_t __return__ (getT()->computePath ());
    return makeServantDownCast<hpp::corbaServer::core_impl::Path,hpp::corbaServer::core_impl::PathVector>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::PathVector_ptr PathPlannerServant<_Base, _Storage>::finishSolve (hpp::core_idl::PathVector_ptr path)
{
  try {
    // automatically generated code.
    hpp::core::PathVectorPtr_t _path = reference_to_servant_base<hpp::core::PathVector>(server_, path)->get();
    hpp::core::PathVectorPtr_t __return__ (getT()->finishSolve (_path));
    return makeServantDownCast<hpp::corbaServer::core_impl::Path,hpp::corbaServer::core_impl::PathVector>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void PathPlannerServant<_Base, _Storage>::interrupt ()
{
  try {
    // automatically generated code.
    
     (getT()->interrupt ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void PathPlannerServant<_Base, _Storage>::maxIterations (hpp::size_type n)
{
  try {
    // automatically generated code.
    
     (getT()->maxIterations (n));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void PathPlannerServant<_Base, _Storage>::timeOut (hpp::value_type seconds)
{
  try {
    // automatically generated code.
    
     (getT()->timeOut (seconds));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__path__planners_hxx__

