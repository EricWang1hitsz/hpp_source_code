#ifndef hpp_core_idl____problem_hxx__
#define hpp_core_idl____problem_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/_problem.idl
//

#include <hpp/core_idl/_problem-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::core_idl::Problem
//
namespace core_impl {
template <typename _Base, typename _Storage>
ProblemServant<_Base, _Storage>::ProblemServant(Server* server, const _Storage& s)
  : ServantBase<hpp::core::Problem, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
ProblemServant<_Base, _Storage>::~ProblemServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
void ProblemServant<_Base, _Storage>::setInitConfig (const hpp::floatSeq& init)
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/_problem.idl:25
    ConfigurationPtr_t q (floatSeqToConfigPtr(getT()->robot(), init, true));
    getT()->initConfig (q);

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* ProblemServant<_Base, _Storage>::getInitConfig ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->initConfig ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ProblemServant<_Base, _Storage>::resetGoalConfigs ()
{
  try {
    // automatically generated code.
    
     (getT()->resetGoalConfigs ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::Constraint_ptr ProblemServant<_Base, _Storage>::getConstraints ()
{
  try {
    // automatically generated code.
    
    hpp::core::ConstraintPtr_t __return__ (getT()->constraints ());
    return makeServantDownCast<hpp::corbaServer::core_impl::Constraint,hpp::corbaServer::core_impl::Constraint>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::Distance_ptr ProblemServant<_Base, _Storage>::getDistance ()
{
  try {
    // automatically generated code.
    
    hpp::core::DistancePtr_t __return__ (getT()->distance ());
    return makeServantDownCast<hpp::corbaServer::core_impl::Distance,hpp::corbaServer::core_impl::Distance>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ProblemServant<_Base, _Storage>::setDistance (hpp::core_idl::Distance_ptr d)
{
  try {
    // automatically generated code.
    hpp::core::DistancePtr_t _d = reference_to_servant_base<hpp::core::Distance>(server_, d)->get();
     (getT()->distance (_d));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::SteeringMethod_ptr ProblemServant<_Base, _Storage>::getSteeringMethod ()
{
  try {
    // automatically generated code.
    
    hpp::core::SteeringMethodPtr_t __return__ (getT()->steeringMethod ());
    return makeServantDownCast<hpp::corbaServer::core_impl::SteeringMethod,hpp::corbaServer::core_impl::SteeringMethod>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ProblemServant<_Base, _Storage>::setSteeringMethod (hpp::core_idl::SteeringMethod_ptr d)
{
  try {
    // automatically generated code.
    hpp::core::SteeringMethodPtr_t _d = reference_to_servant_base<hpp::core::SteeringMethod>(server_, d)->get();
     (getT()->steeringMethod (_d));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::PathValidation_ptr ProblemServant<_Base, _Storage>::getPathValidation ()
{
  try {
    // automatically generated code.
    
    hpp::core::PathValidationPtr_t __return__ (getT()->pathValidation ());
    return makeServantDownCast<hpp::corbaServer::core_impl::PathValidation,hpp::corbaServer::core_impl::PathValidation>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ProblemServant<_Base, _Storage>::setPathValidation (hpp::core_idl::PathValidation_ptr d)
{
  try {
    // automatically generated code.
    hpp::core::PathValidationPtr_t _d = reference_to_servant_base<hpp::core::PathValidation>(server_, d)->get();
     (getT()->pathValidation (_d));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::ConfigurationShooter_ptr ProblemServant<_Base, _Storage>::getConfigurationShooter ()
{
  try {
    // automatically generated code.
    
    hpp::core::ConfigurationShooterPtr_t __return__ (getT()->configurationShooter ());
    return makeServantDownCast<hpp::corbaServer::core_impl::ConfigurationShooter,hpp::corbaServer::core_impl::ConfigurationShooter>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ProblemServant<_Base, _Storage>::setConfigurationShooter (hpp::core_idl::ConfigurationShooter_ptr d)
{
  try {
    // automatically generated code.
    hpp::core::ConfigurationShooterPtr_t _d = reference_to_servant_base<hpp::core::ConfigurationShooter>(server_, d)->get();
     (getT()->configurationShooter (_d));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl____problem_hxx__

