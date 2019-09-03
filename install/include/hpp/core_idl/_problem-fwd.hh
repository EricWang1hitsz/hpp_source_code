#ifndef hpp_core_idl____problem_hpp__
#define hpp_core_idl____problem_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/_problem.idl
//

#include <hpp/core_idl/_problem-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core/problem.hh>
#include <hpp/core_idl/distances.hh>
#include <hpp/core_idl/steering_methods.hh>
#include <hpp/core_idl/path_validations.hh>
#include <hpp/core_idl/configuration_shooters.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::core_idl::Problem
//
namespace core_impl {
template <typename _Base, typename _Storage>
class ProblemServant: public ServantBase<hpp::core::Problem, _Storage>, public virtual _Base
{
public:
  typedef hpp::core::Problem HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::Problem, HppBase);

public:
  // standard constructor
  ProblemServant(Server* server, const _Storage& s);
  virtual ~ProblemServant();

  // methods corresponding to defined IDL attributes and operations
  
  void setInitConfig (const hpp::floatSeq& init);


  hpp::floatSeq* getInitConfig ();


  void resetGoalConfigs ();


  hpp::core_idl::Constraint_ptr getConstraints ();


  hpp::core_idl::Distance_ptr getDistance ();


  void setDistance (hpp::core_idl::Distance_ptr d);


  hpp::core_idl::SteeringMethod_ptr getSteeringMethod ();


  void setSteeringMethod (hpp::core_idl::SteeringMethod_ptr d);


  hpp::core_idl::PathValidation_ptr getPathValidation ();


  void setPathValidation (hpp::core_idl::PathValidation_ptr d);


  hpp::core_idl::ConfigurationShooter_ptr getConfigurationShooter ();


  void setConfigurationShooter (hpp::core_idl::ConfigurationShooter_ptr d);


};

typedef ProblemServant<POA_hpp::core_idl::Problem,hpp::core::ProblemPtr_t > Problem;
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl____problem_hpp__

