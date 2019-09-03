#ifndef hpp_core_idl__path__planners_hpp__
#define hpp_core_idl__path__planners_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/path_planners.idl
//

#include <hpp/core_idl/path_planners-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core/path-planner.hh>
#include <hpp/core_idl/paths.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::core_idl::PathPlanner
//
namespace core_impl {
template <typename _Base, typename _Storage>
class PathPlannerServant: public ServantBase<hpp::core::PathPlanner, _Storage>, public virtual _Base
{
public:
  typedef hpp::core::PathPlanner HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::PathPlanner, HppBase);

public:
  // standard constructor
  PathPlannerServant(Server* server, const _Storage& s);
  virtual ~PathPlannerServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::core_idl::PathVector_ptr solve ();


  void startSolve ();


  void tryConnectInitAndGoals ();


  void oneStep ();


  hpp::core_idl::PathVector_ptr computePath ();


  hpp::core_idl::PathVector_ptr finishSolve (hpp::core_idl::PathVector_ptr path);


  void interrupt ();


  void maxIterations (hpp::size_type n);


  void timeOut (hpp::value_type seconds);


};

typedef PathPlannerServant<POA_hpp::core_idl::PathPlanner,hpp::core::PathPlannerPtr_t > PathPlanner;
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__path__planners_hpp__

