#ifndef hpp_manipulation_idl____problem_hpp__
#define hpp_manipulation_idl____problem_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-manipulation-corba/idl/hpp/manipulation_idl/_problem.idl
//

#include <hpp/manipulation_idl/_problem-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core_idl/_problem.hh>
#include <hpp/manipulation_idl/_graph.hh>
#include <hpp/manipulation/problem.hh>
#include <hpp/manipulation/steering-method/graph.hh>
#include <hpp/manipulation/graph-path-validation.hh>


namespace hpp {

namespace corbaServer {

//
// Class implementing IDL interface hpp::manipulation_idl::Problem
//
namespace manipulation_impl {
template <typename _Base, typename _Storage>
class ProblemServant: public hpp::corbaServer::core_impl::ProblemServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::core_impl::ProblemServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::manipulation_idl::Problem, HppBase);

public:
  // standard constructor
  ProblemServant(Server* server, const _Storage& s);
  virtual ~ProblemServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::manipulation_idl::graph_idl::Graph_ptr getConstraintGraph ();


};

typedef ProblemServant<POA_hpp::manipulation_idl::Problem,hpp::manipulation::ProblemPtr_t > Problem;
} // namespace manipulation_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_manipulation_idl____problem_hpp__

