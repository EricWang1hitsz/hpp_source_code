#ifndef hpp_manipulation_idl____problem_hxx__
#define hpp_manipulation_idl____problem_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-manipulation-corba/idl/hpp/manipulation_idl/_problem.idl
//

#include <hpp/manipulation_idl/_problem-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::manipulation_idl::Problem
//
namespace manipulation_impl {
template <typename _Base, typename _Storage>
ProblemServant<_Base, _Storage>::ProblemServant(Server* server, const _Storage& s)
  : hpp::corbaServer::core_impl::ProblemServant<_Base, _Storage> (server, s)
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
hpp::manipulation_idl::graph_idl::Graph_ptr ProblemServant<_Base, _Storage>::getConstraintGraph ()
{
  try {
    // automatically generated code.
    
    hpp::manipulation::graph::GraphPtr_t __return__ (getT()->constraintGraph ());
    return makeServantDownCast<hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::Graph>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace manipulation_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_manipulation_idl____problem_hxx__

