#ifndef hpp_manipulation_idl____graph_hxx__
#define hpp_manipulation_idl____graph_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-manipulation-corba/idl/hpp/manipulation_idl/_graph.idl
//

#include <hpp/manipulation_idl/_graph-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::manipulation_idl::graph_idl::GraphComponent
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
GraphComponentServant<_Base, _Storage>::GraphComponentServant(Server* server, const _Storage& s)
  : ServantBase<hpp::manipulation::graph::GraphComponent, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
GraphComponentServant<_Base, _Storage>::~GraphComponentServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
char* GraphComponentServant<_Base, _Storage>::name ()
{
  try {
    // automatically generated code.
    
    return ::hpp::corbaServer::c_str (getT()->name ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::size_t GraphComponentServant<_Base, _Storage>::id ()
{
  try {
    // automatically generated code.
    
    return (getT()->id ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::constraints_idl::Implicits* GraphComponentServant<_Base, _Storage>::numericalConstraints ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToSeqServant<hpp::constraints_idl::Implicits,hpp::corbaServer::constraints_impl::Implicit,hpp::corbaServer::constraints_impl::Implicit>(server_) (getT()->numericalConstraints ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::constraints_idl::LockedJoints* GraphComponentServant<_Base, _Storage>::lockedJoints ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToSeqServant<hpp::constraints_idl::LockedJoints,hpp::corbaServer::constraints_impl::Implicit,hpp::corbaServer::constraints_impl::LockedJoint>(server_) (getT()->lockedJoints ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace manipulation_impl

} // namespace graph_impl

//
// Implementational code for IDL interface hpp::manipulation_idl::graph_idl::StateSelector
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
StateSelectorServant<_Base, _Storage>::StateSelectorServant(Server* server, const _Storage& s)
  : hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
StateSelectorServant<_Base, _Storage>::~StateSelectorServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::States* StateSelectorServant<_Base, _Storage>::getStates ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToSeqServant<hpp::manipulation_idl::graph_idl::States,hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::State>(server_) (getT()->getStates ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace manipulation_impl

} // namespace graph_impl

//
// Implementational code for IDL interface hpp::manipulation_idl::graph_idl::Graph
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
GraphServant<_Base, _Storage>::GraphServant(Server* server, const _Storage& s)
  : hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
GraphServant<_Base, _Storage>::~GraphServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::StateSelector_ptr GraphServant<_Base, _Storage>::getStateSelector ()
{
  try {
    // automatically generated code.
    
    hpp::manipulation::graph::StateSelectorPtr_t __return__ (getT()->stateSelector ());
    return makeServantDownCast<hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::StateSelector>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::GraphComponent_ptr GraphServant<_Base, _Storage>::get (hpp::manipulation_idl::graph_idl::size_t id)
{
  try {
    // automatically generated code.
    
    hpp::manipulation::graph::GraphComponentPtr_t __return__ (getT()->get (id));
    return makeServantDownCast<hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::size_t GraphServant<_Base, _Storage>::nbComponents ()
{
  try {
    // automatically generated code.
    
    return (getT()->nbComponents ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void GraphServant<_Base, _Storage>::initialize ()
{
  try {
    // automatically generated code.
    
     (getT()->initialize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace manipulation_impl

} // namespace graph_impl

//
// Implementational code for IDL interface hpp::manipulation_idl::graph_idl::State
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
StateServant<_Base, _Storage>::StateServant(Server* server, const _Storage& s)
  : hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
StateServant<_Base, _Storage>::~StateServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::Edges* StateServant<_Base, _Storage>::neighborEdges ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToSeqServant<hpp::manipulation_idl::graph_idl::Edges,hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::Edge>(server_) (getT()->neighborEdges ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::Edges* StateServant<_Base, _Storage>::hiddenNeighbors ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToSeqServant<hpp::manipulation_idl::graph_idl::Edges,hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::Edge>(server_) (getT()->hiddenNeighbors ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace manipulation_impl

} // namespace graph_impl

//
// Implementational code for IDL interface hpp::manipulation_idl::graph_idl::Edge
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
EdgeServant<_Base, _Storage>::EdgeServant(Server* server, const _Storage& s)
  : hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
EdgeServant<_Base, _Storage>::~EdgeServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::State_ptr EdgeServant<_Base, _Storage>::from ()
{
  try {
    // automatically generated code.
    
    hpp::manipulation::graph::StatePtr_t __return__ (getT()->from ());
    return makeServantDownCast<hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::State>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::State_ptr EdgeServant<_Base, _Storage>::to ()
{
  try {
    // automatically generated code.
    
    hpp::manipulation::graph::StatePtr_t __return__ (getT()->to ());
    return makeServantDownCast<hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::State>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::manipulation_idl::graph_idl::State_ptr EdgeServant<_Base, _Storage>::getState ()
{
  try {
    // automatically generated code.
    
    hpp::manipulation::graph::StatePtr_t __return__ (getT()->state ());
    return makeServantDownCast<hpp::corbaServer::manipulation_impl::graph_impl::GraphComponent,hpp::corbaServer::manipulation_impl::graph_impl::State>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void EdgeServant<_Base, _Storage>::setState (hpp::manipulation_idl::graph_idl::State_ptr st)
{
  try {
    // automatically generated code.
    hpp::manipulation::graph::StatePtr_t _st = reference_to_servant_base<hpp::manipulation::graph::State>(server_, st)->get();
     (getT()->state (_st));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::core_idl::SteeringMethod_ptr EdgeServant<_Base, _Storage>::getSteeringMethod ()
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
hpp::core_idl::PathValidation_ptr EdgeServant<_Base, _Storage>::getPathValidation ()
{
  try {
    // automatically generated code.
    
    hpp::core::PathValidationPtr_t __return__ (getT()->pathValidation ());
    return makeServantDownCast<hpp::corbaServer::core_impl::PathValidation,hpp::corbaServer::core_impl::PathValidation>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace manipulation_impl

} // namespace graph_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_manipulation_idl____graph_hxx__

