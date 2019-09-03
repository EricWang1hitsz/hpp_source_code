#ifndef hpp_manipulation_idl____graph_hpp__
#define hpp_manipulation_idl____graph_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-manipulation-corba/idl/hpp/manipulation_idl/_graph.idl
//

#include <hpp/manipulation_idl/_graph-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/constraints_idl/constraints.hh>
#include <hpp/core_idl/steering_methods.hh>
#include <hpp/core_idl/path_validations.hh>
#include <hpp/manipulation/graph/edge.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <hpp/manipulation/graph/state.hh>
#include <hpp/manipulation/graph/state-selector.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::manipulation_idl::graph_idl::GraphComponent
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
class GraphComponentServant: public ServantBase<hpp::manipulation::graph::GraphComponent, _Storage>, public virtual _Base
{
public:
  typedef hpp::manipulation::graph::GraphComponent HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::manipulation_idl::graph_idl::GraphComponent, HppBase);

public:
  // standard constructor
  GraphComponentServant(Server* server, const _Storage& s);
  virtual ~GraphComponentServant();

  // methods corresponding to defined IDL attributes and operations
  
  char* name ();


  hpp::manipulation_idl::graph_idl::size_t id ();


  hpp::constraints_idl::Implicits* numericalConstraints ();


  hpp::constraints_idl::LockedJoints* lockedJoints ();


};

typedef GraphComponentServant<POA_hpp::manipulation_idl::graph_idl::GraphComponent,hpp::manipulation::graph::GraphComponentPtr_t > GraphComponent;
} // namespace manipulation_impl

} // namespace graph_impl

//
// Class implementing IDL interface hpp::manipulation_idl::graph_idl::StateSelector
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
class StateSelectorServant: public hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::manipulation_idl::graph_idl::StateSelector, HppBase);

public:
  // standard constructor
  StateSelectorServant(Server* server, const _Storage& s);
  virtual ~StateSelectorServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::manipulation_idl::graph_idl::States* getStates ();


};

typedef StateSelectorServant<POA_hpp::manipulation_idl::graph_idl::StateSelector,hpp::manipulation::graph::StateSelectorPtr_t > StateSelector;
} // namespace manipulation_impl

} // namespace graph_impl

//
// Class implementing IDL interface hpp::manipulation_idl::graph_idl::Graph
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
class GraphServant: public hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::manipulation_idl::graph_idl::Graph, HppBase);

public:
  // standard constructor
  GraphServant(Server* server, const _Storage& s);
  virtual ~GraphServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::manipulation_idl::graph_idl::StateSelector_ptr getStateSelector ();


  hpp::manipulation_idl::graph_idl::GraphComponent_ptr get (hpp::manipulation_idl::graph_idl::size_t id);


  hpp::manipulation_idl::graph_idl::size_t nbComponents ();


  void initialize ();


};

typedef GraphServant<POA_hpp::manipulation_idl::graph_idl::Graph,hpp::manipulation::graph::GraphPtr_t > Graph;
} // namespace manipulation_impl

} // namespace graph_impl

//
// Class implementing IDL interface hpp::manipulation_idl::graph_idl::State
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
class StateServant: public hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::manipulation_idl::graph_idl::State, HppBase);

public:
  // standard constructor
  StateServant(Server* server, const _Storage& s);
  virtual ~StateServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::manipulation_idl::graph_idl::Edges* neighborEdges ();


  hpp::manipulation_idl::graph_idl::Edges* hiddenNeighbors ();


};

typedef StateServant<POA_hpp::manipulation_idl::graph_idl::State,hpp::manipulation::graph::StatePtr_t > State;
} // namespace manipulation_impl

} // namespace graph_impl

//
// Class implementing IDL interface hpp::manipulation_idl::graph_idl::Edge
//
namespace manipulation_impl {

namespace graph_impl {
template <typename _Base, typename _Storage>
class EdgeServant: public hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::manipulation_impl::graph_impl::GraphComponentServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::manipulation_idl::graph_idl::Edge, HppBase);

public:
  // standard constructor
  EdgeServant(Server* server, const _Storage& s);
  virtual ~EdgeServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::manipulation_idl::graph_idl::State_ptr from ();


  hpp::manipulation_idl::graph_idl::State_ptr to ();


  hpp::manipulation_idl::graph_idl::State_ptr getState ();


  void setState (hpp::manipulation_idl::graph_idl::State_ptr st);


  hpp::core_idl::SteeringMethod_ptr getSteeringMethod ();


  hpp::core_idl::PathValidation_ptr getPathValidation ();


};

typedef EdgeServant<POA_hpp::manipulation_idl::graph_idl::Edge,hpp::manipulation::graph::EdgePtr_t > Edge;
} // namespace manipulation_impl

} // namespace graph_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_manipulation_idl____graph_hpp__

