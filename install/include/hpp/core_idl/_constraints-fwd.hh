#ifndef hpp_core_idl____constraints_hpp__
#define hpp_core_idl____constraints_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/_constraints.idl
//

#include <hpp/core_idl/_constraints-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core/constraint.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints_idl/constraints.hh>
#include <hpp/constraints/solver/by-substitution.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::core_idl::Constraint
//
namespace core_impl {
template <typename _Base, typename _Storage>
class ConstraintServant: public ServantBase<hpp::core::Constraint, _Storage>, public virtual _Base
{
public:
  typedef hpp::core::Constraint HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::Constraint, HppBase);

public:
  // standard constructor
  ConstraintServant(Server* server, const _Storage& s);
  virtual ~ConstraintServant();

  // methods corresponding to defined IDL attributes and operations
  
  ::CORBA::Boolean apply (const hpp::floatSeq& configin, hpp::floatSeq_out configOut);


  char* name ();


  char* str ();


  ::CORBA::Boolean isSatisfied (const hpp::floatSeq& config);


};

typedef ConstraintServant<POA_hpp::core_idl::Constraint,hpp::core::ConstraintPtr_t > Constraint;
} // namespace core_impl

//
// Class implementing IDL interface hpp::core_idl::ConfigProjector
//
namespace core_impl {
template <typename _Base, typename _Storage>
class ConfigProjectorServant: public hpp::corbaServer::core_impl::ConstraintServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::core_impl::ConstraintServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::ConfigProjector, HppBase);

public:
  // standard constructor
  ConfigProjectorServant(Server* server, const _Storage& s);
  virtual ~ConfigProjectorServant();

  // methods corresponding to defined IDL attributes and operations
  
  void setRightHandSideFromConfig (const hpp::floatSeq& config);


  void setRightHandSideOfConstraintFromConfig (hpp::constraints_idl::Implicit_ptr nm, const hpp::floatSeq& config);


  void setRightHandSide (const hpp::floatSeq& param);


  void setRightHandSideOfConstraint (hpp::constraints_idl::Implicit_ptr nm, const hpp::floatSeq& rhs);


  hpp::floatSeq* getRightHandSide ();


  void setRightHandSideAt (hpp::value_type s);


  ::CORBA::Boolean isConstraintSatisfied (hpp::constraints_idl::Implicit_ptr nm, const hpp::floatSeq& arg, hpp::floatSeq_out err, ::CORBA::Boolean& constraintFound);


};

typedef ConfigProjectorServant<POA_hpp::core_idl::ConfigProjector,hpp::core::ConfigProjectorPtr_t > ConfigProjector;
} // namespace core_impl

//
// Class implementing IDL interface hpp::core_idl::ConstraintSet
//
namespace core_impl {
template <typename _Base, typename _Storage>
class ConstraintSetServant: public hpp::corbaServer::core_impl::ConstraintServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::core_impl::ConstraintServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::ConstraintSet, HppBase);

public:
  // standard constructor
  ConstraintSetServant(Server* server, const _Storage& s);
  virtual ~ConstraintSetServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::core_idl::Constraint_ptr getConfigProjector ();


};

typedef ConstraintSetServant<POA_hpp::core_idl::ConstraintSet,hpp::core::ConstraintSetPtr_t > ConstraintSet;
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl____constraints_hpp__

