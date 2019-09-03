#ifndef hpp_core_idl____constraints_hxx__
#define hpp_core_idl____constraints_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/_constraints.idl
//

#include <hpp/core_idl/_constraints-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::core_idl::Constraint
//
namespace core_impl {
template <typename _Base, typename _Storage>
ConstraintServant<_Base, _Storage>::ConstraintServant(Server* server, const _Storage& s)
  : ServantBase<hpp::core::Constraint, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
ConstraintServant<_Base, _Storage>::~ConstraintServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
::CORBA::Boolean ConstraintServant<_Base, _Storage>::apply (const hpp::floatSeq& configin, hpp::floatSeq_out configOut)
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/_constraints.idl:26
    hpp::core::vector_t q = hpp::corbaServer::floatSeqToVector (configin);
    bool ret = get()->apply(q);
    configOut = hpp::corbaServer::vectorToFloatSeq (q);
    return ret;

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
char* ConstraintServant<_Base, _Storage>::name ()
{
  try {
    // automatically generated code.
    
    return ::hpp::corbaServer::c_str (getT()->name ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
char* ConstraintServant<_Base, _Storage>::str ()
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
::CORBA::Boolean ConstraintServant<_Base, _Storage>::isSatisfied (const hpp::floatSeq& config)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _config = hpp::corbaServer::floatSeqToVector (config);
    return (getT()->isSatisfied (_config));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl

//
// Implementational code for IDL interface hpp::core_idl::ConfigProjector
//
namespace core_impl {
template <typename _Base, typename _Storage>
ConfigProjectorServant<_Base, _Storage>::ConfigProjectorServant(Server* server, const _Storage& s)
  : hpp::corbaServer::core_impl::ConstraintServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
ConfigProjectorServant<_Base, _Storage>::~ConfigProjectorServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
void ConfigProjectorServant<_Base, _Storage>::setRightHandSideFromConfig (const hpp::floatSeq& config)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _config = hpp::corbaServer::floatSeqToVector (config);
     (getT()->rightHandSideFromConfig (_config));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ConfigProjectorServant<_Base, _Storage>::setRightHandSideOfConstraintFromConfig (hpp::constraints_idl::Implicit_ptr nm, const hpp::floatSeq& config)
{
  try {
    // automatically generated code.
    hpp::constraints::ImplicitPtr_t _nm = reference_to_servant_base<hpp::constraints::Implicit>(server_, nm)->get();
      hpp::core::vector_t _config = hpp::corbaServer::floatSeqToVector (config);
     (getT()->rightHandSideFromConfig (_nm, _config));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ConfigProjectorServant<_Base, _Storage>::setRightHandSide (const hpp::floatSeq& param)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _param = hpp::corbaServer::floatSeqToVector (param);
     (getT()->rightHandSide (_param));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ConfigProjectorServant<_Base, _Storage>::setRightHandSideOfConstraint (hpp::constraints_idl::Implicit_ptr nm, const hpp::floatSeq& rhs)
{
  try {
    // automatically generated code.
    hpp::constraints::ImplicitPtr_t _nm = reference_to_servant_base<hpp::constraints::Implicit>(server_, nm)->get();
      hpp::core::vector_t _rhs = hpp::corbaServer::floatSeqToVector (rhs);
     (getT()->rightHandSide (_nm, _rhs));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* ConfigProjectorServant<_Base, _Storage>::getRightHandSide ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->rightHandSide ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ConfigProjectorServant<_Base, _Storage>::setRightHandSideAt (hpp::value_type s)
{
  try {
    // automatically generated code.
    
     (getT()->rightHandSideAt (s));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
::CORBA::Boolean ConfigProjectorServant<_Base, _Storage>::isConstraintSatisfied (hpp::constraints_idl::Implicit_ptr nm, const hpp::floatSeq& arg, hpp::floatSeq_out err, ::CORBA::Boolean& constraintFound)
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/_constraints.idl:39
    hpp::core::vector_t _arg = hpp::corbaServer::floatSeqToVector (arg);
    hpp::constraints::ImplicitPtr_t _nm = reference_to_servant_base<hpp::constraints::Implicit>(server_, nm)->get();
    vector_t errorOut (_nm->function ().outputSpace ()->nv ());
    bool res = getT ()->solver ().isConstraintSatisfied
       (_nm, _arg,errorOut, constraintFound);
    err = hpp::corbaServer::vectorToFloatSeq (errorOut);
    return res;

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl

//
// Implementational code for IDL interface hpp::core_idl::ConstraintSet
//
namespace core_impl {
template <typename _Base, typename _Storage>
ConstraintSetServant<_Base, _Storage>::ConstraintSetServant(Server* server, const _Storage& s)
  : hpp::corbaServer::core_impl::ConstraintServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
ConstraintSetServant<_Base, _Storage>::~ConstraintSetServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::core_idl::Constraint_ptr ConstraintSetServant<_Base, _Storage>::getConfigProjector ()
{
  try {
    // automatically generated code.
    
    hpp::core::ConstraintPtr_t __return__ (getT()->configProjector ());
    return makeServantDownCast<hpp::corbaServer::core_impl::Constraint,hpp::corbaServer::core_impl::Constraint>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl____constraints_hxx__

