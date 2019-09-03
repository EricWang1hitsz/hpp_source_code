#ifndef hpp_constraints_idl__constraints_hxx__
#define hpp_constraints_idl__constraints_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl
//

#include <hpp/constraints_idl/constraints-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::constraints_idl::DifferentiableFunction
//
namespace constraints_impl {
template <typename _Base, typename _Storage>
DifferentiableFunctionServant<_Base, _Storage>::DifferentiableFunctionServant(Server* server, const _Storage& s)
  : ServantBase<hpp::constraints::DifferentiableFunction, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
DifferentiableFunctionServant<_Base, _Storage>::~DifferentiableFunctionServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::floatSeq* DifferentiableFunctionServant<_Base, _Storage>::value (const hpp::floatSeq& arg)
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl:20
    return vectorToFloatSeq (
        (*get()) (floatSeqToVector(arg, get()->inputSize())).vector()
        );

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeqSeq* DifferentiableFunctionServant<_Base, _Storage>::jacobian (const hpp::floatSeq& arg)
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl:20
    matrix_t J (get()->outputDerivativeSize(), get()->inputDerivativeSize());
    get()->jacobian (J, floatSeqToVector(arg));
    return matrixToFloatSeqSeq (J);

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::size_type DifferentiableFunctionServant<_Base, _Storage>::inputSize ()
{
  try {
    // automatically generated code.
    
    return (getT()->inputSize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::size_type DifferentiableFunctionServant<_Base, _Storage>::inputDerivativeSize ()
{
  try {
    // automatically generated code.
    
    return (getT()->inputDerivativeSize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::size_type DifferentiableFunctionServant<_Base, _Storage>::outputSize ()
{
  try {
    // automatically generated code.
    
    return (getT()->outputSize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::size_type DifferentiableFunctionServant<_Base, _Storage>::outputDerivativeSize ()
{
  try {
    // automatically generated code.
    
    return (getT()->outputDerivativeSize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
char* DifferentiableFunctionServant<_Base, _Storage>::name ()
{
  try {
    // automatically generated code.
    
    return ::hpp::corbaServer::c_str (getT()->name ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
char* DifferentiableFunctionServant<_Base, _Storage>::str ()
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

// End of implementational code
} // namespace constraints_impl

//
// Implementational code for IDL interface hpp::constraints_idl::Implicit
//
namespace constraints_impl {
template <typename _Base, typename _Storage>
ImplicitServant<_Base, _Storage>::ImplicitServant(Server* server, const _Storage& s)
  : ServantBase<hpp::constraints::Implicit, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
ImplicitServant<_Base, _Storage>::~ImplicitServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
hpp::constraints_idl::DifferentiableFunction_ptr ImplicitServant<_Base, _Storage>::function ()
{
  try {
    // automatically generated code.
    
    hpp::constraints::DifferentiableFunctionPtr_t __return__ (getT()->functionPtr ());
    return makeServantDownCast<hpp::corbaServer::constraints_impl::DifferentiableFunction,hpp::corbaServer::constraints_impl::DifferentiableFunction>(server_, __return__)._retn();
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
void ImplicitServant<_Base, _Storage>::setRightHandSideFromConfig (const hpp::floatSeq& config)
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
void ImplicitServant<_Base, _Storage>::setRightHandSide (const hpp::floatSeq& rhs)
{
  try {
    // automatically generated code.
    hpp::core::vector_t _rhs = hpp::corbaServer::floatSeqToVector (rhs);
     (getT()->rightHandSide (_rhs));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* ImplicitServant<_Base, _Storage>::getRightHandSide ()
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->rightHandSide ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::size_type ImplicitServant<_Base, _Storage>::rightHandSideSize ()
{
  try {
    // automatically generated code.
    
    return (getT()->rightHandSideSize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::size_type ImplicitServant<_Base, _Storage>::parameterSize ()
{
  try {
    // automatically generated code.
    
    return (getT()->parameterSize ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

template <typename _Base, typename _Storage>
hpp::floatSeq* ImplicitServant<_Base, _Storage>::rightHandSideAt (hpp::value_type s)
{
  try {
    // automatically generated code.
    
    return hpp::corbaServer::vectorToFloatSeq (getT()->rightHandSideAt (s));
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace constraints_impl

//
// Implementational code for IDL interface hpp::constraints_idl::LockedJoint
//
namespace constraints_impl {
template <typename _Base, typename _Storage>
LockedJointServant<_Base, _Storage>::LockedJointServant(Server* server, const _Storage& s)
  : hpp::corbaServer::constraints_impl::ImplicitServant<_Base, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
LockedJointServant<_Base, _Storage>::~LockedJointServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
char* LockedJointServant<_Base, _Storage>::jointName ()
{
  try {
    // automatically generated code.
    
    return ::hpp::corbaServer::c_str (getT()->jointName ());
    
  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace constraints_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_constraints_idl__constraints_hxx__

