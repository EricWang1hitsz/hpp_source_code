// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef hpp_constraints_idl__constraints_hh__
#define hpp_constraints_idl__constraints_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_constraints
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_constraints
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_constraints
#endif



#ifndef hpp_constraints_idl__common_hh_EXTERNAL_GUARD__
#define hpp_constraints_idl__common_hh_EXTERNAL_GUARD__
#include <hpp/common-idl.hh>
#endif



#ifdef USE_stub_in_nt_dll
# ifndef USE_core_stub_in_nt_dll
#  define USE_core_stub_in_nt_dll
# endif
# ifndef USE_dyn_stub_in_nt_dll
#  define USE_dyn_stub_in_nt_dll
# endif
#endif

#ifdef _core_attr
# error "A local CPP macro _core_attr has already been defined."
#else
# ifdef  USE_core_stub_in_nt_dll
#  define _core_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _core_attr
# endif
#endif

#ifdef _dyn_attr
# error "A local CPP macro _dyn_attr has already been defined."
#else
# ifdef  USE_dyn_stub_in_nt_dll
#  define _dyn_attr _OMNIORB_NTDLL_IMPORT
# else
#  define _dyn_attr
# endif
#endif





_CORBA_MODULE hpp

_CORBA_MODULE_BEG

  _CORBA_MODULE constraints_idl

  _CORBA_MODULE_BEG

#ifndef __hpp_mconstraints__idl_mDifferentiableFunction__
#define __hpp_mconstraints__idl_mDifferentiableFunction__

    class DifferentiableFunction;
    class _objref_DifferentiableFunction;
    class _impl_DifferentiableFunction;
    
    typedef _objref_DifferentiableFunction* DifferentiableFunction_ptr;
    typedef DifferentiableFunction_ptr DifferentiableFunctionRef;

    class DifferentiableFunction_Helper {
    public:
      typedef DifferentiableFunction_ptr _ptr_type;

      static _ptr_type _nil();
      static _CORBA_Boolean is_nil(_ptr_type);
      static void release(_ptr_type);
      static void duplicate(_ptr_type);
      static void marshalObjRef(_ptr_type, cdrStream&);
      static _ptr_type unmarshalObjRef(cdrStream&);
    };

    typedef _CORBA_ObjRef_Var<_objref_DifferentiableFunction, DifferentiableFunction_Helper> DifferentiableFunction_var;
    typedef _CORBA_ObjRef_OUT_arg<_objref_DifferentiableFunction,DifferentiableFunction_Helper > DifferentiableFunction_out;

#endif

    // interface DifferentiableFunction
    class DifferentiableFunction {
    public:
      // Declarations for this interface type.
      typedef DifferentiableFunction_ptr _ptr_type;
      typedef DifferentiableFunction_var _var_type;

      static _ptr_type _duplicate(_ptr_type);
      static _ptr_type _narrow(::CORBA::Object_ptr);
      static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
      
      static _ptr_type _nil();

      static inline void _marshalObjRef(_ptr_type, cdrStream&);

      static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
        omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
        if (o)
          return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
        else
          return _nil();
      }

      static _core_attr const char* _PD_repoId;

      // Other IDL defined within this scope.
      
    };

    class _objref_DifferentiableFunction :
      public virtual ::CORBA::Object,
      public virtual omniObjRef
    {
    public:
      floatSeq* value(const ::hpp::floatSeq& arg);
      floatSeqSeq* jacobian(const ::hpp::floatSeq& arg);
      size_type inputSize();
      size_type inputDerivativeSize();
      size_type outputSize();
      size_type outputDerivativeSize();
      char* name();
      char* str();

      inline _objref_DifferentiableFunction()  { _PR_setobj(0); }  // nil
      _objref_DifferentiableFunction(omniIOR*, omniIdentity*);

    protected:
      virtual ~_objref_DifferentiableFunction();

      
    private:
      virtual void* _ptrToObjRef(const char*);

      _objref_DifferentiableFunction(const _objref_DifferentiableFunction&);
      _objref_DifferentiableFunction& operator = (const _objref_DifferentiableFunction&);
      // not implemented

      friend class DifferentiableFunction;
    };

    class _pof_DifferentiableFunction : public _OMNI_NS(proxyObjectFactory) {
    public:
      inline _pof_DifferentiableFunction() : _OMNI_NS(proxyObjectFactory)(DifferentiableFunction::_PD_repoId) {}
      virtual ~_pof_DifferentiableFunction();

      virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
      virtual _CORBA_Boolean is_a(const char*) const;
    };

    class _impl_DifferentiableFunction :
      public virtual omniServant
    {
    public:
      virtual ~_impl_DifferentiableFunction();

      virtual floatSeq* value(const ::hpp::floatSeq& arg) = 0;
      virtual floatSeqSeq* jacobian(const ::hpp::floatSeq& arg) = 0;
      virtual size_type inputSize() = 0;
      virtual size_type inputDerivativeSize() = 0;
      virtual size_type outputSize() = 0;
      virtual size_type outputDerivativeSize() = 0;
      virtual char* name() = 0;
      virtual char* str() = 0;
      
    public:  // Really protected, workaround for xlC
      virtual _CORBA_Boolean _dispatch(omniCallHandle&);

    private:
      virtual void* _ptrToInterface(const char*);
      virtual const char* _mostDerivedRepoId();
      
    };


#ifndef __hpp_mconstraints__idl_mImplicit__
#define __hpp_mconstraints__idl_mImplicit__

    class Implicit;
    class _objref_Implicit;
    class _impl_Implicit;
    
    typedef _objref_Implicit* Implicit_ptr;
    typedef Implicit_ptr ImplicitRef;

    class Implicit_Helper {
    public:
      typedef Implicit_ptr _ptr_type;

      static _ptr_type _nil();
      static _CORBA_Boolean is_nil(_ptr_type);
      static void release(_ptr_type);
      static void duplicate(_ptr_type);
      static void marshalObjRef(_ptr_type, cdrStream&);
      static _ptr_type unmarshalObjRef(cdrStream&);
    };

    typedef _CORBA_ObjRef_Var<_objref_Implicit, Implicit_Helper> Implicit_var;
    typedef _CORBA_ObjRef_OUT_arg<_objref_Implicit,Implicit_Helper > Implicit_out;

#endif

    // interface Implicit
    class Implicit {
    public:
      // Declarations for this interface type.
      typedef Implicit_ptr _ptr_type;
      typedef Implicit_var _var_type;

      static _ptr_type _duplicate(_ptr_type);
      static _ptr_type _narrow(::CORBA::Object_ptr);
      static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
      
      static _ptr_type _nil();

      static inline void _marshalObjRef(_ptr_type, cdrStream&);

      static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
        omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
        if (o)
          return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
        else
          return _nil();
      }

      static _core_attr const char* _PD_repoId;

      // Other IDL defined within this scope.
      
    };

    class _objref_Implicit :
      public virtual ::CORBA::Object,
      public virtual omniObjRef
    {
    public:
      DifferentiableFunction_ptr function();
      void setRightHandSideFromConfig(const ::hpp::floatSeq& config);
      void setRightHandSide(const ::hpp::floatSeq& rhs);
      floatSeq* getRightHandSide();
      size_type rightHandSideSize();
      size_type parameterSize();
      floatSeq* rightHandSideAt(::hpp::value_type s);

      inline _objref_Implicit()  { _PR_setobj(0); }  // nil
      _objref_Implicit(omniIOR*, omniIdentity*);

    protected:
      virtual ~_objref_Implicit();

      
    private:
      virtual void* _ptrToObjRef(const char*);

      _objref_Implicit(const _objref_Implicit&);
      _objref_Implicit& operator = (const _objref_Implicit&);
      // not implemented

      friend class Implicit;
    };

    class _pof_Implicit : public _OMNI_NS(proxyObjectFactory) {
    public:
      inline _pof_Implicit() : _OMNI_NS(proxyObjectFactory)(Implicit::_PD_repoId) {}
      virtual ~_pof_Implicit();

      virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
      virtual _CORBA_Boolean is_a(const char*) const;
    };

    class _impl_Implicit :
      public virtual omniServant
    {
    public:
      virtual ~_impl_Implicit();

      virtual DifferentiableFunction_ptr function() = 0;
      virtual void setRightHandSideFromConfig(const ::hpp::floatSeq& config) = 0;
      virtual void setRightHandSide(const ::hpp::floatSeq& rhs) = 0;
      virtual floatSeq* getRightHandSide() = 0;
      virtual size_type rightHandSideSize() = 0;
      virtual size_type parameterSize() = 0;
      virtual floatSeq* rightHandSideAt(::hpp::value_type s) = 0;
      
    public:  // Really protected, workaround for xlC
      virtual _CORBA_Boolean _dispatch(omniCallHandle&);

    private:
      virtual void* _ptrToInterface(const char*);
      virtual const char* _mostDerivedRepoId();
      
    };


#ifndef __hpp_mconstraints__idl_mLockedJoint__
#define __hpp_mconstraints__idl_mLockedJoint__

    class LockedJoint;
    class _objref_LockedJoint;
    class _impl_LockedJoint;
    
    typedef _objref_LockedJoint* LockedJoint_ptr;
    typedef LockedJoint_ptr LockedJointRef;

    class LockedJoint_Helper {
    public:
      typedef LockedJoint_ptr _ptr_type;

      static _ptr_type _nil();
      static _CORBA_Boolean is_nil(_ptr_type);
      static void release(_ptr_type);
      static void duplicate(_ptr_type);
      static void marshalObjRef(_ptr_type, cdrStream&);
      static _ptr_type unmarshalObjRef(cdrStream&);
    };

    typedef _CORBA_ObjRef_Var<_objref_LockedJoint, LockedJoint_Helper> LockedJoint_var;
    typedef _CORBA_ObjRef_OUT_arg<_objref_LockedJoint,LockedJoint_Helper > LockedJoint_out;

#endif

    // interface LockedJoint
    class LockedJoint {
    public:
      // Declarations for this interface type.
      typedef LockedJoint_ptr _ptr_type;
      typedef LockedJoint_var _var_type;

      static _ptr_type _duplicate(_ptr_type);
      static _ptr_type _narrow(::CORBA::Object_ptr);
      static _ptr_type _unchecked_narrow(::CORBA::Object_ptr);
      
      static _ptr_type _nil();

      static inline void _marshalObjRef(_ptr_type, cdrStream&);

      static inline _ptr_type _unmarshalObjRef(cdrStream& s) {
        omniObjRef* o = omniObjRef::_unMarshal(_PD_repoId,s);
        if (o)
          return (_ptr_type) o->_ptrToObjRef(_PD_repoId);
        else
          return _nil();
      }

      static _core_attr const char* _PD_repoId;

      // Other IDL defined within this scope.
      
    };

    class _objref_LockedJoint :
      public virtual _objref_Implicit
    {
    public:
      char* jointName();

      inline _objref_LockedJoint()  { _PR_setobj(0); }  // nil
      _objref_LockedJoint(omniIOR*, omniIdentity*);

    protected:
      virtual ~_objref_LockedJoint();

      
    private:
      virtual void* _ptrToObjRef(const char*);

      _objref_LockedJoint(const _objref_LockedJoint&);
      _objref_LockedJoint& operator = (const _objref_LockedJoint&);
      // not implemented

      friend class LockedJoint;
    };

    class _pof_LockedJoint : public _OMNI_NS(proxyObjectFactory) {
    public:
      inline _pof_LockedJoint() : _OMNI_NS(proxyObjectFactory)(LockedJoint::_PD_repoId) {}
      virtual ~_pof_LockedJoint();

      virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
      virtual _CORBA_Boolean is_a(const char*) const;
    };

    class _impl_LockedJoint :
      public virtual _impl_Implicit
    {
    public:
      virtual ~_impl_LockedJoint();

      virtual char* jointName() = 0;
      
    public:  // Really protected, workaround for xlC
      virtual _CORBA_Boolean _dispatch(omniCallHandle&);

    private:
      virtual void* _ptrToInterface(const char*);
      virtual const char* _mostDerivedRepoId();
      
    };


  _CORBA_MODULE_END

_CORBA_MODULE_END



_CORBA_MODULE POA_hpp
_CORBA_MODULE_BEG

  _CORBA_MODULE constraints_idl
  _CORBA_MODULE_BEG

    class DifferentiableFunction :
      public virtual hpp::constraints_idl::_impl_DifferentiableFunction,
      public virtual ::PortableServer::ServantBase
    {
    public:
      virtual ~DifferentiableFunction();

      inline ::hpp::constraints_idl::DifferentiableFunction_ptr _this() {
        return (::hpp::constraints_idl::DifferentiableFunction_ptr) _do_this(::hpp::constraints_idl::DifferentiableFunction::_PD_repoId);
      }
    };

    class Implicit :
      public virtual hpp::constraints_idl::_impl_Implicit,
      public virtual ::PortableServer::ServantBase
    {
    public:
      virtual ~Implicit();

      inline ::hpp::constraints_idl::Implicit_ptr _this() {
        return (::hpp::constraints_idl::Implicit_ptr) _do_this(::hpp::constraints_idl::Implicit::_PD_repoId);
      }
    };

    class LockedJoint :
      public virtual hpp::constraints_idl::_impl_LockedJoint,
      public virtual Implicit
    {
    public:
      virtual ~LockedJoint();

      inline ::hpp::constraints_idl::LockedJoint_ptr _this() {
        return (::hpp::constraints_idl::LockedJoint_ptr) _do_this(::hpp::constraints_idl::LockedJoint::_PD_repoId);
      }
    };

  _CORBA_MODULE_END

_CORBA_MODULE_END



_CORBA_MODULE OBV_hpp
_CORBA_MODULE_BEG

  _CORBA_MODULE constraints_idl
  _CORBA_MODULE_BEG

  _CORBA_MODULE_END

_CORBA_MODULE_END





#undef _core_attr
#undef _dyn_attr



inline void
hpp::constraints_idl::DifferentiableFunction::_marshalObjRef(::hpp::constraints_idl::DifferentiableFunction_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}


inline void
hpp::constraints_idl::Implicit::_marshalObjRef(::hpp::constraints_idl::Implicit_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}


inline void
hpp::constraints_idl::LockedJoint::_marshalObjRef(::hpp::constraints_idl::LockedJoint_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_constraints
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_constraints
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_constraints
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_constraints
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_constraints
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_constraints
#endif

#endif  // __constraints_hh__

