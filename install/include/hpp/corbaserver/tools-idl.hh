// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __tools_hh__
#define __tools_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_tools
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_tools
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_tools
#endif



#ifndef __common_hh_EXTERNAL_GUARD__
#define __common_hh_EXTERNAL_GUARD__
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

#ifndef __hpp_mTools__
#define __hpp_mTools__

  class Tools;
  class _objref_Tools;
  class _impl_Tools;
  
  typedef _objref_Tools* Tools_ptr;
  typedef Tools_ptr ToolsRef;

  class Tools_Helper {
  public:
    typedef Tools_ptr _ptr_type;

    static _ptr_type _nil();
    static _CORBA_Boolean is_nil(_ptr_type);
    static void release(_ptr_type);
    static void duplicate(_ptr_type);
    static void marshalObjRef(_ptr_type, cdrStream&);
    static _ptr_type unmarshalObjRef(cdrStream&);
  };

  typedef _CORBA_ObjRef_Var<_objref_Tools, Tools_Helper> Tools_var;
  typedef _CORBA_ObjRef_OUT_arg<_objref_Tools,Tools_Helper > Tools_out;

#endif

  // interface Tools
  class Tools {
  public:
    // Declarations for this interface type.
    typedef Tools_ptr _ptr_type;
    typedef Tools_var _var_type;

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

  class _objref_Tools :
    public virtual ::CORBA::Object,
    public virtual omniObjRef
  {
  public:
    ::CORBA::Boolean loadServerPlugin(const char* context_, const char* pluginName);
    ::CORBA::Boolean createContext(const char* context_);
    void deleteServant(const char* id);
    void shutdown();

    inline _objref_Tools()  { _PR_setobj(0); }  // nil
    _objref_Tools(omniIOR*, omniIdentity*);

  protected:
    virtual ~_objref_Tools();

    
  private:
    virtual void* _ptrToObjRef(const char*);

    _objref_Tools(const _objref_Tools&);
    _objref_Tools& operator = (const _objref_Tools&);
    // not implemented

    friend class Tools;
  };

  class _pof_Tools : public _OMNI_NS(proxyObjectFactory) {
  public:
    inline _pof_Tools() : _OMNI_NS(proxyObjectFactory)(Tools::_PD_repoId) {}
    virtual ~_pof_Tools();

    virtual omniObjRef* newObjRef(omniIOR*,omniIdentity*);
    virtual _CORBA_Boolean is_a(const char*) const;
  };

  class _impl_Tools :
    public virtual omniServant
  {
  public:
    virtual ~_impl_Tools();

    virtual ::CORBA::Boolean loadServerPlugin(const char* context_, const char* pluginName) = 0;
    virtual ::CORBA::Boolean createContext(const char* context_) = 0;
    virtual void deleteServant(const char* id) = 0;
    virtual void shutdown() = 0;
    
  public:  // Really protected, workaround for xlC
    virtual _CORBA_Boolean _dispatch(omniCallHandle&);

  private:
    virtual void* _ptrToInterface(const char*);
    virtual const char* _mostDerivedRepoId();
    
  };


_CORBA_MODULE_END



_CORBA_MODULE POA_hpp
_CORBA_MODULE_BEG

  class Tools :
    public virtual hpp::_impl_Tools,
    public virtual ::PortableServer::ServantBase
  {
  public:
    virtual ~Tools();

    inline ::hpp::Tools_ptr _this() {
      return (::hpp::Tools_ptr) _do_this(::hpp::Tools::_PD_repoId);
    }
  };

_CORBA_MODULE_END



_CORBA_MODULE OBV_hpp
_CORBA_MODULE_BEG

_CORBA_MODULE_END





#undef _core_attr
#undef _dyn_attr



inline void
hpp::Tools::_marshalObjRef(::hpp::Tools_ptr obj, cdrStream& s) {
  omniObjRef::_marshal(obj->_PR_getobj(),s);
}




#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_tools
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_tools
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_tools
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_tools
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_tools
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_tools
#endif

#endif  // __tools_hh__

