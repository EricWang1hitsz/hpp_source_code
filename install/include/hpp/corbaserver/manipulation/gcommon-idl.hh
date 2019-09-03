// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.
#ifndef __gcommon_hh__
#define __gcommon_hh__

#ifndef __CORBA_H_EXTERNAL_GUARD__
#include <omniORB4/CORBA.h>
#endif

#ifndef  USE_stub_in_nt_dll
# define USE_stub_in_nt_dll_NOT_DEFINED_gcommon
#endif
#ifndef  USE_core_stub_in_nt_dll
# define USE_core_stub_in_nt_dll_NOT_DEFINED_gcommon
#endif
#ifndef  USE_dyn_stub_in_nt_dll
# define USE_dyn_stub_in_nt_dll_NOT_DEFINED_gcommon
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

  typedef ::CORBA::Long ID;
  typedef ::CORBA::Long_out ID_out;

  class IDseq_var;

  class IDseq : public _CORBA_Unbounded_Sequence_w_FixSizeElement< ID, 4, 4 >  {
  public:
    typedef IDseq_var _var_type;
    inline IDseq() {}
    inline IDseq(const IDseq& _s)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ID, 4, 4 > (_s) {}

    inline IDseq(_CORBA_ULong _max)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ID, 4, 4 > (_max) {}
    inline IDseq(_CORBA_ULong _max, _CORBA_ULong _len, ID* _val, _CORBA_Boolean _rel=0)
      : _CORBA_Unbounded_Sequence_w_FixSizeElement< ID, 4, 4 > (_max, _len, _val, _rel) {}

  

    inline IDseq& operator = (const IDseq& _s) {
      _CORBA_Unbounded_Sequence_w_FixSizeElement< ID, 4, 4 > ::operator=(_s);
      return *this;
    }
  };

  class IDseq_out;

  class IDseq_var {
  public:
    inline IDseq_var() : _pd_seq(0) {}
    inline IDseq_var(IDseq* _s) : _pd_seq(_s) {}
    inline IDseq_var(const IDseq_var& _s) {
      if( _s._pd_seq )  _pd_seq = new IDseq(*_s._pd_seq);
      else              _pd_seq = 0;
    }
    inline ~IDseq_var() { if( _pd_seq )  delete _pd_seq; }
      
    inline IDseq_var& operator = (IDseq* _s) {
      if( _pd_seq )  delete _pd_seq;
      _pd_seq = _s;
      return *this;
    }
    inline IDseq_var& operator = (const IDseq_var& _s) {
      if( _s._pd_seq ) {
        if( !_pd_seq )  _pd_seq = new IDseq;
        *_pd_seq = *_s._pd_seq;
      } else if( _pd_seq ) {
        delete _pd_seq;
        _pd_seq = 0;
      }
      return *this;
    }
    inline ID& operator [] (_CORBA_ULong _s) {
      return (*_pd_seq)[_s];
    }

  

    inline IDseq* operator -> () { return _pd_seq; }
    inline const IDseq* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
    inline operator IDseq& () const { return *_pd_seq; }
#else
    inline operator const IDseq& () const { return *_pd_seq; }
    inline operator IDseq& () { return *_pd_seq; }
#endif
      
    inline const IDseq& in() const { return *_pd_seq; }
    inline IDseq&       inout()    { return *_pd_seq; }
    inline IDseq*&      out() {
      if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
      return _pd_seq;
    }
    inline IDseq* _retn() { IDseq* tmp = _pd_seq; _pd_seq = 0; return tmp; }
      
    friend class IDseq_out;
    
  private:
    IDseq* _pd_seq;
  };

  class IDseq_out {
  public:
    inline IDseq_out(IDseq*& _s) : _data(_s) { _data = 0; }
    inline IDseq_out(IDseq_var& _s)
      : _data(_s._pd_seq) { _s = (IDseq*) 0; }
    inline IDseq_out(const IDseq_out& _s) : _data(_s._data) {}
    inline IDseq_out& operator = (const IDseq_out& _s) {
      _data = _s._data;
      return *this;
    }
    inline IDseq_out& operator = (IDseq* _s) {
      _data = _s;
      return *this;
    }
    inline operator IDseq*&()  { return _data; }
    inline IDseq*& ptr()       { return _data; }
    inline IDseq* operator->() { return _data; }

    inline ID& operator [] (_CORBA_ULong _i) {
      return (*_data)[_i];
    }

  

    IDseq*& _data;

  private:
    IDseq_out();
    IDseq_out& operator=(const IDseq_var&);
  };

  struct ConfigProjStat {
    typedef _CORBA_ConstrType_Fix_Var<ConfigProjStat> _var_type;

    
    ::CORBA::Long success;

    ::CORBA::Long error;

    ::CORBA::Long nbObs;

  

    void operator>>= (cdrStream &) const;
    void operator<<= (cdrStream &);
  };

  typedef ConfigProjStat::_var_type ConfigProjStat_var;

  typedef ConfigProjStat& ConfigProjStat_out;

  struct GraphComp {
    typedef _CORBA_ConstrType_Variable_Var<GraphComp> _var_type;

    
    ::CORBA::String_member name;

    ::CORBA::Long id;

    ::CORBA::Long start;

    ::CORBA::Long end;

    IDseq waypoints;

  

    void operator>>= (cdrStream &) const;
    void operator<<= (cdrStream &);
  };

  typedef GraphComp::_var_type GraphComp_var;

  typedef _CORBA_ConstrType_Variable_OUT_arg< GraphComp,GraphComp_var > GraphComp_out;

  class GraphComps_t_var;

  class GraphComps_t : public _CORBA_Unbounded_Sequence< GraphComp >  {
  public:
    typedef GraphComps_t_var _var_type;
    inline GraphComps_t() {}
    inline GraphComps_t(const GraphComps_t& _s)
      : _CORBA_Unbounded_Sequence< GraphComp > (_s) {}

    inline GraphComps_t(_CORBA_ULong _max)
      : _CORBA_Unbounded_Sequence< GraphComp > (_max) {}
    inline GraphComps_t(_CORBA_ULong _max, _CORBA_ULong _len, GraphComp* _val, _CORBA_Boolean _rel=0)
      : _CORBA_Unbounded_Sequence< GraphComp > (_max, _len, _val, _rel) {}

  

    inline GraphComps_t& operator = (const GraphComps_t& _s) {
      _CORBA_Unbounded_Sequence< GraphComp > ::operator=(_s);
      return *this;
    }
  };

  class GraphComps_t_out;

  class GraphComps_t_var {
  public:
    inline GraphComps_t_var() : _pd_seq(0) {}
    inline GraphComps_t_var(GraphComps_t* _s) : _pd_seq(_s) {}
    inline GraphComps_t_var(const GraphComps_t_var& _s) {
      if( _s._pd_seq )  _pd_seq = new GraphComps_t(*_s._pd_seq);
      else              _pd_seq = 0;
    }
    inline ~GraphComps_t_var() { if( _pd_seq )  delete _pd_seq; }
      
    inline GraphComps_t_var& operator = (GraphComps_t* _s) {
      if( _pd_seq )  delete _pd_seq;
      _pd_seq = _s;
      return *this;
    }
    inline GraphComps_t_var& operator = (const GraphComps_t_var& _s) {
      if( _s._pd_seq ) {
        if( !_pd_seq )  _pd_seq = new GraphComps_t;
        *_pd_seq = *_s._pd_seq;
      } else if( _pd_seq ) {
        delete _pd_seq;
        _pd_seq = 0;
      }
      return *this;
    }
    inline GraphComp& operator [] (_CORBA_ULong _s) {
      return (*_pd_seq)[_s];
    }

  

    inline GraphComps_t* operator -> () { return _pd_seq; }
    inline const GraphComps_t* operator -> () const { return _pd_seq; }
#if defined(__GNUG__)
    inline operator GraphComps_t& () const { return *_pd_seq; }
#else
    inline operator const GraphComps_t& () const { return *_pd_seq; }
    inline operator GraphComps_t& () { return *_pd_seq; }
#endif
      
    inline const GraphComps_t& in() const { return *_pd_seq; }
    inline GraphComps_t&       inout()    { return *_pd_seq; }
    inline GraphComps_t*&      out() {
      if( _pd_seq ) { delete _pd_seq; _pd_seq = 0; }
      return _pd_seq;
    }
    inline GraphComps_t* _retn() { GraphComps_t* tmp = _pd_seq; _pd_seq = 0; return tmp; }
      
    friend class GraphComps_t_out;
    
  private:
    GraphComps_t* _pd_seq;
  };

  class GraphComps_t_out {
  public:
    inline GraphComps_t_out(GraphComps_t*& _s) : _data(_s) { _data = 0; }
    inline GraphComps_t_out(GraphComps_t_var& _s)
      : _data(_s._pd_seq) { _s = (GraphComps_t*) 0; }
    inline GraphComps_t_out(const GraphComps_t_out& _s) : _data(_s._data) {}
    inline GraphComps_t_out& operator = (const GraphComps_t_out& _s) {
      _data = _s._data;
      return *this;
    }
    inline GraphComps_t_out& operator = (GraphComps_t* _s) {
      _data = _s;
      return *this;
    }
    inline operator GraphComps_t*&()  { return _data; }
    inline GraphComps_t*& ptr()       { return _data; }
    inline GraphComps_t* operator->() { return _data; }

    inline GraphComp& operator [] (_CORBA_ULong _i) {
      return (*_data)[_i];
    }

  

    GraphComps_t*& _data;

  private:
    GraphComps_t_out();
    GraphComps_t_out& operator=(const GraphComps_t_var&);
  };

  struct GraphElements {
    typedef _CORBA_ConstrType_Variable_Var<GraphElements> _var_type;

    
    GraphComps_t nodes;

    GraphComps_t edges;

  

    void operator>>= (cdrStream &) const;
    void operator<<= (cdrStream &);
  };

  typedef GraphElements::_var_type GraphElements_var;

  typedef _CORBA_ConstrType_Variable_OUT_arg< GraphElements,GraphElements_var > GraphElements_out;

_CORBA_MODULE_END



_CORBA_MODULE POA_hpp
_CORBA_MODULE_BEG

_CORBA_MODULE_END



_CORBA_MODULE OBV_hpp
_CORBA_MODULE_BEG

_CORBA_MODULE_END





#undef _core_attr
#undef _dyn_attr





#ifdef   USE_stub_in_nt_dll_NOT_DEFINED_gcommon
# undef  USE_stub_in_nt_dll
# undef  USE_stub_in_nt_dll_NOT_DEFINED_gcommon
#endif
#ifdef   USE_core_stub_in_nt_dll_NOT_DEFINED_gcommon
# undef  USE_core_stub_in_nt_dll
# undef  USE_core_stub_in_nt_dll_NOT_DEFINED_gcommon
#endif
#ifdef   USE_dyn_stub_in_nt_dll_NOT_DEFINED_gcommon
# undef  USE_dyn_stub_in_nt_dll
# undef  USE_dyn_stub_in_nt_dll_NOT_DEFINED_gcommon
#endif

#endif  // __gcommon_hh__

