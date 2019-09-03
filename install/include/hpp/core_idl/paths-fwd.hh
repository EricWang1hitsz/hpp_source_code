#ifndef hpp_core_idl__paths_hpp__
#define hpp_core_idl__paths_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/paths.idl
//

#include <hpp/core_idl/paths-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core/path.hh>
#include <hpp/core/path-vector.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::core_idl::Path
//
namespace core_impl {
template <typename _Base, typename _Storage>
class PathServant: public ServantBase<hpp::core::Path, _Storage>, public virtual _Base
{
public:
  typedef hpp::core::Path HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::Path, HppBase);

public:
  // standard constructor
  PathServant(Server* server, const _Storage& s);
  virtual ~PathServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::size_type outputSize ();


  hpp::size_type outputDerivativeSize ();


  hpp::value_type length ();


  hpp::floatSeq* initial ();


  hpp::floatSeq* end ();


  char* str ();


  hpp::floatSeq* call (hpp::value_type t, ::CORBA::Boolean& success);


  hpp::floatSeq* at (hpp::value_type t, ::CORBA::Boolean& success);


  hpp::floatSeq* derivative (hpp::value_type t, ::CORBA::Short order);


  hpp::core_idl::Path_ptr extract (hpp::value_type tmin, hpp::value_type tmax);


  hpp::core_idl::PathVector_ptr asVector ();


};

typedef PathServant<POA_hpp::core_idl::Path,hpp::core::PathPtr_t > Path;
} // namespace core_impl

//
// Class implementing IDL interface hpp::core_idl::PathVector
//
namespace core_impl {
template <typename _Base, typename _Storage>
class PathVectorServant: public hpp::corbaServer::core_impl::PathServant<_Base, _Storage>, public virtual _Base
{
public:
  typedef typename hpp::corbaServer::core_impl::PathServant<_Base, _Storage>::HppBase HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::PathVector, HppBase);

public:
  // standard constructor
  PathVectorServant(Server* server, const _Storage& s);
  virtual ~PathVectorServant();

  // methods corresponding to defined IDL attributes and operations
  
  hpp::core_idl::size_t numberPaths ();


  hpp::core_idl::Path_ptr pathAtRank (hpp::core_idl::size_t rank);


  void appendPath (hpp::core_idl::Path_ptr p);


  void concatenate (hpp::core_idl::PathVector_ptr p);


};

typedef PathVectorServant<POA_hpp::core_idl::PathVector,hpp::core::PathVectorPtr_t > PathVector;
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__paths_hpp__

