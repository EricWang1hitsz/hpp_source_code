#ifndef hpp_core_idl__path__validations_hpp__
#define hpp_core_idl__path__validations_hpp__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/path_validations.idl
//

#include <hpp/core_idl/path_validations-idl.hh>
#include "hpp/corbaserver/servant-base.hh"

#include <hpp/core/path-validation.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core_idl/paths.hh>


namespace hpp {

namespace corbaServer {


//
// Class implementing IDL interface hpp::core_idl::PathValidation
//
namespace core_impl {
template <typename _Base, typename _Storage>
class PathValidationServant: public ServantBase<hpp::core::PathValidation, _Storage>, public virtual _Base
{
public:
  typedef hpp::core::PathValidation HppBase;

  SERVANT_BASE_TYPEDEFS(hpp::core_idl::PathValidation, HppBase);

public:
  // standard constructor
  PathValidationServant(Server* server, const _Storage& s);
  virtual ~PathValidationServant();

  // methods corresponding to defined IDL attributes and operations
  
  ::CORBA::Boolean validate (hpp::core_idl::Path_ptr p, ::CORBA::Boolean reverse, hpp::core_idl::Path_out validPart, hpp::core_idl::PathValidationReport_out report);


};

typedef PathValidationServant<POA_hpp::core_idl::PathValidation,hpp::core::PathValidationPtr_t > PathValidation;
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__path__validations_hpp__

