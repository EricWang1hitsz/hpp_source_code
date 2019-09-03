#ifndef hpp_core_idl__path__validations_hxx__
#define hpp_core_idl__path__validations_hxx__

//
// Implemention of IDL interfaces in file /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/path_validations.idl
//

#include <hpp/core_idl/path_validations-fwd.hh>

#include <sstream>

#include <hpp/corbaserver/fwd.hh>
#include <hpp/corbaserver/conversions.hh>
#include "hpp/corbaserver/servant-base.hh"

namespace hpp {

namespace corbaServer {

//
// Implementational code for IDL interface hpp::core_idl::PathValidation
//
namespace core_impl {
template <typename _Base, typename _Storage>
PathValidationServant<_Base, _Storage>::PathValidationServant(Server* server, const _Storage& s)
  : ServantBase<hpp::core::PathValidation, _Storage> (server, s)
{
  // add extra constructor code here
}
template <typename _Base, typename _Storage>
PathValidationServant<_Base, _Storage>::~PathValidationServant()
{
  // add extra destructor code here
}

//   Methods corresponding to IDL attributes and operations

template <typename _Base, typename _Storage>
::CORBA::Boolean PathValidationServant<_Base, _Storage>::validate (hpp::core_idl::Path_ptr p, ::CORBA::Boolean reverse, hpp::core_idl::Path_out validPart, hpp::core_idl::PathValidationReport_out report)
{
  try {
    // generated from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/core_idl/path_validations.idl:23
    core::PathPtr_t _p (reference_to_servant_base<core::Path>(server_, p)->get());
    core::PathPtr_t vp;
    core::PathValidationReportPtr_t pvr;

    bool res = get()->validate (_p, reverse, vp, pvr);

    if (pvr) {
      std::ostringstream oss; oss << *pvr;
      std::string res = oss.str();
      report = CORBA::string_dup(res.c_str());
    } else {
      report = CORBA::string_dup("");
    }

    validPart = makeServant<hpp::core_idl::Path_ptr> (server_, new Path (server_, vp));
    return res;

  } catch (const std::exception& e) {
    throw ::hpp::Error (e.what());
  }
}

// End of implementational code
} // namespace core_impl



} // namespace hpp

} // namespace corbaServer

#endif // hpp_core_idl__path__validations_hxx__

