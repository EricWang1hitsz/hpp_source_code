# Python stubs generated by omniidl from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl

import omniORB, _omnipy
from omniORB import CORBA, PortableServer
_0_CORBA = CORBA

_omnipy.checkVersion(3,0, __file__)

# #include "hpp/common.idl"
import hpp_stubs.common_idl
_0_hpp = omniORB.openModule("hpp_stubs.hpp")
_0_hpp__POA = omniORB.openModule("hpp_stubs.hpp__POA")

#
# Start of module "hpp"
#
__name__ = "hpp_idl.hpp"
_0_hpp = omniORB.openModule("hpp_idl.hpp", r"/local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl")
_0_hpp__POA = omniORB.openModule("hpp_idl.hpp__POA", r"/local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl")


#
# Start of module "hpp.constraints_idl"
#
__name__ = "hpp_idl.hpp.constraints_idl"
_0_hpp.constraints_idl = omniORB.openModule("hpp_idl.hpp.constraints_idl", r"/local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl")
_0_hpp__POA.constraints_idl = omniORB.openModule("hpp_idl.hpp__POA.constraints_idl", r"/local/devel/hpp/src/hpp-corbaserver/idl/hpp/constraints_idl/constraints.idl")


# interface DifferentiableFunction
_0_hpp.constraints_idl._d_DifferentiableFunction = (omniORB.tcInternal.tv_objref, "IDL:hpp/constraints_idl/DifferentiableFunction:1.0", "DifferentiableFunction")
omniORB.typeMapping["IDL:hpp/constraints_idl/DifferentiableFunction:1.0"] = _0_hpp.constraints_idl._d_DifferentiableFunction
_0_hpp.constraints_idl.DifferentiableFunction = omniORB.newEmptyClass()
class DifferentiableFunction :
    _NP_RepositoryId = _0_hpp.constraints_idl._d_DifferentiableFunction[1]

    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")

    _nil = CORBA.Object._nil


_0_hpp.constraints_idl.DifferentiableFunction = DifferentiableFunction
_0_hpp.constraints_idl._tc_DifferentiableFunction = omniORB.tcInternal.createTypeCode(_0_hpp.constraints_idl._d_DifferentiableFunction)
omniORB.registerType(DifferentiableFunction._NP_RepositoryId, _0_hpp.constraints_idl._d_DifferentiableFunction, _0_hpp.constraints_idl._tc_DifferentiableFunction)

# DifferentiableFunction operations and attributes
DifferentiableFunction._d_value = ((omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], ), (omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
DifferentiableFunction._d_jacobian = ((omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], ), (omniORB.typeMapping["IDL:hpp/floatSeqSeq:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
DifferentiableFunction._d_inputSize = ((), (omniORB.typeMapping["IDL:hpp/size_type:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
DifferentiableFunction._d_inputDerivativeSize = ((), (omniORB.typeMapping["IDL:hpp/size_type:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
DifferentiableFunction._d_outputSize = ((), (omniORB.typeMapping["IDL:hpp/size_type:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
DifferentiableFunction._d_outputDerivativeSize = ((), (omniORB.typeMapping["IDL:hpp/size_type:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
DifferentiableFunction._d_name = ((), ((omniORB.tcInternal.tv_string,0), ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
DifferentiableFunction._d_str = ((), ((omniORB.tcInternal.tv_string,0), ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})

# DifferentiableFunction object reference
class _objref_DifferentiableFunction (CORBA.Object):
    _NP_RepositoryId = DifferentiableFunction._NP_RepositoryId

    def __init__(self):
        CORBA.Object.__init__(self)

    def value(self, *args):
        return _omnipy.invoke(self, "value", _0_hpp.constraints_idl.DifferentiableFunction._d_value, args)

    def jacobian(self, *args):
        return _omnipy.invoke(self, "jacobian", _0_hpp.constraints_idl.DifferentiableFunction._d_jacobian, args)

    def inputSize(self, *args):
        return _omnipy.invoke(self, "inputSize", _0_hpp.constraints_idl.DifferentiableFunction._d_inputSize, args)

    def inputDerivativeSize(self, *args):
        return _omnipy.invoke(self, "inputDerivativeSize", _0_hpp.constraints_idl.DifferentiableFunction._d_inputDerivativeSize, args)

    def outputSize(self, *args):
        return _omnipy.invoke(self, "outputSize", _0_hpp.constraints_idl.DifferentiableFunction._d_outputSize, args)

    def outputDerivativeSize(self, *args):
        return _omnipy.invoke(self, "outputDerivativeSize", _0_hpp.constraints_idl.DifferentiableFunction._d_outputDerivativeSize, args)

    def name(self, *args):
        return _omnipy.invoke(self, "name", _0_hpp.constraints_idl.DifferentiableFunction._d_name, args)

    def str(self, *args):
        return _omnipy.invoke(self, "str", _0_hpp.constraints_idl.DifferentiableFunction._d_str, args)

    __methods__ = ["value", "jacobian", "inputSize", "inputDerivativeSize", "outputSize", "outputDerivativeSize", "name", "str"] + CORBA.Object.__methods__

omniORB.registerObjref(DifferentiableFunction._NP_RepositoryId, _objref_DifferentiableFunction)
_0_hpp.constraints_idl._objref_DifferentiableFunction = _objref_DifferentiableFunction
del DifferentiableFunction, _objref_DifferentiableFunction

# DifferentiableFunction skeleton
__name__ = "hpp_idl.hpp__POA.constraints_idl"
class DifferentiableFunction (PortableServer.Servant):
    _NP_RepositoryId = _0_hpp.constraints_idl.DifferentiableFunction._NP_RepositoryId


    _omni_op_d = {"value": _0_hpp.constraints_idl.DifferentiableFunction._d_value, "jacobian": _0_hpp.constraints_idl.DifferentiableFunction._d_jacobian, "inputSize": _0_hpp.constraints_idl.DifferentiableFunction._d_inputSize, "inputDerivativeSize": _0_hpp.constraints_idl.DifferentiableFunction._d_inputDerivativeSize, "outputSize": _0_hpp.constraints_idl.DifferentiableFunction._d_outputSize, "outputDerivativeSize": _0_hpp.constraints_idl.DifferentiableFunction._d_outputDerivativeSize, "name": _0_hpp.constraints_idl.DifferentiableFunction._d_name, "str": _0_hpp.constraints_idl.DifferentiableFunction._d_str}

DifferentiableFunction._omni_skeleton = DifferentiableFunction
_0_hpp__POA.constraints_idl.DifferentiableFunction = DifferentiableFunction
omniORB.registerSkeleton(DifferentiableFunction._NP_RepositoryId, DifferentiableFunction)
del DifferentiableFunction
__name__ = "hpp_idl.hpp.constraints_idl"

# interface Implicit
_0_hpp.constraints_idl._d_Implicit = (omniORB.tcInternal.tv_objref, "IDL:hpp/constraints_idl/Implicit:1.0", "Implicit")
omniORB.typeMapping["IDL:hpp/constraints_idl/Implicit:1.0"] = _0_hpp.constraints_idl._d_Implicit
_0_hpp.constraints_idl.Implicit = omniORB.newEmptyClass()
class Implicit :
    _NP_RepositoryId = _0_hpp.constraints_idl._d_Implicit[1]

    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")

    _nil = CORBA.Object._nil


_0_hpp.constraints_idl.Implicit = Implicit
_0_hpp.constraints_idl._tc_Implicit = omniORB.tcInternal.createTypeCode(_0_hpp.constraints_idl._d_Implicit)
omniORB.registerType(Implicit._NP_RepositoryId, _0_hpp.constraints_idl._d_Implicit, _0_hpp.constraints_idl._tc_Implicit)

# Implicit operations and attributes
Implicit._d_function = ((), (omniORB.typeMapping["IDL:hpp/constraints_idl/DifferentiableFunction:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
Implicit._d_setRightHandSideFromConfig = ((omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], ), (), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
Implicit._d_setRightHandSide = ((omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], ), (), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
Implicit._d_getRightHandSide = ((), (omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
Implicit._d_rightHandSideSize = ((), (omniORB.typeMapping["IDL:hpp/size_type:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
Implicit._d_parameterSize = ((), (omniORB.typeMapping["IDL:hpp/size_type:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})
Implicit._d_rightHandSideAt = ((omniORB.typeMapping["IDL:hpp/value_type:1.0"], ), (omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})

# Implicit object reference
class _objref_Implicit (CORBA.Object):
    _NP_RepositoryId = Implicit._NP_RepositoryId

    def __init__(self):
        CORBA.Object.__init__(self)

    def function(self, *args):
        return _omnipy.invoke(self, "function", _0_hpp.constraints_idl.Implicit._d_function, args)

    def setRightHandSideFromConfig(self, *args):
        return _omnipy.invoke(self, "setRightHandSideFromConfig", _0_hpp.constraints_idl.Implicit._d_setRightHandSideFromConfig, args)

    def setRightHandSide(self, *args):
        return _omnipy.invoke(self, "setRightHandSide", _0_hpp.constraints_idl.Implicit._d_setRightHandSide, args)

    def getRightHandSide(self, *args):
        return _omnipy.invoke(self, "getRightHandSide", _0_hpp.constraints_idl.Implicit._d_getRightHandSide, args)

    def rightHandSideSize(self, *args):
        return _omnipy.invoke(self, "rightHandSideSize", _0_hpp.constraints_idl.Implicit._d_rightHandSideSize, args)

    def parameterSize(self, *args):
        return _omnipy.invoke(self, "parameterSize", _0_hpp.constraints_idl.Implicit._d_parameterSize, args)

    def rightHandSideAt(self, *args):
        return _omnipy.invoke(self, "rightHandSideAt", _0_hpp.constraints_idl.Implicit._d_rightHandSideAt, args)

    __methods__ = ["function", "setRightHandSideFromConfig", "setRightHandSide", "getRightHandSide", "rightHandSideSize", "parameterSize", "rightHandSideAt"] + CORBA.Object.__methods__

omniORB.registerObjref(Implicit._NP_RepositoryId, _objref_Implicit)
_0_hpp.constraints_idl._objref_Implicit = _objref_Implicit
del Implicit, _objref_Implicit

# Implicit skeleton
__name__ = "hpp_idl.hpp__POA.constraints_idl"
class Implicit (PortableServer.Servant):
    _NP_RepositoryId = _0_hpp.constraints_idl.Implicit._NP_RepositoryId


    _omni_op_d = {"function": _0_hpp.constraints_idl.Implicit._d_function, "setRightHandSideFromConfig": _0_hpp.constraints_idl.Implicit._d_setRightHandSideFromConfig, "setRightHandSide": _0_hpp.constraints_idl.Implicit._d_setRightHandSide, "getRightHandSide": _0_hpp.constraints_idl.Implicit._d_getRightHandSide, "rightHandSideSize": _0_hpp.constraints_idl.Implicit._d_rightHandSideSize, "parameterSize": _0_hpp.constraints_idl.Implicit._d_parameterSize, "rightHandSideAt": _0_hpp.constraints_idl.Implicit._d_rightHandSideAt}

Implicit._omni_skeleton = Implicit
_0_hpp__POA.constraints_idl.Implicit = Implicit
omniORB.registerSkeleton(Implicit._NP_RepositoryId, Implicit)
del Implicit
__name__ = "hpp_idl.hpp.constraints_idl"

# interface LockedJoint
_0_hpp.constraints_idl._d_LockedJoint = (omniORB.tcInternal.tv_objref, "IDL:hpp/constraints_idl/LockedJoint:1.0", "LockedJoint")
omniORB.typeMapping["IDL:hpp/constraints_idl/LockedJoint:1.0"] = _0_hpp.constraints_idl._d_LockedJoint
_0_hpp.constraints_idl.LockedJoint = omniORB.newEmptyClass()
class LockedJoint (_0_hpp.constraints_idl.Implicit):
    _NP_RepositoryId = _0_hpp.constraints_idl._d_LockedJoint[1]

    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")

    _nil = CORBA.Object._nil


_0_hpp.constraints_idl.LockedJoint = LockedJoint
_0_hpp.constraints_idl._tc_LockedJoint = omniORB.tcInternal.createTypeCode(_0_hpp.constraints_idl._d_LockedJoint)
omniORB.registerType(LockedJoint._NP_RepositoryId, _0_hpp.constraints_idl._d_LockedJoint, _0_hpp.constraints_idl._tc_LockedJoint)

# LockedJoint operations and attributes
LockedJoint._d_jointName = ((), ((omniORB.tcInternal.tv_string,0), ), {_0_hpp.Error._NP_RepositoryId: _0_hpp._d_Error})

# LockedJoint object reference
class _objref_LockedJoint (_0_hpp.constraints_idl._objref_Implicit):
    _NP_RepositoryId = LockedJoint._NP_RepositoryId

    def __init__(self):
        _0_hpp.constraints_idl._objref_Implicit.__init__(self)

    def jointName(self, *args):
        return _omnipy.invoke(self, "jointName", _0_hpp.constraints_idl.LockedJoint._d_jointName, args)

    __methods__ = ["jointName"] + _0_hpp.constraints_idl._objref_Implicit.__methods__

omniORB.registerObjref(LockedJoint._NP_RepositoryId, _objref_LockedJoint)
_0_hpp.constraints_idl._objref_LockedJoint = _objref_LockedJoint
del LockedJoint, _objref_LockedJoint

# LockedJoint skeleton
__name__ = "hpp_idl.hpp__POA.constraints_idl"
class LockedJoint (_0_hpp__POA.constraints_idl.Implicit):
    _NP_RepositoryId = _0_hpp.constraints_idl.LockedJoint._NP_RepositoryId


    _omni_op_d = {"jointName": _0_hpp.constraints_idl.LockedJoint._d_jointName}
    _omni_op_d.update(_0_hpp__POA.constraints_idl.Implicit._omni_op_d)

LockedJoint._omni_skeleton = LockedJoint
_0_hpp__POA.constraints_idl.LockedJoint = LockedJoint
omniORB.registerSkeleton(LockedJoint._NP_RepositoryId, LockedJoint)
del LockedJoint
__name__ = "hpp_idl.hpp.constraints_idl"

#
# End of module "hpp.constraints_idl"
#
__name__ = "hpp_idl.hpp"


#
# End of module "hpp"
#
__name__ = "hpp_stubs.constraints.constraints_idl"

_exported_modules = ( "hpp_idl.hpp", "hpp_idl.hpp.constraints_idl")

# The end.