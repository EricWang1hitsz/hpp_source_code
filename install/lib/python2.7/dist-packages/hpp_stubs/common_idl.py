# Python stubs generated by omniidl from /local/devel/hpp/src/hpp-corbaserver/idl/hpp/common.idl

import omniORB, _omnipy
from omniORB import CORBA, PortableServer
_0_CORBA = CORBA

_omnipy.checkVersion(3,0, __file__)


#
# Start of module "hpp"
#
__name__ = "hpp_idl.hpp"
_0_hpp = omniORB.openModule("hpp_idl.hpp", r"/local/devel/hpp/src/hpp-corbaserver/idl/hpp/common.idl")
_0_hpp__POA = omniORB.openModule("hpp_idl.hpp__POA", r"/local/devel/hpp/src/hpp-corbaserver/idl/hpp/common.idl")


# typedef ... value_type
class value_type:
    _NP_RepositoryId = "IDL:hpp/value_type:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.value_type = value_type
_0_hpp._d_value_type  = omniORB.tcInternal.tv_double
_0_hpp._ad_value_type = (omniORB.tcInternal.tv_alias, value_type._NP_RepositoryId, "value_type", omniORB.tcInternal.tv_double)
_0_hpp._tc_value_type = omniORB.tcInternal.createTypeCode(_0_hpp._ad_value_type)
omniORB.registerType(value_type._NP_RepositoryId, _0_hpp._ad_value_type, _0_hpp._tc_value_type)
del value_type

# typedef ... size_type
class size_type:
    _NP_RepositoryId = "IDL:hpp/size_type:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.size_type = size_type
_0_hpp._d_size_type  = omniORB.tcInternal.tv_longlong
_0_hpp._ad_size_type = (omniORB.tcInternal.tv_alias, size_type._NP_RepositoryId, "size_type", omniORB.tcInternal.tv_longlong)
_0_hpp._tc_size_type = omniORB.tcInternal.createTypeCode(_0_hpp._ad_size_type)
omniORB.registerType(size_type._NP_RepositoryId, _0_hpp._ad_size_type, _0_hpp._tc_size_type)
del size_type

# typedef ... Names_t
class Names_t:
    _NP_RepositoryId = "IDL:hpp/Names_t:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.Names_t = Names_t
_0_hpp._d_Names_t  = (omniORB.tcInternal.tv_sequence, (omniORB.tcInternal.tv_string,0), 0)
_0_hpp._ad_Names_t = (omniORB.tcInternal.tv_alias, Names_t._NP_RepositoryId, "Names_t", (omniORB.tcInternal.tv_sequence, (omniORB.tcInternal.tv_string,0), 0))
_0_hpp._tc_Names_t = omniORB.tcInternal.createTypeCode(_0_hpp._ad_Names_t)
omniORB.registerType(Names_t._NP_RepositoryId, _0_hpp._ad_Names_t, _0_hpp._tc_Names_t)
del Names_t

# exception Error
_0_hpp.Error = omniORB.newEmptyClass()
class Error (CORBA.UserException):
    _NP_RepositoryId = "IDL:hpp/Error:1.0"

    def __init__(self, msg):
        CORBA.UserException.__init__(self, msg)
        self.msg = msg

_0_hpp.Error = Error
_0_hpp._d_Error  = (omniORB.tcInternal.tv_except, Error, Error._NP_RepositoryId, "Error", "msg", (omniORB.tcInternal.tv_string,0))
_0_hpp._tc_Error = omniORB.tcInternal.createTypeCode(_0_hpp._d_Error)
omniORB.registerType(Error._NP_RepositoryId, _0_hpp._d_Error, _0_hpp._tc_Error)
del Error

# typedef ... boolSeq
class boolSeq:
    _NP_RepositoryId = "IDL:hpp/boolSeq:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.boolSeq = boolSeq
_0_hpp._d_boolSeq  = (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_boolean, 0)
_0_hpp._ad_boolSeq = (omniORB.tcInternal.tv_alias, boolSeq._NP_RepositoryId, "boolSeq", (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_boolean, 0))
_0_hpp._tc_boolSeq = omniORB.tcInternal.createTypeCode(_0_hpp._ad_boolSeq)
omniORB.registerType(boolSeq._NP_RepositoryId, _0_hpp._ad_boolSeq, _0_hpp._tc_boolSeq)
del boolSeq

# typedef ... intSeq
class intSeq:
    _NP_RepositoryId = "IDL:hpp/intSeq:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.intSeq = intSeq
_0_hpp._d_intSeq  = (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_long, 0)
_0_hpp._ad_intSeq = (omniORB.tcInternal.tv_alias, intSeq._NP_RepositoryId, "intSeq", (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_long, 0))
_0_hpp._tc_intSeq = omniORB.tcInternal.createTypeCode(_0_hpp._ad_intSeq)
omniORB.registerType(intSeq._NP_RepositoryId, _0_hpp._ad_intSeq, _0_hpp._tc_intSeq)
del intSeq

# typedef ... intSeqSeq
class intSeqSeq:
    _NP_RepositoryId = "IDL:hpp/intSeqSeq:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.intSeqSeq = intSeqSeq
_0_hpp._d_intSeqSeq  = (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:hpp/intSeq:1.0"], 0)
_0_hpp._ad_intSeqSeq = (omniORB.tcInternal.tv_alias, intSeqSeq._NP_RepositoryId, "intSeqSeq", (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:hpp/intSeq:1.0"], 0))
_0_hpp._tc_intSeqSeq = omniORB.tcInternal.createTypeCode(_0_hpp._ad_intSeqSeq)
omniORB.registerType(intSeqSeq._NP_RepositoryId, _0_hpp._ad_intSeqSeq, _0_hpp._tc_intSeqSeq)
del intSeqSeq

# typedef ... floatSeq
class floatSeq:
    _NP_RepositoryId = "IDL:hpp/floatSeq:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.floatSeq = floatSeq
_0_hpp._d_floatSeq  = (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_double, 0)
_0_hpp._ad_floatSeq = (omniORB.tcInternal.tv_alias, floatSeq._NP_RepositoryId, "floatSeq", (omniORB.tcInternal.tv_sequence, omniORB.tcInternal.tv_double, 0))
_0_hpp._tc_floatSeq = omniORB.tcInternal.createTypeCode(_0_hpp._ad_floatSeq)
omniORB.registerType(floatSeq._NP_RepositoryId, _0_hpp._ad_floatSeq, _0_hpp._tc_floatSeq)
del floatSeq

# typedef ... floatSeqSeq
class floatSeqSeq:
    _NP_RepositoryId = "IDL:hpp/floatSeqSeq:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.floatSeqSeq = floatSeqSeq
_0_hpp._d_floatSeqSeq  = (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], 0)
_0_hpp._ad_floatSeqSeq = (omniORB.tcInternal.tv_alias, floatSeqSeq._NP_RepositoryId, "floatSeqSeq", (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:hpp/floatSeq:1.0"], 0))
_0_hpp._tc_floatSeqSeq = omniORB.tcInternal.createTypeCode(_0_hpp._ad_floatSeqSeq)
omniORB.registerType(floatSeqSeq._NP_RepositoryId, _0_hpp._ad_floatSeqSeq, _0_hpp._tc_floatSeqSeq)
del floatSeqSeq

# typedef ... Transform_
class Transform_:
    _NP_RepositoryId = "IDL:hpp/Transform_:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.Transform_ = Transform_
_0_hpp._d_Transform_  = (omniORB.tcInternal.tv_array, omniORB.tcInternal.tv_double, 7)
_0_hpp._ad_Transform_ = (omniORB.tcInternal.tv_alias, Transform_._NP_RepositoryId, "Transform_", (omniORB.tcInternal.tv_array, omniORB.tcInternal.tv_double, 7))
_0_hpp._tc_Transform_ = omniORB.tcInternal.createTypeCode(_0_hpp._ad_Transform_)
omniORB.registerType(Transform_._NP_RepositoryId, _0_hpp._ad_Transform_, _0_hpp._tc_Transform_)
del Transform_

# typedef ... TransformSeq
class TransformSeq:
    _NP_RepositoryId = "IDL:hpp/TransformSeq:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.TransformSeq = TransformSeq
_0_hpp._d_TransformSeq  = (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:hpp/Transform_:1.0"], 0)
_0_hpp._ad_TransformSeq = (omniORB.tcInternal.tv_alias, TransformSeq._NP_RepositoryId, "TransformSeq", (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:hpp/Transform_:1.0"], 0))
_0_hpp._tc_TransformSeq = omniORB.tcInternal.createTypeCode(_0_hpp._ad_TransformSeq)
omniORB.registerType(TransformSeq._NP_RepositoryId, _0_hpp._ad_TransformSeq, _0_hpp._tc_TransformSeq)
del TransformSeq

# typedef ... Quaternion_
class Quaternion_:
    _NP_RepositoryId = "IDL:hpp/Quaternion_:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_hpp.Quaternion_ = Quaternion_
_0_hpp._d_Quaternion_  = (omniORB.tcInternal.tv_array, omniORB.tcInternal.tv_double, 4)
_0_hpp._ad_Quaternion_ = (omniORB.tcInternal.tv_alias, Quaternion_._NP_RepositoryId, "Quaternion_", (omniORB.tcInternal.tv_array, omniORB.tcInternal.tv_double, 4))
_0_hpp._tc_Quaternion_ = omniORB.tcInternal.createTypeCode(_0_hpp._ad_Quaternion_)
omniORB.registerType(Quaternion_._NP_RepositoryId, _0_hpp._ad_Quaternion_, _0_hpp._tc_Quaternion_)
del Quaternion_

#
# End of module "hpp"
#
__name__ = "hpp_stubs.common_idl"

_exported_modules = ( "hpp_idl.hpp", )

# The end.