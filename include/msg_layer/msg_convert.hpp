#pragma once

#include "common/data_convert.hpp"
#include "msg_layer/msg.hpp"
#include "msg_layer/dynamic_param.hpp"

template <>
inline void connector_common::data_convert
    <con_used_msg::NumRealStruct, con_used_msg::NumReal>
    (const con_used_msg::NumRealStruct& a, con_used_msg::NumReal& b) {
    b.num = a.num;
}


template <>
inline void connector_common::data_convert
    <con_used_msg::NumReal, con_used_msg::NumRealStruct>
    (const con_used_msg::NumReal& a, con_used_msg::NumRealStruct& b) {
    b.num = a.num;
}


template <>
inline void connector_common::data_convert<connector_common::Deg, con_used_msg::AngleRelate>
    (const connector_common::Deg& deg, con_used_msg::AngleRelate& t) {
    t.deg.num = deg.val;
    t.rad.num = deg.val * UsefulNum::DEG2RAD;
}


template <>
inline void connector_common::data_convert
    <con_used_msg::PidParamStruct, con_used_msg::PidParam>
    (const con_used_msg::PidParamStruct& a, con_used_msg::PidParam& b) {
    data_convert(a.kp, b.kp);
    data_convert(a.ki, b.ki);
    data_convert(a.kd, b.kd);
    data_convert(a.error_max, b.error_max);
    data_convert(a.irange, b.irange);
    data_convert(a.outmax, b.outmax);
}


template <>
inline void connector_common::data_convert
    <con_used_msg::LqrParamStruct, con_used_msg::LqrParam>
    (const con_used_msg::LqrParamStruct& a, con_used_msg::LqrParam& b) {
    data_convert(a.kp, b.kp);
    data_convert(a.kd, b.kd);
    data_convert(a.outmax, b.outmax);
}
