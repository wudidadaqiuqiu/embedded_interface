#pragma once

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
inline void connector_common::data_convert
    <con_used_msg::PidParamStruct, con_used_msg::PidParam>
    (const con_used_msg::PidParamStruct& a, con_used_msg::PidParam& b) {
    data_convert<decltype(a.kp), decltype(b.kp)>(a.kp, b.kp);
    data_convert<decltype(a.ki), decltype(b.ki)>(a.ki, b.ki);
    data_convert<decltype(a.kd), decltype(b.kd)>(a.kd, b.kd);
    data_convert<decltype(a.error_max), decltype(b.error_max)>(a.error_max, b.error_max);
    data_convert<decltype(a.irange), decltype(b.irange)>(a.irange, b.irange);
    data_convert<decltype(a.outmax), decltype(b.outmax)>(a.outmax, b.outmax);
}
