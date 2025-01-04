#pragma once

#include "common/data_convert.hpp"

using connector_common::real;
using connector_common::get_mid;

namespace controller {

enum ControllerType {
    PID = 0,
};

template <typename FdbTypeT, typename RefTypeT, typename OutTypeT>
struct ControllerBaseConfig {
    using FdbType = FdbTypeT;
    using RefType = RefTypeT;
    using OutType = OutTypeT;
};

template <typename ControllerBaseConfigT>
class ControllerBase {
protected:
    using FdbTypeT = typename ControllerBaseConfigT::FdbType;
    using RefTypeT = typename ControllerBaseConfigT::RefType;
    using OutTypeT = typename ControllerBaseConfigT::OutType;
    FdbTypeT fdb_;
    RefTypeT ref_;
    OutTypeT out_;
public:
    FdbTypeT& fdb() { return fdb_; }
    RefTypeT& ref() { return ref_; }
    OutTypeT& out() { return out_; }
    virtual void update() {}
};

template <ControllerType ControllerTypeT, typename ControllerBaseConfigT>
struct ControllerConfig {};

template <typename ControllerConfigT>
class Controller {};

using ControllerBaseConfigAllReal = ControllerBaseConfig<real, real, real>;
using ControllerBaseConfigNone = ControllerBaseConfig<void, void, void>;

struct DataAndDiff {
    real data;
    real diff;
};

}