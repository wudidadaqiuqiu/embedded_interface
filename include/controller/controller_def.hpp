#pragma once

#include "common/data_convert.hpp"

using connector_common::real;
using connector_common::get_mid;

namespace controller {

enum ControllerType {
    PID = 0,
};


template <typename FdbTypeT, typename RefTypeT, typename OutTypeT>
class ControllerBase {
protected:
    FdbTypeT fdb_;
    RefTypeT ref_;
    OutTypeT out_;
public:
    FdbTypeT& fdb() { return fdb_; }
    RefTypeT& ref() { return ref_; }
    OutTypeT& out() { return out_; }
    virtual void update() {}
};

struct DataAndDiff {
    real data;
    real diff;
};

}