// SPDX-License-Identifier: ISC

// Created by Patrick Kenneally on 5/24/25.
//

#ifndef BASILISK_IMEASUREMENT_H
#define BASILISK_IMEASUREMENT_H

#include "State.hpp"

template <int StateDim>
class IMeasurement {
   public:
    virtual void apply(State<StateDim>& state) const = 0;
    virtual ~IMeasurement() = default;
    virtual double getTimeTag() const = 0;
    virtual bool getValidity() const = 0;
};

#endif  // BASILISK_IMEASUREMENT_H
