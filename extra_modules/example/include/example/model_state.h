/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
namespace example
{
/// @brief Class containing the model of the system to be controlled
class HUMOTO_LOCAL ModelState : public humoto::ModelState, public humoto::config::ConfigurableBase
{
   protected:
#define HUMOTO_CONFIG_SECTION_ID "ModelState"
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_MEMBER_CLASS(com_state_, "com_state");
#include "humoto/config/define_accessors.h"

    /// @brief Sets the default values of the model state
    void setDefaults()
    {
        com_state_.position_ << 0, 0, 0.8;
        com_state_.velocity_ << 0, 0, 0;
        com_state_.acceleration_ << 0, 0, 0;
    }

   public:
    /// @brief State of the CoM
    humoto::rigidbody::PointMassState com_state_;

   public:
    HUMOTO_DEFINE_CONFIG_CONSTRUCTORS(ModelState)

    /// @brief Default constructor
    ModelState() { setDefaults(); }

    /// @brief Returns the current state in vector form:
    /// [x; dx; ddx; y; dy; ddy; z; dz; ddz]
    etools::Vector9 getStateVector() const
    {
        etools::Vector9 currentState;
        currentState(0) = com_state_.position_(0);
        currentState(1) = com_state_.velocity_(0);
        currentState(2) = com_state_.acceleration_(0);
        currentState(3) = com_state_.position_(1);
        currentState(4) = com_state_.velocity_(1);
        currentState(5) = com_state_.acceleration_(1);
        currentState(6) = com_state_.position_(2);
        currentState(7) = com_state_.velocity_(2);
        currentState(8) = com_state_.acceleration_(2);
        return currentState;
    }

    void updateFromVector(const etools::Vector9 &vec)
    {
        com_state_.position_(0) = vec(0);
        com_state_.velocity_(0) = vec(1);
        com_state_.acceleration_(0) = vec(2);
        com_state_.position_(1) = vec(3);
        com_state_.velocity_(1) = vec(4);
        com_state_.acceleration_(1) = vec(5);
        com_state_.position_(2) = vec(6);
        com_state_.velocity_(2) = vec(7);
        com_state_.acceleration_(2) = vec(8);
    }

    /// @brief Log
    ///
    /// @param[in,out] logger logger
    /// @param[in] parent parent
    /// @param[in] name name
    void log(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED, const LogEntryName &parent = LogEntryName(),
             const std::string &name = "model_state") const
    {
        LogEntryName subname = parent;
        subname.add(name);
        com_state_.log(logger, subname, "com_state");
    }
};
}
}

