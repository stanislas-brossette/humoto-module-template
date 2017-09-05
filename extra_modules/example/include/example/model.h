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
/// @brief Wrapper class that handles the model state and its updates
class HUMOTO_LOCAL Model : public humoto::Model
{
  public:
    /// @brief state of the model
    humoto::example::ModelState state_;

  public:
    /// @brief Default constructor
    Model() {}

    /// @brief Update model state
    ///
    /// @param[in] model_state model state
    void updateState(const humoto::ModelState &model_state)
    {
        const humoto::example::ModelState &newState = dynamic_cast<const humoto::example::ModelState &>(model_state);
        state_.com_state_ = newState.com_state_;
    }

    /// @brief Log
    ///
    /// @param[in, out] logger logger
    /// @param[in] parent parent
    /// @param[in] name name
    void log(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED, const LogEntryName &parent = LogEntryName(),
             const std::string &name = "model") const
    {
        LogEntryName subname = parent;
        subname.add(name);

        state_.log(logger, subname, "state");
    }
};
}
}
