/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, INRIA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/// This file defines basic logging macros for Coal, based on Boost.Log.
/// To enable logging, define the preprocessor macro `COAL_ENABLE_LOGGING`.

#ifndef COAL_LOGGING_H
#define COAL_LOGGING_H

#ifdef COAL_ENABLE_LOGGING
#include <boost/log/trivial.hpp>
#define COAL_LOG_INFO(message) BOOST_LOG_TRIVIAL(info) << message
#define COAL_LOG_DEBUG(message) BOOST_LOG_TRIVIAL(debug) << message
#define COAL_LOG_WARNING(message) BOOST_LOG_TRIVIAL(warning) << message
#define COAL_LOG_ERROR(message) BOOST_LOG_TRIVIAL(error) << message
#else
#define COAL_LOG_INFO(message)
#define COAL_LOG_DEBUG(message)
#define COAL_LOG_WARNING(message)
#define COAL_LOG_ERROR(message)
#endif

#endif  // COAL_LOGGING_H
