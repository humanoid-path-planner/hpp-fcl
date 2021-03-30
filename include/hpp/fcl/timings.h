//
// Copyright (c) 2021 INRIA
//

#ifndef HPP_FCL_TIMINGS_FWD_H
#define HPP_FCL_TIMINGS_FWD_H

#include <boost/chrono.hpp>

#include "hpp/fcl/fwd.hh"

namespace hpp { namespace fcl {

  struct CPUTimes
  {
    FCL_REAL wall;
    FCL_REAL user;
    FCL_REAL system;
    
    CPUTimes()
    : wall(0)
    , user(0)
    , system(0)
    {}
    
    void clear()
    {
      wall = user = system = 0;
    }
    
  };

  namespace internal
  {
    inline void get_cpu_times(CPUTimes & current)
    {
      using namespace boost::chrono;
      
      process_real_cpu_clock::time_point wall = process_real_cpu_clock::now();
      process_user_cpu_clock::time_point user = process_user_cpu_clock::now();
      process_system_cpu_clock::time_point system = process_system_cpu_clock::now();
      
      current.wall = time_point_cast<nanoseconds>(wall).time_since_epoch().count()*1e-3;
      current.user = time_point_cast<nanoseconds>(user).time_since_epoch().count()*1e-3;
      current.system = time_point_cast<nanoseconds>(system).time_since_epoch().count()*1e-3;
    }
  }

  struct Timer
  {
    
    Timer()
    {
      start();
    }
    
    CPUTimes elapsed() const
    {
      if(m_is_stopped)
        return m_times;
      
      CPUTimes current;
      internal::get_cpu_times(current);
      current.wall -= m_times.wall;
      current.user -= m_times.user;
      current.system -= m_times.system;
      return current;
    }
    
    void start()
    {
      m_is_stopped = false;
      internal::get_cpu_times(m_times);
    }
    
    void stop()
    {
      if(m_is_stopped)
        return;
      m_is_stopped = true;
      
      CPUTimes current;
      internal::get_cpu_times(current);
      m_times.wall = (current.wall - m_times.wall);
      m_times.user = (current.user - m_times.user);
      m_times.system = (current.system - m_times.system);
      
    }
    
    void resume()
    {
      if(m_is_stopped)
      {
        CPUTimes current(m_times);
        start();
        m_times.wall   -= current.wall;
        m_times.user   -= current.user;
        m_times.system -= current.system;
      }
    }
    
    bool is_stopped() const
    {
      return m_is_stopped;
    }
    
  protected:
    
    CPUTimes m_times;
    bool m_is_stopped;
  };

}}

#endif // ifndef HPP_FCL_TIMINGS_FWD_H
