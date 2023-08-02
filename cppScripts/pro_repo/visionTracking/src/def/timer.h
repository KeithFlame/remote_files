#pragma once

#include <chrono>
#include <typeinfo>

namespace mtimer
{

    enum TimeUnit
    {
        NANOSECOND = 0,
        MICROSECOND = 1,
        MILLISECOND = 2,
        SECOND = 3,
        MINUTE = 4
        // HOUR
        // DAY
    };

    /** @brief Return time duration in milliseconds since epoch
            */
    template<typename Tp = std::chrono::microseconds>
    Tp getDurationSinceEpoch()
    {
        return ::std::chrono::duration_cast<Tp>(::std::chrono::system_clock::now().time_since_epoch());
    }

    /** @brief Return the duration time from start_time_point, the time unit is milliseconds
    */
    template<typename Tp = std::chrono::microseconds>
    float getDurationSince(const Tp& start_time_point, TimeUnit unit = TimeUnit::MILLISECOND)
    {
        auto current_time_point = ::std::chrono::duration_cast<Tp>(::std::chrono::system_clock::now().time_since_epoch());

        float k[3] = { 0 };
        if (typeid(Tp) == typeid(std::chrono::microseconds))
        {
            k[0] = 1.f;
            k[1] = 1.f / 1000.f;
            k[2] = 1.f / 1000000.f;
        }
        else if (typeid(Tp) == typeid(std::chrono::milliseconds))
        {
            k[0] = 1000.f;
            k[1] = 1.f;
            k[2] = 1.f / 1000.f;
        }
        else if (typeid(Tp) == typeid(std::chrono::seconds))
        {
            k[0] = 1000000.f;
            k[1] = 1000.f;
            k[2] = 1.f;
        }
        switch (unit)
        {
        case TimeUnit::MICROSECOND:
            return (current_time_point - start_time_point).count() * k[0];
        case TimeUnit::MILLISECOND:
            return (current_time_point - start_time_point).count() * k[1];
        case TimeUnit::SECOND:
            return (current_time_point - start_time_point).count() * k[2];
        }
        return (current_time_point - start_time_point).count();
    }
}

