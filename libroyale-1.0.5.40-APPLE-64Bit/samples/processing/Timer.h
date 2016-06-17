#if defined (_WIN32) || defined (_WIN64)

#include <windows.h>

#include <iostream>


class Timer
{
    double PCFreq/* = 0.0*/;
    __int64 CounterStart/* = 0*/;
public:
    void StartCounter()
    {
        LARGE_INTEGER li;
        if(!QueryPerformanceFrequency(&li))
            std::cout << "QueryPerformanceFrequency failed!\n";

        PCFreq = double(li.QuadPart)/1000.0;

        QueryPerformanceCounter(&li);
        CounterStart = li.QuadPart;
    }
    double GetCounter()
    {
        LARGE_INTEGER li;
        QueryPerformanceCounter(&li);
        return double(li.QuadPart-CounterStart)/PCFreq;
    }
};

//double Timer::PCFreq = 0.0;
//__int64 Timer::CounterStart = 0;
#else

class Timer
{
public:
    void StartCounter()
    {
        t0 = Clock::now();
    }
    double GetCounter()
    {
        Clock::time_point t1 = Clock::now();
        milliseconds ms = std::chrono::duration_cast<milliseconds>(t1 - t0);

        return ms.count();
    }
private:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::milliseconds milliseconds;
    Clock::time_point t0;
};
#endif