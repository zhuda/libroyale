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