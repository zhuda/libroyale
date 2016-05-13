#pragma once

#ifndef _CONSTANT_H
#define _CONSTANT_H

#include <string>
using namespace std;

#if defined (_WIN32) || defined (_WIN64)
	string datafolder = "\\\\shacomponentnas\\Trans\\Backups\\Danny\\PCL\\data";
	string datafolder2 = "/Users/zhuda/Desktop/github/CamBoard_pico_flexx/libroyale/libroyale-1.0.5.40-APPLE-64Bit/samples/data2";
#else
	string datafolder = "/Users/zhuda/Desktop/github/CamBoard_pico_flexx/libroyale/libroyale-1.0.5.40-APPLE-64Bit/samples/data";
	string datafolder2 = "/Users/zhuda/Desktop/github/CamBoard_pico_flexx/libroyale/libroyale-1.0.5.40-APPLE-64Bit/samples/data2";
#endif
#endif /* end of include guard: _CONSTANT_H */

