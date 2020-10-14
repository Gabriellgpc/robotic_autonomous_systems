#pragma once
#ifndef _COMMON_HPP_
#define _COMMON_HPP_

#include <cstdio>   //FILENAME_MAX, printf
#include <unistd.h> //getcwd
#define GetCurrentDir getcwd

#define CIRCULAR_PATH

namespace COLORS{
    const int RED[3]  = {255,0,0};
    const int GREEN[3]= {0,255,0};
    const int BLUE[3] = {0,0,255}; 
    const int BLACK[3]= {0,0,0};
    const int WHITE[3]= {255,255,255};
}

#endif