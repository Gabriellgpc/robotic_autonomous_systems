#pragma once
#include <cstdio>
#include <cmath>
#include <cstdint>

// xi, yi, thi => ponto e orientação inicial
// xf, yf, thf => ponto e orientação final
// coef : [a0,a1,a2,a3,b0,b1,b2,b3]
void interPoly3(const float xi, 
                const float yi, 
                const float thi, 
                const float xf, 
                const float yf, 
                const float thf, 
                double coef[]);
void pathGenerator(const double coef[], 
                   const uint32_t numPoints, 
                   float x[], float y[], float th[]);
void poly3(const double coef[], 
           const double l, 
           float &x, float &y, float &th);