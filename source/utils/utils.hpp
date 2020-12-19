#pragma once
#include <cstdio>
#include <cmath>
#include <cstdint>

#define NUM_PARAMETERS 8

// xi, yi, thi => ponto e orientação inicial
// xf, yf, thf => ponto e orientação final
// coef : [a0,a1,a2,a3,b0,b1,b2,b3]
void pathComputing(const float xi,
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
double curvature(const double coef[], const double l);
// função que calcula a integral de linha do polinomio de terceiro grau
double poly3Length(const double coef[], double l);

void pioneer_model(float v, float w, float &w_right, float &w_left);