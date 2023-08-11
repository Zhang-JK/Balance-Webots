#pragma once
#include "math.h"

// first gen
// inline float poly1[6][4] = {{72.87, 9.1, -138.0f, -5.51},
//                             {51.36, -55.69, -10.38, -0.766},
//                             {-214.44, 232.57, -91.39, -8.40},
//                             {-95.6, 117.69, -61.89, -7.23},
//                             {-100.60, 296.06, -225.38, 66.46},
//                             {-5.6, 21.384, -19.24, 6.67}};
// inline float poly2[6][4] = {{371.86, -330.517, 84.11, 18.616},
//                             {28.3851, -32.294, 15.4, 2.176},
//                             {-66.38, 190.84, -145.756, 45.329},
//                             {-55.79, 130.91, -95.449, 31.58},
//                             {1229.724, -1332.93, 521.082, 62.0725},
//                             {114.07, -126.48, 51.124, -0.96}};

// // second gen ver 1
// inline float polyNew1[6][4] = {{-232, 311, -250, -3.51},
//                                {27.1, -30.3, -20.42, -0.06},
//                                {-257.7, 257, -91.7, -9.75},
//                                {-157.97, 165.8, -72.5, -7.91},
//                                {-471.4, 607, -308, 76.78},
//                                {-28.2, 41.8, -24.7, 8.08}};

// inline float polyNew2[6][4] = {{471.65, -408.45, 98.7, 22.2474},
//                                {50.506, -52.31, 20.4, 2.2218},
//                                {-294.10, 377.17, -190.2, 45.7884},
//                                {-206.31, 260.11, -130.08, 33.5719},
//                                {1750.1, -1700, 626.5, 56.5758},
//                                {163.44, -169.17, 64.2, -0.0697}};

// second gen ver 2
// inline float polyNewNew1[6][4] = {{-237.86, 311.72, -247.9, -3.9243},
//                                   {24.611, -28.364, -20.5, -0.1130},
//                                   {-246.35, 242.56, -85.0, -10.9656},
//                                   {-151.15, 156.76, -68.1, -8.7342},
//                                   {-546.39, 666.31, -319.6, 74.9057},
//                                   {-30.117, 41.842, -23.3, 7.2897}};

// inline float polyNewNew2[6][4] = {{399.96, -339.21, 76.6, 22.3237},
//                                   {43.72, -44.65, 17.1, 2.3114},
//                                   {-340.42, 413.55, -196.9, 44.5761},
//                                   {-241.75, 288.92, -136.7, 32.7444},
//                                   {1678.4, -1700, 583.2, 64.4443},
//                                   {142.88, -146.44, 54.9, 0.6046}};

// second gen ver 3
// inline float polyNewNewNew1[6][4] = {{-257.12, 327.56, -250.9, -3.7897},
//                                      {31.072, -35.04, -21.3, -0.2992},
//                                      {-203.06, 211.85, -80.4, -10.3377},
//                                      {-102.59, 118.26, -59.4, -8.6331},
//                                      {-311.52, 454.28, -260.7, 73.7605},
//                                      {-45.655, 56.213, -28.4, 8.2625}};
// inline float polyNewNewNew2[6][4] = {{643.63, -578.06, 157.9, 18.4212},
//                                      {61.429, -65.375, 27.0, 1.7321},
//                                      {-174.22, 264.21, -155.3, 43.5620},
//                                      {-121.8, 177.92, -104.0, 31.789},
//                                      {1405.6, -1500, 554.0, 60.368},
//                                      {153.71, -160.22, 61.6, 0.1096}};

inline float polyNewNewNewNew1[6][4] = {{-167.76, 250.89, -229.1, -5.8364},
                                        {35.944, -37.627, -21.1, -0.4439},
                                        {-231.62, 230.48, -83.1, -10.6103},
                                        {-131.03, 139.81, -64.0, -8.7658},
                                        {-342.48, 487.47, -271.2, 73.7442},
                                        {-20.56, 34.497, -22.1, 7.8067}};

inline float polyNewNewNewNew2[6][4] = {{519.64, -448.32, 11.19, 21.421},
                                        {46.482, -49.018, 19.8, 2.3156},
                                        {-208.50, 297.36, -164.8, 43.2756},
                                        {-156.53, 212.08, -114.9, 32.0128},
                                        {1607.4, -1600, 579.3, 61.2271},
                                        {146.31, -151.95, 58.7, 0.4260}};

// void fit(float length, float lqrK[2][6])
// {
//     float length2 = pow(length, 2);
//     float length3 = pow(length, 3);

//     for (int j = 0; j < 6; j++)
//     {
//         lqrK[0][j] = poly1[j][0] * length3 + poly1[j][1] * length2 +
//                      poly1[j][2] * length + poly1[j][3];
//     }
//     for (int j = 0; j < 6; j++)
//     {
//         lqrK[1][j] = poly2[j][0] * length3 + poly2[j][1] * length2 +
//                      poly2[j][2] * length + poly2[j][3];
//     }
// }

// new gen ver 1
// void fit(float length, float lqrK[2][6])
// {
//     float length2 = pow(length, 2);
//     float length3 = pow(length, 3);

//     for (int j = 0; j < 6; j++)
//     {
//         lqrK[0][j] = polyNew1[j][0] * length3 + polyNew1[j][1] * length2 +
//                      polyNew1[j][2] * length + polyNew1[j][3];
//     }
//     for (int j = 0; j < 6; j++)
//     {
//         lqrK[1][j] = polyNew2[j][0] * length3 + polyNew2[j][1] * length2 +
//                      polyNew2[j][2] * length + polyNew2[j][3];
//     }
// }

// new gen ver 2
// void fit(float length, float lqrK[2][6])
// {
//     float length2 = pow(length, 2);
//     float length3 = pow(length, 3);

//     for (int j = 0; j < 6; j++)
//     {
//         lqrK[0][j] = polyNewNew1[j][0] * length3 + polyNewNew1[j][1] *
//         length2 +
//                      polyNewNew1[j][2] * length + polyNewNew1[j][3];
//     }
//     for (int j = 0; j < 6; j++)
//     {
//         lqrK[1][j] = polyNewNew2[j][0] * length3 + polyNewNew2[j][1] *
//         length2 +
//                      polyNewNew2[j][2] * length + polyNewNew2[j][3];
//     }
// }

// new gen ver 3

// void fit(float length, float lqrK[2][6])
// {
//     float length2 = pow(length, 2);
//     float length3 = pow(length, 3);

//     for (int j = 0; j < 6; j++)
//     {
//         lqrK[0][j] = polyNewNewNew1[j][0] * length3 +
//                      polyNewNewNew1[j][1] * length2 +
//                      polyNewNewNew1[j][2] * length + polyNewNewNew1[j][3];
//     }
//     for (int j = 0; j < 6; j++)
//     {
//         lqrK[1][j] = polyNewNewNew2[j][0] * length3 +
//                      polyNewNewNew2[j][1] * length2 +
//                      polyNewNewNew2[j][2] * length + polyNewNewNew2[j][3];
//     }
// }

// new gen ver 4

void fit(float length, float lqrK[2][6])
{
    float length2 = powf(length, 2);
    float length3 = powf(length, 3);

    for (int j = 0; j < 6; j++)
    {
        lqrK[0][j] = polyNewNewNewNew1[j][0] * length3 +
                     polyNewNewNewNew1[j][1] * length2 +
                     polyNewNewNewNew1[j][2] * length + polyNewNewNewNew1[j][3];
    }
    for (int j = 0; j < 6; j++)
    {
        lqrK[1][j] = polyNewNewNewNew2[j][0] * length3 +
                     polyNewNewNewNew2[j][1] * length2 +
                     polyNewNewNewNew2[j][2] * length + polyNewNewNewNew2[j][3];
    }
}

// void fit(float length, float lqrK[2][6])
// {

// float lengtrh2 = pow(length, 2);
// float length3 = pow(length, 3);

// for (int j = 0; j < 6; j++)
// {
//     lqrK[0][j] = poly1[j][0] * length3 + poly1[j][1] * length2 +
//                  poly1[j][2] * length + poly1[j][3];
// }
// for (int j = 0; j < 6; j++)
// {
//     lqrK[1][j] = poly2[j][0] * length3 + poly2[j][1] * length2 +
//                  poly2[j][2] * length + poly2[j][3];
//     // }
// }