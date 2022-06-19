/****
 * Salvatore Marcellini
 * Created starting from the work of @markusbuchholz -
 * https://medium.com/geekculture/trajectory-generation-in-c-da36521128aa
****/

#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#pragma once

using Eigen::MatrixXd;
using Eigen::MatrixXf;

// --- 3 order ---
MatrixXf computeCubicCoeff(float t0, float tf, std::vector<float> vec_q0, std::vector<float> vec_qf);
std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> computeCubicTraj(MatrixXf A, float t0, float tf, int n);

// --- 5 order ---
MatrixXf computeQuinticCoeff(float t0, float tf, std::vector<float> vec_q0, std::vector<float> vec_qf);
std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> computeQuinticTraj(MatrixXf A, float t0, float tf, int n);

// --- 7 order ---
MatrixXf compute7orderCoeff(float t0, float t1, float t2, float t3, std::vector<float> vec_q0x, std::vector<float> vec_qwx, std::vector<float> vec_q3x);
std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> computeWaypointTraj(MatrixXf A, float t0, float t3, int n);


