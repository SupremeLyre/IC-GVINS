/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 * Copyright (C) 2024 i2Nav Group, Wuhan University
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef TIME_DELAY_FACTOR_H
#define TIME_DELAY_FACTOR_H

#include <ceres/ceres.h>

#include <cmath>

class TimeDelayFactor : public ceres::SizedCostFunction<1, 1, 1> {

public:
    explicit TimeDelayFactor(double interval) {
        double sigma = 0.0001; // 0.1 ms * sqrt(s)
        double cov   = sigma * sigma * interval;

        sqrt_info_ = 1.0 / std::sqrt(cov);
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        residuals[0] = (parameters[0][0] - parameters[1][0]) * sqrt_info_;

        if (jacobians) {
            if (jacobians[0]) {
                jacobians[0][0] = sqrt_info_;
            }
            if (jacobians[1]) {
                jacobians[1][0] = -sqrt_info_;
            }
        }

        return true;
    }

private:
    double sqrt_info_;
};

#endif // TIME_DELAY_FACTOR_H
