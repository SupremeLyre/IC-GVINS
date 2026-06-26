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

#ifndef IMU_ZERO_VELOCITY_FACTOR_H
#define IMU_ZERO_VELOCITY_FACTOR_H

#include "preintegration/preintegration.h"

#include <ceres/ceres.h>

class ImuZeroVelocityFactor : public ceres::CostFunction {

public:
    ImuZeroVelocityFactor(Preintegration::PreintegrationOptions options, double vel_std)
        : options_(options)
        , vel_std_(vel_std) {
        *mutable_parameter_block_sizes() = vector<int>{Preintegration::numMixParameter(options_)};
        set_num_residuals(3);
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        for (size_t k = 0; k < 3; k++) {
            residuals[k] = parameters[0][k] / vel_std_;
        }

        if ((options_ == Preintegration::PREINTEGRATION_NORMAL) ||
            (options_ == Preintegration::PREINTEGRATION_EARTH)) {
            if (jacobians && jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> jaco(jacobians[0]);
                jaco.setZero();

                for (size_t k = 0; k < 3; k++) {
                    jaco(k, k) = 1.0 / vel_std_;
                }
            }
        } else if ((options_ == Preintegration::PREINTEGRATION_ODO) ||
                   (options_ == Preintegration::PREINTEGRATION_EARTH_ODO)) {
            if (jacobians && jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 10, Eigen::RowMajor>> jaco(jacobians[0]);
                jaco.setZero();

                for (size_t k = 0; k < 3; k++) {
                    jaco(k, k) = 1.0 / vel_std_;
                }
            }
        }

        return true;
    }

private:
    Preintegration::PreintegrationOptions options_;
    double vel_std_;
};

#endif // IMU_ZERO_VELOCITY_FACTOR_H
