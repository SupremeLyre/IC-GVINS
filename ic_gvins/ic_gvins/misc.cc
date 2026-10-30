/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "misc.h"

#include "common/angle.h"
#include "common/earth.h"
#include "common/logging.h"
#include "common/rotation.h"

#include <algorithm>

size_t MISC::getInsWindowIndex(const std::deque<std::pair<IMU, IntegrationState>> &window, double time) {
    // 返回时间大于输入的第一个索引

    if (window.empty() || (window.front().first.time > time) || (window.back().first.time <= time)) {
        return 0;
    }

    size_t index = 0;
    size_t sta = 0, end = window.size();

    // 二分法搜索
    int counts = 0;
    while (true) {
        auto mid = (sta + end) / 2;

        auto first  = window[mid - 1].first.time;
        auto second = window[mid].first.time;

        if ((first <= time) && (time < second)) {
            index = mid;
            break;
        } else if (first > time) {
            end = mid;
        } else if (second <= time) {
            sta = mid;
        }

        // 65536 / 200 = 327.68
        if (counts++ > 15) {
            LOGE << "Failed to get ins window index at " << Logging::doubleData(time);
            break;
        }
    }

    return index;
}

bool MISC::getCameraPoseFromInsWindow(const std::deque<std::pair<IMU, IntegrationState>> &window, const Pose &pose_b_c,
                                      double time, Pose &pose) {
    // 位置内插
    size_t index = getInsWindowIndex(window, time);
    IntegrationState state;
    if (index > 0) {
        auto state0 = window[index - 1].second;
        auto state1 = window[index].second;

        statePoseInterpolation(state0, state1, time, state);
        pose = stateToCameraPose(state, pose_b_c);
        return true;
    } else {
        pose = stateToCameraPose(window.back().second, pose_b_c);
        return false;
    }
}

void MISC::statePoseInterpolation(const IntegrationState &state0, const IntegrationState &state1, double midtime,
                                  IntegrationState &state) {
    // 仅位姿内插

    Vector3d dp    = state1.p - state0.p;
    Quaterniond dq = state1.q.inverse() * state0.q;
    Vector3d rvec  = Rotation::quaternion2vector(dq);

    double scale = (midtime - state0.time) / (state1.time - state0.time);
    rvec         = rvec * scale;
    dq           = Rotation::rotvec2quaternion(rvec);

    state.p = state0.p + dp * scale;
    state.q = state0.q * dq.inverse();
    state.q.normalize();
}

Pose MISC::stateToCameraPose(const IntegrationState &state, const Pose &pose_b_c) {
    Pose pose;

    pose.t = state.p + state.q.toRotationMatrix() * pose_b_c.t;
    pose.R = state.q.toRotationMatrix() * pose_b_c.R;
    return pose;
}

Eigen::Matrix4d MISC::pose2Twc(const Pose &pose) {
    Eigen::Matrix4d Twc = Eigen::Matrix4d ::Zero();
    Twc(3, 3)           = 1;

    Twc.block<3, 3>(0, 0) = pose.R;
    Twc.block<3, 1>(0, 3) = pose.t;

    return Twc;
}

bool MISC::isTheSameTimeNode(double time0, double time1, double interval) {
    return fabs(time0 - time1) < interval;
}

size_t MISC::getStateDataIndex(const std::deque<double> &timelist, double time, double interval) {
    size_t index = 0;

    size_t sta = 0, end = timelist.size();
    int counts = 0;
    while (true) {
        auto mid      = (sta + end) / 2;
        auto mid_time = timelist[mid];

        if (isTheSameTimeNode(mid_time, time, interval)) {
            index = mid;
            break;
        } else if (mid_time > time) {
            end = mid;
        } else if (mid_time < time) {
            sta = mid;
        }

        if (counts++ > 10) {
            LOGW << "Failed to get state data index at " << Logging::doubleData(time);
            break;
        }
    }

    return index;
}

void MISC::insMechanization(const IntegrationConfiguration &config, const IMU &imu_pre, const IMU &imu_cur,
                            IntegrationState &state) {

    // IMU零偏补偿
    IMU imu_cur2, imu_pre2;
    imu_cur2.dtheta = imu_cur.dtheta - imu_cur.dt * state.bg;
    imu_cur2.dvel   = imu_cur.dvel - imu_cur.dt * state.ba;
    imu_pre2.dtheta = imu_pre.dtheta - imu_pre.dt * state.bg;
    imu_pre2.dvel   = imu_pre.dvel - imu_pre.dt * state.ba;

    if (config.iswithscale) {
        for (int k = 0; k < 3; k++) {
            imu_cur2.dtheta[k] *= (1.0 - state.sg[k]);
            imu_cur2.dvel[k] *= (1.0 - state.sa[k]);
            imu_pre2.dtheta[k] *= (1.0 - state.sg[k]);
            imu_pre2.dvel[k] *= (1.0 - state.sa[k]);
        }
    }

    double dt  = imu_cur.dt;
    state.time = imu_cur.time;

    // 双子样
    Vector3d dvfb = imu_cur2.dvel + 0.5 * imu_cur2.dtheta.cross(imu_cur2.dvel) +
                    1.0 / 12.0 * (imu_pre2.dtheta.cross(imu_cur2.dvel) + imu_pre2.dvel.cross(imu_cur2.dtheta));
    Vector3d dtheta = imu_cur2.dtheta + 1.0 / 12.0 * imu_pre2.dtheta.cross(imu_cur2.dtheta);

    // 速度变化量
    Vector3d dvel;

    if (config.iswithearth) {
        // 哥氏项和重力项
        Vector3d dv_cor_g = (config.gravity - 2.0 * config.iewn.cross(state.v)) * dt;

        // 地球自转补偿项
        Vector3d dnn    = -config.iewn * dt;
        Quaterniond qnn = Rotation::rotvec2quaternion(dnn);

        dvel = 0.5 * (Matrix3d::Identity() + qnn.toRotationMatrix()) * state.q.toRotationMatrix() * dvfb + dv_cor_g;

        // 姿态
        state.q = qnn * state.q * Rotation::rotvec2quaternion(dtheta);
        state.q.normalize();
    } else {
        dvel = state.q.toRotationMatrix() * dvfb + config.gravity * dt;

        // 姿态
        state.q *= Rotation::rotvec2quaternion(dtheta);
        state.q.normalize();
    }

    // 前后历元平均速度计算位置
    state.p += dt * state.v + 0.5 * dt * dvel;
    // 最后更新速度
    state.v += dvel;
}

void MISC::redoInsMechanization(const IntegrationConfiguration &config, const IntegrationState &updated_state,
                                double oldest_required_time, std::deque<std::pair<IMU, IntegrationState>> &ins_windows) {
    // 最新的优化过的位姿
    auto state   = updated_state;
    size_t index = getInsWindowIndex(ins_windows, state.time);

    if (index == 0) {
        LOGE << "Failed to get right index in mechanization";
        return;
    }

    double dt = ins_windows.back().first.time - state.time;
    LOGI << "Redo INS mechanization at " << Logging::doubleData(state.time) << " to "
         << Logging::doubleData(ins_windows.back().first.time) << " is " << dt << " s";
    if (dt > 1.0) {
        LOGW << "Do INS mechanization with a long time " << dt << " s";
    }

    IMU imu0 = ins_windows[index - 1].first;
    IMU imu1 = ins_windows[index].first;

    int isneed = isNeedInterpolation(imu0, imu1, state.time);
    if (isneed == -1) {
        // 前一时刻状态为状态量
        insMechanization(config, imu0, imu1, state);
        ins_windows[index].second = state;
    } else if (isneed == 1) {
        // 当前时刻状态为状态量
        state.time                = imu1.time;
        ins_windows[index].second = state;
    } else if (isneed == 2) {
        imuInterpolation(imu1, imu0, imu1, state.time);
        insMechanization(config, imu0, imu1, state);
        ins_windows[index].second = state;
    }

    // 仅更新IMU时间点的状态
    for (size_t k = index + 1; k < ins_windows.size(); k++) {
        imu0 = imu1;

        imu1 = ins_windows[k].first;
        insMechanization(config, imu0, imu1, state);
        ins_windows[k].second = state;
    }

    // Keep the sliding window's IMU history for delayed GNSS insertion.
    // Retain the sample before the oldest state for interpolation.
    while (ins_windows.size() > 2 && ins_windows[1].first.time < oldest_required_time) {
        ins_windows.pop_front();
    }
}

int MISC::isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    double time = mid;

    // 靠近整秒
    if (imu0.time < time && imu1.time > time) {
        double dt = time - imu0.time;

        // 前一个历元接近
        if (dt < MINIMUM_TIME_INTERVAL) {
            return -1;
        }

        // 后一个历元接近
        dt = imu1.time - time;
        if (dt < MINIMUM_TIME_INTERVAL) {
            return 1;
        }

        // 需内插
        return 2;
    }

    return 0;
}

void MISC::imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid) {
    double time = mid;

    double scale = (imu01.time - time) / imu01.dt;
    IMU buff     = imu01;

    imu00.time   = time;
    imu00.dt     = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel   = buff.dvel * (1 - scale);
    imu00.odovel = buff.odovel * (1 - scale);
    imu00.gps_week = buff.gps_week;

    imu11.time   = buff.time;
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
    imu11.gps_week = buff.gps_week;
}

bool MISC::getImuSeriesFromTo(const std::deque<std::pair<IMU, IntegrationState>> &ins_windows, double start, double end,
                              vector<IMU> &series) {
    series.clear();

    // 获得起点和终点附近的索引
    size_t is = getInsWindowIndex(ins_windows, start);
    size_t ie = getInsWindowIndex(ins_windows, end);
    if (!ins_windows.empty() && end == ins_windows.back().first.time) {
        ie = ins_windows.size() - 1;
    }

    if (is == 0 || ie == 0 || end <= start) {
        LOGE << "Failed to get right IMU series " << Logging::doubleData(start) << " to " << Logging::doubleData(end)
             << ", available " << ins_windows.size() << " samples from "
             << (ins_windows.empty() ? "empty" : Logging::doubleData(ins_windows.front().first.time)) << " to "
             << (ins_windows.empty() ? "empty" : Logging::doubleData(ins_windows.back().first.time));
        return false;
    }

    // The first sample seeds preintegration; its increment is not integrated.
    IMU seed = ins_windows[is - 1].first;
    if (start > seed.time) {
        IMU remainder;
        imuInterpolation(ins_windows[is].first, seed, remainder, start);
    }
    seed.time = start;
    series.push_back(seed);

    // Clip each IMU increment to (start, end], including exact sample boundaries.
    for (size_t k = is; k <= ie; k++) {
        IMU imu = ins_windows[k].first;
        double interval_start = std::max(start, ins_windows[k - 1].first.time);
        double interval_end   = std::min(end, imu.time);
        if (interval_end <= interval_start) {
            continue;
        }
        double dt    = interval_end - interval_start;
        double scale = dt / imu.dt;
        imu.time     = interval_end;
        imu.dt       = dt;
        imu.dtheta *= scale;
        imu.dvel *= scale;
        imu.odovel *= scale;
        series.push_back(imu);
    }

    return true;
}

bool MISC::detectZeroVelocity(const vector<IMU> &imu_buffer, double imudatarate, vector<double> &average) {

    auto size          = static_cast<double>(imu_buffer.size());
    double size_invert = 1.0 / size;

    double sum[6];
    double std[6];

    average.resize(6);
    average[0] = average[1] = average[2] = average[3] = average[4] = average[5] = 0;
    for (const auto &imu : imu_buffer) {
        average[0] += imu.dtheta.x();
        average[1] += imu.dtheta.y();
        average[2] += imu.dtheta.z();
        average[3] += imu.dvel.x();
        average[4] += imu.dvel.y();
        average[5] += imu.dvel.z();
    }

    average[0] *= size_invert;
    average[1] *= size_invert;
    average[2] *= size_invert;
    average[3] *= size_invert;
    average[4] *= size_invert;
    average[5] *= size_invert;

    sum[0] = sum[1] = sum[2] = sum[3] = sum[4] = sum[5] = 0;
    for (const auto &imu : imu_buffer) {
        sum[0] += (imu.dtheta.x() - average[0]) * (imu.dtheta.x() - average[0]);
        sum[1] += (imu.dtheta.y() - average[1]) * (imu.dtheta.y() - average[1]);
        sum[2] += (imu.dtheta.z() - average[2]) * (imu.dtheta.z() - average[2]);
        sum[3] += (imu.dvel.x() - average[3]) * (imu.dvel.x() - average[3]);
        sum[4] += (imu.dvel.y() - average[4]) * (imu.dvel.y() - average[4]);
        sum[5] += (imu.dvel.z() - average[5]) * (imu.dvel.z() - average[5]);
    }

    // 速率形式
    std[0] = sqrt(sum[0] * size_invert) * imudatarate;
    std[1] = sqrt(sum[1] * size_invert) * imudatarate;
    std[2] = sqrt(sum[2] * size_invert) * imudatarate;
    std[3] = sqrt(sum[3] * size_invert) * imudatarate;
    std[4] = sqrt(sum[4] * size_invert) * imudatarate;
    std[5] = sqrt(sum[5] * size_invert) * imudatarate;

    if ((std[0] < ZERO_VELOCITY_GYR_THRESHOLD) && (std[1] < ZERO_VELOCITY_GYR_THRESHOLD) &&
        (std[2] < ZERO_VELOCITY_GYR_THRESHOLD) && (std[3] < ZERO_VELOCITY_ACC_THRESHOLD) &&
        (std[4] < ZERO_VELOCITY_ACC_THRESHOLD) && (std[5] < ZERO_VELOCITY_ACC_THRESHOLD)) {

        return true;
    }

    return false;
}

void MISC::writeNavResult(const IntegrationConfiguration &config, const IntegrationState &state, int gps_week,
                          const FileSaver::Ptr &navfile, const FileSaver::Ptr &errfile,
                          const FileSaver::Ptr &trajfile) {
    static int counts = 0;
    if (counts++ % 10) {
        return;
    }

    // 保存结果
    vector<double> result;

    double time  = state.time;
    Pose global  = Earth::local2global(config.origin, Pose{state.q.toRotationMatrix(), state.p});
    Vector3d pos = global.t;
    pos.segment(0, 2) *= R2D;
    Vector3d att = Rotation::matrix2euler(global.R) * R2D;
    Vector3d vel = state.v;
    Vector3d bg  = state.bg * R2D * 3600;
    Vector3d ba  = state.ba * 1e5;

    {
        // navigation file
        result.clear();

        result.push_back(gps_week);
        result.push_back(time);
        result.push_back(pos[0]);
        result.push_back(pos[1]);
        result.push_back(pos[2]);
        result.push_back(vel[0]);
        result.push_back(vel[1]);
        result.push_back(vel[2]);
        result.push_back(att[0]);
        result.push_back(att[1]);
        result.push_back(att[2]);
        navfile->dump(result);
        navfile->flush();
    }

    {
        // imu error file
        result.clear();

        result.push_back(time);
        result.push_back(bg[0]);
        result.push_back(bg[1]);
        result.push_back(bg[2]);
        result.push_back(ba[0]);
        result.push_back(ba[1]);
        result.push_back(ba[2]);

        if (config.iswithscale) {
            Vector3d sg = state.sg * 1e6;
            Vector3d sa = state.sa * 1e6;
            result.push_back(sg[0]);
            result.push_back(sg[1]);
            result.push_back(sg[2]);
            result.push_back(sa[0]);
            result.push_back(sa[1]);
            result.push_back(sa[2]);
        }

        result.push_back(state.sodo);
        errfile->dump(result);
        errfile->flush();
    }

    {
        // trajectory
        result.clear();

        result.push_back(time);
        result.push_back(state.p[0]);
        result.push_back(state.p[1]);
        result.push_back(state.p[2]);
        result.push_back(state.q.x());
        result.push_back(state.q.y());
        result.push_back(state.q.z());
        result.push_back(state.q.w());

        trajfile->dump(result);
    }
}
