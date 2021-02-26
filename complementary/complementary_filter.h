/*

  @author Can Altineller <altineller@gmail.com>
  based on the work of Roberto G. Valenti

*/

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "stdint.h"

class ComplementaryFilter {

  public:
    ComplementaryFilter();

    bool setGainAcc(float gain);
    bool setGainMag(float gain);
    float getGainAcc() const;
    float getGainMag() const;
    float getGain();

    bool setBiasAlpha(float bias_alpha);
    float getBiasAlpha() const;

    // When the filter is in the steady state, bias estimation will occur (if the parameter is enabled).
    bool getSteadyState() const;

    void setDoBiasEstimation(bool do_bias_estimation);
    bool getDoBiasEstimation() const;

    void setDoAdaptiveGain(bool do_adaptive_gain);
    bool getDoAdaptiveGain() const;

    bool setKAngularVelocityThreshold(float thres);
    bool setKAccelerationThreshold(float thres);
    bool setKDeltaAngularVelocityThreshold(float thres);

    bool setSteadyLimit(int limit);

    float getAngularVelocityBiasX() const;
    float getAngularVelocityBiasY() const;
    float getAngularVelocityBiasZ() const;

    // Set the orientation, as a Hamilton Quaternion, of the body frame wrt the fixed frame.
    void setOrientation(float q0, float q1, float q2, float q3);

    // Get the orientation, as a Hamilton Quaternion, of the body frame wrt the fixed frame.
    void getOrientation(float& q0, float& q1, float& q2, float& q3) const;

    // [ax, ay, az]: normalized gravity vector
    // [wx, wy, wz]: angular velocity, in rad/s
    // [mx, my, mz]: magnetic field, units irrelevant
    // dt: time delta, in seconds.

    // update from accelerometer, gyroscope, and magnetometer data.
    void update(float ax, float ay, float az, float wx, float wy, float wz, float mx, float my, float mz, double dt);

    // update from accelerometer and gyroscope
    void update(float ax, float ay, float az, float wx, float wy, float wz, double dt);

  private:
    static const float kGravity;

    // Bias estimation steady state thresholds
    float kAngularVelocityThreshold;
    float kAccelerationThreshold;
    float kDeltaAngularVelocityThreshold;

    // Gain parameter for the complementary filter, belongs in [0, 1].
    float gain_acc_;
    float gain_mag_;

    float gain_;

    // Bias estimation gain parameter, belongs in [0, 1].
    float bias_alpha_;

    // Parameter whether to do bias estimation or not.
    bool do_bias_estimation_;
    
    // Parameter whether to do adaptive gain or not.
    bool do_adaptive_gain_;

    bool initialized_;
    int steady_limit_;

    bool steady_state = false;
    bool steady_state_momentary = false;
    int steady_state_count = 0;

    // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
    // the orientation of the fixed frame wrt the body frame.
    float q0_, q1_, q2_, q3_; 

    // Bias in angular velocities;
    float wx_prev_, wy_prev_, wz_prev_;

    // Bias in angular velocities;
    float wx_bias_, wy_bias_, wz_bias_;

    void updateBiases(float ax, float ay, float az, float wx, float wy, float wz);
    bool checkState(float ax, float ay, float az, float wx, float wy, float wz) const;
    void getPrediction(float wx, float wy, float wz, double dt, float& q0_pred, float& q1_pred, float& q2_pred, float& q3_pred) const;
    void getMeasurement(float ax, float ay, float az, float& q0_meas, float& q1_meas, float& q2_meas, float& q3_meas);
    void getMeasurement(float ax, float ay, float az, float mx, float my, float mz, float& q0_meas, float& q1_meas, float& q2_meas, float& q3_meas);
    void getAccCorrection(float ax, float ay, float az, float p0, float p1, float p2, float p3, float& dq0, float& dq1, float& dq2, float& dq3);
    void getMagCorrection(float mx, float my, float mz, float p0, float p1, float p2, float p3, float& dq0, float& dq1, float& dq2, float& dq3);
    float getAdaptiveGain(float alpha, float ax, float ay, float az);
};

// Utility math functions:
void normalizeVector(float& x, float& y, float& z);
void normalizeQuaternion(float& q0, float& q1, float& q2, float& q3);
void scaleQuaternion(float gain, float& dq0, float& dq1, float& dq2, float& dq3);
void invertQuaternion(float q0, float q1, float q2, float q3, float& q0_inv, float& q1_inv, float& q2_inv, float& q3_inv);
void quaternionMultiplication(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3, float& r0, float& r1, float& r2, float& r3);
void rotateVectorByQuaternion(float x, float y, float z, float q0, float q1, float q2, float q3, float& vx, float& vy, float& vz);

#endif