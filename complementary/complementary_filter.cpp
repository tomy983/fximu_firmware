/*

  @author Can Altineller <altineller@gmail.com>
  based on the work of Roberto G. Valenti

*/

#include "complementary_filter.h"

#include <cmath>

const float ComplementaryFilter::kGravity = 9.80665F;

ComplementaryFilter::ComplementaryFilter():
    gain_acc_(0.02),
    gain_mag_(0.01),
    bias_alpha_(0.1),
    do_bias_estimation_(true),
    do_adaptive_gain_(true),
    initialized_(false),
    steady_limit_(8),
    q0_(1), q1_(0), q2_(0), q3_(0),
    wx_prev_(0), wy_prev_(0), wz_prev_(0),
    wx_bias_(0), wy_bias_(0), wz_bias_(0) { }

void ComplementaryFilter::setDoBiasEstimation(bool do_bias_estimation) {
  do_bias_estimation_ = do_bias_estimation;
}

bool ComplementaryFilter::getDoBiasEstimation() const {
  return do_bias_estimation_;
}

void ComplementaryFilter::setDoAdaptiveGain(bool do_adaptive_gain) {
  do_adaptive_gain_ = do_adaptive_gain;
}

bool ComplementaryFilter::getDoAdaptiveGain() const {
  return do_adaptive_gain_;
}

bool ComplementaryFilter::setGainAcc(float gain) {
  if(gain >= 0 && gain <= 1.0) {
    gain_acc_ = gain;
    return true;
  } else {
    return false;
  }
}

bool ComplementaryFilter::setGainMag(float gain) {
  if(gain >= 0 && gain <= 1.0) {
    gain_mag_ = gain;
    return true;
  } else {
    return false;
  }
}

float ComplementaryFilter::getGainAcc() const  {
  return gain_acc_;
}

float ComplementaryFilter::getGainMag() const {
  return gain_mag_;
}

float ComplementaryFilter::getGain() {
  return gain_;
}

bool ComplementaryFilter::getSteadyState() const {
  return steady_state;
}

bool ComplementaryFilter::setBiasAlpha(float bias_alpha) {
  if(bias_alpha >= 0 && bias_alpha <= 1.0) {
    bias_alpha_ = bias_alpha;
    return true;
  } else {
    return false;
  }
}

float ComplementaryFilter::getBiasAlpha() const {
  return bias_alpha_;
}

bool ComplementaryFilter::setKAngularVelocityThreshold(float thres) {
    if(thres >= 0 && thres <= 1.0) {
        kAngularVelocityThreshold = thres;
        return true;
    } else {
        return false;
    }
}

bool ComplementaryFilter::setKAccelerationThreshold(float thres) {
    if(thres >= 0 && thres <= 1.0) {
        kAccelerationThreshold = thres;
        return true;
    } else {
        return false;
    }
}

bool ComplementaryFilter::setKDeltaAngularVelocityThreshold(float thres) {
    if(thres >= 0 && thres <= 1.0) {
        kDeltaAngularVelocityThreshold = thres;
        return true;
    } else {
        return false;
    }
}


void ComplementaryFilter::setOrientation(float q0, float q1, float q2, float q3) {
  // Set the state to inverse (state is fixed wrt body).
  invertQuaternion(q0, q1, q2, q3, q0_, q1_, q2_, q3_);
}

float ComplementaryFilter::getAngularVelocityBiasX() const {
  return wx_bias_;
}

float ComplementaryFilter::getAngularVelocityBiasY() const {
  return wy_bias_;
}

float ComplementaryFilter::getAngularVelocityBiasZ() const {
  return wz_bias_;
}

void ComplementaryFilter::update(float ax, float ay, float az, float wx, float wy, float wz, double dt) {

  if(!initialized_) {
    // First time - ignore prediction:
    getMeasurement(ax, ay, az, q0_, q1_, q2_, q3_);
    initialized_ = true;
    return;
  }

  // Bias estimation.
  if(do_bias_estimation_) {
    updateBiases(ax, ay, az, wx, wy, wz);
  }

  // Prediction.
  float q0_pred, q1_pred, q2_pred, q3_pred;
  getPrediction(wx, wy, wz, dt, q0_pred, q1_pred, q2_pred, q3_pred);

  // Correction (from acc):
  // q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
  // where qI = identity quaternion
  float dq0_acc, dq1_acc, dq2_acc, dq3_acc;
  getAccCorrection(ax, ay, az, q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

  if(do_adaptive_gain_) {
    gain_ = getAdaptiveGain(gain_acc_, ax, ay, az);
  } else {
    gain_ = gain_acc_;
  }

  scaleQuaternion(gain_, dq0_acc, dq1_acc, dq2_acc, dq3_acc);
  quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc, dq1_acc, dq2_acc, dq3_acc, q0_, q1_, q2_, q3_);
  normalizeQuaternion(q0_, q1_, q2_, q3_);

}

void ComplementaryFilter::update(float ax, float ay, float az, float wx, float wy, float wz, float mx, float my, float mz, double dt) {

  if(!initialized_) {
    // First time - ignore prediction:
    getMeasurement(ax, ay, az, mx, my, mz, q0_, q1_, q2_, q3_);
    initialized_ = true;
    return;
  }

  // Bias estimation.
  if(do_bias_estimation_) {
    updateBiases(ax, ay, az, wx, wy, wz);
  }

  // Prediction.
  float q0_pred, q1_pred, q2_pred, q3_pred;
  getPrediction(wx, wy, wz, dt, q0_pred, q1_pred, q2_pred, q3_pred);
     
  // Correction (from acc): 
  // q_temp = q_pred * [(1-gain) * qI + gain * dq_acc]
  // where qI = identity quaternion
  float dq0_acc, dq1_acc, dq2_acc, dq3_acc;  
  getAccCorrection(ax, ay, az, q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

  if(do_adaptive_gain_) {
    gain_ = getAdaptiveGain(gain_acc_, ax, ay, az);
  } else {
    gain_ = gain_acc_;
  }

  scaleQuaternion(gain_, dq0_acc, dq1_acc, dq2_acc, dq3_acc);

  float q0_temp, q1_temp, q2_temp, q3_temp;
  quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc, dq1_acc, dq2_acc, dq3_acc, q0_temp, q1_temp, q2_temp, q3_temp);
  
  // Correction (from mag):
  // q_ = q_temp * [(1-gain) * qI + gain * dq_mag]
  // where qI = identity quaternion
  float dq0_mag, dq1_mag, dq2_mag, dq3_mag;  
  getMagCorrection(mx, my, mz, q0_temp, q1_temp, q2_temp, q3_temp, dq0_mag, dq1_mag, dq2_mag, dq3_mag);

  scaleQuaternion(gain_mag_, dq0_mag, dq1_mag, dq2_mag, dq3_mag);

  quaternionMultiplication(q0_temp, q1_temp, q2_temp, q3_temp, dq0_mag, dq1_mag, dq2_mag, dq3_mag, q0_, q1_, q2_, q3_);

  normalizeQuaternion(q0_, q1_, q2_, q3_);

}


bool ComplementaryFilter::checkState(float ax, float ay, float az, float wx, float wy, float wz) const {
  float acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
  if(fabs(acc_magnitude - kGravity) > kAccelerationThreshold) {
    return false;
  }
  if(fabs(wx - wx_prev_) > kDeltaAngularVelocityThreshold || fabs(wy - wy_prev_) > kDeltaAngularVelocityThreshold || fabs(wz - wz_prev_) > kDeltaAngularVelocityThreshold) {
    return false;
  }
  if(fabs(wx - wx_bias_) > kAngularVelocityThreshold || fabs(wy - wy_bias_) > kAngularVelocityThreshold || fabs(wz - wz_bias_) > kAngularVelocityThreshold) {
    return false;
  }
  return true;
}

void ComplementaryFilter::updateBiases(float ax, float ay, float az,  float wx, float wy, float wz) {

  steady_state_momentary = checkState(ax, ay, az, wx, wy, wz);

  if(steady_state_momentary) { if(steady_state_count <= steady_limit_) { steady_state_count++; } } else { steady_state_count = 0; }
  if(steady_state_count > steady_limit_) { steady_state = true; } else { steady_state = false; }

  if(steady_state) {
    wx_bias_ += bias_alpha_ * (wx - wx_bias_);
    wy_bias_ += bias_alpha_ * (wy - wy_bias_);
    wz_bias_ += bias_alpha_ * (wz - wz_bias_);
  }

  wx_prev_ = wx; 
  wy_prev_ = wy; 
  wz_prev_ = wz;

}

bool ComplementaryFilter::setSteadyLimit(int limit)  {
    if(limit >= 2 && limit <= 127) {
        steady_limit_ = limit;
        return true;
    } else {
        steady_limit_ = 8;
        return false;
    }
}

void ComplementaryFilter::getPrediction(float wx, float wy, float wz, double dt, float& q0_pred, float& q1_pred, float& q2_pred, float& q3_pred) const {

  float wx_unb = wx - wx_bias_;
  float wy_unb = wy - wy_bias_;
  float wz_unb = wz - wz_bias_;

  // notice F
  q0_pred = q0_ + 0.5F*dt*( wx_unb*q1_ + wy_unb*q2_ + wz_unb*q3_);
  q1_pred = q1_ + 0.5F*dt*(-wx_unb*q0_ - wy_unb*q3_ + wz_unb*q2_);
  q2_pred = q2_ + 0.5F*dt*( wx_unb*q3_ - wy_unb*q0_ - wz_unb*q1_);
  q3_pred = q3_ + 0.5F*dt*(-wx_unb*q2_ + wy_unb*q1_ - wz_unb*q0_);
  
  normalizeQuaternion(q0_pred, q1_pred, q2_pred, q3_pred);
}

void ComplementaryFilter::getMeasurement(float ax, float ay, float az, float mx, float my, float mz, float& q0_meas, float& q1_meas, float& q2_meas, float& q3_meas) {

  // q_acc is the quaternion obtained from the acceleration vector representing 
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
  float q0_acc, q1_acc, q2_acc, q3_acc;
    
  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);
  if(az >=0) {
      q0_acc =  sqrt((az + 1) * 0.5);	
      q1_acc = -ay/(2.0 * q0_acc);
      q2_acc =  ax/(2.0 * q0_acc);
      q3_acc = 0;
  } else {
      float X = sqrt((1 - az) * 0.5);
      q0_acc = -ay/(2.0 * X);
      q1_acc = X;
      q2_acc = 0;
      q3_acc = ax/(2.0 * X);
  }
  
  // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
  // frame by the inverse of q_acc.
  // l = R(q_acc)^-1 m
  float lx = (q0_acc*q0_acc + q1_acc*q1_acc - q2_acc*q2_acc)*mx + 2.0 * (q1_acc*q2_acc)*my - 2.0 * (q0_acc*q2_acc)*mz;
  float ly = 2.0 * (q1_acc*q2_acc)*mx + (q0_acc*q0_acc - q1_acc*q1_acc + q2_acc*q2_acc)*my + 2.0 * (q0_acc*q1_acc)*mz;
  
  // q_mag is the quaternion that rotates the Global frame (North West Up) into
  // the intermediary frame. q1_mag and q2_mag are defined as 0.
  float gamma = lx*lx + ly*ly;
  float beta = sqrt(gamma + lx*sqrt(gamma));
  float q0_mag = beta / (sqrt(2.0 * gamma));
  float q3_mag = ly / (sqrt(2.0) * beta);
    
  // The quaternion multiplication between q_acc and q_mag represents the 
  // quaternion, orientation of the Global frame wrt the local frame.  
  // q = q_acc times q_mag 
  quaternionMultiplication(q0_acc, q1_acc, q2_acc, q3_acc, q0_mag, 0, 0, q3_mag, q0_meas, q1_meas, q2_meas, q3_meas );
  //q0_meas = q0_acc*q0_mag;     
  //q1_meas = q1_acc*q0_mag + q2_acc*q3_mag;
  //q2_meas = q2_acc*q0_mag - q1_acc*q3_mag;
  //q3_meas = q0_acc*q3_mag;
}


void ComplementaryFilter::getMeasurement(float ax, float ay, float az, float& q0_meas, float& q1_meas, float& q2_meas, float& q3_meas) {
  // q_acc is the quaternion obtained from the acceleration vector representing 
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
     
  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);

  if(az >=0) {
    q0_meas =  sqrt((az + 1) * 0.5);	
    q1_meas = -ay/(2.0 * q0_meas);
    q2_meas =  ax/(2.0 * q0_meas);
    q3_meas = 0;
  } else {
    float X = sqrt((1 - az) * 0.5);
    q0_meas = -ay/(2.0 * X);
    q1_meas = X;
    q2_meas = 0;
    q3_meas = ax/(2.0 * X);
  }

}

void ComplementaryFilter::getAccCorrection(float ax, float ay, float az, float p0, float p1, float p2, float p3, float& dq0, float& dq1, float& dq2, float& dq3) {

  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);
  
  // Acceleration reading rotated into the world frame by the inverse predicted
  // quaternion (predicted gravity):
  float gx, gy, gz;
  rotateVectorByQuaternion(ax, ay, az, p0, -p1, -p2, -p3, gx, gy, gz);
  
  // Delta quaternion that rotates the predicted gravity into the real gravity:
  dq0 =  sqrt((gz + 1) * 0.5);	
  dq1 = -gy/(2.0 * dq0);
  dq2 =  gx/(2.0 * dq0);
  dq3 =  0.0;

}

void ComplementaryFilter::getMagCorrection(float mx, float my, float mz, float p0, float p1, float p2, float p3, float& dq0, float& dq1, float& dq2, float& dq3) {

  // Magnetic reading rotated into the world frame by the inverse predictedvquaternion:
  float lx, ly, lz;
  rotateVectorByQuaternion(mx, my, mz, p0, -p1, -p2, -p3, lx, ly, lz);
   
  // Delta quaternion that rotates the l so that it lies in the xz-plane (points north):
  float gamma = lx*lx + ly*ly;
  float beta = sqrt(gamma + lx*sqrt(gamma));
  dq0 = beta / (sqrt(2.0 * gamma));
  dq1 = 0.0;
  dq2 = 0.0;  
  dq3 = ly / (sqrt(2.0) * beta);

}
 
void ComplementaryFilter::getOrientation(float& q0, float& q1, float& q2, float& q3) const {
  // Return the inverse of the state (state is fixed wrt body).
  invertQuaternion(q0_, q1_, q2_, q3_, q0, q1, q2, q3);
}

float ComplementaryFilter::getAdaptiveGain(float alpha, float ax, float ay, float az) {
    float a_mag = sqrt(ax*ax + ay*ay + az*az);
    float error = fabs(a_mag - kGravity)/kGravity;
    float factor;
    float error1 = 0.1;
    float error2 = 0.2;
    float m = 1.0/(error1 - error2);
    float b = 1.0 - m*error1;
    if(error < error1) {
        factor = 1.0;
    } else if (error < error2) {
        factor = m*error + b;
    } else {
        factor = 0.0;
    }
    return factor*alpha;
}

void normalizeVector(float& x, float& y, float& z) {
    float norm = sqrt(x*x + y*y + z*z);
    x /= norm;
    y /= norm;
    z /= norm;
}

void normalizeQuaternion(float& q0, float& q1, float& q2, float& q3) {
    float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void invertQuaternion(float q0, float q1, float q2, float q3, float& q0_inv, float& q1_inv, float& q2_inv, float& q3_inv) {
    // Assumes quaternion is normalized.
    q0_inv = q0;
    q1_inv = -q1;
    q2_inv = -q2;
    q3_inv = -q3;
}

void scaleQuaternion(float gain, float& dq0, float& dq1, float& dq2, float& dq3) {
    // 0.9
    if (dq0 < 0.0) {
        // Slerp (Spherical linear interpolation):
        float angle = acos(dq0);
        float A = sin(angle*(1.0 - gain))/sin(angle);
        float B = sin(angle * gain)/sin(angle);
        dq0 = A + B * dq0;
        dq1 = B * dq1;
        dq2 = B * dq2;
        dq3 = B * dq3;
    } else {
        // Lerp (Linear interpolation):
        dq0 = (1.0 - gain) + gain * dq0;
        dq1 = gain * dq1;
        dq2 = gain * dq2;
        dq3 = gain * dq3;
    }
    normalizeQuaternion(dq0, dq1, dq2, dq3);
}

void quaternionMultiplication(float p0, float p1, float p2, float p3, float q0, float q1, float q2, float q3, float& r0, float& r1, float& r2, float& r3) {
  // r = p q
  r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
  r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
  r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1;
  r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0;
}

void rotateVectorByQuaternion(float x, float y, float z, float q0, float q1, float q2, float q3, float& vx, float& vy, float& vz) {
  vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z;
  vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z;
  vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z;
}