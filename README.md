# 2D Kalman Filter for iOS Navigation

A lightweight **2D Kalman Filter** implementation in Swift that fuses CoreMotion accelerometer data and CoreLocation GPS readings to produce accurate, smoothed estimates of position and velocity. Designed for real-time navigation apps where accuracy is critical.

---

## 🚀 Features

* **Predict & Update Steps**: Integrates IMU (accelerometer) data for prediction and GPS data for correction.
* **Local ENU Projection**: Converts latitude/longitude to East–North–Up (ENU) flat meters relative to a reference origin.
* **Velocity Estimation**: Maintains estimates for both position (`px`, `py`) and velocity (`vx`, `vy`).
* **Configurable Noise Models**: Easy-to-tune process and measurement noise covariance parameters.
* **Minimal Dependencies**: Built entirely on `simd`, `CoreLocation`, and `CoreMotion`.

---

## 🏗️ Getting Started

### Requirements

* iOS 18.0+
* Xcode 16+
* Swift 6+

### Installation

Copy the necessary files into your Xcode project:

```swift
// Add KalmanFilter.swift to your project directory
// Add Math.swift to your project directory
```

---

## 💡 Usage

1. **Initialize** with a GPS reference point:

   ```swift
   import CoreLocation

   let origin = CLLocation(latitude: 40.7128, longitude: -74.0060)
   let kf = KalmanFilter(origin: origin)
   ```

2. **Predict** on each accelerometer update:

   ```swift
   // Example: IMU callback from CMMotionManager
   func motionManager(_ manager: CMMotionManager, didUpdateAcc data: CMAccelerometerData) {
       // In this case, converting gravity units to meters^2
       let ax = data.acceleration.x * 9.81
       let ay = data.acceleration.y * 9.81
       let heading = currentHeadingRadians
       let dt = timestamp - lastTimestamp
       kf.predict(ax: ax, ay: ay, headingRadians: heading, dt: dt)
   }
   ```

3. **Update** on each GPS reading:

   ```swift
   func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
       guard let gps = locations.last else { return }
       kf.update(with: gps)
       print("Position: (\(kf.px), \(kf.py)), Velocity: (\(kf.vx), \(kf.vy))")
   }
   ```

---

## 📐 Math & Implementation Details

* **State Vector**: `[px, py, vx, vy]ᵀ` — position and velocity in local meters.
* **State Transition (F)** & **Control (B)** matrices: Based on constant-acceleration kinematics.
* **Process Noise (Q)**: Derived from assumed acceleration variance.
* **Observation Model (H)**: Projects the state to measurement space (position only).
* **Measurement Noise (R)**: Based on GPS horizontal accuracy.

*For full derivations and theoretical background, refer to standard Kalman filter literature.*

---

## 📝 License

Distributed under the MIT License.

