//
//  KalmanFilter.swift
//  KalmanFilter
//
//  Created by Jialiang Cao on 7/22/25.
//

import CoreLocation
import CoreMotion
import simd

// 2D Kalman Filter for position (x, y) and velocity (vx, vy)
class KalmanFilter {
    // MARK: - State (X subk = [px, py, vx, vy])
    private(set) var state = simd_double4(0, 0, 0, 0)
    
    // MARK: - Covariance & Noise Matrices
    // P: 4x4 state covariance, how certain we are about how each value affects another from [-1, 1]: negative correlation -> positive correlation
    private var P = simd_double4x4(0)
    
   // H: 4x4 Observation Model
    private let H = simd_double4x4(rows: [
        simd_double4(1, 0, 0, 0),
        simd_double4(0, 1, 0, 0),
        simd_double4(0, 0, 0, 0),
        simd_double4(0, 0, 0, 0)
    ])
    
    // MARK: - Origin for local ENU projection
    private let originLat: Double
    private let originLon: Double
    private var count = 0
    
    // MARK: - Init
    // - Parameters:
    //      - origin: the reference GPS coordinate from which local meters is computed
    //      - initialUncertainty: how uncertain you are about initial position/velocity (in meters^2)
    //      - processNoise: assumed process noise variance on acceleration (m^2 / s^4)
    init
    (
        origin: CLLocation,
    ) {
        let coord = origin.coordinate
        self.originLat = coord.latitude
        self.originLon = coord.longitude
        
        // Convert horizontal accuracy (95% radius) to 1-sigma std dev for position
        // 95% confidence ~ 2 sigma, so sigma = radius / 2
        
        //let sigma_p = origin.horizontalAccuracy / 2.0
        // ^ Doesn't work as expected because GPS is not warmed up and returns accuracy of 0 (very high confidence)
        let sigma_p = 5.0
        let sigma_p2 = sigma_p * sigma_p
        
        // Velocity initial uncertainty m^2/s
        let sigma_v = 0.25
        let sigma_v2 = sigma_v * sigma_v
        
        for i in 0..<2 {
            P[i][i] = sigma_p2
        }
        
        for i in 2..<4 {
            P[i][i] = sigma_v2
        }
    }
    
    // MARK: - Predict Step
    //
    // Called every IMU update with gravity compensated acceleration
    // - Parameters:
    //      - ax: acceleration in x (m/s^2)
    //      - ay: acceleration in y (m/s^2)
    //      - dt: time delta since last predict call (s)
    
    func predict(ax: Double, ay: Double, headingRadians: Double, dt: Double) {
        // State transition F for constant‐velocity + control:
        // [ px ]   [1 0 dt 0 ][px]   [½dt²  0   ][ax]
        // [ py ] = [0 1 0  dt][py] + [0     ½dt²][ay]
        // [ vx ]   [0 0 1  0 ][vx]   [dt    0   ][ax]
        // [ vy ]   [0 0 0  1 ][vy]   [0     dt  ][ay]
        //  -   vx * dt = distance traveled keeping old velocity for dt seconds
        //  -   ax * dt2 = extra distance added by acceleration
        //  -   Finally update vx, speed, for next step

        // Rotate according to true north
        let accX = ax * cos(-headingRadians) - ay * sin(-headingRadians)
        let accY = ax * sin(-headingRadians) + ay * cos(-headingRadians)

        let dt2 = 0.5 * dt * dt // Formula for distance under constant acceleration
        
        // Damping factor on velocity
        let F = simd_double4x4(rows: [
            simd_double4(1, 0, dt, 0),
            simd_double4(0, 1, 0, dt),
            simd_double4(0, 0, 1, 0),
            simd_double4(0, 0, 0, 1)
        ])

        let B = simd_double4x4(rows: [
            simd_double4(dt2, 0, 0, 0),
            simd_double4(0, dt2, 0, 0),
            simd_double4(dt, 0, 0, 0),
            simd_double4(0, dt, 0, 0)
        ])
        
        let u = simd_double4(accX, accY, 0, 0)
        state = F * state + B * u
        
        // Process noise covariance Q scaled by dt:
         // For constant acceleration model, Q is:
         // Q_pos = (dt^4)/4 * accelNoiseVar
         // Q_vel = dt^2 * accelNoiseVar
         // Off-diagonal terms = (dt^3)/2 * accelNoiseVar
         
        let accelNoiseVar = 1.5 * 1.5
        // Position variance due to integrated acceleration noise
         let q11 = (dt * dt * dt * dt) / 4.0 * accelNoiseVar
        // Covariance between position and velocity
         let q13 = (dt * dt * dt) / 2.0 * accelNoiseVar
        // Velocity variance due to acceleration noise
         let q33 = dt * dt * accelNoiseVar
         
         let Q = simd_double4x4(rows: [
             simd_double4(q11, 0,   q13, 0),
             simd_double4(0,   q11, 0,   q13),
             simd_double4(q13, 0,   q33, 0),
             simd_double4(0,   q13, 0,   q33)
         ])
        
        // P' = F * P * F^T (transposed) + Q
        // Or P = F·P·Fᵀ + B·Q_acc·Bᵀ + Q
        // But for simplicity, approximating process noise by adding Q directly:
        P = F * P * F.transpose + Q
    }
    
    // MARK: - Update Step
    //
    // - Parameters:
    //      - GPS CoreLocation reading
    //
    // Steps:
    //    Compute predicted measurement: z^=Hx^z^=Hx^
    //
    //    Calculate innovation (residual): y=z−z^y=z−z^
    //
    //    Compute innovation covariance: S=HPHT+RS=HPHT+R
    //
    //    Compute Kalman gain: K=PHTS−1K=PHTS−1
    //
    //    Update state estimate: x^=x^+Kyx^=x^+Ky
    //
    //    Update estimate covariance: P=(I−KH)PP=(I−KH)P
    func update(with gps: CLLocation) {
        // Convert GPS lat/lon to local x,y in meters
        let (zpx, zpy) = gpsToXY(coord: gps.coordinate)
        let z = simd_double4(zpx, zpy, 0, 0)
        
        // Measurement Noise Covariance
        let variance_p = gps.horizontalAccuracy * gps.horizontalAccuracy / 2.0
        // asumming sqrt(sigma_x^2 + sigma_y^2) is the horizontalAccuracy formula

        // Use large velocity variance because update does not provide info on velocity
        let R = simd_double4x4(diagonal: simd_double4(
            variance_p,
            variance_p,
            1e7,
            1e7,
        ))
        
        // Residual, difference between GPS reading and predicted position
        let y = z - H * state
        
        // Innovation covariance matrix
        let S = H * P * H.transpose + R
        
        // Kalman Gain
        let K = P * H.transpose * S.inverse
        
        // Update State estimate
        state += K * y
        
        // Update estimate covariance
        let I = matrix_identity_double4x4
        P = (I - K * H) * P
    }

    // Accessors
    var px: Double { state[0] }
    var py: Double { state[1] }
    var vx: Double { state[2] }
    var vy: Double { state[3] }
    
    // MARK: - Coordinate Helpers
    
    // Convert GPS to local flat meters (ENU projection)
    func gpsToXY(coord: CLLocationCoordinate2D) -> (x: Double, y: Double) {
        let R = 6378137.0 // Earth radius in meters
        let dLat = (coord.latitude - originLat) * Double.pi / 180.0
        let dLon = (coord.longitude - originLon) * Double.pi / 180.0
        
        let x = dLon * R * cos(originLat * Double.pi / 180.0)
        let y = dLat * R
        return (x, y)
    }
    
    func xyToGps(x: Double, y: Double) -> CLLocationCoordinate2D {
        let dLat = y / 6378137.0
        let dLon = x / (6378137.0 * cos(originLat * Double.pi / 180.0))
        
        let lat = originLat + dLat * 180.0 / Double.pi
        let lon = originLon + dLon * 180.0 / Double.pi
        
        return CLLocationCoordinate2D(latitude: lat, longitude: lon)
    }
}
