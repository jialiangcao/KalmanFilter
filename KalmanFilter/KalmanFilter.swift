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

    // Q: 4x4 Process noise covariance, how much we expect the IMU to be noisy or rough
    private let Q : simd_double4x4
    
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
    
    // MARK: - Init
    // - Parameters:
    //      - origin: the reference GPS coordinate from which local meters is computed
    //      - initialUncertainty: how uncertain you are about initial position/velocity (in meters^2)
    //      - processNoise: assumed process noise variance on acceleration (m^2 / s^4)
    init
    (
        origin: CLLocationCoordinate2D,
        initialUncertainty: Double = 3, // Small 1-10 trusts initial estimate, large > 100 is very uncertain
    ) {
        self.originLat = origin.latitude
        self.originLon = origin.longitude
        
        for i in 0..<4 {
            P[i][i] = initialUncertainty
        }
        
        // Process Noise Covariance
        // Variance of noise (square of standard deviation) in that state variable
        // Represents how uncertain we are between model predictions
        // Experiment with sigma_p
        let sigma_p = 0.5
        let sigma_v = 0.0447
        Q = simd_double4x4(rows: [
            simd_double4(sigma_p*sigma_p, 0, 0, 0),
            simd_double4(0, sigma_p*sigma_p, 0, 0),
            simd_double4(0, 0, sigma_v*sigma_v, 0),
            simd_double4(0, 0, 0, sigma_v*sigma_v)
        ])
    }
    
    // MARK: - Predict Step
    //
    // Called every IMU update with gravity compensated acceleration
    // - Parameters:
    //      - ax: acceleration in x (m/s^2)
    //      - ay: acceleration in y (m/s^2)
    //      - dt: time delta since last predict call (s)
    
    // TODO: Velocity should not be updated with noisy IMU here
    func predict(ax: Double, ay: Double, dt: Double) {
        // State transition F for constant‐velocity + control:
        // [ px ]   [1 0 dt 0 ][px]   [½dt²  0   ][ax]
        // [ py ] = [0 1 0  dt][py] + [0     ½dt²][ay]
        // [ vx ]   [0 0 1  0 ][vx]   [dt    0   ][ax]
        // [ vy ]   [0 0 0  1 ][vy]   [0     dt  ][ay]
        //  -   vx * dt = distance traveled keeping old velocity for dt seconds
        //  -   ax * dt2 = extra distance added by acceleration
        //  -   Finally update vx, speed, for next step
        let dt2 = 0.5 * dt * dt // Formula for distance under constant acceleration
        
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
        
        let u = simd_double4(ax, ay, 0, 0)
        state = F * state + B * u
        
        // P' = F * P * F^T (transposed) + Q
        // Or P = F·P·Fᵀ + B·Q_acc·Bᵀ + Q
        // But for simplicity, we approximate process noise by adding Q directly:
        P = F * P * F.transpose + Q
    }
    
    // MARK: - Update Step
    //
    // - Parameters:
    //      - GPS CoreLocation reading
    //      TODO: Add velocity in update
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
        // z, standard notation for noisy measurement
        let (zpx, zpy) = gpsToXY(coord: gps.coordinate)
        let z = simd_double4(zpx, zpy, 0, 0)
        
        // Measurement Noise Covariance
        // Represents how noisy or uncertain the measurements are
        let sigma_p = gps.horizontalAccuracy
        let sigma_v = 1e6 // velocity is not being measured here
        let sp2 = sigma_p * sigma_p

        let R = simd_double4x4(diagonal: simd_double4(
            sp2,
            sp2,
            sigma_v,
            sigma_v
        ))
        
        // Residual, difference between GPS reading and predicted position
        // y = z - H * x
        let y = z - H * state
        
        // Innovation covariance matrix
        // S = H * P * H^T + R
        let S = H * P * H.transpose + R
        
        // Kalman Gain
        // Compute "trust GPS vs prediction"
        // K = P * H^T * S^-1
        let K = P * H.transpose * S.inverse
        
        // Update State estimate
        // x = x + K * y
        state += K * y
        
        // Update estimate covariance
        // P = (I - K * H) * P
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
