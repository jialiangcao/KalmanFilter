//
//  KalmanFilter.swift
//  KalmanFilter
//
//  Created by Jialiang Cao on 7/22/25.
//

import CoreLocation
import CoreMotion

// 2D Kalman Filter for position (x, y) and velocity (vx, vy)
class KalmanFilter {
    // MARK: - State (X subk = [px, py, vx, vy])
    private(set) var px: Double = 0
    private(set) var py: Double = 0
    private(set) var vx: Double = 0
    private(set) var vy: Double = 0
    
    // MARK: - Covariance & Noise Matrices
    // P: 4x4 state covariance, how certain we are about how each value affects another from [-1, 1]: negative correlation -> positive correlation
    private var P: [[Double]] = Array(repeating: Array(repeating: 0, count: 4), count: 4)
   // H: 2x4 Observation Model (maps state to GPS measurement) only 2x4 because GPS only tells us about px and py
    private let H: [[Double]] = [
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ]
    // Q: 4x4 Process noise covariance, how much we expect the IMU to be noisy or rough
    private let Q: [[Double]]
 
    
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
        initialUncertainty: Double = 10,
        processNoise: Double = 0.05 // Might need to update to reflect meters
    ) {
        self.originLat = origin.latitude
        self.originLon = origin.longitude
        
        for i in 0..<4 {
            P[i][i] = initialUncertainty
        }
        
        // Higher processNoise trusts GPS more
        Q = [
            [processNoise, 0, 0, 0],
            [0, processNoise, 0, 0],
            [0, 0, processNoise, 0],
            [0, 0, 0, processNoise]
        ]
    }
    
    // MARK: - Predict Step
    //
    // Called every IMU update with gravity compensated acceleration
    // - Parameters:
    //      - ax: acceleration in x (m/s^2)
    //      - ay: acceleration in y (m/s^2)
    //      - dt: time delta since last predict call (s)
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
        px += vx * dt + ax * dt2
        py += vy * dt + ay * dt2
        vx += ax * dt
        vy += ay * dt
        
        let F: [[Double]] = [
            [1,   0,  dt, 0],
            [0,   1,  0,  dt],
            [0,   0,  1,  0],
            [0,   0,  0,  1]
        ]

        let B: [[Double]] = [
            [dt2,  0],
            [0,   dt2],
            [dt,   0],
            [0,    dt]
        ]
        
        // Convert P to Matrix type, do P = F·P·Fᵀ + B·Q_acc·Bᵀ + Q
        // But for simplicity, we approximate process noise by adding Q directly:
        P = multiply4x4(F, multiply4x4(P, transpose4x4(F)))
        P = add4x4(P, Q)
    }
    
    // MARK: - Update Step
    //
    // Called every GPS update
    // Uses 'horizontalAccuracy' as 1sd in meters
    //
    // - Parameters:
    //      - GPS CoreLocation reading
    func update(with gps: CLLocation) {
        // Convert GPS lat/lon to local x,y in meters
        let (zpx, zpy) = gpsToXY(coord: gps.coordinate)
        let sd = gps.horizontalAccuracy
        let R: [[Double]] = [
            [sd*sd, 0],
            [0, sd*sd]
        ]
        
        // Difference between GPS reading and predicted position
        // y = z - H * x
        let y0 = zpx - px
        let y1 = zpy - py
        
        // Blend uncertainties
        // S = H * P * H^T + R (2x2)
        let Ht = transpose4x2(H)
        let PHt = multiply4x4_4x2(P, Ht)
        let S = add2x2(
            multiply2x4_4x2(H, PHt),
            R
        )
        
        // Kalman Gain
        // Compute "trust GPS vs prediction"
        // K = P * H^T * S^-1 (4x2)
        let K = multiply4x2_2x2(
            PHt,
            invert2x2(S)
        )
        
        // Apply blend to correct position & speed
        // x = x + K * y
        let dy = [y0, y1]
        let dx = multiply4x2Vector(K, dy) // 4x2 * 2x1 -> 4x1 vector
        px += dx[0]
        py += dx[1]
        vx += dx[2]
        vy += dx[3]
        
        // Update covariance
        // P = (I sub4 - K * H) * P
        let KH = multiply4x2_2x4(K, H)
        let I = identity4x4()
        let IminusKH = subtract4x4(I, KH)
        P = multiply4x4(IminusKH, P)
    }
    
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

// MARK: - Matrix Helper Functions

/// Multiplies two 4x4 matrices
func multiply4x4(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
    var result = Array(repeating: Array(repeating: 0.0, count: 4), count: 4)
    for i in 0..<4 {
        for j in 0..<4 {
            for k in 0..<4 {
                result[i][j] += A[i][k] * B[k][j]
            }
        }
    }
    return result
}

/// Multiplies a 4x4 matrix by a 4x2 matrix
func multiply4x4_4x2(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
    var result = Array(repeating: Array(repeating: 0.0, count: 2), count: 4)
    for i in 0..<4 {
        for j in 0..<2 {
            for k in 0..<4 {
                result[i][j] += A[i][k] * B[k][j]
            }
        }
    }
    return result
}

/// Multiplies a 4x2 matrix by a 2x2 matrix
func multiply4x2_2x2(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
    var result = Array(repeating: Array(repeating: 0.0, count: 2), count: 4)
    for i in 0..<4 {
        for j in 0..<2 {
            for k in 0..<2 {
                result[i][j] += A[i][k] * B[k][j]
            }
        }
    }
    return result
}

/// Multiplies a 4x2 matrix by a 2x1 vector
func multiply4x2Vector(_ A: [[Double]], _ v: [Double]) -> [Double] {
    var result = [Double](repeating: 0.0, count: 4)
    for i in 0..<4 {
        for k in 0..<2 {
            result[i] += A[i][k] * v[k]
        }
    }
    return result
}

/// Multiplies a 2x4 matrix by a 4x2 matrix
func multiply2x4_4x2(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
    var result = Array(repeating: Array(repeating: 0.0, count: 2), count: 2)
    for i in 0..<2 {
        for j in 0..<2 {
            for k in 0..<4 {
                result[i][j] += A[i][k] * B[k][j]
            }
        }
    }
    return result
}

/// Multiplies a 4x2 matrix by a 2x4 matrix
func multiply4x2_2x4(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
    var result = Array(repeating: Array(repeating: 0.0, count: 4), count: 4)
    for i in 0..<4 {
        for j in 0..<4 {
            for k in 0..<2 {
                result[i][j] += A[i][k] * B[k][j]
            }
        }
    }
    return result
}

/// Transposes a 4x4 matrix
func transpose4x4(_ A: [[Double]]) -> [[Double]] {
    var result = Array(repeating: Array(repeating: 0.0, count: 4), count: 4)
    for i in 0..<4 {
        for j in 0..<4 {
            result[j][i] = A[i][j]
        }
    }
    return result
}

/// Transposes a 2x4 matrix to 4x2
func transpose4x2(_ A: [[Double]]) -> [[Double]] {
    var result = Array(repeating: Array(repeating: 0.0, count: 2), count: 4)
    for i in 0..<2 {
        for j in 0..<4 {
            result[j][i] = A[i][j]
        }
    }
    return result
}

/// Adds two 4x4 matrices
func add4x4(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
    var result = A
    for i in 0..<4 {
        for j in 0..<4 {
            result[i][j] += B[i][j]
        }
    }
    return result
}

/// Adds two 2x2 matrices
func add2x2(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
    var result = A
    for i in 0..<2 {
        for j in 0..<2 {
            result[i][j] += B[i][j]
        }
    }
    return result
}

/// Subtracts two 4x4 matrices
func subtract4x4(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
    var result = A
    for i in 0..<4 {
        for j in 0..<4 {
            result[i][j] -= B[i][j]
        }
    }
    return result
}

/// Creates 4x4 identity matrix
func identity4x4() -> [[Double]] {
    var I = Array(repeating: Array(repeating: 0.0, count: 4), count: 4)
    for i in 0..<4 {
        I[i][i] = 1.0
    }
    return I
}

/// Inverts a 2x2 matrix
func invert2x2(_ A: [[Double]]) -> [[Double]] {
    let det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
    return [
        [A[1][1]/det, -A[0][1]/det],
        [-A[1][0]/det, A[0][0]/det]
    ]
}
