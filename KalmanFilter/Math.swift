//
//  Math.swift
//  KalmanFilter
//
//  Created by Jialiang Cao on 7/29/25.
//

import Foundation

func gaussian(x: Double, mean: Double, sigma: Double) -> Double {
    let exponent = -pow(x - mean, 2) / (2 * pow(sigma, 2))
    let coefficient = 1.0 / (sigma * sqrt(2.0 * .pi))
    return coefficient * exp(exponent)
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

