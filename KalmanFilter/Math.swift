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
