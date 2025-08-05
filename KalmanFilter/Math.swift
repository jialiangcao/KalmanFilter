//
//  Math.swift
//  KalmanFilter
//
//  Created by Jialiang Cao on 7/29/25.
//

import Foundation
import CoreLocation

func gaussian(x: Double, mean: Double, sigma: Double) -> Double {
    let exponent = -pow(x - mean, 2) / (2 * pow(sigma, 2))
    let coefficient = 1.0 / (sigma * sqrt(2.0 * .pi))
    return coefficient * exp(exponent)
}

// Helper to compute bearing (radians) from one GPS coordinate to another, measured clockwise from true north. 0 -> 6.283..
func headingRadians(from: CLLocationCoordinate2D, to: CLLocationCoordinate2D) -> Double {
    let lat1 = from.latitude * .pi / 180
    let lon1 = from.longitude * .pi / 180
    let lat2 = to.latitude * .pi / 180
    let lon2 = to.longitude * .pi / 180
    let dLon = lon2 - lon1

    let y = sin(dLon) * cos(lat2)
    let x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
    var bearing = atan2(y, x)

    // Normalize to 0...2π
    if bearing < 0 {
        bearing += 2 * .pi
    }
    return bearing
}

