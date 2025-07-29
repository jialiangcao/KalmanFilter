//
//  KalmanFilterTests.swift
//  KalmanFilterTests
//
//  Created by Jialiang Cao on 7/22/25.
//

import XCTest
import CoreLocation

@testable import KalmanFilter

final class KalmanFilterTests: XCTestCase {
    func testGPStoMeters() throws {
        // BMCC 245 Greenwich bottom left
        let point1 = CLLocationCoordinate2D(latitude: 40.713481315279516, longitude: -74.01104871696431)
        // BMCC 245 Greenwich bottom right, meaning walking east on the South side
        let point2 = CLLocationCoordinate2D(latitude: 40.713858307316414, longitude: -74.01073053689558)
        
        let loc1 = CLLocation(latitude: point1.latitude, longitude: point1.longitude)
        let loc2 = CLLocation(latitude: point2.latitude, longitude: point2.longitude)
        
        let kf = KalmanFilter(origin: point1)
        let dist = kf.gpsToXY(coord: point2) // Custom solution
        let approxSquared = dist.x * dist.x + dist.y * dist.y
        let haversineDistance = loc1.distance(from: loc2) // Apple implementation
        XCTAssertEqual(sqrt(approxSquared), haversineDistance, accuracy: 0.5)
    }

    func testRoundTripProjection() {
        let origin = CLLocationCoordinate2D(latitude: 40.71348, longitude: -74.01105)
        let point = CLLocationCoordinate2D(latitude: 40.71385, longitude: -74.01073)
        let kf = KalmanFilter(origin: origin)

        let (x, y) = kf.gpsToXY(coord: point)
        let roundTrip = kf.xyToGps(x: x, y: y)

        XCTAssertEqual(roundTrip.latitude, point.latitude, accuracy: 1e-6)
        XCTAssertEqual(roundTrip.longitude, point.longitude, accuracy: 1e-6)
    }

    
    func testWalkWithNoise() throws {
        let point1 = CLLocationCoordinate2D(latitude: 40.713481315279516,
                                            longitude: -74.01104871696431)
        let point2 = CLLocationCoordinate2D(latitude: 40.713858307316414,
                                            longitude: -74.01073053689558)
        let kf = KalmanFilter(origin: point1)

        let totalDistance = CLLocation(latitude: point1.latitude,
                                       longitude: point1.longitude)
            .distance(from: CLLocation(latitude: point2.latitude,
                                       longitude: point2.longitude))
        let walkingSpeed = 1.4 // m/s
        let duration = totalDistance / walkingSpeed
        let durationInt = Int(ceil(duration))
        var groundTruthXY: [(x: Double, y: Double)] = []
        for sec in 0...durationInt {
            let t = Double(sec) / duration
            let baseX = t * kf.gpsToXY(coord: point2).x
            let baseY = t * kf.gpsToXY(coord: point2).y
            groundTruthXY.append((x: baseX, y: baseY))
        }
        
        var noisyXY: [(x: Double, y: Double)] = []

        var noisyGPS: [CLLocationCoordinate2D] = []
        for gt in groundTruthXY {
            let angle = Double.random(in: 0..<(2 * .pi))
            let radius = Double.random(in: 5...10)
            let nx = gt.x + radius * cos(angle)
            let ny = gt.y + radius * sin(angle)
            noisyXY.append((x: nx, y: ny))
            noisyGPS.append(kf.xyToGps(x: nx, y: ny))
        }

        let accelHz = 100.0
        let totalAccelSamples = Int(duration * accelHz)
        var accelStream: [(ax: Double, ay: Double)] = []
        for _ in 0..<totalAccelSamples {
            let noise = Double.random(in: -0.002...0.002)
            accelStream.append((ax: noise, ay: noise))
        }

        var filteredXY: [(x: Double, y: Double)] = []
        var gpsIndex = 0
        let dt = 1.0 / accelHz
        for i in 0..<totalAccelSamples {
            let (ax, ay) = accelStream[i]
            kf.predict(ax: ax, ay: ay, dt: dt)

            if i % Int(accelHz) == 0, gpsIndex < noisyGPS.count {
                let fix = CLLocation(coordinate: noisyGPS[gpsIndex],
                                     altitude: 0,
                                     horizontalAccuracy: 7.5,
                                     verticalAccuracy: 5,
                                     timestamp: Date())
                kf.update(with: fix)
                filteredXY.append((x: kf.px, y: kf.py))
                gpsIndex += 1
            }
        }

        var sumRawError = 0.0
        for (i, gpsPoint) in noisyGPS.enumerated() {
            let gt = groundTruthXY[i]
            let (zx, zy) = kf.gpsToXY(coord: gpsPoint)
            let dx = zx - gt.x
            let dy = zy - gt.y
            sumRawError += sqrt(dx*dx + dy*dy)
        }
        let avgRawError = sumRawError / Double(noisyGPS.count)

        var sumFiltError = 0.0
        for (i, est) in filteredXY.enumerated() {
            let gt = groundTruthXY[i]
            let dx = est.x - gt.x
            let dy = est.y - gt.y
            sumFiltError += sqrt(dx*dx + dy*dy)
        }
        let avgFiltError = sumFiltError / Double(filteredXY.count)
        print("")
        for i in 0..<min(groundTruthXY.count, filteredXY.count) {
            print("==== Time Step \(i) ====")
            print("Ground Truth: \(groundTruthXY[i])")
            print("Noisy Input: \(noisyXY[i])")
            print("Filtered Output: \(filteredXY[i])")
        }
        print("======================================")
        print("Average RAW GPS error      = \(avgRawError) m")
        print("Average FILTERED GPS error = \(avgFiltError) m")
        print("Error reduction            = \((1 - (avgFiltError/avgRawError))*100)% better")
        print("======================================")
    }

}
