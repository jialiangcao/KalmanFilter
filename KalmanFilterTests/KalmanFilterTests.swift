import XCTest
import CoreLocation
@testable import KalmanFilter

/// Helper to compute bearing (radians) from one GPS coordinate to another, measured clockwise from true north. 0 -> 6.283..
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

class KalmanFilterTests: XCTestCase {
    /// Test walking in a straight line with noise
    func testWalkStraightWithNoise() throws -> (raw: Double, filtered: Double) {
        // Setup origin and destination
        let start = CLLocationCoordinate2D(latitude: 40.7134813, longitude: -74.0110487)
        let mid = CLLocationCoordinate2D(latitude: 40.7136813, longitude: -74.0108487)
        let end = CLLocationCoordinate2D(latitude: 40.7138813, longitude: -74.0106487)
        let origin = CLLocation(latitude: start.latitude, longitude: start.longitude)

        // Sequence of headings: constant
        let headings = [headingRadians(from: start, to: mid), headingRadians(from: mid, to: end)]

        let kf = KalmanFilter(origin: origin)
        let totalDist = CLLocation(latitude: start.latitude, longitude: start.longitude)
            .distance(from: CLLocation(latitude: end.latitude, longitude: end.longitude))
        let walkingSpeed = 1.4 // m/s
        let duration = totalDist / walkingSpeed
        let dt = 1.0 / 100.0

        // Generate ground truth positions at each second
        var groundTruth: [(Double, Double)] = []
        let endXY = kf.gpsToXY(coord: end)
        let steps = Int(ceil(duration))
        for t in 0...steps {
            let frac = Double(t) / Double(steps)
            groundTruth.append((endXY.x * frac, endXY.y * frac))
        }

        // Simulate noisy GPS and IMU
        var noisyGPS: [CLLocationCoordinate2D] = []
        for gt in groundTruth {
            let angle = Double.random(in: 0..<(2 * .pi))
            let radius = Double.random(in: 20...40)
            noisyGPS.append(kf.xyToGps(x: gt.0 + radius * cos(angle), y: gt.1 + radius * sin(angle)))
        }
        let totalAccel = Int(duration * 100.0)
        var accelStream = [(ax: Double, ay: Double)]()
        for _ in 0..<totalAccel {
            accelStream.append((ax: Double.random(in: -0.02...0.02), ay: Double.random(in: -0.02...0.02)))
        }

        // Run filter
        var filtered: [(Double, Double)] = []
        var gpsIndex = 0
        for i in 0..<totalAccel {
            let h = headings[gpsIndex < headings.count ? gpsIndex : headings.count - 1]
            kf.predict(ax: accelStream[i].ax, ay: accelStream[i].ay, headingRadians: h, dt: dt)
            if i % 100 == 0 && gpsIndex < noisyGPS.count {
                kf.update(with: CLLocation(coordinate: noisyGPS[gpsIndex], altitude: 0,
                                           horizontalAccuracy: 30, verticalAccuracy: 1,
                                           timestamp: Date()))
                filtered.append((kf.px, kf.py))
                gpsIndex += 1
            }
        }

        // Assert reduced error
        let rawError = zip(noisyGPS.indices, noisyGPS).map { i, gps in
            let gt = groundTruth[i]
            let xy = kf.gpsToXY(coord: gps)
            return hypot(xy.x - gt.0, xy.y - gt.1)
        }.reduce(0, +) / Double(noisyGPS.count)

        let filtError = zip(filtered, groundTruth).map { est, gt in
            hypot(est.0 - gt.0, est.1 - gt.1)
        }.reduce(0, +) / Double(filtered.count)

        XCTAssertTrue(filtError < rawError, "Filter should reduce error for straight walk")
        return (raw: rawError, filtered: filtError)
    }

    /// Test walking with a 90° turn in the middle
    func testWalkWithTurn() throws -> (raw: Double, filtered: Double) {
        // Define a path that turns right after halfway
        let p1 = CLLocationCoordinate2D(latitude: 40.7135, longitude: -74.0110)
        let p2 = CLLocationCoordinate2D(latitude: 40.7140, longitude: -74.0110) // North
        let p3 = CLLocationCoordinate2D(latitude: 40.7140, longitude: -74.0105) // East
        let origin = CLLocation(latitude: p1.latitude, longitude: p1.longitude)

        let headings = [headingRadians(from: p1, to: p2), headingRadians(from: p2, to: p3)]
        let waypoints = [p2, p3]

        let kf = KalmanFilter(origin: origin)
        let totalDist = CLLocation(latitude: p1.latitude, longitude: p1.longitude)
            .distance(from: CLLocation(latitude: p3.latitude, longitude: p3.longitude))
        let duration = totalDist / 1.4
        let dt = 0.01

        var groundTruth: [(Double, Double)] = []
        let dist12 = CLLocation(latitude: p1.latitude, longitude: p1.longitude)
            .distance(from: CLLocation(latitude: p2.latitude, longitude: p2.longitude))
        let steps1 = Int(ceil((dist12 / 1.4) / dt))
        let xy2 = kf.gpsToXY(coord: p2)
        for i in 0...steps1 {
            let frac = Double(i) / Double(steps1)
            groundTruth.append((xy2.x * frac, xy2.y * frac))
        }
        let xy3 = kf.gpsToXY(coord: p3)
        let remaining = Int(ceil(((totalDist - dist12) / 1.4) / dt))
        for i in 1...remaining {
            let frac = Double(i) / Double(remaining)
            groundTruth.append((xy2.x + (xy3.x - xy2.x) * frac,
                                 xy2.y + (xy3.y - xy2.y) * frac))
        }

        // Simulate noise and accel
        var noisyGPS: [CLLocationCoordinate2D] = groundTruth.map { gt in
            let angle = Double.random(in: 0..<(2 * .pi))
            let r = Double.random(in: 20...40)
            return kf.xyToGps(x: gt.0 + r * cos(angle), y: gt.1 + r * sin(angle))
        }
        let totalAccel = groundTruth.count * Int(1/dt)
        var accelStream = [(Double, Double)]()
        for _ in 0..<totalAccel {
            accelStream.append((Double.random(in: -0.02...0.02),
                                Double.random(in: -0.02...0.02)))
        }

        // Filter
        var filtered: [(Double, Double)] = []
        var waypointIndex = 0
        for i in 0..<totalAccel {
            let currentHeading = headings[min(waypointIndex, headings.count-1)]
            kf.predict(ax: accelStream[i].0, ay: accelStream[i].1,
                       headingRadians: currentHeading, dt: dt)
            if i % Int(1/dt) == 0 && waypointIndex < noisyGPS.count {
                kf.update(with: CLLocation(coordinate: noisyGPS[waypointIndex], altitude:0,
                                           horizontalAccuracy:30, verticalAccuracy:1,
                                           timestamp:Date()))
                filtered.append((kf.px, kf.py))
                waypointIndex += 1
            }
        }

        let rawErr = zip(noisyGPS, groundTruth).map { gps, gt in
            let xy = kf.gpsToXY(coord: gps)
            return hypot(xy.x-gt.0, xy.y-gt.1)
        }.reduce(0,+)/Double(noisyGPS.count)
        let filtErr = zip(filtered, groundTruth).map { est, gt in
            hypot(est.0-gt.0, est.1-gt.1)
        }.reduce(0,+)/Double(filtered.count)

        XCTAssertTrue(filtErr < rawErr, "Filter should handle a 90° turn")
        return (raw: rawErr, filtered: filtErr)
    }

    /// Test walking with speed changes (slowdown then speedup)
    func testWalkSpeedChange() throws -> (raw: Double, filtered: Double) {
        let pStart = CLLocationCoordinate2D(latitude: 40.7135, longitude: -74.0110)
        let pEnd = CLLocationCoordinate2D(latitude: 40.7145, longitude: -74.0100)
        let origin = CLLocation(latitude: pStart.latitude, longitude: pStart.longitude)
        let kf = KalmanFilter(origin: origin)
        
        // Create ground truth with variable speed: half time slow, half time fast
        let totalDist = CLLocation(latitude: pStart.latitude, longitude: pStart.longitude)
            .distance(from: CLLocation(latitude: pEnd.latitude, longitude: pEnd.longitude))
        let slowSpeed = 0.7 // m/s
        let fastSpeed = 2.0 // m/s
        let distSlow = totalDist * 0.4
        let distFast = totalDist * 0.6
        let tSlow = distSlow / slowSpeed
        let tFast = distFast / fastSpeed
        let dt = 0.01
        let stepsSlow = Int(ceil(tSlow / dt))
        let stepsFast = Int(ceil(tFast / dt))
        let endXY = kf.gpsToXY(coord: pEnd)

        var groundTruth: [(Double, Double)] = []
        // Slow segment
        for i in 0..<stepsSlow {
            let frac = Double(i) / Double(stepsSlow)
            groundTruth.append((endXY.x * 0.4 * frac, endXY.y * 0.4 * frac))
        }
        // Fast segment
        for i in 0...stepsFast {
            let frac = Double(i) / Double(stepsFast)
            groundTruth.append((endXY.x * (0.4 + 0.6 * frac), endXY.y * (0.4 + 0.6 * frac)))
        }

        var noisyGPS: [CLLocationCoordinate2D] = groundTruth.map { gt in
            let angle = Double.random(in: 0..<(2 * .pi))
            let r = Double.random(in: 20...40)
            return kf.xyToGps(x: gt.0 + r * cos(angle), y: gt.1 + r * sin(angle))
        }
        var accelStream: [(Double, Double)] = []
        for _ in 0..<groundTruth.count * Int(1/dt) {
            accelStream.append((Double.random(in: -0.02...0.02),
                                Double.random(in: -0.02...0.02)))
        }

        var filtered: [(Double, Double)] = []
        var gpsIndex = 0
        for i in 0..<accelStream.count {
            // Heading always from start to end
            let h = headingRadians(from: pStart, to: pEnd)
            kf.predict(ax: accelStream[i].0, ay: accelStream[i].1,
                       headingRadians: h, dt: dt)
            if i % Int(1/dt) == 0 && gpsIndex < noisyGPS.count {
                kf.update(with: CLLocation(coordinate: noisyGPS[gpsIndex], altitude:0,
                                           horizontalAccuracy:30, verticalAccuracy:1,
                                           timestamp:Date()))
                filtered.append((kf.px, kf.py))
                gpsIndex += 1
            }
        }

        let rawErr = zip(noisyGPS, groundTruth).map { gps, gt in
            let xy = kf.gpsToXY(coord: gps)
            return hypot(xy.x-gt.0, xy.y-gt.1)
        }.reduce(0,+)/Double(noisyGPS.count)
        let filtErr = zip(filtered, groundTruth).map { est, gt in
            hypot(est.0-gt.0, est.1-gt.1)
        }.reduce(0,+)/Double(filtered.count)

        XCTAssertTrue(filtErr < rawErr, "Filter should handle speed changes gracefully")
        return (raw: rawErr, filtered: filtErr)
    }
    
    func runMultipleTimes(_ runs: Int, simulate: () -> (raw: Double, filtered: Double)) -> (avgRaw: Double, avgFiltered: Double) {
        var totalRaw = 0.0
        var totalFiltered = 0.0

        for _ in 0..<runs {
            let (raw, filtered) = simulate()
            totalRaw += raw
            totalFiltered += filtered
        }

        return (totalRaw / Double(runs), totalFiltered / Double(runs))
    }

    func testTurningCorner() throws {
        let (straightRaw, straightFilt) = runMultipleTimes(5) {
            try! testWalkStraightWithNoise()
        }
        
        let (cornerRaw, cornerFilt) = runMultipleTimes(5) {
            try! testWalkWithTurn()
        }
        
        let (speedRaw, speedFilt) = runMultipleTimes(5) {
            try! testWalkSpeedChange()
        }

        print("===== Results =====")
        print("= Straight Walk =")
        print("Avg Raw Error: \(straightRaw)")
        print("Avg Filtered Error: \(straightFilt)")
        print("= Corner Walk =")
        print("Avg Raw Error: \(cornerRaw)")
        print("Avg Filtered Error: \(cornerFilt)")
        print("= Speed Walk =")
        print("Avg Raw Error: \(speedRaw)")
        print("Avg Filtered Error: \(speedFilt)")
        print("===================")
        let totalRaw = straightRaw + cornerRaw + speedRaw
        let totalFilt = straightFilt + cornerFilt + speedFilt
        let totalReduction = totalRaw - totalFilt
        let averageReduction = 100 - totalReduction / 3.0
        print("===================")
        print("Total Average Reduction: \(averageReduction)")
        print("===================")
        
        XCTAssert(averageReduction > 50)
    }

}
