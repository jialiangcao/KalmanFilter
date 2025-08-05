import XCTest
import CoreLocation
@testable import KalmanFilter

class KalmanFilterTests: XCTestCase {
    /// Test walking with speed changes (slowdown then speedup)
    func testWalkWithNoiseAndVelocityChange() throws {
        let pStart = CLLocationCoordinate2D(latitude: 40.7135, longitude: -74.0110)
        let pEnd = CLLocationCoordinate2D(latitude: 40.7145, longitude: -74.0100)
        let origin = CLLocation(latitude: pStart.latitude, longitude: pStart.longitude)
        let kf = KalmanFilter(origin: origin)

        let endXY = kf.gpsToXY(coord: pEnd)
        let totalDist = hypot(endXY.x, endXY.y)
        let slowSpeed = 0.7 // m^2/s
        let fastSpeed = 2.0 // m^2/s
        let distSlow = totalDist * 0.4
        let distFast = totalDist * 0.6
        let tSlow = distSlow / slowSpeed
        let tFast = distFast / fastSpeed
        let totalTime = tSlow + tFast

        let ux = endXY.x / totalDist
        let uy = endXY.y / totalDist

        let secCount = Int(ceil(totalTime))
        var groundTruth: [(x: Double, y: Double)] = []
        for s in 0...secCount {
            let t = Double(s)
            let traveled: Double
            if t <= tSlow {
                traveled = slowSpeed * t
            } else {
                traveled = distSlow + fastSpeed * (t - tSlow)
            }
            groundTruth.append((x: ux * traveled,
                                y: uy * traveled))
        }

        var noisyXY: [(x: Double, y: Double)] = []
        let noisyGPS: [CLLocationCoordinate2D] = groundTruth.map { gt in
            let angle = Double.random(in: 0..<(2 * .pi))
            let radius = Double.random(in: 10...20)
            let nx = gt.x + radius * cos(angle)
            let ny = gt.y + radius * sin(angle)
            noisyXY.append((x: nx, y: ny))
            return kf.xyToGps(x: nx, y: ny)
        }

        let accelHz = 100.0
        let totalSamples = Int(totalTime * accelHz)
        let dt = 1.0 / accelHz
        var accelStream: [(ax: Double, ay: Double)] = []
        accelStream.reserveCapacity(totalSamples)
        for _ in 0..<totalSamples {
            let n = Double.random(in: -0.02...0.02)
            accelStream.append((ax: n, ay: n))
        }

        var filtered: [(x: Double, y: Double)] = []
        filtered.reserveCapacity(noisyGPS.count)
        var gpsIndex = 0

        let heading = headingRadians(from: pStart, to: pEnd)

        for i in 0..<totalSamples {
            let (ax, ay) = accelStream[i]
            kf.predict(ax: ax, ay: ay, headingRadians: heading, dt: dt)

            if i % Int(accelHz) == 0 && gpsIndex < noisyGPS.count {
                let fix = CLLocation(
                    coordinate: noisyGPS[gpsIndex],
                    altitude: 0,
                    horizontalAccuracy: 15,
                    verticalAccuracy: 1,
                    timestamp: Date()
                )
                kf.update(with: fix)
                filtered.append((x: kf.px, y: kf.py))
                gpsIndex += 1
            }
        }

        let img = renderMultiLinePlot(
            series: [
                ("Ground Truth", groundTruth),
                ("Noisy GPS",    noisyXY),
                ("Filtered",     filtered)
            ],
            size: CGSize(width: 500, height: 400),
            margin: 40
        )

        let attachment = XCTAttachment(image: img)
        attachment.name = "WalkWithSpeed Plot"
        attachment.lifetime = .keepAlways
        add(attachment)

        XCTAssertNotNil(img)
    }
    
    /// Test walking with a 90° turn in the middle
    func testWalkWithTurn() throws {
        let p1 = CLLocationCoordinate2D(latitude: 40.7135, longitude: -74.0110)
        let p2 = CLLocationCoordinate2D(latitude: 40.7140, longitude: -74.0110) // north
        let p3 = CLLocationCoordinate2D(latitude: 40.7140, longitude: -74.0105) // east
        let origin = CLLocation(latitude: p1.latitude, longitude: p1.longitude)
        let kf = KalmanFilter(origin: origin)

        let xy2 = kf.gpsToXY(coord: p2)
        let xy3 = kf.gpsToXY(coord: p3)
        let dist12 = hypot(xy2.x, xy2.y)
        let dist23 = hypot(xy3.x - xy2.x, xy3.y - xy2.y)
        let walkingSpeed = 1.4 // m/s
        let t12 = dist12 / walkingSpeed
        let t23 = dist23 / walkingSpeed
        let totalTime = t12 + t23

        let h12 = headingRadians(from: p1, to: p2)
        let h23 = headingRadians(from: p2, to: p3)

        let secCount = Int(ceil(totalTime))
        var groundTruth: [(x: Double, y: Double)] = []
        for s in 0...secCount {
            let t = Double(s)
            let traveled: Double
            if t <= t12 {
                traveled = walkingSpeed * t
            } else {
                traveled = dist12 + walkingSpeed * (t - t12)
            }
            let pt: (x: Double, y: Double)
            if t <= t12 {
                let frac = traveled / dist12
                pt = (x: xy2.x * frac, y: xy2.y * frac)
            } else {
                let frac2 = (traveled - dist12) / dist23
                pt = (x: xy2.x + (xy3.x - xy2.x) * frac2,
                      y: xy2.y + (xy3.y - xy2.y) * frac2)
            }
            groundTruth.append(pt)
        }

        var noisyXY: [(x: Double, y: Double)] = []
        let noisyGPS: [CLLocationCoordinate2D] = groundTruth.map { gt in
            let angle = Double.random(in: 0..<(2 * .pi))
            let radius = Double.random(in: 5...10)
            let nx = gt.x + radius * cos(angle)
            let ny = gt.y + radius * sin(angle)
            noisyXY.append((x: nx, y: ny))
            return kf.xyToGps(x: nx, y: ny)
        }

        let accelHz = 100.0
        let totalSamples = Int(totalTime * accelHz)
        let dt = 1.0 / accelHz
        var accelStream = [(ax: Double, ay: Double)]()
        accelStream.reserveCapacity(totalSamples)
        for _ in 0..<totalSamples {
            let n = Double.random(in: -0.02...0.02)
            accelStream.append((ax: n, ay: n))
        }

        var filtered: [(x: Double, y: Double)] = []
        filtered.reserveCapacity(noisyGPS.count)
        var gpsIndex = 0

        for i in 0..<totalSamples {
            let tNow = Double(i) * dt
            let heading = tNow <= t12 ? h12 : h23

            let (ax, ay) = accelStream[i]
            kf.predict(ax: ax, ay: ay, headingRadians: heading, dt: dt)

            if i % Int(accelHz) == 0 && gpsIndex < noisyGPS.count {
                let fix = CLLocation(
                    coordinate: noisyGPS[gpsIndex],
                    altitude: 0,
                    horizontalAccuracy: 7.5,
                    verticalAccuracy: 1,
                    timestamp: Date()
                )
                kf.update(with: fix)
                filtered.append((x: kf.px, y: kf.py))
                gpsIndex += 1
            }
        }

        let rawErr = zip(noisyGPS, groundTruth)
            .map { gps, gt in
                let xy = kf.gpsToXY(coord: gps)
                return hypot(xy.x - gt.x, xy.y - gt.y)
            }.reduce(0,+) / Double(noisyGPS.count)

        let filtErr = zip(filtered, groundTruth)
            .map { est, gt in hypot(est.x - gt.x, est.y - gt.y) }
            .reduce(0,+) / Double(filtered.count)

        XCTAssertTrue(filtErr < rawErr, "Filter should handle a 90° turn")

        let img = renderMultiLinePlot(
            series: [
                ("Ground Truth", groundTruth),
                ("Noisy GPS",    noisyXY),
                ("Filtered",     filtered)
            ],
            size: CGSize(width: 500, height: 400),
            margin: 40
        )
        let attach = XCTAttachment(image: img)
        attach.name = "WalkWithTurn Plot"
        attach.lifetime = .keepAlways
        add(attach)
    }

    func testWalkWithNoise() throws {
        let point1 = CLLocationCoordinate2D(latitude: 40.713481315279516,
                                            longitude: -74.01104871696431)
        let origin = CLLocation(latitude: point1.latitude, longitude: point1.longitude)
        let point2 = CLLocationCoordinate2D(latitude: 40.713858307316414,
                                            longitude: -74.01073053689558)
        let kf = KalmanFilter(origin: origin)
        let constantHeading = headingRadians(from: point1, to: point2)
        
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
            let radius = Double.random(in: 5.0...10.0) // Lower/upper bound inaccuracy in m^2
            let nx = gt.x + radius * cos(angle)
            let ny = gt.y + radius * sin(angle)
            noisyXY.append((x: nx, y: ny))
            noisyGPS.append(kf.xyToGps(x: nx, y: ny))
        }
        
        let accelHz = 100.0
        let totalAccelSamples = Int(duration * accelHz)
        var accelStream: [(ax: Double, ay: Double)] = []
        for _ in 0..<totalAccelSamples {
            let noise = Double.random(in: -0.002...0.002) // Mean zero acceleration
            accelStream.append((ax: noise, ay: noise))
        }
        
        var filteredXY: [(x: Double, y: Double)] = []
        var gpsIndex = 0
        let dt = 1.0 / accelHz
        for i in 0..<totalAccelSamples {
            let (ax, ay) = accelStream[i]
            kf.predict(ax: ax, ay: ay, headingRadians: constantHeading, dt: dt)
            
            if i % Int(accelHz) == 0, gpsIndex < noisyGPS.count {
                let fix = CLLocation(coordinate: noisyGPS[gpsIndex],
                                     altitude: 0,
                                     horizontalAccuracy: 7.5,
                                     verticalAccuracy: 1,
                                     timestamp: Date())
                kf.update(with: fix)
                filteredXY.append((x: kf.px, y: kf.py))
                gpsIndex += 1
            }
        }
            
        let img = renderMultiLinePlot(
            series: [
                ("Ground Truth", groundTruthXY),
                ("Noisy GPS",    noisyXY),
                ("Filtered",     filteredXY)
            ],
            size: CGSize(width: 500, height: 400),
            margin: 40
        )
        
        let attachment = XCTAttachment(image: img)
        attachment.name = "WalkWithNoise Plot"
        attachment.lifetime = .keepAlways
        add(attachment)
        
        XCTAssertNotNil(img)
    }

    func renderMultiLinePlot(
        series: [(label: String, data: [(x: Double, y: Double)])],
        size: CGSize,
        margin: CGFloat
    ) -> UIImage {
        let allX = series.flatMap { $0.data.map { $0.x } }
        let allY = series.flatMap { $0.data.map { $0.y } }
        guard let minX = allX.min(), let maxX = allX.max(),
              let minY = allY.min(), let maxY = allY.max()
        else { return UIImage() }
        
        let colors: [UIColor] = [.systemGreen, .systemRed, .systemBlue, .systemOrange, .systemPurple]
        
        return UIGraphicsImageRenderer(size: size).image { ctx in
            let cg = ctx.cgContext
            cg.setLineWidth(1)
            cg.setStrokeColor(UIColor.lightGray.cgColor)
            
            let origin = CGPoint(x: margin, y: size.height - margin)
            let xEnd = CGPoint(x: size.width - margin, y: origin.y)
            let yEnd = CGPoint(x: origin.x, y: margin)
            cg.move(to: origin); cg.addLine(to: xEnd)
            cg.move(to: origin); cg.addLine(to: yEnd)
            cg.strokePath()
            
            func scale(_ pt: (x: Double, y: Double)) -> CGPoint {
                let xN = (pt.x - minX)/(maxX-minX)
                let yN = (pt.y - minY)/(maxY-minY)
                let xP = origin.x + CGFloat(xN)*(size.width - 2*margin)
                let yP = origin.y - CGFloat(yN)*(size.height - 2*margin)
                return CGPoint(x: xP, y: yP)
            }
            
            for (idx, (_, data)) in series.enumerated() {
                let color = colors[idx % colors.count].cgColor
                cg.setStrokeColor(color)
                cg.setLineWidth(2)
                for (i, pt) in data.enumerated() {
                    let p = scale(pt)
                    if i == 0 { cg.move(to: p) }
                    else      { cg.addLine(to: p) }
                }
                cg.strokePath()
            }
            
            let legendX = size.width - margin + 5
            var legendY = margin
            let sw: CGFloat = 12, sh: CGFloat = 12
            let fm = UIFont.systemFont(ofSize: 12)
            for (idx, (label, _)) in series.enumerated() {
                let color = colors[idx % colors.count]
                let rect = CGRect(x: legendX, y: legendY, width: sw, height: sh)
                cg.setFillColor(color.cgColor)
                cg.fill(rect)
                let txtPoint = CGPoint(x: legendX + sw + 4, y: legendY - 2)
                (label as NSString).draw(at: txtPoint, withAttributes: [.font: fm, .foregroundColor: UIColor.label])
                legendY += sh + 6
            }
        }
    }
}
