//
//  ViewModel.swift
//  KalmanFilter
//
//  Created by Jialiang Cao on 7/22/25.
//

import Foundation
import CoreLocation
import CoreMotion
import Combine

class ViewModel: NSObject, ObservableObject {
    // MARK: - Published Properties
    @Published var filteredCoordinate: CLLocationCoordinate2D?
    @Published var rawCoordinate: CLLocationCoordinate2D?
    @Published var filteredPath: [CLLocationCoordinate2D] = []
    @Published var rawPath: [CLLocationCoordinate2D] = []
    @Published var error: Error?
    
    // MARK: - Managers and Filter
    private let motionManager = CMMotionManager()
    private let locationManager = CLLocationManager()
    private var kalmanFilter: KalmanFilter?
    private var currentHeadingRadians: Double = 0.0
    
    // MARK: - Initialization
    override init() {
        super.init()
        locationManager.delegate = self
        locationManager.headingFilter = kCLHeadingFilterNone
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.distanceFilter = kCLDistanceFilterNone
        locationManager.requestWhenInUseAuthorization()
        locationManager.startUpdatingHeading()
        locationManager.startUpdatingLocation()
        
        startAccelerometer()
    }
    
    // MARK: - Accelerometer Handling
    private func startAccelerometer() {
        guard motionManager.isAccelerometerAvailable else { return }
        motionManager.deviceMotionUpdateInterval = 0.05
        motionManager.startDeviceMotionUpdates(to: .main) { [weak self] data, err in
            guard let self = self, let acc = data?.userAcceleration else { return }
            // Converts to meters
            let ax = acc.x * 9.81
            let ay = acc.y * 9.81
            let dt = self.motionManager.deviceMotionUpdateInterval
//            print("Heading Radians: \(self.currentHeadingRadians)")
            self.kalmanFilter?.predict(ax: ax, ay: ay, headingRadians: self.currentHeadingRadians, dt: dt)
            self.publishFilteredCoordinate()
        }
    }
    
    // MARK: - Publish Filtered Output
    private func publishFilteredCoordinate() {
        guard let filter = kalmanFilter else { return }
        let latLon = filter.xyToGps(x: filter.px, y: filter.py)
        DispatchQueue.main.async {
            self.filteredCoordinate = latLon
            self.filteredPath.append(latLon)
        }
    }
}

// MARK: - CLLocationManagerDelegate
extension ViewModel: CLLocationManagerDelegate {
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let loc = locations.last else { return }
        let coord = loc.coordinate
        DispatchQueue.main.async {
            self.rawCoordinate = coord
            self.rawPath.append(coord)
        }
        // Initialize Kalman filter at first GPS fix
        if kalmanFilter == nil {
            kalmanFilter = KalmanFilter(origin: loc)
        }
        // Run update step
        kalmanFilter?.update(with: loc)
        publishFilteredCoordinate()
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        let headingDegrees = newHeading.trueHeading > 0 ? newHeading.trueHeading : newHeading.magneticHeading
        currentHeadingRadians = headingDegrees * .pi / 180
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        DispatchQueue.main.async {
            self.error = error
        }
    }
}
