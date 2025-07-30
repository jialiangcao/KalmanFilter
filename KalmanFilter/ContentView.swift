//
//  ContentView.swift
//  KalmanFilter
//
//  Created by Jialiang Cao on 7/22/25.
//
import SwiftUI
import MapKit

struct ContentView: View {
    @StateObject private var viewModel = ViewModel()

    var body: some View {
        ZStack {
            MapView(rawCoordinates: viewModel.rawPath, filteredCoordinates: viewModel.filteredPath)
                .edgesIgnoringSafeArea(.all)

            VStack {
                HStack {
                    Circle().fill(Color.blue).frame(width: 10, height: 10)
                    Text("Raw Path")
                    Circle().fill(Color.red).frame(width: 10, height: 10)
                    Text("Filtered Path")
                }
                .padding(8)
                .background(Color.white.opacity(0.8))
                .cornerRadius(8)
                Spacer()
            }
            .padding()
        }
    }
}

// MARK: - MapView Representable
struct MapView: UIViewRepresentable {
    var rawCoordinates: [CLLocationCoordinate2D]
    var filteredCoordinates: [CLLocationCoordinate2D]

    func makeUIView(context: Context) -> MKMapView {
        let mapView = MKMapView()
        mapView.delegate = context.coordinator
        mapView.showsUserLocation = false
        mapView.mapType = .standard
        return mapView
    }

    func updateUIView(_ uiView: MKMapView, context: Context) {
        uiView.removeOverlays(uiView.overlays)
        
        // Raw path polyline
        let rawLine = MKPolyline(coordinates: rawCoordinates, count: rawCoordinates.count)
        rawLine.title = "raw"
        uiView.addOverlay(rawLine)
        
        // Filtered path polyline
        let filteredLine = MKPolyline(coordinates: filteredCoordinates, count: filteredCoordinates.count)
        filteredLine.title = "filtered"
        uiView.addOverlay(filteredLine)
        
        // Center on first filtered point
        if let last = filteredCoordinates.last {
            let region = MKCoordinateRegion(center: last,
                                            span: MKCoordinateSpan(latitudeDelta: 0.0015, longitudeDelta: 0.0015))
            uiView.setRegion(region, animated: false)
        }
    }

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    class Coordinator: NSObject, MKMapViewDelegate {
        func mapView(_ mapView: MKMapView, rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
            guard let polyline = overlay as? MKPolyline else {
                return MKOverlayRenderer(overlay: overlay)
            }
            let renderer = MKPolylineRenderer(polyline: polyline)
            if polyline.title == "raw" {
                renderer.strokeColor = .blue
            } else {
                renderer.strokeColor = .red
            }
            renderer.lineWidth = 3
            return renderer
        }
    }
}

#Preview {
    ContentView()
}
