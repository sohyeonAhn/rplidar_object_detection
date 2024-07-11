import sys
from PyQt5.QtCore import Qt, QUrl, pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLineEdit, QLabel
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
import os

class CustomWebEnginePage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, message, line_number, source_id):
        print(f"Console message: {message} (line {line_number}): {source_id}")

class MapApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Google Maps with PyQt5")
        self.setGeometry(100, 100, 1200, 800)

        # API Key for Google Maps
        self.api_key = "AIzaSyBUBAhu3jl8NIC54-BXqEggLmJo-YNCIcw"

        # Initialize UI elements
        self.map_view = QWebEngineView()
        self.map_view.setPage(CustomWebEnginePage(self.map_view))
        self.lat_input = QLineEdit()
        self.lon_input = QLineEdit()
        self.add_marker_button = QPushButton("Add Marker and Record Path")
        self.zoom_in_button = QPushButton("Zoom In")
        self.zoom_out_button = QPushButton("Zoom Out")
        self.status_label = QLabel("Enter latitude and longitude and click 'Add Marker and Record Path'.")

        self.init_ui()
        self.load_map()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        controls_layout = QHBoxLayout()
        controls_layout.addWidget(QLabel("Latitude:"))
        self.lat_input.setPlaceholderText("Enter latitude")
        controls_layout.addWidget(self.lat_input)
        controls_layout.addWidget(QLabel("Longitude:"))
        self.lon_input.setPlaceholderText("Enter longitude")
        controls_layout.addWidget(self.lon_input)
        controls_layout.addWidget(self.add_marker_button)
        controls_layout.addWidget(self.zoom_in_button)
        controls_layout.addWidget(self.zoom_out_button)
        layout.addLayout(controls_layout)

        layout.addWidget(self.map_view)
        layout.addWidget(self.status_label)

        self.add_marker_button.clicked.connect(self.add_marker)
        self.zoom_in_button.clicked.connect(lambda: self.execute_js("map.setZoom(map.getZoom() + 1);"))
        self.zoom_out_button.clicked.connect(lambda: self.execute_js("map.setZoom(map.getZoom() - 1);"))

    def load_map(self):
        html_content = self.get_map_html()
        with open('map.html', 'w') as f:
            f.write(html_content)

        file_path = os.path.abspath("map.html")
        print("HTML file path:", file_path)  # 파일 경로를 출력하여 확인
        self.map_view.setUrl(QUrl.fromLocalFile(file_path))

    def get_map_html(self):
        return f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Google Maps</title>
            <style>
                body, html {{
                    height: 100%;
                    margin: 0;
                    padding: 0;
                }}
                #map {{
                    height: 100%;
                }}
            </style>
            <script>
                let map;
                let polyline;
                let path = [];

                function initMap() {{
                    map = new google.maps.Map(document.getElementById('map'), {{
                        center: {{lat: 37.5665, lng: 126.9780}},  // 서울의 위도와 경도
                        zoom: 8  // 더 작은 줌 레벨로 설정하여 지도를 축소
                    }});

                    polyline = new google.maps.Polyline({{
                        path: path,
                        geodesic: true,
                        strokeColor: '#FF0000',
                        strokeOpacity: 1.0,
                        strokeWeight: 2
                    }});
                    polyline.setMap(map);
                }}

                function addMarker(lat, lng) {{
                    var position = new google.maps.LatLng(lat, lng);
                    var marker = new google.maps.Marker({{
                        position: position,
                        map: map
                    }});
                    path.push(position);
                    polyline.setPath(path);
                }}
            </script>
        </head>
        <body>
            <div id="map"></div>
            <script src="https://maps.googleapis.com/maps/api/js?key={self.api_key}&callback=initMap" async defer></script>
        </body>
        </html>
        """

    @pyqtSlot()
    def add_marker(self):
        try:
            lat = float(self.lat_input.text())
            lon = float(self.lon_input.text())
            self.execute_js(f"addMarker({lat}, {lon});")
            self.status_label.setText(f"Marker added at ({lat}, {lon}) and path recorded.")
        except ValueError:
            self.status_label.setText("Invalid input. Please enter valid latitude and longitude.")

    def execute_js(self, script):
        self.map_view.page().runJavaScript(script)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MapApp()
    window.show()
    sys.exit(app.exec_())