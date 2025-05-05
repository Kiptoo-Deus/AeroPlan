Flight Planner
A C++/Qt-based flight planning application for Microsoft Flight Simulator (MSFS), X-Plane, and Prepar3D. It generates optimal routes using the A* algorithm, displays real-time maps via MapTiler, and exports flight plans in .pln, .fms, and .lnmpln formats. The modern GUI includes a toolbar, route summary, tabified chart/tools docks, and pilot tools (METAR, NOTAMs, ATIS).

<img width="2213" alt="Screenshot 2025-05-05 at 8 29 12 PM" src="https://github.com/user-attachments/assets/4ee153db-c777-4fd8-85c4-832c13b2a8d5" />


Features

Route Planning: A* algorithm optimizes routes using global airport/VOR data from OurAirports, factoring in wind data from NOAA.
Map Display: Real-time MapTiler map with clickable waypoints (blue for airports, red for VORs), green route lines, and white labels.
Export: Supports .pln (MSFS/Prepar3D), .fms (X-Plane), and .lnmpln (Little Navmap) formats.
GUI: Dark-themed interface with toolbar (Export, Refresh Weather), route summary (waypoints, distance), and tabified docks for charts (mock SIDs/STARs) and pilot tools.
Weather: Fetches METAR data, updates every 15 minutes or on demand.
Cross-Platform: Runs on macOS and Windows.

Prerequisites

CMake: 3.10 or higher
Qt 6: Widgets and WebEngineWidgets modules
libcurl: For NOAA weather API
C++17 Compiler: Clang (macOS) or MSVC (Windows)
Dependencies:
json.hpp (nlohmann/json v3.11.3)
csv.hpp (csv-parser)


Data Files:
airports.csv (OurAirports)
navaids.csv (OurAirports)


Internet: For MapTiler and NOAA APIs

Setup and Installation
macOS

Install Homebrew:/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"


Install Dependencies:brew install cmake qt libcurl


Clone Repository:git clone https://github.com/yourusername/flight-planner.git
cd flight-planner


Download Dependencies:mkdir -p include
curl -L https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp -o include/json.hpp
curl -L https://github.com/vincentlaucsb/csv-parser/raw/master/single_include/csv.hpp -o include/csv.hpp


Download Data Files:mkdir -p data
curl -L https://davidmegginson.github.io/ourairports-data/airports.csv -o data/airports.csv
curl -L https://davidmegginson.github.io/ourairports-data/navaids.csv -o data/navaids.csv


Create Resource File:echo '<RCC><qresource prefix="/"><file>world_map.jpg</file></qresource></RCC>' > resources.qrc


Build:mkdir build && cd build
cmake .. -DQt6_DIR=/opt/homebrew/lib/cmake/Qt6 -DCURL_DIR=/opt/homebrew/lib/cmake/CURL
make


Run:./flight-planner



Windows

Install Visual Studio:
Download Visual Studio Community 2022 from https://visualstudio.microsoft.com.
Include "Desktop development with C++" workload.


Install CMake:
Download from https://cmake.org/download, add to PATH.


Install vcpkg:git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat


Install Dependencies via vcpkg:.\vcpkg install qt6-base qt6-webengine curl --triplet x64-windows


Clone Repository:git clone https://github.com/yourusername/flight-planner.git
cd flight-planner


Download Dependencies:mkdir include
curl -L https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp -o include\json.hpp
curl -L https://github.com/vincentlaucsb/csv-parser/raw/master/single_include/csv.hpp -o include\csv.hpp


Download Data Files:mkdir data
curl -L https://davidmegginson.github.io/ourairports-data/airports.csv -o data\airports.csv
curl -L https://davidmegginson.github.io/ourairports-data/navaids.csv -o data\navaids.csv


Create Resource File:echo ^<RCC^>^<qresource prefix="/"^>^<file^>world_map.jpg^</file^>^</qresource^>^</RCC^> > resources.qrc


Build:mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:\path\to\vcpkg\scripts\buildsystems\vcpkg.cmake
cmake --build . --config Release

Replace C:\path\to\vcpkg with your vcpkg directory.
Run:.\Release\flight-planner.exe



Usage

Route Planning: Select departure and arrival airports (e.g., EGLL to KJFK) from dropdowns. The A* algorithm generates an optimal route.
Map: View waypoints (blue for airports, red for VORs) and green route lines on a zoomable MapTiler map. Click waypoints to see details.
Export: Click "Export Route" to save as .pln (MSFS/Prepar3D), .fms (X-Plane), or .lnmpln (Little Navmap).
MSFS/Prepar3D: Copy .pln to C:\ProgramData\Garmin\Trainers\GTN\FPLN (Windows) or load via "Open Flight Plan".
X-Plane: Copy .fms to Output/FMS plans and load via FMS menu.
Little Navmap: Import .lnmpln and re-export if needed.


Charts: View mock SIDs/STARs in the Charts dock.
Pilot Tools: Check METAR, NOTAMs, and ATIS in the Tools dock.
Weather: Click "Refresh Weather" or wait 15 minutes for updated METAR data.

Troubleshooting

Build Errors: Ensure Qt6, libcurl, and CMake are installed. Verify include/json.hpp and data/airports.csv.
Map Not Rendering: Check internet and MapTiler API key:curl -I https://api.maptiler.com/maps/basic/style.json?key=tZjDLrbVj0oBGiaBHZJC


Export Fails: Verify file permissions:dir ~/route.pln


JavaScript Errors: Enable debug logs by adding to main() before app.exec():qputenv("QT_LOGGING_RULES", "qt5.webenginecontext=true");

Rebuild and check console.


Acknowledgments

OurAirports for airport/VOR data
MapTiler for map rendering
NOAA for METAR data

