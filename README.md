# ✈️ Flight Planner

A C++/Qt-based flight planning application for Microsoft Flight Simulator (MSFS), X-Plane, and Prepar3D. It generates optimal routes using the A\* algorithm, displays real-time maps via MapTiler, and exports flight plans in `.pln`, `.fms`, and `.lnmpln` formats.

<img width="2213" alt="Screenshot 2025-05-05 at 8 29 12 PM" src="https://github.com/user-attachments/assets/0d6d0178-dfa8-47fd-83b3-87d9e42d3e4e" />


## 🌟 Features

* **Route Planning**: A\* algorithm optimized with global airport and VOR data from OurAirports, considering wind data from NOAA.
* **Map Display**: Real-time MapTiler map with clickable waypoints

  * 🟦 Airports
  * 🔴 VORs
  * 🟩 Route lines
  * ⚪ White labels
* **Export Formats**:

  * `.pln` (MSFS/Prepar3D)
  * `.fms` (X-Plane)
  * `.lnmpln` (Little Navmap)
* **Modern GUI**:

  * Dark theme
  * Toolbar (Export, Refresh Weather)
  * Route summary (waypoints, distance)
  * Tabified docks (charts, pilot tools)
* **Weather Tools**: Fetches METAR data every 15 minutes or on demand
* **Cross-Platform**: macOS & Windows

---

## ⚙️ Prerequisites

* **CMake**: ≥ 3.10

* **Qt 6**: `Widgets`, `WebEngineWidgets`

* **libcurl**: For NOAA weather API

* **C++17 Compiler**: Clang (macOS) or MSVC (Windows)

* **Dependencies**:

  * [`json.hpp`](https://github.com/nlohmann/json) (v3.11.3)
  * [`csv.hpp`](https://github.com/vincentlaucsb/csv-parser)

* **Data Files**:

  * `airports.csv` (OurAirports)
  * `navaids.csv` (OurAirports)

* **Internet Access**: Required for MapTiler & NOAA API

---

## 🖥️ Installation

### macOS

1. **Install Homebrew**

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

2. **Install Dependencies**

```bash
brew install cmake qt libcurl
```

3. **Clone Repository**

```bash
git clone https://github.com/yourusername/flight-planner.git
cd flight-planner
```

4. **Download Headers**

```bash
mkdir -p include
curl -L https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp -o include/json.hpp
curl -L https://github.com/vincentlaucsb/csv-parser/raw/master/single_include/csv.hpp -o include/csv.hpp
```

5. **Download Data Files**

```bash
mkdir -p data
curl -L https://davidmegginson.github.io/ourairports-data/airports.csv -o data/airports.csv
curl -L https://davidmegginson.github.io/ourairports-data/navaids.csv -o data/navaids.csv
```

6. **Create Resource File**

```bash
echo '<RCC><qresource prefix="/"><file>world_map.jpg</file></qresource></RCC>' > resources.qrc
```

7. **Build & Run**

```bash
mkdir build && cd build
cmake .. -DQt6_DIR=/opt/homebrew/lib/cmake/Qt6 -DCURL_DIR=/opt/homebrew/lib/cmake/CURL
make
./flight-planner
```

---

### Windows

1. **Install Visual Studio**

   * Download [Visual Studio 2022 Community](https://visualstudio.microsoft.com/)
   * Include **"Desktop development with C++"** workload

2. **Install CMake**

   * [Download](https://cmake.org/download/)
   * Add to `PATH`

3. **Install vcpkg**

```powershell
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```

4. **Install Dependencies**

```powershell
.\vcpkg install qt6-base qt6-webengine curl --triplet x64-windows
```

5. **Clone Repository**

```powershell
git clone https://github.com/yourusername/flight-planner.git
cd flight-planner
```

6. **Download Headers**

```powershell
mkdir include
curl -L https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp -o include\json.hpp
curl -L https://github.com/vincentlaucsb/csv-parser/raw/master/single_include/csv.hpp -o include\csv.hpp
```

7. **Download Data Files**

```powershell
mkdir data
curl -L https://davidmegginson.github.io/ourairports-data/airports.csv -o data\airports.csv
curl -L https://davidmegginson.github.io/ourairports-data/navaids.csv -o data\navaids.csv
```

8. **Create Resource File**

```powershell
echo ^<RCC^>^<qresource prefix="/"^>^<file^>world_map.jpg^</file^>^</qresource^>^</RCC^> > resources.qrc
```

9. **Build & Run**

```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:\path\to\vcpkg\scripts\buildsystems\vcpkg.cmake
cmake --build . --config Release
.\Release\flight-planner.exe
```

> 🔁 Replace `C:\path\to\vcpkg` with your actual vcpkg path.

---

## 🚀 Usage

* **Route Planning**: Select departure/arrival airports (e.g., `EGLL` to `KJFK`); A\* algorithm generates route.
* **Map Viewer**:

  * 🟦 Airports, 🔴 VORs
  * 🟩 Green route line
  * Zoomable, clickable interface
* **Export**:

  * `.pln`: Copy to `C:\ProgramData\Garmin\Trainers\GTN\FPLN` or load in MSFS/Prepar3D
  * `.fms`: Copy to `Output/FMS plans` for X-Plane
  * `.lnmpln`: Import into Little Navmap
* **Charts & Tools**:

  * View mock SIDs/STARs
  * METAR, NOTAMs, and ATIS
  * "Refresh Weather" for updates

---

## 🛠️ Troubleshooting

* **Build Errors**:

  * Confirm Qt6, libcurl, and CMake are installed
  * Check file presence: `include/json.hpp`, `include/csv.hpp`, `data/*.csv`

* **Map Not Rendering**:

  * Verify internet access
  * Test MapTiler API:

    ```bash
    curl -I https://api.maptiler.com/maps/basic/style.json?key=YOUR_API_KEY
    ```

* **Export Issues**:

  * Check file permissions:

    ```bash
    ls -l ~/route.pln
    ```

* **JavaScript Errors in WebView**:
  Add this before `app.exec()`:

  ```cpp
  qputenv("QT_LOGGING_RULES", "qt5.webenginecontext=true");
  ```

---

## 🤝 Contributing

Contributions are welcome!

1. Fork the repo
2. Create a feature branch
3. Submit a pull request

---


## 🙏 Acknowledgments

* [OurAirports](https://ourairports.com) – Airport & VOR data
* [MapTiler](https://maptiler.com) – Map rendering
* [NOAA](https://aviationweather.gov/metar) – METAR data

---
