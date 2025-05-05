#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QWebEngineView>
#include <QDockWidget>
#include <QComboBox>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QListWidget>
#include <QTimer>
#include <QStyleFactory>
#include <QToolBar>
#include <QPushButton>
#include <QFileDialog>
#include <QLabel>
#include <QXmlStreamWriter>
#include <QFile>
#include <QMessageBox>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <limits>
#include <cmath>
#include <curl/curl.h>
#include <json.hpp>
#include <csv.hpp>
#include <stdexcept>
using json = nlohmann::json;

struct Waypoint {
    std::string id;
    double lat, lon;
    std::string type;
    Waypoint() : id(""), lat(0.0), lon(0.0), type("") {}
    Waypoint(std::string id_, double lat_, double lon_, std::string type_)
        : id(id_), lat(lat_), lon(lon_), type(type_) {}
};

struct Edge {
    std::string to;
    double distance;
    Edge(std::string t, double d) : to(t), distance(d) {}
};

struct Weather {
    double wind_speed;
    double wind_dir;
    std::string metar;
    Weather() : wind_speed(0.0), wind_dir(0.0), metar("") {}
    Weather(double speed, double dir, std::string m) : wind_speed(speed), wind_dir(dir), metar(m) {}
};

class MapWidget : public QWidget {
private:
    QWebEngineView* webView;
    std::unordered_map<std::string, Waypoint>& waypoints;
    std::vector<std::string>& route;
    QTextEdit* infoDisplay;

public:
    MapWidget(std::unordered_map<std::string, Waypoint>& wp, std::vector<std::string>& r, QWidget* parent = nullptr)
        : QWidget(parent), waypoints(wp), route(r) {
        QVBoxLayout* layout = new QVBoxLayout(this);
        webView = new QWebEngineView(this);
        infoDisplay = new QTextEdit(this);
        infoDisplay->setReadOnly(true);
        infoDisplay->setFixedHeight(100);
        layout->addWidget(webView);
        layout->addWidget(infoDisplay);

        QString html = R"(
            <!DOCTYPE html>
            <html>
            <head>
                <meta charset='utf-8' />
                <title>Flight Planner Map</title>
                <meta name='viewport' content='initial-scale=1,maximum-scale=1,user-scalable=no' />
                <script src='https://unpkg.com/maplibre-gl@3.6.2/dist/maplibre-gl.js'></script>
                <link href='https://unpkg.com/maplibre-gl@3.6.2/dist/maplibre-gl.css' rel='stylesheet' />
                <style>
                    body { margin: 0; padding: 0; }
                    #map { position: absolute; top: 0; bottom: 100px; width: 100%; }
                    #info { position: absolute; bottom: 0; width: 100%; height: 100px; background: #333; color: #fff; padding: 5px; }
                </style>
            </head>
            <body>
            <div id='map'></div>
            <div id='info'>Click a waypoint to see details</div>
            <script>
                const map = new maplibregl.Map({
                    container: 'map',
                    style: 'https://api.maptiler.com/maps/basic/style.json?key=tZjDLrbVj0oBGiaBHZJC',
                    center: [0, 0],
                    zoom: 1
                });

                map.on('load', () => {
                    console.log('Map loaded');
                    map.addSource('waypoints', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
                    map.addSource('route', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
                    map.addLayer({
                        id: 'waypoints',
                        type: 'circle',
                        source: 'waypoints',
                        paint: {
                            'circle-radius': 5,
                            'circle-color': ['match', ['get', 'type'], 'airport', '#0000FF', 'VOR', '#FF0000', '#000000']
                        }
                    });
                    map.addLayer({
                        id: 'route',
                        type: 'line',
                        source: 'route',
                        layout: { 'line-join': 'round', 'line-cap': 'round' },
                        paint: { 'line-color': '#00FF00', 'line-width': 2 }
                    });
                    map.addLayer({
                        id: 'waypoint-labels',
                        type: 'symbol',
                        source: 'waypoints',
                        layout: {
                            'text-field': ['get', 'id'],
                            'text-offset': [1, 0],
                            'text-anchor': 'left',
                            'text-size': 12
                        },
                        paint: {
                            'text-color': '#FFFFFF',
                            'text-halo-color': '#000000',
                            'text-halo-width': 1
                        }
                    });

                    if (window.initialWaypoints && window.initialRoute) {
                        updateWaypoints(window.initialWaypoints);
                        updateRoute(window.initialRoute, window.initialWaypoints);
                    }
                });

                function updateWaypoints(waypoints) {
                    if (!map.getSource('waypoints')) {
                        console.error('updateWaypoints: Waypoints source not ready');
                        return;
                    }
                    console.log('Updating waypoints:', waypoints.length);
                    const features = waypoints.map(wp => ({
                        type: 'Feature',
                        properties: { id: wp.id, type: wp.type },
                        geometry: { type: 'Point', coordinates: [wp.lon, wp.lat] }
                    }));
                    map.getSource('waypoints').setData({ type: 'FeatureCollection', features });
                }

                function updateRoute(route, waypoints) {
                    if (!map.getSource('route')) {
                        console.error('updateRoute: Route source not ready');
                        return;
                    }
                    console.log('Updating route:', route);
                    const coords = route.map(id => {
                        const wp = waypoints.find(w => w.id === id);
                        return wp ? [wp.lon, wp.lat] : [0, 0];
                    });
                    map.getSource('route').setData({
                        type: 'FeatureCollection',
                        features: [{
                            type: 'Feature',
                            geometry: { type: 'LineString', coordinates: coords }
                        }]
                    });
                }

                map.on('click', 'waypoints', (e) => {
                    const id = e.features[0].properties.id;
                    console.log('Waypoint clicked:', id);
                    document.getElementById('info').innerText = 'Clicked waypoint: ' + id;
                });
            </script>
            </body>
            </html>
        )";
        webView->setHtml(html);

        QTimer::singleShot(2000, this, &MapWidget::updateMap);
        resize(800, 600);
    }

    void updateMap() {
        QString jsWaypoints = "[";
        for (const auto& [id, wp] : waypoints) {
            jsWaypoints += QString("{'id': '%1', 'lat': %2, 'lon': %3, 'type': '%4'},")
                .arg(QString::fromStdString(id).replace("'", "\\'"))
                .arg(wp.lat)
                .arg(wp.lon)
                .arg(QString::fromStdString(wp.type));
        }
        jsWaypoints.chop(1);
        jsWaypoints += "]";

        QStringList routeList;
        for (const auto& id : route) {
            routeList.append(QString("'%1'").arg(QString::fromStdString(id).replace("'", "\\'")));
        }
        QString jsRoute = "[" + routeList.join(",") + "]";

        QString jsCode = QString(
            "window.initialWaypoints = %1;"
            "window.initialRoute = %2;"
            "if (map && map.loaded()) {"
            "  console.log('Map ready, updating waypoints and route');"
            "  updateWaypoints(window.initialWaypoints);"
            "  updateRoute(window.initialRoute, window.initialWaypoints);"
            "} else {"
            "  console.log('Map not loaded yet, waypoints and route stored');"
            "}"
        ).arg(jsWaypoints, jsRoute);

        webView->page()->runJavaScript(jsCode, [](const QVariant& result) {
            if (!result.isValid()) {
                std::cerr << "Failed to execute initial map update\n";
            } else {
                std::cout << "Initial map update executed successfully\n";
            }
        });
    }

    void setInfo(const QString& text) {
        infoDisplay->setText(text);
    }
};

class FlightPlanner {
private:
    std::unordered_map<std::string, Waypoint> waypoints;
    std::unordered_map<std::string, std::vector<Edge>> graph;
    std::unordered_map<std::string, Weather> weather_cache;
    std::string project_root;

    double greatCircleDistance(const Waypoint& a, const Waypoint& b) {
        const double R = 3440.0;
        double lat1 = a.lat * M_PI / 180.0, lon1 = a.lon * M_PI / 180.0;
        double lat2 = b.lat * M_PI / 180.0, lon2 = b.lon * M_PI / 180.0;
        return R * std::acos(std::sin(lat1) * std::sin(lat2) +
                             std::cos(lat1) * std::cos(lat2) * std::cos(lon2 - lon1));
    }

    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s) {
        size_t newLength = size * nmemb;
        s->append((char*)contents, newLength);
        return newLength;
    }

    Weather fetchWeather(double lat, double lon) {
        std::string cache_key = std::to_string(lat) + "," + std::to_string(lon);
        if (weather_cache.count(cache_key)) {
            return weather_cache.at(cache_key);
        }
        CURL* curl = curl_easy_init();
        std::string response;
        if (curl) {
            std::string url = "https://aviationweather.gov/api/data/metar?ids=" + waypoints.begin()->second.id +
                              "&format=json";
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
            CURLcode res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);

            if (res == CURLE_OK && !response.empty()) {
                try {
                    json j = json::parse(response);
                    if (j.is_array() && !j.empty() && j[0].contains("wspd") && j[0].contains("wdir") && j[0].contains("rawOb")) {
                        double speed = j[0]["wspd"].get<double>() * 1.94384;
                        double dir = j[0]["wdir"].get<double>();
                        std::string metar = j[0]["rawOb"].get<std::string>();
                        Weather w(speed, dir, metar);
                        weather_cache.emplace(cache_key, w);
                        return w;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "JSON parse error for lat=" << lat << ", lon=" << lon << ": " << e.what() << "\n";
                }
            } else {
                std::cerr << "CURL error for lat=" << lat << ", lon=" << lon << ": " << curl_easy_strerror(res) << "\n";
            }
        }
        Weather w(0.0, 0.0, "No METAR available");
        weather_cache.emplace(cache_key, w);
        return w;
    }

    double calculateFuelCost(const Waypoint& from, const Waypoint& to, double distance) {
        Weather w = fetchWeather(from.lat, from.lon);
        double bearing = std::atan2(
            std::sin(to.lon - from.lon) * std::cos(to.lat * M_PI / 180.0),
            std::cos(from.lat * M_PI / 180.0) * std::sin(to.lat * M_PI / 180.0) -
            std::sin(from.lat * M_PI / 180.0) * std::cos(to.lat * M_PI / 180.0) * std::cos(to.lon - from.lon)
        ) * 180.0 / M_PI;
        double relative_angle = std::fmod(w.wind_dir - bearing + 360.0, 360.0);
        double effective_speed = 450.0 + w.wind_speed * std::cos(relative_angle * M_PI / 180.0);
        double fuel_per_nm = 10.0 / effective_speed;
        return distance * fuel_per_nm;
    }

    void loadAirports(const std::string& filename) {
        try {
            csv::CSVReader reader(filename);
            for (auto& row : reader) {
                if (row["type"].get<std::string>() == "large_airport" ||
                    row["type"].get<std::string>() == "medium_airport") {
                    std::string id = row["ident"].get<std::string>();
                    double lat = row["latitude_deg"].get<double>();
                    double lon = row["longitude_deg"].get<double>();
                    waypoints.emplace(id, Waypoint(id, lat, lon, "airport"));
                }
            }
            std::cout << "Loaded airports from " << filename << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Error loading airports: " << e.what() << "\n";
        }
    }

    void loadNavaids(const std::string& filename) {
        try {
            csv::CSVReader reader(filename);
            for (auto& row : reader) {
                if (row["type"].get<std::string>().find("VOR") != std::string::npos) {
                    std::string id = row["ident"].get<std::string>();
                    double lat = row["latitude_deg"].get<double>();
                    double lon = row["longitude_deg"].get<double>();
                    waypoints.emplace(id, Waypoint(id, lat, lon, "VOR"));
                }
            }
            std::cout << "Loaded navaids from " << filename << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Error loading navaids: " << e.what() << "\n";
        }
    }

    void buildGraph(double max_distance = 500.0) {
        for (const auto& [id1, wp1] : waypoints) {
            for (const auto& [id2, wp2] : waypoints) {
                if (id1 != id2) {
                    double dist = greatCircleDistance(wp1, wp2);
                    if (dist <= max_distance) {
                        graph[id1].emplace_back(id2, dist);
                    }
                }
            }
        }
        for (const auto& [id1, wp1] : waypoints) {
            if (wp1.type == "VOR") {
                for (const auto& [id2, wp2] : waypoints) {
                    if (id2 != id1 && wp2.type == "VOR") {
                        double dist = greatCircleDistance(wp1, wp2);
                        if (dist <= 1000.0) {
                            graph[id1].emplace_back(id2, dist);
                        }
                    }
                }
            }
        }
    }

public:
    FlightPlanner(const std::string& root) : project_root(root) {}

    bool exportToPln(const std::vector<std::string>& route, const QString& filename) {
        QFile file(filename);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            std::cerr << "Failed to open " << filename.toStdString() << " for writing\n";
            return false;
        }

        QXmlStreamWriter xml(&file);
        xml.setAutoFormatting(true);
        xml.writeStartDocument();
        xml.writeStartElement("SimBase.Document");
        xml.writeAttribute("Type", "FlightPlan");
        xml.writeAttribute("version", "1,0");
        xml.writeTextElement("Title", QString::fromStdString(route.front() + " to " + route.back()));
        xml.writeTextElement("Descr", "Generated by Flight Planner");
        xml.writeStartElement("FlightPlan.FlightPlan");

        for (const auto& id : route) {
            if (!this->waypoints.count(id)) continue;
            const auto& wp = this->waypoints.at(id);
            xml.writeStartElement("WorldPosition");
            xml.writeCharacters(QString("%1,%2").arg(wp.lat, 0, 'f', 6).arg(wp.lon, 0, 'f', 6));
            xml.writeEndElement();
            xml.writeStartElement("ATCWaypoint");
            xml.writeAttribute("id", QString::fromStdString(id).left(10));
            xml.writeTextElement("ATCWaypointType", wp.type == "airport" ? "Airport" : "VOR");
            xml.writeTextElement("WorldPosition", QString("%1,%2,0").arg(wp.lat, 0, 'f', 6).arg(wp.lon, 0, 'f', 6));
            xml.writeEndElement();
        }

        xml.writeEndElement();
        xml.writeEndElement();
        xml.writeEndDocument();
        file.close();
        std::cout << "Exported .pln to " << filename.toStdString() << "\n";
        return true;
    }

    bool exportToFms(const std::vector<std::string>& route, const QString& filename) {
        QFile file(filename);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            std::cerr << "Failed to open " << filename.toStdString() << " for writing\n";
            return false;
        }

        QTextStream out(&file);
        out << "I\n1100 Version\nCYCLE 2310\nADEP " << QString::fromStdString(route.front()) << "\n";
        out << "ADES " << QString::fromStdString(route.back()) << "\n";
        out << "NUMENR " << route.size() << "\n";

        int index = 0;
        for (const auto& id : route) {
            if (!this->waypoints.count(id)) continue;
            const auto& wp = this->waypoints.at(id);
            QString wp_id = QString::fromStdString(id).replace(" ", "_");
            out << "1 " << wp_id << " 0.000000 " << QString::number(wp.lat, 'f', 6)
                << " " << QString::number(wp.lon, 'f', 6) << " 0\n";
            index++;
        }

        file.close();
        std::cout << "Exported .fms to " << filename.toStdString() << "\n";
        return true;
    }

    bool exportToLnmpln(const std::vector<std::string>& route, const QString& filename) {
        QFile file(filename);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            std::cerr << "Failed to open " << filename.toStdString() << " for writing\n";
            return false;
        }

        QXmlStreamWriter xml(&file);
        xml.setAutoFormatting(true);
        xml.writeStartDocument();
        xml.writeStartElement("LittleNavmap");
        xml.writeAttribute("SchemaVersion", "2.0");
        xml.writeStartElement("Flightplan");
        xml.writeTextElement("Header", QString::fromStdString(route.front() + " to " + route.back()));
        xml.writeStartElement("Waypoints");

        for (const auto& id : route) {
            if (!this->waypoints.count(id)) continue;
            const auto& wp = this->waypoints.at(id);
            xml.writeStartElement("Waypoint");
            xml.writeTextElement("Ident", QString::fromStdString(id));
            xml.writeTextElement("Type", wp.type == "airport" ? "AIRPORT" : "VOR");
            xml.writeTextElement("Pos", QString("%1 %2").arg(wp.lat, 0, 'f', 6).arg(wp.lon, 0, 'f', 6));
            xml.writeEndElement();
        }

        xml.writeEndElement();
        xml.writeEndElement();
        xml.writeEndElement();
        xml.writeEndDocument();
        file.close();
        std::cout << "Exported .lnmpln to " << filename.toStdString() << "\n";
        return true;
    }

    void loadData(const std::string& airports_file, const std::string& navaids_file) {
        std::string airports_path = this->project_root + "/" + airports_file;
        std::string navaids_path = this->project_root + "/" + navaids_file;
        loadAirports(airports_path);
        loadNavaids(navaids_path);
        buildGraph();
        std::cout << "Loaded " << waypoints.size() << " waypoints\n";
    }

    std::vector<std::string> findOptimalRoute(const std::string& start, const std::string& end) {
        struct Node {
            std::string id;
            double cost;
            double heuristic;
            std::vector<std::string> path;
            bool operator>(const Node& other) const { return cost + heuristic > other.cost + other.heuristic; }
        };

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
        std::unordered_map<std::string, double> min_cost;
        std::unordered_map<std::string, std::vector<std::string>> best_path;

        if (!waypoints.count(start) || !waypoints.count(end)) {
            std::cerr << "Start or end waypoint not found\n";
            return {};
        }

        pq.push({start, 0.0, greatCircleDistance(waypoints[start], waypoints[end]), {start}});
        min_cost[start] = 0.0;

        while (!pq.empty()) {
            Node current = pq.top();
            pq.pop();

            if (current.id == end) {
                return current.path;
            }

            if (current.cost > min_cost[current.id]) continue;

            for (const auto& edge : graph[current.id]) {
                double new_cost = current.cost + calculateFuelCost(waypoints[current.id], waypoints[edge.to], edge.distance);
                double heuristic = greatCircleDistance(waypoints[edge.to], waypoints[end]);

                if (!min_cost.count(edge.to) || new_cost < min_cost[edge.to]) {
                    min_cost[edge.to] = new_cost;
                    std::vector<std::string> new_path = current.path;
                    new_path.push_back(edge.to);
                    pq.push({edge.to, new_cost, heuristic, new_path});
                }
            }
        }

        std::cerr << "No route found from " << start << " to " << end << "\n";
        return {};
    }

    void printGraph(const std::string& start_id) const {
        if (graph.count(start_id)) {
            std::cout << start_id << ":\n";
            for (const auto& edge : graph.at(start_id)) {
                std::cout << "  -> " << edge.to << " (" << edge.distance << " nm)\n";
            }
        } else {
            std::cout << "Waypoint " << start_id << " not found\n";
        }
    }

    std::unordered_map<std::string, Weather>& getWeatherCache() { return weather_cache; }
    std::unordered_map<std::string, Waypoint>& getWaypoints() { return waypoints; }
    std::vector<std::string> getWaypointIds() const {
        std::vector<std::string> ids;
        for (const auto& [id, _] : waypoints) {
            ids.push_back(id);
        }
        std::sort(ids.begin(), ids.end());
        return ids;
    }

    std::string getMockChart(const std::string& airport_id) {
        if (waypoints.count(airport_id)) {
            return "SID " + airport_id + "-1: Depart runway, climb to 5000ft, proceed to nearest VOR.\n"
                   "STAR " + airport_id + "-1: From VOR, descend to 3000ft, approach runway.";
        }
        return "No chart available for " + airport_id;
    }

    double calculateRouteDistance(const std::vector<std::string>& route) {
        double total = 0.0;
        for (size_t i = 0; i < route.size() - 1; ++i) {
            if (waypoints.count(route[i]) && waypoints.count(route[i + 1])) {
                total += greatCircleDistance(waypoints.at(route[i]), waypoints.at(route[i + 1]));
            }
        }
        return total;
    }
};

int main(int argc, char *argv[]) {
    curl_global_init(CURL_GLOBAL_ALL);
    QApplication app(argc, argv);
    app.setStyle(QStyleFactory::create("Fusion"));
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    app.setPalette(darkPalette);
    app.setStyleSheet(
        "QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }"
        "QPushButton { background-color: #4a4a4a; color: white; padding: 5px; border-radius: 5px; font-family: 'Arial'; }"
        "QPushButton:hover { background-color: #5a5a5a; }"
        "QComboBox { background-color: #333; color: white; padding: 5px; border-radius: 3px; }"
        "QDockWidget::title { background: #3a3a3a; color: white; padding: 5px; }"
        "QTextEdit, QListWidget { background-color: #2a2a2a; color: white; border: 1px solid #555; }"
        "QLabel { color: white; font-family: 'Arial'; }"
    );

    std::string project_root = "/Users/joel/Documents/GitHub/flight-planner";
    FlightPlanner planner(project_root);
    planner.loadData("data/airports.csv", "data/navaids.csv");
    planner.printGraph("KJFK");

    std::vector<std::string> route = planner.findOptimalRoute("EGLL", "KJFK");
    std::cout << "\nOptimal Route (EGLL to KJFK):\n";
    if (!route.empty()) {
        for (const auto& wp : route) {
            std::cout << wp << " -> ";
        }
        std::cout << "END\n";
    } else {
        std::cout << "No route found\n";
    }

    QMainWindow window;
    window.setWindowTitle("Flight Planner");

    QToolBar* toolbar = window.addToolBar("Tools");
    toolbar->setMovable(false);
    QPushButton* exportButton = new QPushButton("Export Route", &window);
    toolbar->addWidget(exportButton);
    QPushButton* refreshButton = new QPushButton("Refresh Weather", &window);
    toolbar->addWidget(refreshButton);

    MapWidget* map = new MapWidget(planner.getWaypoints(), route, &window);
    window.setCentralWidget(map);

    QDockWidget* routeDock = new QDockWidget("Route Planner", &window);
    window.addDockWidget(Qt::LeftDockWidgetArea, routeDock);
    QWidget* routeWidget = new QWidget(routeDock);
    QVBoxLayout* routeLayout = new QVBoxLayout(routeWidget);
    QComboBox* startCombo = new QComboBox(routeWidget);
    QComboBox* endCombo = new QComboBox(routeWidget);
    QLabel* routeSummary = new QLabel("Route: 0 waypoints, 0 nm", routeWidget);
    routeLayout->addWidget(new QLabel("Departure", routeWidget));
    routeLayout->addWidget(startCombo);
    routeLayout->addWidget(new QLabel("Arrival", routeWidget));
    routeLayout->addWidget(endCombo);
    routeLayout->addWidget(routeSummary);
    routeLayout->addStretch();
    routeDock->setWidget(routeWidget);

    QDockWidget* chartDock = new QDockWidget("Charts", &window);
    QWidget* chartWidget = new QWidget(chartDock);
    QVBoxLayout* chartLayout = new QVBoxLayout(chartWidget);
    QListWidget* chartList = new QListWidget(chartWidget);
    chartLayout->addWidget(chartList);
    QTextEdit* chartDisplay = new QTextEdit(chartWidget);
    chartDisplay->setReadOnly(true);
    chartLayout->addWidget(chartDisplay);
    chartDock->setWidget(chartWidget);

    QDockWidget* toolsDock = new QDockWidget("Pilot Tools", &window);
    QTextEdit* toolsDisplay = new QTextEdit(toolsDock);
    toolsDisplay->setReadOnly(true);
    toolsDock->setWidget(toolsDisplay);

    window.tabifyDockWidget(chartDock, toolsDock);
    window.addDockWidget(Qt::RightDockWidgetArea, chartDock);

    for (const auto& id : planner.getWaypointIds()) {
        startCombo->addItem(QString::fromStdString(id));
        endCombo->addItem(QString::fromStdString(id));
    }
    startCombo->setCurrentText("EGLL");
    endCombo->setCurrentText("KJFK");

    chartList->addItem("EGLL SID-1");
    chartList->addItem("KJFK STAR-1");
    chartDisplay->setText(QString::fromStdString(planner.getMockChart("EGLL")));
    toolsDisplay->setText("METAR EGLL: " + QString::fromStdString(planner.getWeatherCache().begin()->second.metar) +
                          "\nNOTAM: No active NOTAMs\nATIS: Runway 27R in use.");
    routeSummary->setText(QString("Route: %1 waypoints, %2 nm")
        .arg(route.size())
        .arg(planner.calculateRouteDistance(route), 0, 'f', 1));

    QTimer* weatherTimer = new QTimer(&window);
    auto updateRoute = [&](const std::string& start, const std::string& end) {
        route = planner.findOptimalRoute(start, end);
        std::cout << "\nRefreshed Route (" << start << " to " << end << "):\n";
        if (!route.empty()) {
            for (const auto& wp : route) {
                std::cout << wp << " -> ";
            }
            std::cout << "END\n";
        } else {
            std::cout << "No route found\n";
        }
        map->updateMap();
        routeSummary->setText(QString("Route: %1 waypoints, %2 nm")
            .arg(route.size())
            .arg(planner.calculateRouteDistance(route), 0, 'f', 1));
        toolsDisplay->setText("METAR " + QString::fromStdString(start) + ": " +
                              QString::fromStdString(planner.getWeatherCache().begin()->second.metar) +
                              "\nNOTAM: No active NOTAMs\nATIS: Runway in use.");
    };

    QObject::connect(weatherTimer, &QTimer::timeout, [&]() {
        planner.getWeatherCache().clear();
        std::string start = startCombo->currentText().toStdString();
        std::string end = endCombo->currentText().toStdString();
        updateRoute(start, end);
    });
    weatherTimer->start(900000);

    QObject::connect(refreshButton, &QPushButton::clicked, [&]() {
        planner.getWeatherCache().clear();
        std::string start = startCombo->currentText().toStdString();
        std::string end = endCombo->currentText().toStdString();
        updateRoute(start, end);
    });

    QObject::connect(startCombo, &QComboBox::currentTextChanged, [&](const QString& text) {
        std::string start = text.toStdString();
        std::string end = endCombo->currentText().toStdString();
        updateRoute(start, end);
        chartList->clear();
        chartList->addItem(QString::fromStdString(start + " SID-1"));
        chartList->addItem(QString::fromStdString(end + " STAR-1"));
        chartDisplay->setText(QString::fromStdString(planner.getMockChart(start)));
    });

    QObject::connect(endCombo, &QComboBox::currentTextChanged, [&](const QString& text) {
        std::string start = startCombo->currentText().toStdString();
        std::string end = text.toStdString();
        updateRoute(start, end);
        chartList->clear();
        chartList->addItem(QString::fromStdString(start + " SID-1"));
        chartList->addItem(QString::fromStdString(end + " STAR-1"));
        chartDisplay->setText(QString::fromStdString(planner.getMockChart(end)));
    });

    QObject::connect(chartList, &QListWidget::itemClicked, [&](QListWidgetItem* item) {
        std::string id = item->text().split("-")[0].toStdString();
        chartDisplay->setText(QString::fromStdString(planner.getMockChart(id)));
    });

    QObject::connect(exportButton, &QPushButton::clicked, [&]() {
        QStringList formats;
        formats << "MSFS/Prepar3D (*.pln)" << "X-Plane (*.fms)" << "Little Navmap (*.lnmpln)";
        QString selectedFilter = formats[0];
        QString filename = QFileDialog::getSaveFileName(
            &window,
            "Export Flight Plan",
            QDir::homePath(),
            formats.join(";;"),
            &selectedFilter
        );
        if (filename.isEmpty()) return;

        bool success = false;
        if (selectedFilter.contains(".pln")) {
            if (!filename.endsWith(".pln")) filename += ".pln";
            success = planner.exportToPln(route, filename);
        } else if (selectedFilter.contains(".fms")) {
            if (!filename.endsWith(".fms")) filename += ".fms";
            success = planner.exportToFms(route, filename);
        } else if (selectedFilter.contains(".lnmpln")) {
            if (!filename.endsWith(".lnmpln")) filename += ".lnmpln";
            success = planner.exportToLnmpln(route, filename);
        }

        QMessageBox::information(&window, "Export",
            success ? "Flight plan exported successfully" : "Failed to export flight plan");
    });

    window.resize(1000, 700);
    window.show();

    curl_global_cleanup();
    return app.exec();
}

#include "main.moc"
