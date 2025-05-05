#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QResource>
#include <QDockWidget>
#include <QComboBox>
#include <QVBoxLayout>
#include <QTimer>
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
    Weather() : wind_speed(0.0), wind_dir(0.0) {}
    Weather(double speed, double dir) : wind_speed(speed), wind_dir(dir) {}
};

class MapWidget : public QWidget {
private:
    std::unordered_map<std::string, Waypoint>& waypoints;
    std::vector<std::string>& route;
    QImage map_image;
    double zoom;
    double offset_x, offset_y;
    QPointF last_mouse_pos;

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        double map_width = map_image.width() * zoom;
        double map_height = map_image.height() * zoom;
        painter.drawImage(QRectF(offset_x, offset_y, map_width, map_height), map_image);

        auto toPixel = [&](double lat, double lon) -> QPointF {
            double x = (lon + 180.0) / 360.0 * map_image.width();
            double y = (90.0 - lat) / 180.0 * map_image.height();
            return QPointF(x * zoom + offset_x, y * zoom + offset_y);
        };

        for (const auto& [id, wp] : waypoints) {
            QPointF pixel = toPixel(wp.lat, wp.lon);
            painter.setPen(Qt::black);
            painter.setBrush(wp.type == "airport" ? Qt::blue : Qt::red);
            painter.drawEllipse(pixel, 5, 5);
            if (zoom > 2.0) { // Show labels only when zoomed in
                painter.drawText(pixel + QPointF(8, 0), QString::fromStdString(id));
            }
        }

        if (!route.empty()) {
            painter.setPen(QPen(Qt::green, 2));
            for (size_t i = 1; i < route.size(); ++i) {
                const Waypoint& wp1 = waypoints.at(route[i-1]);
                const Waypoint& wp2 = waypoints.at(route[i]);
                QPointF p1 = toPixel(wp1.lat, wp1.lon);
                QPointF p2 = toPixel(wp2.lat, wp2.lon);
                painter.drawLine(p1, p2);
            }
        }
    }

    void wheelEvent(QWheelEvent* event) override {
        double old_zoom = zoom;
        zoom *= event->angleDelta().y() > 0 ? 1.1 : 0.9;
        zoom = std::max(0.5, std::min(zoom, 10.0));
        double mouse_x = event->position().x();
        double mouse_y = event->position().y();
        offset_x += mouse_x * (old_zoom - zoom);
        offset_y += mouse_y * (old_zoom - zoom);
        update();
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (event->button() == Qt::LeftButton) {
            last_mouse_pos = event->pos();
            double x = (event->pos().x() - offset_x) / zoom;
            double y = (event->pos().y() - offset_y) / zoom;
            double lon = (x / map_image.width() * 360.0) - 180.0;
            double lat = 90.0 - (y / map_image.height() * 180.0);
            for (const auto& [id, wp] : waypoints) {
                double dist = std::sqrt(std::pow(wp.lat - lat, 2) + std::pow(wp.lon - lon, 2));
                if (dist < 1.0) {
                    std::cout << "Clicked waypoint: " << id << "\n";
                    break;
                }
            }
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (event->buttons() & Qt::LeftButton) {
            offset_x += event->pos().x() - last_mouse_pos.x();
            offset_y += event->pos().y() - last_mouse_pos.y();
            last_mouse_pos = event->pos();
            update();
        }
    }

public:
    MapWidget(std::unordered_map<std::string, Waypoint>& wp, std::vector<std::string>& r, QWidget* parent = nullptr)
        : QWidget(parent), waypoints(wp), route(r), zoom(1.0), offset_x(0), offset_y(0) {
        setMouseTracking(true);
        map_image.load(":/world_map.jpg");
        if (map_image.isNull()) {
            std::cerr << "Failed to load world_map.jpg\n";
        }
        resize(800, 600);
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
            std::string url = "http://api.openweathermap.org/data/2.5/weather?lat=" + std::to_string(lat) +
                              "&lon=" + std::to_string(lon) + "&appid=83e163f68a192a2705bcfd9f6e1dfb93&units=metric";
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
            CURLcode res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);

            if (res == CURLE_OK) {
                try {
                    json j = json::parse(response);
                    double speed = j["wind"]["speed"].get<double>() * 1.94384;
                    double dir = j["wind"]["deg"].get<double>();
                    Weather w(speed, dir);
                    weather_cache.emplace(cache_key, w);
                    return w;
                } catch (...) {
                    std::cerr << "JSON parse error for lat=" << lat << ", lon=" << lon << "\n";
                }
            } else {
                std::cerr << "CURL error: " << curl_easy_strerror(res) << "\n";
            }
        }
        Weather w(0.0, 0.0);
        weather_cache.emplace(cache_key, w);
        return w;
    }

    double calculateFuelCost(const Waypoint& from, const Waypoint& to, double distance) {
        Weather w = fetchWeather(from.lat, from.lon);
        double bearing = std::atan2(
            std::sin(to.lon - from.lon) * std::cos(to.lat),
            std::cos(from.lat) * std::sin(to.lat) - std::sin(from.lat) * std::cos(to.lat) * std::cos(to.lon - from.lon)
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
    }

public:
    FlightPlanner(const std::string& root) : project_root(root) {}

    void loadData(const std::string& airports_file, const std::string& navaids_file) {
        std::string airports_path = project_root + "/" + airports_file;
        std::string navaids_path = project_root + "/" + navaids_file;
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
};

int main(int argc, char *argv[]) {
    curl_global_init(CURL_GLOBAL_ALL);
    QApplication app(argc, argv);

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

    MapWidget* map = new MapWidget(planner.getWaypoints(), route, &window);
    window.setCentralWidget(map);

    QDockWidget* dock = new QDockWidget("Route Planner", &window);
    window.addDockWidget(Qt::LeftDockWidgetArea, dock);
    QWidget* dockWidget = new QWidget(dock);
    QVBoxLayout* layout = new QVBoxLayout(dockWidget);
    QComboBox* startCombo = new QComboBox(dockWidget);
    QComboBox* endCombo = new QComboBox(dockWidget);
    layout->addWidget(startCombo);
    layout->addWidget(endCombo);
    dock->setWidget(dockWidget);

    for (const auto& id : planner.getWaypointIds()) {
        startCombo->addItem(QString::fromStdString(id));
        endCombo->addItem(QString::fromStdString(id));
    }
    startCombo->setCurrentText("EGLL");
    endCombo->setCurrentText("KJFK");

    QTimer* weatherTimer = new QTimer(&window);
    QObject::connect(weatherTimer, &QTimer::timeout, [&]() {
        planner.getWeatherCache().clear();
        std::string start = startCombo->currentText().toStdString();
        std::string end = endCombo->currentText().toStdString();
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
        map->update();
    });
    weatherTimer->start(900000); // 15 minutes

    QObject::connect(startCombo, &QComboBox::currentTextChanged, [&](const QString& text) {
        std::string start = text.toStdString();
        std::string end = endCombo->currentText().toStdString();
        route = planner.findOptimalRoute(start, end);
        std::cout << "\nUpdated Route (" << start << " to " << end << "):\n";
        if (!route.empty()) {
            for (const auto& wp : route) {
                std::cout << wp << " -> ";
            }
            std::cout << "END\n";
        } else {
            std::cout << "No route found\n";
        }
        map->update();
    });

    QObject::connect(endCombo, &QComboBox::currentTextChanged, [&](const QString& text) {
        std::string start = startCombo->currentText().toStdString();
        std::string end = text.toStdString();
        route = planner.findOptimalRoute(start, end);
        std::cout << "\nUpdated Route (" << start << " to " << end << "):\n";
        if (!route.empty()) {
            for (const auto& wp : route) {
                std::cout << wp << " -> ";
            }
            std::cout << "END\n";
        } else {
            std::cout << "No route found\n";
        }
        map->update();
    });

    window.resize(800, 600);
    window.show();

    curl_global_cleanup();
    return app.exec();
}
