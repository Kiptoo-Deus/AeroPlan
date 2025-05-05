#include <QApplication>
#include <QMainWindow>
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
    Weather() : wind_speed(0.0), wind_dir(0.0) {} // Default constructor
    Weather(double speed, double dir) : wind_speed(speed), wind_dir(dir) {}
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
                    double speed = j["wind"]["speed"].get<double>() * 1.94384; // m/s to knots
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
};

int main(int argc, char *argv[]) {
    curl_global_init(CURL_GLOBAL_ALL);
    QApplication app(argc, argv);

    std::string project_root = "/Users/joel/Documents/GitHub/flight-planner";
    FlightPlanner planner(project_root);
    planner.loadData("data/airports.csv", "data/navaids.csv");
    planner.printGraph("KJFK");

    auto route = planner.findOptimalRoute("KJFK", "KLAX");
    std::cout << "\nOptimal Route:\n";
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
    window.resize(800, 600);
    window.show();

    curl_global_cleanup();
    return app.exec();
}
