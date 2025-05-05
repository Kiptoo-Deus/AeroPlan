#include <QApplication>
#include <QMainWindow>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <csv.hpp>
#include <stdexcept>

struct Waypoint {
    std::string id;
    double lat, lon;
    std::string type; // "airport" or "VOR"
    Waypoint() : id(""), lat(0.0), lon(0.0), type("") {}
    Waypoint(std::string id_, double lat_, double lon_, std::string type_)
        : id(id_), lat(lat_), lon(lon_), type(type_) {}
};

struct Edge {
    std::string to;
    double distance;
    Edge(std::string t, double d) : to(t), distance(d) {}
};

class FlightPlanner {
private:
    std::unordered_map<std::string, Waypoint> waypoints;
    std::unordered_map<std::string, std::vector<Edge>> graph;
    std::string project_root;

    double greatCircleDistance(const Waypoint& a, const Waypoint& b) {
        const double R = 3440.0;
        double lat1 = a.lat * M_PI / 180.0, lon1 = a.lon * M_PI / 180.0;
        double lat2 = b.lat * M_PI / 180.0, lon2 = b.lon * M_PI / 180.0;
        return R * std::acos(std::sin(lat1) * std::sin(lat2) +
                             std::cos(lat1) * std::cos(lat2) * std::cos(lon2 - lon1));
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
    QApplication app(argc, argv);

    std::string project_root = "/Users/joel/Documents/GitHub/flight-planner";
    FlightPlanner planner(project_root);
    planner.loadData("data/airports.csv", "data/navaids.csv");
    planner.printGraph("KJFK");

    QMainWindow window;
    window.setWindowTitle("Flight Planner");
    window.resize(800, 600);
    window.show();

    return app.exec();
}
