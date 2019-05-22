#pragma once

#include <string>
#include <vector>
#include <string_view>
#include <set>
#include <unordered_map>
#include <deque>
#include <memory>
#include <optional>
#include "point.h"
#include "graph.h"
#include "router.h"

class TransportManager {
public:
	void AddStop(
			std::string_view new_stop_name,
			const Point& new_stop_location,
			const std::vector<std::pair<std::string_view, int>>& distances
	);

	void AddBus(
			std::string_view new_bus_name,
			const std::vector<std::string_view>& new_bus_route,
			bool new_bus_is_circle
	);

	void SetParams(int new_bus_wait_time, double new_bus_velocity);
	void InitRouter();

private:
	struct Stop;
	struct Bus;

public:
	struct StopInfo {
		const std::set<std::string_view>& buses_for_stop;
	};

	struct BusInfo {
		size_t stops_count;
		size_t unique_stops_count;
		int route_length;
		double curvature;
	};

	struct RouteInfo {
		struct RouteItem {
			RouteItem(std::string type, double time) : type_(type), time_(time) {}

			const std::string type_;
			double time_;
		};

		struct WaitItem : RouteItem {
			WaitItem(double time, std::string_view stop_name)
			        : RouteItem("Wait", time)
			        , stop_name_(stop_name) {}

			std::string_view stop_name_;
		};

		struct BusItem : RouteItem {
			BusItem(double time, std::string_view bus, size_t span_count)
			       : RouteItem("Bus", time)
			       , bus_(bus)
			       , span_count_(span_count) {}

			std::string_view bus_;
			size_t span_count_;
		};

		double total_time_ = 0;
		std::vector<std::unique_ptr<WaitItem>> wait_items_;
		std::vector<std::unique_ptr<BusItem>> bus_items_;
 	};

	std::optional<BusInfo> GetBusInfo(const std::string& requested_bus_name) const;
	std::optional<StopInfo> GetStopInfo(const std::string& requested_stop_name) const;
	std::optional<RouteInfo> GetRouteInfo(const std::string& from, const std::string& to) const;

private:
	struct Stop {
		Stop& SetLocation(const Point& new_location);

		size_t id;
		std::string_view name;
		std::unique_ptr<Point> location;
		std::set<std::string_view> buses_list;
		std::unordered_map<const Stop*, int> real_distances;
	};

	struct Bus {
		std::string_view name;
		std::vector<const Stop*> route;
		bool is_circle;
		mutable std::optional<BusInfo> cached_info;
	};

	std::unordered_map<std::string_view, Stop> stops;
	std::unordered_map<std::string_view, Bus> buses;

	static int GetDistBtwStops(const Stop* from, const Stop* to);
	static std::pair<int, double> GetRouteLengthAndCurvature(const Bus& bus);

	double bus_wait_time = 0;
	double bus_velocity = 0;

	struct EdgeInfo {
		std::string_view bus_name;
		size_t span_count;
	};

	std::unordered_map<Graph::VertexId, std::string_view> vertex_ids;
	std::unordered_map<Graph::EdgeId, EdgeInfo> edge_ids;

	using TransportGraph = Graph::DirectedWeightedGraph<double>;
	using TransportRouter = Graph::Router<double>;

	std::unique_ptr<TransportGraph> graph;
	std::unique_ptr<TransportRouter> router;

	template <typename RouteIt>
	void AddRouteToGraph(RouteIt begin, RouteIt end, const Bus* bus) {
		for (RouteIt it1 = begin; it1 < end; ++it1) {
			int cur_dist = 0;
			for (RouteIt it2 = it1 + 1; it2 < end; ++it2) {
				cur_dist += GetDistBtwStops(*prev(it2), *it2);
				size_t edge_id = graph->AddEdge({
					(*it1)->id,
					(*it2)->id,
					bus_wait_time + cur_dist / bus_velocity
				});
				edge_ids[edge_id] = {bus->name, static_cast<size_t>(it2 - it1)};
			}
		}
	}

	Stop& CreateStop(std::string_view name);
};
