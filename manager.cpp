#include "manager.h"
#include <unordered_set>
#include <iostream>

using namespace std;

TransportManager::Stop& TransportManager::Stop::SetLocation(const Point& new_location) {
	location = make_unique<Point>(new_location);
	return *this;
}

TransportManager::Stop& TransportManager::CreateStop(string_view name) {
	if (!stops.count(name)) {
		stops[name].name = name;
		stops[name].id = stops.size() - 1;
		vertex_ids[stops.size() - 1] = name;
	}

	return stops[name];
}

void TransportManager::AddStop(
		string_view new_stop_name,
		const Point& new_stop_location,
		const vector<pair<string_view, int>>& distances
) {
	auto& new_stop = CreateStop(new_stop_name).SetLocation(new_stop_location);
	for (const auto& item : distances) {
		auto& other_stop = CreateStop(item.first);
		new_stop.real_distances[&other_stop] = item.second;
		if (!other_stop.real_distances.count(&new_stop)) {
			other_stop.real_distances[&new_stop] = item.second;
		}
	}
}

void TransportManager::AddBus(
		string_view new_bus_name,
		const vector<string_view>& new_bus_route,
		bool new_bus_is_circle
) {
	Bus& new_bus = buses[new_bus_name];
	new_bus.is_circle = new_bus_is_circle;
	for (const auto new_stop_name : new_bus_route) {
		auto& new_stop = CreateStop(new_stop_name);
		new_bus.route.push_back(&new_stop);
		new_stop.buses_list.insert(new_bus_name);
	}

	new_bus.name = new_bus_name;
}

optional<TransportManager::BusInfo> TransportManager::GetBusInfo(const string& requested_bus_name) const {
	if (!buses.count(requested_bus_name)) {
		return nullopt;
	}

	const auto& requested_bus = buses.at(requested_bus_name);
	if (!requested_bus.cached_info) {
		BusInfo info;

		info.stops_count = requested_bus.route.size();
		if (!requested_bus.is_circle) {
			info.stops_count *= 2;
			--info.stops_count;
		}

		info.unique_stops_count = (unordered_set<const Stop*>(
				begin(requested_bus.route),
				end(requested_bus.route)
		)).size();

		auto rc = GetRouteLengthAndCurvature(requested_bus);
		info.route_length = rc.first;
		info.curvature = rc.second;

		requested_bus.cached_info = info;
	}

	return *(requested_bus.cached_info);
}

optional<TransportManager::StopInfo> TransportManager::GetStopInfo(const string& requested_stop_name) const {
	if (!stops.count(requested_stop_name)) {
		return nullopt;
	}

	return TransportManager::StopInfo{stops.at(requested_stop_name).buses_list};
}

int TransportManager::GetDistBtwStops(const Stop* from, const Stop* to) {
	return from->real_distances.at(to);
}

pair<int, double> TransportManager::GetRouteLengthAndCurvature(const Bus& bus) {
	int real_length = 0;
	double earth_length = 0;

	for (size_t i = 1; i < bus.route.size(); ++i) {
		real_length += GetDistBtwStops(bus.route[i - 1], bus.route[i]);
		earth_length += GetDistance(
				*(bus.route[i - 1]->location),
				*(bus.route[i]->location)
		);
	}

	if (!bus.is_circle) {
		earth_length *= 2;
		for (size_t i = 1; i < bus.route.size(); ++i) {
			real_length += GetDistBtwStops(bus.route[i], bus.route[i - 1]);
		}
	}

	return {real_length, real_length / earth_length};
}

void TransportManager::SetParams(int new_bus_wait_time, double new_bus_velocity) {
	bus_wait_time = static_cast<double>(new_bus_wait_time);
	bus_velocity = 1000.0 * new_bus_velocity / 60.0;
}

void TransportManager::InitRouter() {
	graph = make_unique<TransportGraph>(stops.size());

	for (const auto& bus : buses) {
		AddRouteToGraph(
				bus.second.route.begin(),
				bus.second.route.end(),
				&bus.second
		);

		if (bus.second.is_circle) continue;

		AddRouteToGraph(
				bus.second.route.rbegin(),
				bus.second.route.rend(),
				&bus.second
		);
	}

	router = make_unique<TransportRouter>(*graph);
}

optional<TransportManager::RouteInfo> TransportManager::GetRouteInfo(const string& from, const string& to) const {
	auto builded_route = router->BuildRoute(stops.at(from).id, stops.at(to).id);
	if (!builded_route) return nullopt;

	RouteInfo info;
	info.total_time_ = builded_route->weight;


	for (size_t i = 0; i < builded_route->edge_count; ++i) {
		auto edge_id = router->GetRouteEdge(builded_route->id, i);
		const auto& edge = graph->GetEdge(edge_id);

		info.wait_items_.push_back(make_unique<RouteInfo::WaitItem>(bus_wait_time, vertex_ids.at(edge.from)));
		info.bus_items_.push_back(make_unique<RouteInfo::BusItem>(
				edge.weight - bus_wait_time,
				edge_ids.at(edge_id).bus_name,
				edge_ids.at(edge_id).span_count
		));
	}

	router->ReleaseRoute(builded_route->id);
	return info;
}
