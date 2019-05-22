#include "json.h"
#include <unordered_set>

using namespace std;

namespace Json {
	ostream& operator<<(ostream& os, const Node& node) {
		try {
			const auto& value = node.AsArray();
			os << "[";
			size_t i = 0;
			for (const auto& n : value) {
				if (i != 0) os << ", ";
				os << n;
				++i;
			}
			os << "]";
			return os;
		} catch (bad_variant_access&) {}

		try {
			const auto& value = node.AsMap();
			os << "{";
			size_t i = 0;
			for (const auto& p : value) {
				if (i != 0) os << ", ";
				os << "\"" << p.first << "\": ";
				os << p.second;
				++i;
			}
			os << "}";
			return os;
		} catch (bad_variant_access&) {}

		try {
			const auto& value = node.AsInt();
			os << value;
			return os;
		} catch (bad_variant_access&) {}

		try {
			const auto& value = node.AsString();
			os << "\"" << value << "\"";
			return os;
		} catch (bad_variant_access&) {}

		try {
			const auto& value = node.AsDouble();
			os << value;
			return os;
		} catch (bad_variant_access&) {}

		try {
			const auto& value = node.AsBool();
			if (value) {
				os << "true";
			} else {
				os << "false";
			}
			return os;
		} catch (bad_variant_access&) {}

		os << "empty";
		return os;
	}

	ostream& operator<<(ostream& os, const Document& doc) {
		os << doc.GetRoot();
		return os;
	}

  Document::Document(Node root) : root(move(root)) {
  }

  const Node& Document::GetRoot() const {
    return root;
  }

  Node LoadNode(istream& input, bool loadint = true);

  Node LoadArray(istream& input) {
    vector<Node> result;

    for (char c; input >> c && c != ']'; ) {
      if (c != ',') {
        input.putback(c);
      }
      result.push_back(LoadNode(input));
    }

    return Node(move(result));
  }

  Node LoadInt(istream& input) {
    int result = 0;
    while (isdigit(input.peek())) {
      result *= 10;
      result += input.get() - '0';
    }
    return Node(result);
  }

  Node LoadDouble(istream& input) {
  	  double result = 0;

  	  double sign = 1;
  	  if (input.peek()  == '-') {
  		  sign = -1;
  		  input.get();
  	  }

  	  bool before_dot = true;
  	  double coef = 1;
  	  while (isdigit(input.peek()) || input.peek() == '.') {
  		  if (before_dot) {
  			  if (input.peek() == '.') {
  				  before_dot = false;
  				  input.get();
  				  continue;
  			  } else {
  				  result *= 10;
  				  result += static_cast<double>(input.get() - '0');
  			  }
  		  } else {
  			  coef /= 10;
  			  result += coef * static_cast<double>(input.get() - '0');
  		  }
  	  }

  	  return Node(result * sign);
  }

  Node LoadBool(istream& input) {
  	  string str;
  	  while (isalpha(input.peek())) {
  		  str += input.get();
  	  }

  	  if (str == "true") {
  		  return Node(true);
  	  } else if (str == "false") {
  		  return Node(false);
  	  } else {
  		  throw runtime_error("Invalid bool value: " + str);
  	  }
  }

  Node LoadString(istream& input) {
    string line;
    getline(input, line, '"');
    return Node(move(line));
  }

  const unordered_set<string> loadint_keywords = {
		  "road_distances",
		  "id",
		  "route_length",
		  "request_id",
		  "stop_count",
		  "unique_stop_count",
		  "bus_wait_time"
  };

  const unordered_set<string> loaddouble_keywords = {
		  "longitude",
		  "latitude",
		  "curvature",
		  "bus_velocity"
  };

  Node LoadDict(istream& input, bool loadint = true) {
    map<string, Node> result;

    for (char c; input >> c && c != '}'; ) {
      if (c == ',') {
        input >> c;
      }

      string key = LoadString(input).AsString();
      input >> c;
      if (loadint_keywords.count(key)) {
    	  loadint = true;
      }
      if (loaddouble_keywords.count(key)) {
    	  loadint = false;
      }
      result.emplace(move(key), LoadNode(input, loadint));
    }

    return Node(move(result));
  }

  Node LoadNode(istream& input, bool loadint) {
    char c;
    input >> c;

    if (c == '[') {
      return LoadArray(input);
    } else if (c == '{') {
      return LoadDict(input);
    } else if (c == '"') {
      return LoadString(input);
    } else if (isalpha(c)){
      input.putback(c);
      return LoadBool(input);
    } else {
      input.putback(c);
      if (loadint) {
    	  return LoadInt(input);
      }
      return LoadDouble(input);
    }
  }

  Document LoadQueries(istream& input) {
    return Document{LoadNode(input)};
  }

  Document ProcessQueries(const Document& doc) {
	  TransportManager manager;

	  const auto& base_requests = doc.GetRoot().AsMap().at("base_requests").AsArray();
	  const auto& stat_requests = doc.GetRoot().AsMap().at("stat_requests").AsArray();
	  const auto& params = doc.GetRoot().AsMap().at("routing_settings").AsMap();

	  for (const auto& base_request_node : base_requests) {
		  const auto& type = base_request_node.AsMap().at("type").AsString();
		  if (type == "Stop") {
			  double lat = base_request_node.AsMap().at("latitude").AsDouble();
			  double lon = base_request_node.AsMap().at("longitude").AsDouble();
			  string_view name = base_request_node.AsMap().at("name").AsString();

			  const auto& distlist = base_request_node.AsMap().at("road_distances").AsMap();

			  vector<pair<string_view, int>> distances;
			  distances.reserve(distlist.size());
			  for (const auto& dist : distlist) {
				  distances.push_back({dist.first, dist.second.AsInt()});
			  }

			  manager.AddStop(name, Point(lat, lon), distances);
		  } else if (type == "Bus") {
			  string_view name = base_request_node.AsMap().at("name").AsString();
			  bool is_roundtrip = base_request_node.AsMap().at("is_roundtrip").AsBool();

			  const auto& stoplist = base_request_node.AsMap().at("stops").AsArray();

			  vector<string_view> stops;
			  stops.reserve(stoplist.size());
			  for (const auto& stop : stoplist) {
				  stops.push_back(stop.AsString());
			  }

			  manager.AddBus(name, stops, is_roundtrip);
		  }
	  }

	  manager.SetParams(params.at("bus_wait_time").AsInt(), params.at("bus_velocity").AsDouble());
	  manager.InitRouter();

	  vector<Node> stat_answers;
	  for (const auto& stat_request_node : stat_requests) {
		  const auto& type = stat_request_node.AsMap().at("type").AsString();
		  map<string, Node> stat_answer;

		  if (type == "Stop") {
			  const string& name = stat_request_node.AsMap().at("name").AsString();
			  int id = stat_request_node.AsMap().at("id").AsInt();

			  auto info = manager.GetStopInfo(name);

			  stat_answer["request_id"] = id;
			  if (info) {
				  vector<Node> buses;
				  for (const auto bus : (*info).buses_for_stop) {
					  buses.push_back(Node(string(bus)));
				  }
				  stat_answer["buses"] = move(buses);
			  } else {
				  stat_answer["error_message"] = "not found"s;
			  }
		  } else if (type == "Bus") {
			  const string& name = stat_request_node.AsMap().at("name").AsString();
			  int id = stat_request_node.AsMap().at("id").AsInt();

			  auto info = manager.GetBusInfo(name);

			  stat_answer["request_id"] = id;
			  if (info) {
					stat_answer["route_length"] = Node((*info).route_length);
					stat_answer["curvature"] = Node((*info).curvature);
					stat_answer["stop_count"] = Node(static_cast<int>((*info).stops_count));
					stat_answer["unique_stop_count"] = Node(static_cast<int>((*info).unique_stops_count));
			  } else {
				  stat_answer["error_message"] = "not found"s;
			  }
		  } else if (type == "Route") {
			  int id = stat_request_node.AsMap().at("id").AsInt();
			  const string& from = stat_request_node.AsMap().at("from").AsString();
			  const string& to = stat_request_node.AsMap().at("to").AsString();

			  auto info = manager.GetRouteInfo(from, to);

			  stat_answer["request_id"] = id;
			  if (info) {
				  stat_answer["total_time"] = info->total_time_;
				  vector<Node> items;
				  items.reserve(info->wait_items_.size());
				  for (size_t i = 0; i < info->wait_items_.size(); ++i) {
					  map<string, Node> wait_item;
					  wait_item["type"] = "Wait"s;
					  wait_item["time"] = static_cast<int>(info->wait_items_[i]->time_);
					  wait_item["stop_name"] = string(info->wait_items_[i]->stop_name_);

					  map<string, Node> bus_item;
					  bus_item["type"] = "Bus"s;
					  bus_item["time"] = info->bus_items_[i]->time_;
					  bus_item["bus"] = string(info->bus_items_[i]->bus_);
					  bus_item["span_count"] = static_cast<int>(info->bus_items_[i]->span_count_);

					  items.push_back(Node(move(wait_item)));
					  items.push_back(Node(move(bus_item)));
				  }
				  stat_answer["items"] = Node(move(items));
			  } else {
				  stat_answer["error_message"] = "not found"s;
			  }
		  }

		  stat_answers.push_back(move(stat_answer));
	  }

	  return Document{Node(stat_answers)};
  }

}
