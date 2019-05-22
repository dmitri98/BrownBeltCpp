#include <iostream>
#include "manager.h"
#include "json.h"

using namespace std;

int main() {
	cout.precision(7);

	cout << Json::ProcessQueries(Json::LoadQueries(cin));

	return 0;
}
