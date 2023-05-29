import numpy as np
from scipy.spatial import distance_matrix
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

# Define the locations
locations = [
    {'id': 1, 'latitude': 23.8728568, 'longitude': 90.3984184, 'address': 'Uttara Branch'},
    {'id': 2, 'latitude': 23.8513998, 'longitude': 90.3944536, 'address': 'City Bank Airport'},
    {'id': 3, 'latitude': 23.8330429, 'longitude': 90.4092871, 'address': 'City Bank Nikunja'},
    {'id': 4, 'latitude': 23.8679743, 'longitude': 90.3840879, 'address': 'City Bank Beside Uttara Diagnostic'},
    {'id': 5, 'latitude': 23.8248293, 'longitude': 90.3551134, 'address': 'City Bank Mirpur 12'},
    {'id': 6, 'latitude': 23.827149, 'longitude': 90.4106238, 'address': 'City Bank Le Meridien'},
    {'id': 7, 'latitude': 23.8629078, 'longitude': 90.3816318, 'address': 'City Bank Shaheed Sarani'},
    {'id': 8, 'latitude': 23.8673789, 'longitude': 90.429412, 'address': 'City Bank Narayanganj'},
    {'id': 9, 'latitude': 23.8248938, 'longitude': 90.3549467, 'address': 'City Bank Pallabi'},
    {'id': 10, 'latitude': 23.813316, 'longitude': 90.4147498, 'address': 'City Bank JFP'}
]

# Extract coordinates from the location data
coordinates = np.array([(loc['latitude'], loc['longitude']) for loc in locations])

# Calculate distance matrix based on coordinates
dist_matrix = distance_matrix(coordinates, coordinates)

# Create a routing index manager
manager = pywrapcp.RoutingIndexManager(len(locations), 1, 0)

# Create a routing model
routing = pywrapcp.RoutingModel(manager)

# Define the distance callback
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return dist_matrix[from_node][to_node]

# Register the distance callback
transit_callback_index = routing.RegisterTransitCallback(distance_callback)

# Set the cost function
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Set the search parameters
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

# Solve the TSP
solution = routing.SolveWithParameters(search_parameters)

# Get the optimal route
route = []
index = routing.Start(0)
while not routing.IsEnd(index):
    route.append(manager.IndexToNode(index))
    index = solution.Value(routing.NextVar(index))
route.append(manager.IndexToNode(index))

# Print the optimized route
for idx in route:
    location = locations[idx]
    print(f"Location: {location['address']} (Latitude: {location['latitude']}, Longitude: {location['longitude']})")
