#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
    return data[id].lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
    return data[id].lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
    return data[id].name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
    return data[id].neighbors;
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  std::string res = "";
  for(auto it = data.begin(); it != data.end(); it++){
    if( (it->second).name == name){
      res = it->first;
      break;
    }
  }
  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  auto id = GetID(name);
  if (id != ""){
    results.first = GetLat(id);
    results.second = GetLon(id);
  }
  return results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b){
  int m = a.size();
  int n = b.size();
  std::vector< std::vector<int> > d(m+1, std::vector<int>(n+1, 0)); 

  for (int i = 0; i <= m; i++){
    for (int j = 0; j<= n; j++){
      if (i == 0 && j == 0){
        d[i][j] = 0;
      } else if (i != 0 && j == 0){
        d[i][j] = i;
      } else if (i == 0 && j != 0){
        d[i][j] = j;
      } else if(a[i-1] == b[j-1]){
        d[i][j] =  d[i-1][j-1];
      } else {
        d[i][j] = 1 + std::min(std::min(d[i-1][j], d[i][j-1]), d[i-1][j-1]);
      }
    }
  }

}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  auto lname = name;
  std::transform(name.begin(),name.end(),lname.begin(), ::tolower);
  int minimum = INT_MAX;
  int distance = 0;
  std::string tmp = "";
  auto id = GetID(name);
  if (id.empty()){
    for (auto it = data.begin(); it != data.end(); it++){
      auto n = (it->second).name;
      std::transform((it->second).name.begin(),(it->second).name.end(),n.begin(), ::tolower);
      if (n == lname){
        tmp = (it->second).name;
      } else if (!n.empty()){
        distance = CalculateEditDistance(lname, n);
        if (distance < minimum){
          minimum = distance;
          tmp = (it->second).name;
        }
      }
    }
  } else {
    tmp = name;
  }
  return tmp;
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  if(name.length() > 1 && name[name.length() - 1] == ' '){name.pop_back();}
  if(name.length() > 1 && name[0]== ' '){name.erase(name.begin());}
  int size = name.length();
  std::transform(name.begin(),name.end(),name.begin(), ::tolower);
  std::unordered_map<std::string, Node>::iterator it = data.begin();
  while(it != data.end()){
    std::string e = GetName(it->first);
    std::string temp = e;
    std::transform(e.begin(),e.end(),temp.begin(), ::tolower);
    if (size == 0 || name == " "){
      if(e != ""){results.push_back(e);}}
    else if (temp.substr(0,size) == name){
      results.push_back(e);
    }
    it++;
  }
  std::sort(results.begin(),results.end());
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;
  
  // Get ID's of Start Location and End Location
  std::string start_ID_= GetID(location1_name);
  std::string end_ID_ = GetID(location2_name);

  // Node start_node_ = data[start_ID_];
  // Node end_node = data[end_ID_];

  // Create Min Heap using Priority Queue
  std::priority_queue< std::pair<double, std::string>, std::vector< std::pair< double,std::string > >, std::greater< std::pair< double, std::string >>> q_;

  // Stores the Shortest Distance of Each Node using Unordered Map
  std::unordered_map< std::string, double > distance_;

  // Stores the predecessors for each node using unordered map
  std::unordered_map< std::string, std::vector< std::string > > shortest_path_;

  // Strores Visted Nodes using Unordered Map
  std::unordered_map< std::string, bool > visited_;

  // set start_node as visited and its shortest_distance as 0 and set the predecessor path as empty vector;
  // visited_[start_ID_] = true;
  distance_[start_ID_] = 0;
  // visited_[start_ID_] = true;
  std::vector< std::string > current_path_ = {};
  shortest_path_[start_ID_] = current_path_;
  // Push the start node into maxheap
  q_.push(std::make_pair(distance_[start_ID_], start_ID_));
  // Run Dijkstra Unitil the Priority Queue is Empty
  while( q_.empty() != true){
    // Get The Shortest Distance Node ( Top One in priority Queue)
    auto qtop = q_.top();
    auto cid = qtop.second;
    auto cdist = qtop.first;
    q_.pop();

    // Check if The current Node is same as the End Node
    // If Not Proceed Inside the for Loop
    if( cid != end_ID_ ){
      // Check if the Current Distance is Greater than the shortest Distance of the Node
      // If Not Proceed with If loop
      if ( cdist <= distance_[cid] ){
        if (visited_.find(cid) == visited_.end()){
          visited_[cid] = false;
        }
        if ( visited_[cid] == false) {
          visited_[cid] = true;
          auto neighbors = GetNeighborIDs(cid);
          for (auto neighbor : neighbors){
            double new_dist_ = cdist + CalculateDistance(cid, neighbor);
            if ( distance_.find(neighbor) == distance_.end() ){
              distance_[neighbor] = INT_MAX;
            }
            if (new_dist_ < distance_[neighbor]){
              distance_[neighbor] = new_dist_;
              auto previous_path = shortest_path_[cid];
              previous_path.push_back(cid);
              shortest_path_[neighbor] = previous_path;
              q_.push(std::make_pair(distance_[neighbor], neighbor));
            }
          }
        }
      }
    } else if ( cid == end_ID_ ) {
      path = shortest_path_[cid];
      path.push_back(cid);
      break;
    }
  }

  // std::cout << path[0] << " " << start_ID_ << " " << end_ID_ << std::endl;
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
  std::vector<std::string> path;

  // Getting start and End Node Id's
  std::string start_ID_ = GetID(location1_name);
  std::string end_ID_ = GetID(location2_name);

  // Shortest Distance of Each Node using Unordered Map
  std::unordered_map< std::string, double > distance_;

  // Shortest path to cirrent Node using unordered map
  std::unordered_map< std::string, std::vector< std::string > > shortest_path_;

  // set a boolean stop sign to true
  bool stop_sign = true;
  std::vector<std::string> empty_predecessor_ = {};

  for (auto d : data){
    distance_[d.first] = INT_MAX;
    shortest_path_[d.first] = empty_predecessor_;

  distance_[start_ID_] = 0.0;
  }
  for ( int i = 1 ; i <= data.size() - 1 ; i ++ ){
    std::cout << i << std::endl;
    for ( auto d : data ) {
      auto neighbors = GetNeighborIDs(d.first);
      for( auto neighbor : neighbors ) {
        if(distance_[d.first] != INT_MAX){
          auto new_dist_ = distance_[d.first] + CalculateDistance(d.first, neighbor);
          if ( new_dist_ < distance_[neighbor]){
            distance_[neighbor] = new_dist_;
            auto cpath = shortest_path_[d.first];
            cpath.push_back(d.first);
            shortest_path_[neighbor] = cpath;
            stop_sign = false;
          } else {
            stop_sign = true;
            break;
          }
        }
      }

    }
  }
  path = shortest_path_[end_ID_];
  path.push_back(end_ID_);
  return path;  
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  return result;                                                     
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
