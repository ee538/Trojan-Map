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
  return d[m][n];
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
  // if(name.length() > 1 && name[name.length() - 1] == ' '){name.pop_back();}
  // if(name.length() > 1 && name[0]== ' '){name.erase(name.begin());}
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
  
  std::vector<std::string> path ={};

  if ((location1_name.empty()) || (location2_name.empty())){
    return path;
  }

  
  // Get ID's of Start Location and End Location
  std::string start_ID_= GetID(location1_name);
  std::string end_ID_ = GetID(location2_name);
  if ((start_ID_.empty()) || (end_ID_.empty())){
    return path;
  }
  // Create Min Heap using Priority Queue
  std::priority_queue< std::pair<double, std::string>, std::vector< std::pair< double,std::string > >, std::greater< std::pair< double, std::string >>> q_;

  // Stores the Shortest Distance of Each Node using Unordered Map
  std::unordered_map< std::string, double > distance_;

  // Stores the predecessors for each node using unordered map
  std::unordered_map< std::string, std::string > parent_;

  // Strores Visted Nodes using Unordered Map
  std::unordered_map< std::string, bool > visited_;

  // set start_node as visited and its shortest_distance as 0 and set the predecessor path as empty vector;
  // visited_[start_ID_] = true;
  distance_[start_ID_] = 0;

  // Push the start node into maxheap
  q_.push(std::make_pair(distance_[start_ID_], start_ID_));

  // Run Dijkstra Unitil the Priority Queue is Empty or When you reach destination

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

        if ( visited_.count(cid) == 0) {

          visited_[cid] = true;

          auto neighbors = GetNeighborIDs(cid);
          for (auto neighbor : neighbors){

            double new_dist_ = cdist + CalculateDistance(cid, neighbor);

            if ( distance_.count(neighbor) == 0){
              distance_[neighbor] = INT_MAX;
            }

            if (new_dist_ < distance_[neighbor]){
              distance_[neighbor] = new_dist_;
              parent_[neighbor] = cid;
              q_.push(std::make_pair(distance_[neighbor], neighbor));
            }
          }
        }
      }
    } else if (cid == end_ID_) {
      break;
    }
  }
  path = {};
  path.push_back(end_ID_);
  auto id = parent_[end_ID_];
  while(id != start_ID_){
    path.push_back(id);
    id = parent_[id];
  }
  path.push_back(start_ID_);
  std::reverse(path.begin(), path.end());
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
  std::vector<std::string> path = {};

  if ((location1_name.empty()) || (location2_name.empty())){
    return path;
  }
  // Getting start and End Node Id's
  std::string start_ID_ = GetID(location1_name);
  std::string end_ID_ = GetID(location2_name);
  if ((start_ID_.empty()) || (end_ID_.empty())){
    return path;
  }
  // Shortest Distance of Each Node using Unordered Map
  std::unordered_map< std::string, double > distance_;

  // Shortest path to cirrent Node using unordered map
  std::unordered_map< std::string, std::string > parent_;

  // set a boolean stop sign to true
  bool stop_sign = true;

  // for (auto d : data){
  //   parent_[d.first] = "";
  // }

  distance_[start_ID_] = 0.0;
  
  int length = data.size();
  for ( int i = 1 ; i <= length - 1 ; i ++ ){
    // Set Stop sign to true while starting to iterate through new data map
    // If this is changed we will continue else it will break
    stop_sign = true;

    // iterating through data which has a distance
    for ( auto d : distance_ ) {

      auto neighbors = GetNeighborIDs(d.first);
      for( auto neighbor : neighbors ) {
        auto new_dist_ = distance_[d.first] + CalculateDistance(d.first, neighbor);

        if (distance_.count(neighbor) == 0){
          distance_[neighbor] = INT_MAX;
        }
        if (distance_[neighbor] > new_dist_){

          distance_[neighbor] = new_dist_;

          parent_[neighbor] = d.first;
          stop_sign = false;  
        } 
      }      
    } 
    if (stop_sign == true){
      break;
    }
  }
  
  path = {};
  path.push_back(end_ID_);
  auto id = parent_[end_ID_];
  while(id != start_ID_){
    path.push_back(id);
    id = parent_[id];
    
  }
  path.push_back(start_ID_);
  std::reverse(path.begin(), path.end());
  return path;  
}


/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(std::vector<std::string> location_ids) {
  auto start = std::chrono::high_resolution_clock::now();
  std::pair<double, std::vector<std::vector<std::string>>> records;
  // std::cout <<" Hello ";
  auto result = this->BruteForceHelper(location_ids);
  int best_path_loc = 0;
  double distance_ = INT_MAX;

  for(int i = 0; i < result.size(); i++){
    result[i].push_back(result[i][0]);
    auto path_distance = CalculatePathLength(result[i]);
    if (path_distance < distance_){
      distance_ = path_distance;
      best_path_loc = i;
    }
  }
  std::swap(result[best_path_loc], result[result.size() - 1]);
  records.first = distance_;
  records.second = result;
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  // std::cout << " COST = " << distance_ << " TIME = " << duration.count()/1000 << std::endl;
  return records;
}

std::vector<std::vector<std::string>> TrojanMap::BruteForceHelper(std::vector<std::string> location_ids){
  std::vector<std::vector<std::string>> result;

  if(location_ids.size() == 1){
    result.push_back(location_ids);
    return result;
  }

  for (int i = 0; i < location_ids.size(); i++){
    auto current = location_ids[i];
    std::vector<std::string> next_locations(location_ids.begin(), location_ids.end());
    // std::copy(location_ids.begin(), location_ids.end(), next_locations);
    next_locations.erase(next_locations.begin() + i);

    auto next_result = BruteForceHelper(next_locations);

    for (auto &e : next_result){
      e.insert(e.begin(), current);
      result.push_back(e);
    }
  }

  return result;
}


std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(std::vector<std::string> location_ids) {
  auto start = std::chrono::high_resolution_clock::now();
  std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::vector<std::string>> result;
  std::vector<std::string> curResult;
  double distance_ = INT_MAX;
  BacktrackingHelper(location_ids, result, curResult, distance_);
  int best_path_loc = 0;
  distance_ = INT_MAX;
  for(int i = 0; i < result.size(); i++){
    // result[i].push_back(result[i][0]);
    auto path_distance = CalculatePathLength(result[i]);
    if (path_distance < distance_){
      distance_ = path_distance;
      best_path_loc = i;
    }
  }

  std::swap(result[best_path_loc], result[result.size() - 1]);

  records.first = CalculatePathLength(result[result.size() - 1]);
  records.second = result;

  
  // std::cout << " " << " Distance = " << distance_ << std::endl;
  // for(auto &e : result[result.size() - 1]){
  //   std::cout << e <<" ";
  // }
  // std::cout << std::endl;

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  // std::cout << " COST = " << distance_ << " TIME = " << duration.count()/1000 << std::endl;
  return records;
}

void TrojanMap::BacktrackingHelper(std::vector<std::string> &location_ids, std::vector<std::vector<std::string>> &result, std::vector<std::string> &curResult, double distance_){

  if(curResult.size() == location_ids.size()){
    auto r1 = curResult;
    r1.push_back(r1[0]);
    auto currDist = CalculatePathLength(location_ids);
    if (currDist < distance_){
      distance_ = currDist;
      result.push_back(r1);
    }
    return;  
  }

  for (int i = 0; i < location_ids.size(); i++){
    if( std::find(curResult.begin(), curResult.end(), location_ids[i]) != curResult.end()){
      continue;
    }
    curResult.push_back(location_ids[i]);
    if (CalculatePathLength(curResult) < distance_){
      BacktrackingHelper(location_ids, result, curResult, distance_);
    }
    curResult.pop_back();

  }

}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(std::vector<std::string> location_ids){
  auto start = std::chrono::high_resolution_clock::now();
  std::pair<double, std::vector<std::vector<std::string>>> records;
  TravellingTrojan_2optHelper(location_ids, records);
  double best_distance = INT_MAX;
  int idx = 0;
  for(int i = 0; i< records.second.size(); i++){
    auto path = records.second[i];
    double distance = CalculatePathLength(path);
    if (distance < best_distance){
      best_distance = distance;
      idx = i;}
  }
  // for(auto n: records.second[idx]){
  //   std::cout<<n<<" ";
  // }
  std::swap(records.second[idx],records.second[records.second.size()-1]);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // std::cout << " COST = " << best_distance << " TIME = " << duration.count()/1000 << std::endl;
  return records;
}

void TrojanMap::TravellingTrojan_2optHelper(std::vector<std::string> location_ids,   std::pair<double, std::vector<std::vector<std::string>>> &records){
  location_ids.push_back(location_ids[0]);
  double best_distance = CalculatePathLength(location_ids);
  location_ids.pop_back();
  auto best_path = location_ids;
  for(int i = 1; i < location_ids.size(); i++){
    for(int j = i+1;j<location_ids.size(); j++){
      auto new_route = Swap2opt(location_ids,i,j);
      new_route.push_back(new_route[0]);
      double new_distance = CalculatePathLength(new_route);
      records.second.push_back(new_route);
      new_route.pop_back();
      if (new_distance < best_distance){
        best_distance = new_distance;
        best_path = new_route;
        records.first = new_distance;
        TravellingTrojan_2optHelper(best_path, records);
      }
    }
  }
}


std::vector<std::string> TrojanMap::Swap2opt(std::vector<std::string> location_ids, int s, int k){
  std::vector<std::string> sub;
  if(s > location_ids.size() || k > location_ids.size()){return sub;}
  int j = k;
  int i = 0;
  while(i < location_ids.size()){
    if (i < s){
      sub.push_back(location_ids[i]);
    }
    else if(i >= s && j >= s){
      sub.push_back(location_ids[j]);
      j = j - 1;
    }
    else{
      sub.push_back(location_ids[i]);
    }
    i+=1;
  }
  return sub;
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
   std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    location_names_from_csv.push_back(line);
    }
  fin.close();
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
  std::vector<std::string> temp;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);
    int count = 0;
    while (getline(s, word, ',')) {
      if (count == 0){
        temp.push_back(word);
        }
      else if (count == 1){
        temp.push_back(word);
        }
      count++;
      }
      dependencies_from_csv.push_back(temp);
      temp = {};
    }
    fin.close();
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
  std::unordered_map<std::string, std::vector<std::string>> adj;
  for (auto loc: locations){
    std::vector<std::string> temp;
    adj[loc] = temp;
  }
  for(auto dep: dependencies){
    adj[dep[0]].push_back(dep[1]);
  }
  bool cycle = TopoCycle(locations,adj);
  if(cycle){return {};}
  std::map<std::string, bool> visited;
  auto it = adj.begin();
  while(it != adj.end()){
    if(!visited[it->first]){
      TopoSortHelper(it->first,visited,adj,result);
    }
    it++;
  }
  std::reverse(result.begin(), result.end());
  return result;                                                     
}

/**
 * TopoSortHelper: It is a topological sorting recursive helper function. Modifes the topo_sort
 * vector based on DFS algorithm which in turn can be used to find the topological sort.
 
 *
 * @param  {std::vector<std::string>} locations                              : locations
 * @param  {std::map<std::string, bool>} visited                             : visited locations
 * @param  {std::unordered_map<std::string, std::vector<std::string>>} adj   : DAG
 * @param  {std::vector<std::string>} topo_list                              : DFS Topological vector
 */

void TrojanMap::TopoSortHelper(std::string location, std::map<std::string, bool> &visited, std::unordered_map<std::string, std::vector<std::string>> adj ,std::vector<std::string> &topo_list){
  visited[location] = true;
  for(auto child: adj[location]){
    if(!visited[child]){
      TopoSortHelper(child, visited, adj, topo_list);
    }   
  }
  topo_list.push_back(location);
}

/**
 * TopoCycle: A helper function for DeliveryTrojan function. Function is used to check if
 * There is a cycle from given Adjacency matrix. Returns TRUE if there exists a cycle and FALSE
 * if there is no cycle and a valid DAG.
 
 *
 * @param  {std::vector<std::string>} locations                              : locations
 * @param  {std::unordered_map<std::string, std::vector<std::string>>} adj   : DAG
 */

bool TrojanMap::TopoCycle(std::vector<std::string> locations,std::unordered_map<std::string, std::vector<std::string>> adj) {
  std::map<std::string, bool> visited;
  std::map<std::string, bool> backedge;
  for(auto n: locations){
    visited[n] = false;
    backedge[n] = false;
  }
  for(auto n: locations){
    if(!visited[n] && TopoCycleHelper(n, visited, adj, backedge)){
      return true;
    }
  }
  return false;
}

/**
 * TopoCycleHelper: A helper function for TopoCycle function which is called recursively and
 * used to find if there exsists a cycle from given Adjacency matrix. It performs DFS on the 
 * given graph data and a backedge vector is used to keep track of back edges. 
 
 *
 * @param  {std::string} current_id                                          : Current location
 * @param  {std::map<std::string, bool>} visited                             : visited locations
 * @param  {std::unordered_map<std::string, std::vector<std::string>>} adj   : DAG
 * @param  {std::map<std::string, bool>} backedge                            : Back Edge vector
 */

bool TrojanMap::TopoCycleHelper(std::string current_id, std::map<std::string, bool> &visited,std::unordered_map<std::string, std::vector<std::string>> adj,std::map<std::string, bool> &backedge) {
  if(!visited[current_id]){
    visited[current_id] = true;
    backedge[current_id] = true;
    auto neigh = adj[current_id];
    for(auto n: neigh){
      if(!visited[n] && TopoCycleHelper(n, visited, adj, backedge)){
        return true;
      }
      else if(backedge[n]){
        return true;
      }
    }
  }
  backedge[current_id] = false;
  return false;
}
  

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  double lat = GetLat(id);
  double lon = GetLon(id);
  if (lat <= square[2] && lat >= square[3] && lon <= square[1] && lon >= square[0]){
    return true;
  }
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
  //std::unordered_map<std::string, Node>::iterator it = data.begin();
  for (auto it = data.begin(); it != data.end(); it++){
    if (inSquare(it->first, square)){
      subgraph.push_back(it->first);
    }

  }
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
  std::map<std::string, bool> visited;
  //std::cout<<"Number of points in the subgraph: "<<subgraph.size()<<std::endl;
  std::unordered_map<std::string,std::string> predecessor;
  for(auto n: subgraph){
    visited[n] = false;
    //predecessor[n] = {};
  }
  std::string parent_id = "-1";
  bool result = hasCycle(subgraph[0],visited,parent_id, square, predecessor);
  std::vector<std::string> plot;
  if (result){
  auto it = predecessor.begin();
  while(it != predecessor.end()){
    //std::cout<< it->first<<" "<< it->second<<std::endl;
    plot.push_back(predecessor[it->second]);
    it++;
  } 
  // PlotPath(plot);
  }
  return result;
}

/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */

std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(33.9990000, -118.3210000);
  std::pair<double, double> upperRight(34.0410000, -118.2490000);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1280,
                                   (1 - (lat - bottomLeft.first) / h) * 900);
  return result;
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */

void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/map.png");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  // cv::resize(img, img, cv::Size(img.cols, img.rows));
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < int(location_ids.size()); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * hasCycle: A helper function which is called recursively and used to find if 
 * there exsists a cycle. It performs DFS on the given graph data and return true 
 * if there exists a cycle in the graph. 
 
 *
 * @param  {std::string} current_id                                          : Current location
 * @param  {std::map<std::string, bool>} visited                             : visited locations
 * @param  {std::string} parent_id                                           : Parent location to the current location
 * @param  {std::unordered_map<std::string,std::string>} predecessor         : predecessor location for the current location
 * @param  {sstd::vector<double>} square                                     : Square coordinates
 */

bool TrojanMap::hasCycle(std::string current_id, std::map<std::string, bool> &visited, std::string parent_id,std::vector<double> &square,std::unordered_map<std::string,std::string> &predecessor) {
  visited[current_id] = true;
  std::vector<std::string> neighb = GetNeighborIDs(current_id);
  for (auto n: neighb){
    predecessor[n] = current_id;
    if (!visited[n] && inSquare(current_id, square)){
      if (hasCycle(n, visited, current_id, square, predecessor)){
        return true;
      }
    }
    else if(visited[n] && n != parent_id && inSquare(current_id, square)){
      return true;
    }
  }
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
  std::vector<std::pair<double, std::string>> temp;
  std::pair<double,std::string> temp_pair;
  if(k == 0){return res;}

  for (auto it = data.begin(); it != data.end(); it++){
      auto id = it->first;
      auto ref_name = GetName(id);
      auto att = data[id].attributes;

      if(ref_name.size() != 0 && att.size() != 0){
        auto base_id = GetID(name);
        auto id_vec = {id,base_id};
        if(att.count(attributesName)){
          auto length = CalculatePathLength(id_vec);
          if(length <= r && id != base_id){
            temp_pair.first = length;
            temp_pair.second = id;
            temp.push_back(temp_pair);
          }
        }
      }
    }
  std::make_heap(temp.begin(),temp.end());
  std::sort_heap(temp.begin(),temp.end());
  int ctr = 0;
  int i = 0;
  while(ctr < k && i < temp.size()){;
    res.push_back(temp[i].second);
    ctr+=1;
    i+=1;
  }
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

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_3opt(std::vector<std::string> location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> records;
  TravellingTrojan_3optHelper(location_ids, records);
  std::cout<<"Helper working";
  double best_distance = INT_MAX;
  int idx = 0;
  for(int i = 0; i< records.second.size(); i++){
    auto path = records.second[i];
    double distance = CalculatePathLength(path);
    if (distance < best_distance){
      best_distance = distance;
      idx = i;}
  }
  std::swap(records.second[idx],records.second[records.second.size()-1]);
  return records;
}

void TrojanMap::TravellingTrojan_3optHelper(std::vector<std::string> location_ids,   std::pair<double, std::vector<std::vector<std::string>>> &records){
  location_ids.push_back(location_ids[0]);
  double best_distance = CalculatePathLength(location_ids);
  location_ids.pop_back();
  auto best_path = location_ids;
  for(int i = 1; i < location_ids.size(); i++){
    for(int j = i+1;j<location_ids.size(); j++){
      for(int k = j+1;k<location_ids.size(); k++){
          std::vector<int> vec = {i,j,k};
          do{              
            auto new_route = Swap3opt(location_ids,vec[0],vec[1]);
            new_route.push_back(new_route[0]);
            double new_distance = CalculatePathLength(new_route);
            records.second.push_back(new_route);
            new_route.pop_back();
            if (new_distance < best_distance){
              best_distance = new_distance;
              best_path = new_route;
              records.first = new_distance;
              TravellingTrojan_3optHelper(best_path, records);
            }
          }while(std::next_permutation(vec.begin(), vec.end()));
      }    
    }
  }
}

std::vector<std::string> TrojanMap::Swap3opt(std::vector<std::string> location_ids, int s, int k){
  std::vector<std::string> sub;
  std::vector<int> vec = {s,k};
  std::sort(vec.begin(),vec.end());
  s = vec[0];
  k = vec[1];
  if(s > location_ids.size() || k > location_ids.size()){return sub;}
  int j = k;
  int i = 0;
  while(i < location_ids.size()){
    if (i < s){
      sub.push_back(location_ids[i]);
    }
    else if(i >= s && j >= s){
      sub.push_back(location_ids[j]);
      j = j - 1;
    }
    else{
      sub.push_back(location_ids[i]);
    }
    i+=1;
  }
  return sub;
}