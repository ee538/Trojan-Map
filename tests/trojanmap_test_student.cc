#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, GetLat) {
  TrojanMap tm;
  std::string id;
  double lat, res;
  id ="653725";
  lat = 34.0360852;
  res = tm.GetLat(id);
  EXPECT_EQ(lat, res);
  id ="3699276746";
  lat = 34.0111333;
  res = tm.GetLat(id);
  EXPECT_EQ(lat, res);
  id ="3396325727";
  lat = 34.0234721;
  res = tm.GetLat(id);
  EXPECT_EQ(lat, res);
}

TEST(TrojanMapStudentTest, GetLon) {
  TrojanMap tm;
  std::string id;
  double lon, res;
  id ="653725";
  lon = -118.3212048;
  res = tm.GetLon(id);
  EXPECT_EQ(lon, res);
  id ="3699276746";
  lon = -118.2957046;
  res = tm.GetLon(id);
  EXPECT_EQ(lon, res);
  id ="3396325727";
  lon = -118.2743162;
  res = tm.GetLon(id);
  EXPECT_EQ(lon, res);
}

TEST(TrojanMapStudentTest, GetName) {
  TrojanMap tm;
  std::string id;
  std::string name;
  std::string res;
  id = "3646948889";
  name = "St. Francis Center - Food Bank";
  res = tm.GetName(id);
  EXPECT_EQ(name, res);
  id = "3854179745";
  name = "Chevron 1";
  res = tm.GetName(id);
  EXPECT_EQ(name, res);
  id = "3810143804";
  name = "Starbucks";
  res = tm.GetName(id);
  EXPECT_EQ(name, res);
}

TEST(TrojanMapStudentTest, GetID) {
  TrojanMap tm;
  std::string id;
  std::string name;
  std::string res;
  id = "3646948889";
  name = "St. Francis Center - Food Bank";
  res = tm.GetID(name);
  EXPECT_EQ(id, res);
  id = "3854179745";
  name = "Chevron 1";
  res = tm.GetID(name);
  EXPECT_EQ(id, res);
  id = "3810143804";
  name = "Starbucks";
  res = tm.GetID(name);
  EXPECT_EQ(id, res);
}

TEST(TrojanMapStudentTest, GetNeighborIDs){
  TrojanMap tm;
  std::string id;
  std::vector<std::string> nIDs = {};
  std::vector<std::string> res = {};
  id = "3646948889";
  nIDs = {"269987071"};
  res = tm.GetNeighborIDs(id);
  EXPECT_EQ(nIDs, res);
  id = "3854179745";
  nIDs = {"7298792416"};
  res = tm.GetNeighborIDs(id);
  EXPECT_EQ(nIDs, res);
  id = "3810143804";
  nIDs = {"4837730315"};
  res = tm.GetNeighborIDs(id);
  EXPECT_EQ(nIDs, res);
}

TEST(TrojanMapStudentTest, GetPosition){
  TrojanMap tm;
  std::string name;
  std::pair<double, double> position;
  name = "St. Francis Center - Food Bank";
  position.first = 34.0339159;
  position.second = -118.2695622;
  auto res = tm.GetPosition(name);
  EXPECT_EQ(position, res);
  name = "Chevron 1";
  position.first = 34.0336409;
  position.second = -118.2620798;
  res = tm.GetPosition(name);
  EXPECT_EQ(position, res);
  name = "Chevr";
  position.first = -1;
  position.second = -1;
  res = tm.GetPosition(name);
  EXPECT_EQ(position, res);
  name = "Starbucks";
  position.first = 34.0390615;
  position.second = -118.2611170;
  res = tm.GetPosition(name);
  EXPECT_EQ(position, res);
  name = "starbucks";
  position.first = -1;
  position.second = -1;
  res = tm.GetPosition(name);
  EXPECT_EQ(position, res);
}

TEST(TrojanMapStudentTest, Autocomplete) {
  TrojanMap tm;
  std::string sub = "chi";
  std::vector<std::string> results = {"Chick-fil-A", "Chinese Street Food","Chipotle"};
  EXPECT_EQ(tm.Autocomplete(sub),results);
  sub = "CHI";
  EXPECT_EQ(tm.Autocomplete(sub),results);
  sub = "ChI";
  EXPECT_EQ(tm.Autocomplete(sub),results);
  sub = "ch";
  results = { "Chase", "Chase Plaza Heliport", "Cheebos Burger", "Chevron", "Chevron 1", "Chevron 2", "Chick-fil-A", "Chinese Street Food", "Chipotle", "Chucks Chicken & Waffles", "Church of Christ" };
  EXPECT_EQ(tm.Autocomplete(sub),results);
  sub = "CH";
  EXPECT_EQ(tm.Autocomplete(sub),results);  
  sub = "saint";
  results = {"Saint Agnes Elementary School","Saint Cecilia School","Saint James Park","Saint John Baptist Church","Saint Lukes Missionary Baptist Church","Saint Marks Lutheran Church","Saint Patrick School","Saint Patricks Catholic Church","Saint Phillips Episcopal Church"};
  EXPECT_EQ(tm.Autocomplete(sub),results);
  sub = "SAIN";
  // EXPECT_EQ(tm.Autocomplete(sub),results);
  // sub = "SAIN ";
  // EXPECT_EQ(tm.Autocomplete(sub),results);
  // sub = " SAIN ";
  // EXPECT_EQ(tm.Autocomplete(sub),results);
  // sub = "";
  // EXPECT_EQ(tm.Autocomplete(sub),results);
}

TEST(TrojanMapTest, CalculateEditDistance) {
  TrojanMap tm;
  int distance = 6;
  int result = tm.CalculateEditDistance("Saint Agnes Elementary School", "Saint Elementary School");
  EXPECT_EQ(distance, result);
  distance = 0;
  result = tm.CalculateEditDistance("", "");
  EXPECT_EQ(distance, result);
  distance = 3;
  result = tm.CalculateEditDistance("Horse", "Ros");
  EXPECT_EQ(distance, result);
  distance = 5;
  result = tm.CalculateEditDistance("Horse", "");
  EXPECT_EQ(distance, result);
  distance = 3;
  result = tm.CalculateEditDistance("", "ROS");
  EXPECT_EQ(distance, result);
  distance = 2;
  result = tm.CalculateEditDistance("Targeety", "Target");
  EXPECT_EQ(distance, result);
  distance = 2;
  result = tm.CalculateEditDistance("Trider Joe", "Trader Joes");
  EXPECT_EQ(distance, result);
  distance = 2;
  result = tm.CalculateEditDistance("Western & 36th","Western & 36th 1");
  EXPECT_EQ(distance, result);
  distance = 6;
  result = tm.CalculateEditDistance("Which wich?","Which wich???????");
  EXPECT_EQ(distance, result);

}

TEST(TrojanMapTest, FindClosestName) {
  TrojanMap m;
  std::string name, result;
  name = "Ralphs";
  result = m.FindClosestName("Rolphs");
  EXPECT_EQ(name, result);
  name = "Trader Joes";
  result = m.FindClosestName("Trider Joe");
  EXPECT_EQ(name, result);
  name = "Target";
  result = m.FindClosestName("Targeety");
  EXPECT_EQ(name, result);
  name = "Ralphs";
  result = m.FindClosestName("ROLPHS");
  EXPECT_EQ(name, result);
  name = "Trader Joes";
  result = m.FindClosestName("TRADER JOE");
  EXPECT_EQ(name, result);
  name = "";
  result = m.FindClosestName("");
  EXPECT_EQ(name, result);
}

TEST(TrojanMap, Dijkstra){
  TrojanMap m;
  std::string start_name = "Ralphs";
  std::string end_name = "Target";
  std::vector< std::string > result = m.CalculateShortestPath_Dijkstra(start_name, end_name);
  std::vector< std::string > path = {"2578244375","4380040154","4380040158","4380040167","6805802087",
                                     "8410938469","6813416131","7645318201","6813416130","6813416129",
                                     "123318563","452688940","6816193777","123408705","6816193774",
                                     "452688933","452688931","123230412","6816193770","6787470576",
                                     "4015442011","6816193692","6816193693","6816193694","4015377691",
                                     "544693739","6816193696","6804883323","6807937309","6807937306",
                                     "6816193698","4015377690","4015377689","122814447","6813416159",
                                     "6813405266","4015372488","4015372487","6813405229","122719216",
                                     "6813405232","4015372486","7071032399","4015372485","6813379479",
                                     "6813379584","6814769289","5237417650"};
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  result = m.CalculateShortestPath_Dijkstra(end_name, start_name);
  std::reverse(path.begin(), path.end());
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  start_name = "Ralphs";
  end_name = "Chick-fil-A";
  result = m.CalculateShortestPath_Dijkstra(start_name, end_name);
  path = {"2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919",
          "6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145",
          "6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785",
          "6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809",
          "4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483",
          "3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391",
          "123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015",
          "1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556",
          "6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107",
          "2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516",
          "6814916515","6820935910","4547476733"};
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  result = m.CalculateShortestPath_Dijkstra(end_name, start_name);
  std::reverse(path.begin(), path.end());
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());

  start_name = "Washington & Broadway 1";
  end_name = "Pico & Maple";
  result = m.CalculateShortestPath_Dijkstra(start_name, end_name);
  path = {"9596558262","4060083400","21098546","1866577831","1738419621","72092084",
          "2611833620","6805256739","1873055923","6818460838","6818460837","6818460836",
          "8164416107","122840581","250745156","1614922726","1918477977","122986158",
          "122607557","1837231044","122868247","3010199904","2611809734","4291117324"};
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  result = m.CalculateShortestPath_Dijkstra(end_name, start_name);
  std::reverse(path.begin(), path.end());
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());

  start_name = "Western & Washington";
  end_name = "Adams & Hoover";
  result = m.CalculateShortestPath_Dijkstra(start_name, end_name);
  path = {"8189262087","8382458681","4009709283","6819144993","1771004838","4009709281","4009709279",
          "4037559775","4009673255","122532758","9035099537","4009673254","660937747","660937746",
          "122537361","1614656116","1614656215","1614656221","5345018537","5345018536","5345018534",
          "604242638","452688855","452688846","123161895","2613120766","123161891","122935678","1771004849",
          "123161885","123561991","6819212758","8410938475","8410938474","8410938473","8410938476","8410938479",
          "8410938477","122740210","6914795320","6799110594","122740207","122817133","123183657","6816193765",
          "6807583301","123183652","7477442524","1922565220","1841016364","123408705","6816193774","452688933",
          "452688931","123230412","6816193770","6787470576","4015442011","6816193692","123408740","6816193688",
          "4015442010","4015442009","9561513950"};
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  result = m.CalculateShortestPath_Dijkstra(end_name, start_name);
  std::reverse(path.begin(), path.end());
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
}

TEST(TrojanMap, Bellman_Ford){
  TrojanMap m;
  std::string start_name = "Ralphs";
  std::string end_name = "Target";
  std::vector< std::string > result = m.CalculateShortestPath_Bellman_Ford(start_name, end_name);
  std::vector< std::string > path = {"2578244375","4380040154","4380040158","4380040167","6805802087",
                                     "8410938469","6813416131","7645318201","6813416130","6813416129",
                                     "123318563","452688940","6816193777","123408705","6816193774",
                                     "452688933","452688931","123230412","6816193770","6787470576",
                                     "4015442011","6816193692","6816193693","6816193694","4015377691",
                                     "544693739","6816193696","6804883323","6807937309","6807937306",
                                     "6816193698","4015377690","4015377689","122814447","6813416159",
                                     "6813405266","4015372488","4015372487","6813405229","122719216",
                                     "6813405232","4015372486","7071032399","4015372485","6813379479",
                                     "6813379584","6814769289","5237417650"};
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  result = m.CalculateShortestPath_Bellman_Ford(end_name, start_name);
  std::reverse(path.begin(), path.end());
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  start_name = "Ralphs";
  end_name = "Chick-fil-A";
  result = m.CalculateShortestPath_Bellman_Ford(start_name, end_name);
  path = {"2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919",
          "6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145",
          "6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785",
          "6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809",
          "4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483",
          "3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391",
          "123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015",
          "1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556",
          "6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107",
          "2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516",
          "6814916515","6820935910","4547476733"};
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  result = m.CalculateShortestPath_Bellman_Ford(end_name, start_name);
  std::reverse(path.begin(), path.end());
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());

  start_name = "Washington & Broadway 1";
  end_name = "Pico & Maple";
  result = m.CalculateShortestPath_Bellman_Ford(start_name, end_name);
  path = {"9596558262","4060083400","21098546","1866577831","1738419621","72092084",
          "2611833620","6805256739","1873055923","6818460838","6818460837","6818460836",
          "8164416107","122840581","250745156","1614922726","1918477977","122986158",
          "122607557","1837231044","122868247","3010199904","2611809734","4291117324"};
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  result = m.CalculateShortestPath_Bellman_Ford(end_name, start_name);
  std::reverse(path.begin(), path.end());
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());

  start_name = "Western & Washington";
  end_name = "Adams & Hoover";
  result = m.CalculateShortestPath_Bellman_Ford(start_name, end_name);
  path = {"8189262087","8382458681","4009709283","6819144993","1771004838","4009709281","4009709279",
          "4037559775","4009673255","122532758","9035099537","4009673254","660937747","660937746",
          "122537361","1614656116","1614656215","1614656221","5345018537","5345018536","5345018534",
          "604242638","452688855","452688846","123161895","2613120766","123161891","122935678","1771004849",
          "123161885","123561991","6819212758","8410938475","8410938474","8410938473","8410938476","8410938479",
          "8410938477","122740210","6914795320","6799110594","122740207","122817133","123183657","6816193765",
          "6807583301","123183652","7477442524","1922565220","1841016364","123408705","6816193774","452688933",
          "452688931","123230412","6816193770","6787470576","4015442011","6816193692","123408740","6816193688",
          "4015442010","4015442009","9561513950"};
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
  result = m.CalculateShortestPath_Bellman_Ford(end_name, start_name);
  std::reverse(path.begin(), path.end());
  EXPECT_EQ(path, result);
  EXPECT_EQ(m.CalculatePathLength(path), m.CalculatePathLength(result));
  EXPECT_EQ(path.size(), result.size());
}


TEST(TrojanMap, CycleDet){
  TrojanMap m;
  std::vector<double> square = {-118.275, -118.274, 34.015, 34.014};
  std::vector<std::string> subgraph = m.GetSubgraph(square);
  EXPECT_EQ(m.CycleDetection(subgraph, square), false);

  square = {-118.264, -118.260, 34.014, 34.010};
  subgraph = m.GetSubgraph(square);
  EXPECT_EQ(m.CycleDetection(subgraph, square), true);

  square = {-118.312, -118.311, 34.004, 34.003};
  subgraph = m.GetSubgraph(square);
  EXPECT_EQ(m.CycleDetection(subgraph, square), false);

  square = {-118.300, -118.299, 34.002, 34.001};
  subgraph = m.GetSubgraph(square);
  EXPECT_EQ(m.CycleDetection(subgraph, square), false);

  square = {-118.300, -118.299, 34.002, 34.000};
  subgraph = m.GetSubgraph(square);
  EXPECT_EQ(m.CycleDetection(subgraph, square), false);

  square = {-118.290, -118.289, 34.010, 34.004};
  subgraph = m.GetSubgraph(square);
  EXPECT_EQ(m.CycleDetection(subgraph, square), false);
}

TEST(TrojanMap, DeliveryTrojan){
  TrojanMap m;
  std::vector<std::vector<std::string>> dep = {{"Ralphs","Chick-fil-A"},{"Ralphs","KFC"},{"Chick-fil-A","KFC"}};
  std::vector<std::string> loc = {"Ralphs", "KFC", "Chick-fil-A"};
  std::vector<std::string> res = {"Ralphs", "Chick-fil-A", "KFC"};
  EXPECT_EQ(m.DeliveringTrojan(loc, dep), res);

  dep = {{"Chick-fil-A","Ralphs"},{"Ralphs","KFC"},{"Chick-fil-A","KFC"}};
  res = {"Chick-fil-A", "Ralphs", "KFC"};
  EXPECT_EQ(m.DeliveringTrojan(loc, dep), res);

  dep = {{"Chick-fil-A","Ralphs"}};
  res = {"KFC","Chick-fil-A", "Ralphs"};
  EXPECT_EQ(m.DeliveringTrojan(loc, dep), res);

  dep = {{"Chick-fil-A","Ralphs"},{"Chick-fil-A","KFC"}};
  res = {"Chick-fil-A", "KFC","Ralphs"};
  EXPECT_EQ(m.DeliveringTrojan(loc, dep), res);

  dep = {{"Chick-fil-A","Ralphs"},{"Ralphs","Chick-fil-A"}};
  res = {};
  EXPECT_EQ(m.DeliveringTrojan(loc, dep), res);

  std::vector<std::vector<std::string>> empty;
  loc = {"Ralphs"};
  res = {"Ralphs"};
  EXPECT_EQ(m.DeliveringTrojan(loc, empty), res);
}