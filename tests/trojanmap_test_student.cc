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
  EXPECT_EQ(tm.Autocomplete(sub),results);
  sub = "SAIN ";
  EXPECT_EQ(tm.Autocomplete(sub),results);
  sub = " SAIN ";
  EXPECT_EQ(tm.Autocomplete(sub),results);
  sub = "";
  EXPECT_EQ(tm.Autocomplete(sub),results);
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
  int result = tm.CalculateEditDistance("Horse", "Ros");
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