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
  name = "Starbucks";
  position.first = 34.0390615;
  position.second = -118.2611170;
  res = tm.GetPosition(name);
  EXPECT_EQ(position, res);

}
