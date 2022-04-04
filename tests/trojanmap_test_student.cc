#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, GetLat) {
  TrojanMap tm;
  std::string id ="653725";
  double lat = 34.0360852;
  double res = tm.GetLat(id);
  EXPECT_EQ(lat, res);
}

