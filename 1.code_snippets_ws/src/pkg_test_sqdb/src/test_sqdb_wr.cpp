#include <ros/ros.h>
#include <boost/thread.hpp>
#include <common/sqdb.h>

int main()
{
  //写

  sqdb::Db my_db("/home/mantou/core.db");
  std::string query_str;
  std::string room("living room");
  std::string elec("light");
  query_str = "select id, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w, np_x, np_y, np_z, no_x,no_y,no_z,no_w from thing where room_name=\""
      + room + "\" and name=\"" + elec + "\"; ";
  sqdb::Statement s = my_db.Query(query_str.c_str());
  if(s.Next())
    ROS_INFO("%.2f", s.GetField(1).GetDouble());

  //插入
//    sqdb::Statement i= my_db.Query ("insert into thing(room_name, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w, name, area, np_x, np_y, np_z, no_x,no_y,no_z,no_w) values (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?) ");
//    std::string str("666");
//    double real = 666;
//    i.Bind(1, str);
//    i.Bind(9, str);
//    i.Next();

  //替换更新
  char buffer[256];

  sprintf(buffer,"UPDATE thing SET position_x=\"%.2f\" WHERE id=\"%d\" AND room_name=\"%s\"", 6.0, 1, "living room");
  ROS_INFO(buffer);
  s = my_db.Query (buffer);
  s.Next ();

  return 0;
}
