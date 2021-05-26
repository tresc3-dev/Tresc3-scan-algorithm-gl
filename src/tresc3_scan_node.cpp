#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include "gl_driver.h"

class Tresc3Scanner {
 public:
  Tresc3Scanner();
  ~Tresc3Scanner();

  void InitROS(void);

  void Run(void);
  geometry_msgs::Point32 CalPoint(double r, double theta);
  void SetColor(void);

 private:
  // ros
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{"~"};

  ros::Publisher pcl_pub;

  sensor_msgs::PointCloud pcl_msg;
  sensor_msgs::ChannelFloat32 channel;

  // launch variables
  std::string serial_port_name = std::string("/dev/ttyUSB0");
  std::string frame_id = std::string("pcl_data");
  std::string pub_topicname_pcl = std::string("pcl_data");
  std::string direction = std::string("up");
  double velocity = 0.0;
  double height = 0.0;

  // local variables
  double z;
  double step;
  int r = 255;
  int g = 0;
  int b = 0;
  unsigned int color = r * 65536 + g * 256 + b;
  float float_rgb = *reinterpret_cast<float*>(&color);

  // gl lidar
  Gl* gl;
};

Tresc3Scanner::Tresc3Scanner() {
  // ros init
  InitROS();

  // GL init
  gl = new Gl(serial_port_name, 921600);
  std::cout << "Serial Num : " << gl->GetSerialNum() << std::endl;
  gl->SetFrameDataEnable(true);
}

Tresc3Scanner::~Tresc3Scanner() { gl->SetFrameDataEnable(false); }

void Tresc3Scanner::InitROS(void) {
  nh_priv.param("serial_port_name", serial_port_name, serial_port_name);
  nh_priv.param("frame_id", frame_id, frame_id);
  nh_priv.param("pub_topicname_pcl", pub_topicname_pcl, pub_topicname_pcl);
  nh_priv.param("direction", direction, direction);
  nh_priv.param("velocity", velocity, velocity);
  nh_priv.param("height", height, height);

  // set initial value
  if (direction == "up") {
    step = velocity * -0.05;
  } else if (direction == "down") {
    step = velocity * 0.05;
  } else {
    ROS_INFO("param 'direction' must be up or down!!");
    ros::shutdown();
  }

  z = height;

  pcl_pub = nh.advertise<sensor_msgs::PointCloud>(pub_topicname_pcl, 10);
  pcl_msg.header.frame_id = frame_id;
  channel.name = "rgb";
}

void Tresc3Scanner::Run(void) {
  Gl::framedata_t frame_data;
  gl->ReadFrameData(frame_data);

  int num_data = frame_data.distance.size();
  if (num_data > 0) {
    pcl_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < num_data; i++) {
      geometry_msgs::Point32 p =
          CalPoint(frame_data.distance[i], frame_data.angle[i]);
      pcl_msg.points.push_back(p);
      channel.values.push_back(float_rgb);
    }
    pcl_msg.channels.clear();
    pcl_msg.channels.push_back(channel);
    pcl_pub.publish(pcl_msg);
  }
  SetColor();
  z += step;
}

geometry_msgs::Point32 Tresc3Scanner::CalPoint(double r, double theta) {
  geometry_msgs::Point32 p;
  p.x = r * cos(theta);
  p.y = r * sin(theta);
  p.z = z;

  return p;
}

void Tresc3Scanner::SetColor(void) {
  if (r > 0 && b == 0) {
    r--;
    g++;
  } else if (g > 0 && r == 0) {
    g--;
    b++;
  } else if (b > 0 && g == 0) {
    r++;
    b--;
  }
  color = r * 65536 + g * 256 + b;
  float_rgb = *reinterpret_cast<float*>(&color);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Tresc3_scanner");

  Tresc3Scanner scanner;

  // loop
  ros::Rate loop_rate(20);
  while (ros::ok()) {
    scanner.Run();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}