#include <ros/ros.h>
#include <std_msgs/String.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// For Serial Communication with arduino board
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <iostream>
#include <iostream>

#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>

#include <string>

using namespace std;

class ImageConverter 
{
    std_msgs::String d;
    ros::Subscriber sub;
    ros::Publisher pub;

public:
    ImageConverter(ros::NodeHandle nh, const char *usbPort);

    ~ImageConverter() {}

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
    
    int fd,ret;
    char red[88];
};

ImageConverter::ImageConverter(ros::NodeHandle nh, const char *usbPort)
{
    // Subscribe to input video feed and publish output video feed
    sub = nh.subscribe("/point_cloud2", 1,
                               &ImageConverter::cloud_cb, this);
    pub = nh.advertise<std_msgs::String>("robot_movement", 1000);

    std::cout<<"class initialized"<<std::endl;
    fd = open(usbPort, O_RDWR | O_NOCTTY | O_NDELAY);
    struct termios SerialPortSettings;  // Create the structure
    tcgetattr(fd, &SerialPortSettings);
    cfsetispeed(&SerialPortSettings,B115200); // Set Read  Speed as 115200
    cfsetospeed(&SerialPortSettings,B115200);
    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
      printf("Error while setting attributes \n");
      printf("\nfunction name is  %s >>\n",__func__);
      printf("reading from Serial port== %c >>\n",red);
      printf("returned fd is :%d\n",fd );
      if (fd == -1)
      {
        perror("open_port: Unable to open /dev/ttyACM0 - ");
      }
}

void Write(int fd,std::string data,std::size_t del_time) {
  for(int i=0;i<data.length();i++) {
      int ret=write(fd,&data[i],1);
  }
  std::this_thread::sleep_for (std::chrono::microseconds (del_time));
}

void ImageConverter::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);
	std::string data = "C";
	for(auto pnt : cloud) {
		if(pnt.z <= 0.40 && pnt.z > 0) {
			data = "S";
		}
		else if(pnt.z > 0.40 && pnt.z <0.60) {
			data = "B";
		}
	}
	Write(fd,data,0);
	d.data = data;
	pub.publish(d);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "deye");
    ros::NodeHandle nh;
    
    const char *usbPort = "/dev/ttyACM0";
    ImageConverter ic(nh, usbPort);
    ros::spin();
    return 0;
}



