#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/Imu.h> // TODO Ros2
#include <cmath>
#include <iostream>

class JitterFilter : public rclcpp::Node
{
public:
    explicit JitterFilter()
    : Node("imu_jitter_filter_node"),
      last_stamp_{ 0, 0 },
      publisher_{ this->create_publisher<sensor_msgs::Imu>("imu/data_jitter_filtered", 10) }
    { }

    void filter(const sensor_msgs::Imu::ConstPtr &imu_ptr) {
        //const auto now = ros::Time::now();
        //const auto stamp = imu_ptr->header.stamp;

        sensor_msgs::Imu new_data = *imu_ptr;
        static sensor_msgs::Imu prev_data = *imu_ptr;

        const float difference = 0.1;
	//ROS_INFO("in filter");

	/*
        if (stamp <= last_stamp_ || stamp > now) {
	    ROS_INFO("stuff2");
            return;
        }
	*/

        bool lin_accel_ok = dataFilter(new_data.linear_acceleration, prev_data.linear_acceleration, difference);
        bool ang_vel_ok = dataFilter(new_data.angular_velocity, prev_data.angular_velocity, difference);

        //last_stamp_ = stamp;

        if (lin_accel_ok && ang_vel_ok)
	{
        	publisher_.publish(new_data);
	}
	else
	{
		std::cout << "Jitter Filter rejected incoming IMU message" << std::endl;
	}
	
        prev_data = new_data;
    }

private:
    rclcpp::Time last_stamp_;
    rclcpp::Publisher<sensor_msgs::Imu> publisher_;
    bool dataFilter(const auto &data, const auto &prev_data, const float difference)
    {
    	return abs(data.x - prev_data.x) >= difference &&
            abs(data.y - prev_data.y) >= difference &&
            abs(data.z - prev_data.z) >= difference
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JitterFilter>();
	ROS_INFO("Jitter Filter launched");

    const auto subscriber =
        node->create_subscription<sensor_msgs::Imu>("imu/data_filtered", 10,
                                         &JitterFilter::filter, &filter);

    rclcpp::spin(node);
}
