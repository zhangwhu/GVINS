#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <gnss_ros.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <sstream>

using namespace gnss_comm;

// GPS is now ahead of UTC by 18 seconds
#define GPS_LEAP_SECOND 18

/* str：IMU传感器采集坐标系 
   目标系：   前-右-下
   目标单位： rad、m/s^2 */
/* imu数据转bag，bag包以IMU数据为基础 */
void imu2bag(rosbag::Bag &bag, const std::string imuFile, const std::string outBag, int gpsWeek, 
    std::string LorR, std::string frameID, const gtime_t &ts, const gtime_t &te, const char *str)
{
    gtime_t ti;
    std::ifstream file(imuFile);
    if (!file.is_open())
    {
        ROS_ERROR_STREAM("Failed to open file!");
        exit(1);
    }

    bag.open(outBag, rosbag::bagmode::Write);

    std::string line;
    std::getline(file, line);   // 跳过第一行

    while (std::getline(file, line))
    {
        // 将每行数据分割为各个字段
        std::istringstream iss(line);
        double time, gyro[3], accel[3];
        if (!(iss >> time >> gyro[0] >> gyro[1] >> gyro[2] >> accel[0] >> accel[1] >> accel[2]))
        {
            ROS_WARN_STREAM("Failed to parse line: " << line);
            continue;
        }
        ti = gpst2time(gpsWeek, time);
        if (timediff(ti, ts) < -0.001 || timediff(ti, te) > 0.001) continue;

        // 创建IMU消息
        sensor_msgs::Imu imu_msg;
        // 315964800是GPS起始时间和计算机起始时间的一个固定差值
        time = time + 315964800 + 604800 * gpsWeek - 8 * 3600;
        imu_msg.header.stamp = ros::Time(time);
        imu_msg.header.frame_id = frameID;
        for (int i = 0; i < 3; i++) {
            switch (str[i]) {
                case 'F': imu_msg.angular_velocity.x    =  gyro[i];
                          imu_msg.linear_acceleration.x =  accel[i]; break;
                case 'B': imu_msg.angular_velocity.x    = -gyro[i];
                          imu_msg.linear_acceleration.x = -accel[i]; break;
                case 'R': imu_msg.angular_velocity.y    =  gyro[i];
                          imu_msg.linear_acceleration.y =  accel[i]; break;
                case 'L': imu_msg.angular_velocity.y    = -gyro[i];
                          imu_msg.linear_acceleration.y = -accel[i]; break;
                case 'D': imu_msg.angular_velocity.z    =  gyro[i]; 
                          imu_msg.linear_acceleration.z =  accel[i]; break;
                case 'U': imu_msg.angular_velocity.z    = -gyro[i]; 
                          imu_msg.linear_acceleration.z = -accel[i]; break;
            }
        }
        /* 转弧度 */
        imu_msg.angular_velocity.x *= (PI/180.0);   
        imu_msg.angular_velocity.y *= (PI/180.0);
        imu_msg.angular_velocity.z *= (PI/180.0);

        // 写入ROSbag文件
        bag.write(LorR, ros::Time(time), imu_msg);
    }

    bag.close();
    file.close();
    std::cout << "imu data convert finished!" << std::endl;
}

/* image转bag */
void image2bag(rosbag::Bag &bag, const std::string &strPathToImage, const std::string outBag, int gpsWeek, 
    std::string LorR, std::string frameID, const gtime_t &ts, const gtime_t &te)
{
    gtime_t ti;
    std::ifstream fTime;
    std::string strPathToTime = strPathToImage + "/timestamps.txt";
    fTime.open(strPathToTime);
    if (!fTime.is_open())
    {
        ROS_ERROR_STREAM("Failed to open timestamp file!");
        exit(1);
    }

    // 保存时间戳
    std::vector<double> vTimeStamp;
    while(!fTime.eof())
    {
        std::string s;
        getline(fTime, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeStamp.push_back(t);
        }
    }
    fTime.close();

    // 开始转换图片
    bag.open(outBag, rosbag::bagmode::Append);
    double time;
    cv::Mat image;

    for (int i = 0; i < vTimeStamp.size(); i++)
    {
        ti = gpst2time(gpsWeek, vTimeStamp[i]);
        if (timediff(ti, ts) < -0.001 || timediff(ti, te) > 0.001) continue;
        
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(10) << i;
        std::string imageName = strPathToImage + "/data/" + ss.str() + ".png"; 
        image = cv::imread(imageName, cv::IMREAD_GRAYSCALE);
        time = vTimeStamp[i] + 315964800 + 604800 * gpsWeek - 8 * 3600;

        sensor_msgs::ImagePtr rosImg;
        cv_bridge::CvImage rosImage;
        rosImage.encoding = "mono8";
        rosImage.image = image;

        rosImg = rosImage.toImageMsg();   // cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
        rosImg->header.stamp = ros::Time(time);
        rosImg->header.frame_id = frameID;
        // char id[32];
        // time2str(ti, id, 0);
        // printf("%s %f\n", id, time);
        bag.write(LorR, ros::Time(time), rosImg);
        std::cout << "done: " << i << "/" << vTimeStamp.size() << std::endl; 
    }
    bag.close();
    std::cout << "image convert finished!" << std::endl;
}

/* obs转bag */
void obs2bag(rosbag::Bag &bag, const std::vector<std::vector<obsd_t>> &rinex_meas, const std::string outBag, 
    std::string LorR, std::string frameID) 
{
    int week, i = 0;
    double tow, time;
    GnssMeasMsg rosObs;

    bag.open(outBag, rosbag::bagmode::Append);
    for (auto meas : rinex_meas) {
        double tow = time2gpst(meas[0].time, &week);
        time = tow + 315964800 + 604800 * week - 8 * 3600;

        rosObs = meas2msg(meas);
        rosObs.header.stamp = ros::Time(time);
        rosObs.header.frame_id = frameID;
        bag.write(LorR, ros::Time(time), rosObs);
        std::cout << "done: " << ++i << "/" << rinex_meas.size() << "\t" << meas.size() << std::endl; 
    }
    bag.close();
    std::cout << "GNSS convert finished!" << std::endl;
}

int main(int argc, char **argv)
{
    gtime_t ts, te;
    char t1[32] = "2023 04 16 09 53 45";
    char t2[32] = "2023 04 16 10 08 10";

    str2time(t1, 0, 32, &ts);
    str2time(t2, 0, 32, &te);

    ros::init(argc, argv, "data_to_rosbag");
    if (argc != 7) {
        ROS_ERROR_STREAM("Usage: data2bag gps_week imu_file left_image_path right_image_path out_bag");
        exit(1);
    }
    ros::NodeHandle nh;
    
    // 创建rosbag文件
    rosbag::Bag bag;
    int gpsWeek = std::atoi(argv[1]);
    std::string imuFile = argv[2];
    std::string LstrPathToImage = argv[3];
    std::string RstrPathToImage = argv[4];
    std::string rinexfile = argv[5];
    std::string outBag = argv[6];
    
    // imu转bag
    imu2bag(bag, imuFile, outBag, gpsWeek, "/gvi/imu0", "imu0", ts, te, "RFU");     // IMU的类型较多，要根据实际进行修改

    // image转bag
    image2bag(bag, LstrPathToImage, outBag, gpsWeek, "/gvi/image0", "cam0", ts, te);
    // image2bag(bag, RstrPathToImage, outBag, gpsWeek, "/gvi/cam1", "cam1");

    // rinex转bag
    prcopt_t prcopt = prcopt_default;
    std::vector<std::vector<obsd_t>> rinex_meas;    

    strcpy(prcopt.rnxopt[0],"-SYS=GEC");  
    prcopt.ts = ts;     prcopt.te = te;     prcopt.ti = 1;            
    rinex2obs(rinexfile, prcopt, rinex_meas);
    obs2bag(bag, rinex_meas, outBag, "/gvi/gnss0", "gnss0");          
    return 0;
}