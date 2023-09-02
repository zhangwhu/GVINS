
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <gnss_ros.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <opencv2/highgui.hpp>
using cv::Mat;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

static void unix2gps(double unixs, int &week, double &sow) {
        double seconds = unixs + 8 * 3600 - 315964800;

        week = floor(seconds / 604800);
        sow  = seconds - week * 604800;
};


void imuCallback(const sensor_msgs::ImuConstPtr &imumsg) {
    double unixsecond = imumsg->header.stamp.toSec();
    double weeksec;
    int week;
    unix2gps(unixsecond, week, weeksec);
    printf("%4d %8.2f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f\n", week, weeksec, 
        imumsg->angular_velocity.x*(180/PI), imumsg->angular_velocity.y*(180/PI), 
        imumsg->angular_velocity.z*(180/PI),
        imumsg->linear_acceleration.x, imumsg->linear_acceleration.y, imumsg->linear_acceleration.z);
}
void gnssCallback(const gnss_comm::GnssMeasMsgConstPtr &gnssmsg) {
    char id[32];
    gtime_t tt;
    tt = gpst2time(gnssmsg->meas[0].time.week, gnssmsg->meas[0].time.tow);
    time2str(tt, id, 0);

    double unixsecond = gnssmsg->header.stamp.toSec();
    double weeksec;
    int week;
    unix2gps(unixsecond, week, weeksec);
    printf("id %4d %8.2f\n", id, week, weeksec);

    obs_t obs = msg2meas(gnssmsg);

    // for (auto meas : gnssmsg->meas) {
    //     satno2id(meas.sat, id);
    //     // printf("%s %15.4f %15.4f %15.4f\n", id, meas.psr[0], meas.psr[1], meas.psr[2]);
    //     // printf("%s %10.4f %10.4f %10.4f\n", id, meas.dopp[0], meas.dopp[1], meas.dopp[2]);
    //     // printf("%s %5d %5d %5d\n", id, meas.LLI[0], meas.LLI[1], meas.LLI[2]);
    //     // printf("%s %5d %5d %5d\n", id, meas.SNR[0], meas.SNR[1], meas.SNR[2]);
    // }
}
void imageCallback(const sensor_msgs::ImageConstPtr &imagemsg) {
    double unixsecond = imagemsg->header.stamp.toSec();
    double weeksec;
    int week;
    unix2gps(unixsecond, week, weeksec);
    printf("image %4d %8.2f\n", week, weeksec);

    Mat image;
    // Copy image data
    // if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
    //     image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
    //     memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    // } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
    //     image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
    //     memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
    // }
}

struct mdd {
    Vector3d d1;
};

#include <deque>
int main(int argc, char *argv[]) {
    // CGNSSConfig *config = new CGNSSConfig();
    // config->LoadYAML("/media/zhuhang/D/Data/PPPRTK_INS_VISUAL_Wuh-230416/gvins.yaml");
    // CGNSSManage *mm = new CGNSSManage();
    // mm.Init(config.prcopt, config.solopt, config.filopt, config.sta);
    // Quaterniond q_b_c(0.497766, 0.502679, 0.501396, 0.498141);

    // std::vector<double> mcc;
    // mcc.push_back(6);   mcc.push_back(7);   mcc.push_back(8);   mcc.push_back(9);
    // Eigen::MatrixXd noise_;
    // Eigen::Vector3d ee(6,7,8);
    // noise_.setIdentity(9, 9);
    // auto ff = Vector3d(ee.array().pow(2)).asDiagonal();
    // ee = Vector3d(mcc.data())*D2R;
    // std::cout << ee <<std::endl;
    // noise_.block<3, 3>(0, 0) = ff;
    // for (size_t i = 0; i < 9; i++) {
    //     for (size_t j = 0; j < 9; j++) {
    //         printf("%12.8f", noise_(i,j));
    //     }
    //     printf("\n");
    // }
    // getchar();

    // std::unordered_map<ulong, double> dd;
    // dd[0] = 20;
    // dd[1] = 30;
    // dd[2] = 40;
    // dd.erase(1);
    
    // // printf("size=%4d %10.2f\n", dd.size(),dd[1]);
    // if (dd.find(1) == dd.end()) {
    //     printf("mei zhao dao\n");
    // } else {
    //     printf("cc\n");
    // }
    // getchar();

    // mdd c1;
    // printf("%10.2f, %10.2f, %10.2f\n", c1.d1[0], c1.d1[1], c1.d1[2]);
    // getchar();

    Eigen::Vector3d eulerAngle( 90.32/180*PI, 0.29/180*PI, 88.11/180*PI);   // roll, pitch, yaw (欧拉角逆时针旋转为正，顺时针旋转为负)
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Vector3d::UnitZ()));
    Quaterniond mm = yawAngle * pitchAngle * rollAngle;
    Vector3d vm = mm.matrix().eulerAngles(2,1,0);
    printf("qq=[%12.6f, %12.6f, %12.6f, %12.6f]\n", mm.w(), mm.x(), mm.y(), mm.z());
    printf("vm=[%12.6f, %12.6f, %12.6f]\n", vm[0]*R2D, vm[1]*R2D, vm[2]*R2D);

    Quaterniond q_b_c(0.497766, 0.502679, 0.501396, 0.498141);
    Vector3d ve = q_b_c.matrix().eulerAngles(2,1,0);

    Matrix3d cc = q_b_c.toRotationMatrix();
    printf("qq=[%12.6f, %12.6f, %12.6f, %12.6f]\n", q_b_c.w(), q_b_c.x(), q_b_c.y(), q_b_c.z());
    printf("ve=[%12.6f, %12.6f, %12.6f]\n", ve[0]*R2D, ve[1]*R2D, ve[2]*R2D);
    printf("cc=[%12.6f, %12.6f, %12.6f]\n", cc(0, 0), cc(0, 1), cc(0, 2));
    printf("   [%12.6f, %12.6f, %12.6f]\n", cc(1, 0), cc(1, 1), cc(1, 2));
    printf("   [%12.6f, %12.6f, %12.6f]\n", cc(2, 0), cc(2, 1), cc(2, 2));
    getchar();

    // std::queue<Vector3d> mm;
    // Vector3d dd(1,2,3);
    // mm.push(dd);
    // auto cc = mm.front();
    // Matrix3d qq;
    // qq(0,0) = 4;
    // printf("%10.2f  %10.2f\n",cc[1], qq(0,0));

    ros::init(argc, argv, "receive");
    ros::NodeHandle nh;                 //全局命名空间
    // subscribe message
    ros::Subscriber imu_sub   = nh.subscribe<sensor_msgs::Imu>("/gvi/imu0", 100, &imuCallback);  
    ros::Subscriber gnss_sub  = nh.subscribe<gnss_comm::GnssMeasMsg>("/gvi/gnss0", 1, &gnssCallback);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/gvi/image0", 10, &imageCallback);
    ros::spin();
}