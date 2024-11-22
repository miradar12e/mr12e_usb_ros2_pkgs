#include "geometry_msgs/msg/point.hpp"
#include "math.h"
#include "miradar.h"
#include "miradar_msgs/msg/ppi.hpp"
#include "miradar_msgs/msg/ppi_data.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"


#define DEG2RAG(deg) (((deg) / 360) * 2 * M_PI)

#define ST_RQT_MODE_RADAR       "01:sensor_mode"
#define ST_RQT_RMAX             "11:distance_max"
#define ST_RQT_RMIN             "12:distance_min"
#define ST_RQT_RALARM           "13:distance_alarm"
#define ST_RQT_RDIV             "14:distance_div"
#define ST_RQT_THMAX            "21:angle_max"
#define ST_RQT_THDIV            "22:angle_div"
#define ST_RQT_TXPWR            "31:tx_power"
#define ST_RQT_RXHPF            "32:rx_hpf_gain"
#define ST_RQT_RXPGA            "33:rx_pga_gain"
#define ST_RQT_DBMAX            "34:rx_dB_max"
#define ST_RQT_DBMIN            "35:rx_dB_min"
#define ST_RQT_MEASITVL         "41:duration"
#define ST_RQT_NUMPPI           "42:num_ppi_plot"
#define ST_RQT_MODE_SCAN        "51:scan_mode"

class MiRadarROS2 {
public:
    rcl_interfaces::msg::SetParametersResult setParam(
        const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        RCLCPP_INFO(node->get_logger(), "Applying Parameter Change.");
        for (const auto &parameter : parameters) {
            setParamImpl(parameter);
        }

        isConfigUpdate = true;
        return result;
    }

    void setParamImpl(const rclcpp::Parameter &parameter) {
        if (parameter.get_name() == "devicename" &&
            parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            deviceFile = parameter.as_string();
        } else if (parameter.get_name() == ST_RQT_MODE_RADAR &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            std::cout << "sensor mode changing" << std::endl;
            sensorMode = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_RMAX &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_DOUBLE) {
            miradarParam.maxDistance =
                static_cast<int>(parameter.as_double() * 1000);
            RCLCPP_INFO(node->get_logger(), "max distance set");
            RCLCPP_INFO(node->get_logger(), "distance : %f, converted : %d",
                        parameter.as_double(), miradarParam.maxDistance);
        } else if (parameter.get_name() == ST_RQT_RMIN &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_DOUBLE) {
            miradarParam.minDistance =
                static_cast<int>(parameter.as_double() * 1000.0);
        } else if (parameter.get_name() == ST_RQT_RALARM &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_DOUBLE) {
            miradarParam.alarmDistance =
                static_cast<int>(parameter.as_double() * 1000.0);
        } else if (parameter.get_name() == ST_RQT_DBMAX &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            std::cout << parameter.as_int() << std::endl;
            miradarParam.maxDb = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_DBMIN &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.minDb = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_RDIV &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.nDistance = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_THMAX &&                  // 2023_1116 ST added
                   parameter.get_type() ==                                  //
                       rclcpp::ParameterType::PARAMETER_INTEGER) {          //
            miradarParam.maxAngle = parameter.as_int();                     //
        } else if (parameter.get_name() == ST_RQT_THDIV &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.nAngle = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_TXPWR &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.txPower = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_RXHPF &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.hpfGain = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_RXPGA &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.pgaGain = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_MEASITVL &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            miradarParam.duration = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_MODE_SCAN &&
                   parameter.get_type() ==
                       rclcpp::ParameterType::PARAMETER_INTEGER) {
            laserscanMode = parameter.as_int();
        } else if (parameter.get_name() == ST_RQT_NUMPPI &&                 // 2023_1116 ST added
                   parameter.get_type() ==                                  //
                       rclcpp::ParameterType::PARAMETER_INTEGER) {          //
            miradarParam.nNumPpiPlot = parameter.as_int();                  //
        }
    }

    explicit MiRadarROS2() {
        //: sensorMode(0), isConfigUpdate(false), laserscanMode(1) {
        sensorMode = 0;                 // 2023_1206 ST for colcon build warning
        isConfigUpdate = false;
        laserscanMode = 1;
        node = rclcpp::Node::make_shared("miradar_node");
        pub = node->create_publisher<miradar_msgs::msg::PPIData>(
            "/miradar/ppidata", 20);
        mapPub = node->create_publisher<sensor_msgs::msg::Image>(
            "/miradar/image_raw", 20);
        laserPub = node->create_publisher<sensor_msgs::msg::LaserScan>(
            "/miradar/scan", 20);
        pcPub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/miradar/points", 20);
        declareParams();

        deviceFile =
            ((int)(deviceFile.find("/dev/tty")) == -1) ? STDEF_INIT_COMM_PORT : deviceFile;

        initConnection();

        callback_handle = node->add_on_set_parameters_callback(
                            std::bind(&MiRadarROS2::setParam, this, std::placeholders::_1));
                                                                        // 2023_1206 ST  changed from
                                                                        // set_on_parameters_set_callback() for humble
    }

    void declareParams() {
        node->declare_parameter(ST_RQT_MODE_RADAR, 0);
        node->declare_parameter(ST_RQT_RMAX, STDEF_INIT_RMAX);
        node->declare_parameter(ST_RQT_RMIN, STDEF_INIT_RMIN);
        node->declare_parameter(ST_RQT_THMAX, STDEF_INIT_THMAX);
        node->declare_parameter(ST_RQT_DBMAX, STDEF_INIT_RXDBMAX);
        node->declare_parameter(ST_RQT_DBMIN, STDEF_INIT_RXDBMIN);

        node->declare_parameter(ST_RQT_RALARM, STDEF_INIT_RALM);
        node->declare_parameter(ST_RQT_RDIV, STDEF_INIT_RDIV);
        node->declare_parameter(ST_RQT_THDIV, STDEF_INIT_THDIV);
        node->declare_parameter(ST_RQT_TXPWR, STDEF_INIT_TXPWR);
        node->declare_parameter(ST_RQT_RXHPF, STDEF_INIT_RXHPF);
        node->declare_parameter(ST_RQT_RXPGA, STDEF_INIT_RXPGA);
        node->declare_parameter(ST_RQT_MEASITVL, STDEF_INIT_MEASITVL);
        node->declare_parameter(ST_RQT_MODE_SCAN, 0);
        node->declare_parameter(ST_RQT_NUMPPI, STDEF_INIT_NUMPPIPLOT);
    }

    void initConnection() {
        int fd;

        fd = serial.CommInit(deviceFile);
        if (fd < 0) {
            RCLCPP_INFO(node->get_logger(), "device open failed.");
            exit(-1);
        }
        miradar.setSerial(serial);
        RCLCPP_INFO(node->get_logger(), "Connected to %s",
                    (char *)deviceFile.c_str());
        //miradar.setParam();
    }

    void changeSensorState() {
        RCLCPP_INFO(node->get_logger(), "changing");
        miradar.setSensorState(sensorMode);
    }

    void publishMap() {
        if ((miradar.nAngle * miradar.nDistance) == (int)(miradar.map.size())) {
            auto image = sensor_msgs::msg::Image();
            image.height = miradarParam.nAngle;
            image.width = miradarParam.nDistance;
            image.step = image.width;
            image.header.frame_id = "miradar";
            image.encoding = "mono8";
            std::copy(miradar.map.begin(), miradar.map.end(),
                      std::back_inserter(image.data));
            mapPub->publish(image);
        } else {
            //RCLCPP_INFO(node->get_logger(), "map is corrupt. %d  %d  %dsize",
            //            miradar.map.size(), (miradar.nDistance), (miradar.nAngle));
        }
    }

    void checkForEmptyParam() {
        miradarParam.maxAngle = (miradarParam.maxAngle == 0)
                                    ? miradar.radarParam.maxAngle
                                    : miradarParam.maxAngle;
        miradarParam.minDistance = (miradarParam.minDistance == 0)
                                       ? miradar.radarParam.minDistance
                                       : miradarParam.minDistance;
        miradarParam.maxDistance = (miradarParam.maxDistance == 0)
                                       ? miradar.radarParam.maxDistance
                                       : miradarParam.maxDistance;
        miradarParam.nAngle =
            (miradarParam.nAngle == 0) ? miradar.nAngle : miradarParam.nAngle;
        miradarParam.nDistance = (miradarParam.nDistance == 0)
                                     ? miradar.nDistance
                                     : miradarParam.nDistance;
    }

    void publishLaserScan() {
        //RCLCPP_INFO(node->get_logger(), "%d, %d", miradarParam.nAngle,
        //            miradarParam.nDistance);
        //RCLCPP_INFO(node->get_logger(), "%d, %d", miradar.radarParam.nAngle,
        //            miradar.radarParam.nDistance);

        if((miradarParam.nAngle * miradarParam.nDistance) == (int)(miradar.map.size())) {
            auto ls = sensor_msgs::msg::LaserScan();
            ls.header.frame_id = "miradar_scan";
            ls.angle_min = DEG2RAG(-static_cast<double>(miradarParam.maxAngle));
            ls.angle_max = DEG2RAG(static_cast<double>(miradarParam.maxAngle));
            ls.range_max =
                static_cast<double>(miradarParam.maxDistance) / 1000.0;
            ls.angle_increment = (ls.angle_max - ls.angle_min) /
                                 static_cast<double>(miradarParam.nAngle);
            ls.time_increment = 0.001;
            float a = (ls.range_max) / (static_cast<double>(miradarParam.nDistance - 1));
            //RCLCPP_INFO(node->get_logger(), "nAngle : %d", miradarParam.nAngle);

            for (int i = 0; i < miradarParam.nAngle; i++) {
                int max = -1;
                float distance = -1;
                float intensity = -1;

                for (int j = 0; j < miradarParam.nDistance; j++) {
                    if (laserscanMode == 0) {
                        if (MiRadar::pixel2DB(
                                miradar.map[i * miradarParam.nDistance + j]) >
                            miradarParam.minDb) {
                            distance = a * j;
                            intensity =
                                static_cast<double>(
                                    miradar
                                        .map[i * miradarParam.nDistance + j]) /
                                255.0;
                            ls.ranges.push_back(distance);
                            ls.intensities.push_back(intensity);
                            break;
                        } else if (j == miradarParam.nDistance - 1) {
                            ls.ranges.push_back(999);
                            ls.intensities.push_back(0);
                        }
                    } else {
                        if (miradar.map[i * miradarParam.nDistance + j] > max) {
                            max = miradar.map[i * miradarParam.nDistance + j];
                            distance = a * j;
                            intensity =
                                static_cast<double>(
                                    miradar
                                        .map[i * miradarParam.nDistance + j]) /
                                255.0;
                        }
                    }
                }
                if (laserscanMode == 1) {
                    ls.ranges.push_back(distance);
                    ls.intensities.push_back(intensity);
                }
            }
            ls.header.stamp = node->now();
            laserPub->publish(ls);
        }
    }

    void publishPointClouds() {
        pcl::PointCloud<pcl::PointXYZI> ladarpc;

        if ((miradarParam.nAngle * miradarParam.nDistance) == (int)(miradar.map.size())) {
            auto pointcloud_msg = sensor_msgs::msg::PointCloud2();

            double angle_min =
                DEG2RAG(-static_cast<double>(miradarParam.maxAngle));
            double angle_max =
                DEG2RAG(static_cast<double>(miradarParam.maxAngle));
            double range_max =
                static_cast<double>(miradarParam.maxDistance) / 1000.0;
            double angle_increment = (angle_max - angle_min) /
                                     static_cast<double>(miradarParam.nAngle);
            double distance_increment = range_max / static_cast<double>(miradarParam.nDistance);
            float a = range_max / (static_cast<double>(miradarParam.nDistance - 1));
            float a_angle = (angle_max - angle_min) /
                            (static_cast<double>(miradarParam.nAngle - 1));

            for (int i = 0; i < miradarParam.nAngle; i++) {
                float distance = -1;
                float intensity = -1;
                float angle = 0;

                for (int j = 0; j < miradarParam.nDistance; j++) {
                    if(MiRadar::pixel2DB(
                            miradar.map[i * miradarParam.nDistance + j]) >
                        miradarParam.minDb) {
                        distance = a * j + distance_increment / 2;
                        angle = a_angle * i + angle_min + angle_increment / 2;
                        intensity = static_cast<double>(miradar.map[i * miradarParam.nDistance + j]) / 255.0;
                                                // 2023_1116 ST debug  from  [i * miradar.radarParam.nDistance + j]
                        double y = distance * sin(angle);
                        double x = distance * cos(angle);
                        pcl::PointXYZI point;
                        point.x = x;
                        point.y = y;
                        point.z = 0;
                        point.intensity = intensity;
                        ladarpc.push_back(point);
                    }
                }
            }
            pcl::toROSMsg(ladarpc, pointcloud_msg);
            pointcloud_msg.header.frame_id = "miradar";
            pcPub->publish(pointcloud_msg);
        }
    }



    void publishPPI() {
        if (miradar.ppiEntries.size() > 0) {
            auto ppidata = miradar_msgs::msg::PPIData();

            for (int i = 0; i < (int)(miradar.ppiEntries.size()); i++) {
                auto ppi = miradar_msgs::msg::PPI();
                double distance =
                    static_cast<double>(miradar.ppiEntries[i].distance) /
                    1000.0;
                double rad =
                    DEG2RAG(static_cast<double>(miradar.ppiEntries[i].angle));
                ppi.position.y = distance * sin(rad);
                ppi.position.x = distance * cos(rad);
                ppi.position.z = 0;

                ppi.speed = miradar.ppiEntries[i].speed;
                ppi.db = miradar.ppiEntries[i].db;
                
                //-------------------------------- ST added 2022_1018
                if(((miradarParam.nNumPpiPlot)-1)<i) {
                    continue;
                }
                //-------------------------------- ST added 2022_1018
                
                ppidata.data.push_back(ppi);
            }
            pub->publish(ppidata);
        }
    }

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle;      // 2023_1206 ST

    void run() {
        miradar.run();
        if (sensorMode == 1) {
            publishPPI();
        } else if (sensorMode == 2) {
            publishMap();
            publishLaserScan();
            publishPointClouds();
        }

        if (isConfigUpdate) {
            changeSensorState();
            miradar.setParam(miradarParam);
            isConfigUpdate = false;
        }
    }

private:
    Serial serial;
    MiRadarParam miradarParam;
    bool isConfigUpdate;
    MiRadar miradar;
    std::string deviceFile;
    int sensorMode;
    int laserscanMode;

    rclcpp::Publisher<miradar_msgs::msg::PPIData>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mapPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcPub;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    MiRadarROS2 miradarROS;

    while (rclcpp::ok()) {
        miradarROS.run();
        rclcpp::spin_some(miradarROS.node);
    }
}
