#include <iostream>
#include <fstream>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <chrono> //計測用，xxmsという書き方ができるようにする
#include <functional> //ラムダ式などを使えるようにする
#include <memory> //メモリを扱えるようにする
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "catchrobo_msgs/msg/slave_control.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "../include/dynamixel_workbench.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
//#include "dynamixel_sdk/dynamixel_sdk.h"

// git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
// cd /DynamixelSDK/c++/build/linux64
// make
// sudo make install


using namespace std;
using namespace std::chrono_literals;

class DynamixelNode : public rclcpp::Node{ //rclcpp::Nodeを継承してMinimalPubSubクラスを作成
    private:
    int dxl_num;
    const char *log;
    bool result;
    DynamixelWorkbench dxl_wb;
    DynamixelDriver dx_dr;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<catchrobo_msgs::msg::SlaveControl>::SharedPtr command_sub;
    std::vector<int> ids = {};
    std::vector<std::string> mode = {};
    string port;
    int rate;
    std::vector<float> current_pos = std::vector<float>(6, 0.0);
    std::vector<float> command_pos = std::vector<float>(6, 0.0);
    float speed = 0.0175; //rad

    public:
    DynamixelNode() : Node("dynamixel_node"){ //Node関数をオーバーライド
        // yamlファイルの読み込み
        try{
            std::string package_path = ament_index_cpp::get_package_share_directory("dynamixel_control");
            YAML::Node config = YAML::LoadFile(package_path + "/config/dynamixel.yml");
            port = config["port"].as<string>();
            cout << "port:" << port << endl;
            rate = config["rate"].as<int>();
            cout << "rate:" << rate << endl;
            for(auto motor : config["motors"]){
                ids.push_back(motor["id"].as<int>());
                mode.push_back(motor["mode"].as<std::string>());
            }
        }catch(YAML::Exception& e){
            cerr << "yamlファイルの読み込みに失敗しました:\n" << e.what() << endl;
        }

        result = false;
        dxl_num = ids.size();

        // portの初期化。成功したらTrueを返す。
        result = dxl_wb.init(port.c_str(), rate, &log);
        if (result == false){
            cout << "portの初期化に失敗:" << log << endl;
        }else{
            cout << "portの初期化に成功" << endl;
        }

        cout << "dynamixelの数:" << dxl_num << endl;
        for(int i = 0; i < dxl_num; i++){
            uint16_t model;
            // pingを送りモデル番号を取得
            result = dxl_wb.ping((uint8_t)ids[i], &model, &log);
            if (result == false){
                cout << "ID:" << ids[i] << " model:" << model << " pingに失敗\n" << log << endl;
            }else{
                cout << "ID:" << ids[i] << " model:" << model << " pingに成功" << endl;
            }
        }
        // モードの設定
        for(int i = 0; i < dxl_num; i++){
            if(mode[i]=="position"){
                // jointモードを設定。トルクがオンになる。
                result = dxl_wb.jointMode(ids[i], 0, 0, &log);
                if (result == false){
                    cout << "ID:" << ids[i] << " JointModeの設定に失敗\n" << log << endl;
                }else{
                    cout << "ID:" << ids[i] << " JointModeの設定に成功" << endl;
                    // モードの設定に成功したら位置を初期化
                    int32_t value = dxl_wb.convertRadian2Value(ids[i], M_PI);
                    result = dxl_wb.goalPosition(ids[i], value, &log);
                    if (result == false){
                        cout << "ID:" << ids[i] << " GoalPositionの設定に失敗\n" << log << endl;
                    }
                }
            }else if(mode[i]=="velocity"){
                // ホイールモードを設定。トルクがオンになる。
                result = dxl_wb.wheelMode(ids[i], 0, &log);
                if (result == false){
                    cout << "ID:" << ids[i] << " WheelModeの設定に失敗\n" << log << endl;
                }else{
                    cout << "ID:" << ids[i] << " WheelModeの設定に成功" << endl;
                    // モードの設定に成功したら速度を初期化
                    dxl_wb.goalVelocity(ids[i], (int32_t)0);
                }
            }else if(mode[i]=="current"){
                // 電流ベース位置制御モードを設定。トルクがオンになる。
                //7.5A
                switch(ids[i]){
                    case 1:
                    result = dxl_wb.currentBasedPositionMode(ids[i], 1500, &log);
                    break;
                    case 2:
                    result = dxl_wb.currentBasedPositionMode(ids[i], 2000, &log);
                    break;
                    case 3:
                    result = dxl_wb.currentBasedPositionMode(ids[i], 1500, &log);
                    break;
                    case 4:
                    result = dxl_wb.currentBasedPositionMode(ids[i], 800, &log);
                    break;
                    case 5:
                    result = dxl_wb.currentBasedPositionMode(ids[i], 600, &log);
                    break;
                    case 6:
                    result = dxl_wb.currentBasedPositionMode(ids[i], 600, &log);
                    break;
                    case 7:
                    result = dxl_wb.currentBasedPositionMode(ids[i], 500, &log);
                    break;
                }
                if (result == false){
                    cout << "ID:" << ids[i] << " CurrentBasePositionModeの設定に失敗\n" << log << endl;
                }else{
                    cout << "ID:" << ids[i] << " CurrentBasePositionModeの設定に成功" << endl;
                    int32_t value = dxl_wb.convertRadian2Value(ids[i], 0.0);
                    if(i == 7){
                        result = dxl_wb.goalPosition(ids[i], 500, &log);
                    }else{
                        result = dxl_wb.goalPosition(ids[i], value, &log);
                    }
                    if (result == false){
                        cout << "ID:" << ids[i] << " GoalPositionの設定に失敗\n" << log << endl;
                    }
                    current_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                }
            }
        }


        auto timer_callback = [this]() -> void{
            //ROS1で言うところのwhileループの中身
            int32_t input_data = 0;
            for(int i = 0; i < 6; i++){
                if(command_pos[i] - current_pos[i] < -speed){ //commandがcurrentより大幅に小さい場合は、current-speedを入力とする。
                    current_pos[i] -= speed;
                }else if(command_pos[i] - current_pos[i] > speed){ //commandがcurrentより大幅に大きい場合は、current+speedを入力とする
                    current_pos[i] += speed;
                }else{
                    current_pos[i] = command_pos[i]; //それ以外の場合はそのままの値を入力とする
                }
                result = dxl_wb.torqueOn(ids[i], &log);
                if (result == false){
                    cout << "TorqueOnに失敗:" << log << endl;
                }
                input_data = dxl_wb.convertRadian2Value(ids[i], current_pos[i]);
                result = dxl_wb.goalPosition(ids[i], input_data, &log);
                if (result == false){
                    cout << "ID:" << ids[i] << " GoalPositionの設定に失敗\n" << log << endl;
                }
            }
        };

        auto topic_callback = [this](const catchrobo_msgs::msg::SlaveControl& msg) -> void{
            command_pos[0] = msg.sholder1;
            command_pos[1] = msg.sholder2;
            command_pos[2] = msg.elbow1;
            command_pos[3] = msg.elbow2;
            command_pos[4] = msg.wrist1;
            command_pos[5] = msg.wrist2;
            result = dxl_wb.torqueOn(ids[6], &log);
            if (result == false){
                cout << "TorqueOnに失敗:" << log << endl;
            }
            if(msg.grip){
                result = dxl_wb.goalPosition(ids[6], 1890); //-60
            }else{
                result = dxl_wb.goalPosition(ids[6], 500); //50
            }
            if (result == false){
                cout << "ID:" << ids[6] << " GoalPositionの設定に失敗\n" << log << endl;
            }
        };

        // qosの設定
        rmw_qos_profile_t qos = rmw_qos_profile_default;
        qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos.depth = 1;

        //state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/dynamixel_state", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos)));
        command_sub = this->create_subscription<catchrobo_msgs::msg::SlaveControl>("/dynamixel_command", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos)), topic_callback);
        timer = this->create_wall_timer(10ms, timer_callback);
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    //タイマからのコールバックを開始する
    std::shared_ptr<DynamixelNode> dynamixel_node = std::make_shared<DynamixelNode>();
    rclcpp::spin(dynamixel_node);
    rclcpp::shutdown();
    return 0;
}


/*
int id = 5;
float radian = 0.2;

DynamixelDriver dx_dr;
DynamixelWorkbench dx_wb;
bool result = 0;
uint32_t value = 0;
value = convertRadian2Value(id, radian);
result = dx_dr.writeRegister(id, "Goal_Position", value, log);

bool DynamixelWorkbench::goalPosition(uint8_t id, float radian, const char **log)
{
  bool result = 0;
  uint32_t value = 0;

  value = convertRadian2Value(id, radian);

  result = goalPosition(id, (int32_t)value, log);
  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set goal position!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal position!";
  return true;
}
bool DynamixelWorkbench::goalPosition(uint8_t id, int value, const char **log)
{
//   goalPosition(id, (int32_t)value, log);
// }

// bool DynamixelWorkbench::goalPosition(uint8_t id, int32_t value, const char **log)
// {
  bool result = false;
  
  result = itemWrite(id, "Goal_Position", value, log);

  if (result == false)
  {
    if (log != NULL) *log = "[DynamixelWorkbench] Failed to set goal position!";
    return false;
  }

  if (log != NULL) *log = "[DynamixelWorkbench] Succeeded to set goal position!";
  return result;
}
bool DynamixelWorkbench::itemWrite(uint8_t id, const char *item_name, int32_t data, const char **log)
{
  return writeRegister(id, item_name, data, log);
}*/

