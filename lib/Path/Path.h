/* ---------- path_manager.h ---------- */
#pragma once
#include <deque>
#include <Arduino.h>
#include <vector>
#include "serial.h"
#include <utility>
#include <String>
#include <map>
#include <Preferences.h>
#include <ErriezCRC32.h>
#include <set>
#include <cstring>
#include "BLE.h"
#include "IMU901.h"
const uint8_t MODE_BUTTON_PIN = 23;
extern volatile bool mode_changed;
const unsigned long debounce_delay = 200;
enum class MoveType
{
    FORWARD_CM,
    TURN_LEFT_DEG,
    TURN_RIGHT_DEG,
    STOP
};
enum class MotionState
{
    IDLE,
    MOVING,
    TURNING,
    STOPPING,
   
};
enum class WorkMode
{
    PATROLLING,
    NAVIGATION,
    WAITING,
};

struct PathTask
{
    MoveType type;
    float value;
    String location;
};

struct EncoderDelta
{
    int64_t left;
    int64_t right;
};
struct FlashData {
    char location[16]; // 固定长度存储位置名称
    int32_t value;
};

class BLEClientManager; // 前向声明
class PathManager
{
public:
    PathManager(float wheel_diameter, int encoder_ppr,BLEClientManager& ble,IMU901& imu);

    // 任务管理接口
    void add_task(MoveType type, float value, String location = "");
    void clear_tasks();
    void execute();
    bool is_executing() const;
    const String point_order_[12] = {
        "A1", "A2", "A3", "B3", "B2", "B1",
        "C1", "C2", "C3", "D3", "D2", "D1"};
    // 数据采集接口
    void storeData(const String &location, int value);
    void initStorage();
    const std::vector<std::pair<String, int>> &get_collected_data() const { return collected_data_; }
    void save_as_flash_data();
    void load_from_flash_data();
    int stop_counter_ =0;
    // 导航
    void switch_mode(WorkMode new_mode);
    WorkMode current_mode() const { return work_mode_; }

    void load_navigation_task(const String &point);
    String findPointByValue(int targetValue) const;
    void printStoredData();
    void check_mode();
    /*------------------Private------------------*/
    // 任务执行相关
   
    
private:
Preferences prefs_;
    struct Task
    {
        MoveType type;
        float value;
        String location;
        Task(MoveType t, float v, String loc = "") : type(t), value(v), location(loc) {}
    };
    WorkMode work_mode_ = WorkMode::WAITING;
    std::set<String> sent_locations_;
    std::map<String, std::deque<Task>> navigation_paths_; // 导航路径存储
    std::deque<Task> task_queue_;                         // 改为deque容器
    const float wheel_circumference_;
    const int encoder_ppr_;
    MotionState motion_state_ = MotionState::IDLE;
    EncoderDelta total_delta_;
    float target_yaw_ = 0;
    bool BLEFlag=false;
    int current_nav_value_ = 0;      // 存储当前导航数值
    String current_target_point_;    // 存储当前目标点位
     //蓝牙
     BLEClientManager& ble_manager_; 
    // 数据采集相关
    std::vector<std::pair<String, int>> collected_data_;
    // 闪存成员
    
    const char *FLASH_NAMESPACE = "patrol_data";
    IMU901& imu_;
    void reset_patrol_counter();
    // 私有方法
    int cm_to_pulses(float cm) const;
    void start_task(MoveType type, float value);
    void stop_motion_control();
    bool check_distance(int target_pulses);
    // 数据存储相关
    uint32_t calculate_crc() const;
    
};
void Path_Init();