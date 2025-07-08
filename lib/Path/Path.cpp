/* ---------- path_manager.cpp ---------- */
#include "Path.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "pid.h"
#include "BLE.h"
// 外部变量声明
extern PID_t Angle_Outer, Left_Inner, Right_Inner;
extern volatile bool pidComputeFlag;
extern int16_t basespeed;
extern PathManager path_manager;
constexpr float ANGLE_TOLERANCE = 1.0f;
extern IMU901 imu;
extern HardwareSerial SerialPort;
extern volatile bool OKFLAG;
extern int current_nav_value ;
extern String current_target_point;
extern int inputValue;
extern String targetPoint;
PathManager::PathManager(float wheel_diameter, int encoder_ppr, BLEClientManager &ble, IMU901 &imu)
    : prefs_(),
    ble_manager_(ble),
      imu_(imu),
      stop_counter_(0),
      wheel_circumference_(PI * wheel_diameter),
      encoder_ppr_(encoder_ppr),
      point_order_({"A1", "A2", "A3", "B3", "B2", "B1",
                    "C1", "C2", "C3", "D3", "D2", "D1"}) { }

int PathManager::cm_to_pulses(float cm) const
{
    return static_cast<int>((cm / wheel_circumference_) * encoder_ppr_);
}

void PathManager::add_task(MoveType type, float value, String location)
{
    task_queue_.emplace_back(type, value, location);
}

void PathManager::clear_tasks()
{
    task_queue_.clear();
}

void PathManager::start_task(MoveType type, float value)
{
    switch (type)
    {
    case MoveType::FORWARD_CM:
        Encoder_Reset();
        total_delta_ = {0, 0};
        basespeed = 15;
        motion_state_ = MotionState::MOVING;
        break;

    case MoveType::TURN_LEFT_DEG:
        target_yaw_ = imu.getAttitude().yaw + value; // 左转时增加角度值
        Angle_Outer.Target = target_yaw_;
        motion_state_ = MotionState::TURNING;
        break;

    case MoveType::TURN_RIGHT_DEG:
        target_yaw_ = imu.getAttitude().yaw - value; // 右转时减少角度值
        Angle_Outer.Target = target_yaw_;
        motion_state_ = MotionState::TURNING;
        break;

    case MoveType::STOP:
        motion_state_ = MotionState::STOPPING;
        break;
    }
}

bool PathManager::is_executing() const
{
    return motion_state_ != MotionState::IDLE;
}

void PathManager::stop_motion_control()
{
    Motor_Control(0, 0);
    pidComputeFlag = false;
    Encoder_Reset();
}

bool PathManager::check_distance(int target_pulses)
{
     int avg_pulses = (abs(total_delta_.left) + abs(total_delta_.right)) / 2;
     //Serial.printf("[DEBUG] Avg Pulses: %d, Target Pulses: %d\n", avg_pulses, target_pulses);
    return (abs(total_delta_.left) >= target_pulses) ||
           (abs(total_delta_.right) >= target_pulses);
}
/*------------------路径执行------------------*/
void PathManager::execute()
{
    Serial.printf("[DEBUG] Queue:%d State:%d\n", task_queue_.size(), static_cast<int>(motion_state_));
    if (work_mode_ == WorkMode::WAITING)
    {
        return;
    }
    if (task_queue_.empty())
    {
        OKFLAG = true;
        motion_state_ = MotionState::IDLE;
        stop_motion_control();
        return;
    }
    if (work_mode_ == WorkMode::NAVIGATION && task_queue_.empty())
        return;
    if (!task_queue_.empty())
    {
        auto &current_task = task_queue_.front();

        switch (motion_state_)
        {
        case MotionState::IDLE:
            start_task(current_task.type, current_task.value);
            break;

        case MotionState::MOVING:
        {

            EncoderDelta current = {Encoder_Get1(), Encoder_Get2()};
            total_delta_.left += current.left;
            total_delta_.right += current.right;

            if (check_distance(cm_to_pulses(current_task.value)))
            {
                motion_state_ = MotionState::IDLE;
                task_queue_.pop_front();
                
            }
            break;
        }

        case MotionState::TURNING:
        {
            float current_yaw = imu.getAttitude().yaw;
            if (fabs(target_yaw_ - current_yaw) <= ANGLE_TOLERANCE)
            {
                task_queue_.pop_front();
                motion_state_ = MotionState::IDLE;
                Motor_Control(0, 0);
            }
            break;
        }

        case MotionState::STOPPING:
        {
            stop_motion_control();
            // 同时支持巡逻和导航模式的STOP任务
            /*-----------------遍历模式的停止----------------*/
            if (work_mode_ == WorkMode::PATROLLING  && !task_queue_.empty()) 
            {
                auto& current_task = task_queue_.front();
                if (current_task.type == MoveType::STOP && current_task.location != "") 
                {
                    String current_point = current_task.location;
                    Serial.printf("\n[INPUT] Waiting for %s: ", current_point.c_str());
                    // 非阻塞读取串口数据
                    if (SerialPort.available()) {
                        // 清除前置非数字字符
                        while (SerialPort.available() && !isdigit(SerialPort.peek())) {
                            SerialPort.read();
                        }
        
                        // 解析有效数据
                        if (SerialPort.available() > 0) {
                            int val = SerialPort.parseInt();
                            storeData(current_point, val); // 存储数据
                            
                            // 清除残留字符
                            while (SerialPort.available()) {
                                SerialPort.read();
                            }
        
                            // 弹出已完成的STOP任务
                            task_queue_.pop_front();
                            motion_state_ = MotionState::IDLE;
                            Serial.printf("Received %d for %s. Resuming...\n", val, current_point.c_str());
                        }
                    }
                }
                }
/*-----------------导航模式的停止----------------*/
             if (work_mode_ == WorkMode::NAVIGATION && !task_queue_.empty()) {
            auto& current_task = task_queue_.front();
                // 自动完成任务（不等待输入）
                 if (targetPoint != "") {
                    Serial.printf("\n[INPUT] Waiting for %s: ", targetPoint.c_str());
            // 立即通过蓝牙发送（无需等待STOP）
            if(ble_manager_.isConnected()){
                std::string ble_msg = 
                    current_target_point.c_str() + 
                    std::string(" ") + 
                    std::to_string(current_nav_value);
                ble_manager_.sendData(ble_msg);
                Serial.printf("[Instant] Sent: %s\n", ble_msg.c_str());
            }
                task_queue_.pop_front();
                motion_state_ = MotionState::IDLE;
              
                
            
               
            
        }}
           
        }
         break;
        }
    }
}

void PathManager::storeData(const String &location, int value) {
    // 数据有效性验证
    if (location.isEmpty() || value == 0) {
        Serial.printf("[ERROR] Invalid data: %s:%d\n", location.c_str(), value);
        return;
    }

    // 检查是否已存在该点位数据
    bool exists = false;
    for (auto& item : collected_data_) {
        if (item.first == location) {
            item.second = value; // 更新已有数据
            exists = true;
            Serial.printf("[DATA] Updated %s:%d\n", location.c_str(), value);
            break;
        }
    }
    
    if (!exists) {
        collected_data_.emplace_back(location, value);
        Serial.printf("[DATA] New %s:%d\n", location.c_str(), value);
    }
    
    // 蓝牙发送（带重复检查）
    if (ble_manager_.isConnected()) {
        std::string ble_data = location.c_str();
        ble_data += " " + std::to_string(value);
        
        if (sent_locations_.find(location) == sent_locations_.end()) {
                ble_manager_.sendData(ble_data) ;
                sent_locations_.insert(location);
                Serial.printf("[BLE] Sent: %s\n", ble_data.c_str());
            
        } else {
            Serial.printf("[BLE] %s already sent\n", location.c_str());
        }
    }
    save_as_flash_data(); // 立即保存到闪存
   
}
void PathManager::reset_patrol_counter()
{
    prefs_.begin(FLASH_NAMESPACE);
    // 仅当键存在时才删除
    if (prefs_.isKey("collected")) prefs_.remove("collected");
    if (prefs_.isKey("crc")) prefs_.remove("crc");
    if (prefs_.isKey("counter")) prefs_.remove("counter");
    prefs_.end();
    collected_data_.clear();
    sent_locations_.clear();
}
void PathManager::initStorage()
{
    
    load_from_flash_data();
}
uint32_t PathManager::calculate_crc() const
{
    if (collected_data_.empty())
        return CRC32_INITIAL; // 空数据返回初始值

    // 将 vector 数据转换为字节流
    const uint8_t *data_ptr = reinterpret_cast<const uint8_t *>(collected_data_.data());
    size_t data_len = collected_data_.size() * sizeof(std::pair<String, int>);

    // 计算CRC32
    return crc32Buffer(data_ptr, data_len);
}

void PathManager::save_as_flash_data() {
    prefs_.begin(FLASH_NAMESPACE, false);
    
    prefs_.remove("data");
    
    std::vector<FlashData> flash_data;
    for (const auto& item : collected_data_) {
        FlashData data;
        // 确保字符串截断并添加终止符
        strncpy(data.location, item.first.c_str(), sizeof(data.location)-1);
        data.location[sizeof(data.location)-1] = '\0';
        data.value = item.second;
        flash_data.push_back(data);
    }
    
    prefs_.putBytes("data", flash_data.data(), flash_data.size() * sizeof(FlashData));
    
    uint32_t crc = crc32Buffer(
        reinterpret_cast<const uint8_t*>(flash_data.data()),
        flash_data.size() * sizeof(FlashData)
    );
    prefs_.putUInt("crc", crc);
    prefs_.putInt("counter", stop_counter_);
    
    prefs_.end(); // 确保提交更改
}

void PathManager::load_from_flash_data() {
    prefs_.begin(FLASH_NAMESPACE, true);
    
    collected_data_.clear();
    stop_counter_ = 0;
    
    if (prefs_.isKey("data")) {
        size_t data_size = prefs_.getBytesLength("data");
        size_t item_count = data_size / sizeof(FlashData);
        
        if (data_size > 0 && (data_size % sizeof(FlashData) == 0)) {
            std::vector<FlashData> flash_data(item_count);
            prefs_.getBytes("data", flash_data.data(), data_size);
            
            uint32_t stored_crc = prefs_.getUInt("crc", CRC32_INITIAL);
            uint32_t calc_crc = crc32Buffer(
                reinterpret_cast<const uint8_t*>(flash_data.data()),
                data_size
            );
            
            if (stored_crc == calc_crc) {
                for (const auto& data : flash_data) {
                    String location(data.location);
                    collected_data_.emplace_back(location, static_cast<int>(data.value));
                }
                stop_counter_ = prefs_.getInt("counter", 0);
            } else {
                Serial.println("CRC校验失败,清除损坏数据");
                prefs_.remove("data");
                prefs_.remove("crc");
                prefs_.remove("counter");
            }
        }
    }
    prefs_.end();
}
void PathManager::printStoredData() {
    prefs_.begin(FLASH_NAMESPACE);
    // 打印前先同步数据（确保加载最新数据）
    load_from_flash_data(); // 确保加载最新数据

    // 检查是否有数据
    if (collected_data_.empty()) {
        Serial.println("[INFO] No patrol data in memory");
        prefs_.end();
        return;
    }

    Serial.println("\nCurrent Patrol Data:");
    Serial.println("---------------------");
    for (const auto& item : collected_data_) {
        Serial.printf("Point: %-4s | Value: %4d\n", 
                      item.first.c_str(), item.second);
    }
    Serial.println("---------------------");
    prefs_.end();
}
void PathManager::switch_mode(WorkMode new_mode) {
   if (work_mode_ != new_mode) {
        work_mode_ = new_mode;
        clear_tasks();
        stop_motion_control();
        
        // 重置编码器计数
        Encoder_Reset();
        
        // 重置陀螺仪目标角度为当前方向
        Angle_Outer.Target = imu_.getAttitude().yaw;
        
        // 清除可能的残留转向误差
        Angle_Outer.Out = 0;

        // 模式切换日志
        Serial.printf("[MODE] Switched to %s. Encoder & IMU reset.\n", 
                      (work_mode_ == WorkMode::NAVIGATION) ? "NAVIGATION" : "PATROLLING");

        switch (work_mode_) {
            case WorkMode::PATROLLING:
                Serial.println("[INFO] Switching to PATROLLING mode");
                Path_Init(); // 仅巡逻模式加载巡逻路径
                Angle_Outer.Target = imu.getAttitude().yaw;
                break;
            case WorkMode::NAVIGATION:
                Serial.println("[INFO] Switching to NAVIGATION mode");
                 Motor_Control(0, 0);
                  clear_tasks();
                  Angle_Outer.Target = imu.getAttitude().yaw;
                break;
            case WorkMode::WAITING:
                Serial.println("[INFO] Switching to WAITING mode");
                motion_state_ = MotionState::IDLE;
                break;
        }
    }
}
void PathManager::check_mode()
{
    if (work_mode_ == WorkMode::PATROLLING)
    {
        Serial.println("[INFO] Switching to PATROLLING mode");
        Path_Init();            // 重新初始化巡逻路径
    }
    if (work_mode_ == WorkMode::WAITING)
    {Serial.println("[INFO] Switching to WAITING mode");
        motion_state_ = MotionState::IDLE;
        Motor_Control(0, 0);
    }
    if( work_mode_ == WorkMode::NAVIGATION)
    {Serial.println("[INFO] Switching to NAVIGATION mode");
        clear_tasks(); // 清空任务队列
        stop_motion_control(); // 停止运动控制
        
    }
}



// PathManager.cpp

String PathManager::findPointByValue(int targetValue) const
{
    for (const auto &item : collected_data_)
    {
        if (item.second == targetValue)
        {
            return item.first;
        }
    }
    return ""; // 如果未找到，返回空字符串
}
// 加载导航任务
void PathManager::load_navigation_task(const String &point)
{
    motion_state_ = MotionState::IDLE;
    clear_tasks(); // 清空旧任务

    /*------------------ A区点位 ------------------*/
    if (point == "A1") {
        add_task(MoveType::FORWARD_CM, 50);
        add_task(MoveType::STOP, 0, "A1");
        add_task(MoveType::FORWARD_CM, 97);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 155);
    } 
     else if (point == "A2") {
        add_task(MoveType::FORWARD_CM, 80);
        add_task(MoveType::STOP, 0, "A2");
        add_task(MoveType::FORWARD_CM, 67);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 155);
    }
    else if (point == "A3") {
        add_task(MoveType::FORWARD_CM, 110);
        add_task(MoveType::STOP, 0, "A3");
        add_task(MoveType::FORWARD_CM, 37);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 155);
    }

    /*------------------ B区点位 ------------------*/
     else if (point == "B3") {
        add_task(MoveType::FORWARD_CM, 130);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 55);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 25);
        add_task(MoveType::STOP, 0, "B3");
        add_task(MoveType::TURN_LEFT_DEG, 180);
        add_task(MoveType::FORWARD_CM, 45);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 62);
    }
     else if (point == "B2") {
        add_task(MoveType::FORWARD_CM, 130);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 55);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 55);
        add_task(MoveType::STOP, 0, "B2");
        add_task(MoveType::TURN_LEFT_DEG, 180);
        add_task(MoveType::FORWARD_CM, 75);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 62);
    }
     else if (point == "B1") {
        add_task(MoveType::FORWARD_CM, 130);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 55);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 85);
        add_task(MoveType::STOP, 0, "B1");
        add_task(MoveType::TURN_LEFT_DEG, 180);
        add_task(MoveType::FORWARD_CM, 105);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 62);
    }

    /*------------------ C区点位 ------------------*/
     else if (point == "C1") {
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 75);
        add_task(MoveType::TURN_LEFT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 40);
         add_task(MoveType::STOP, 0, "C1");
        add_task(MoveType::FORWARD_CM, 98);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 65);
    }
     else if (point == "C2") {
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 75);
        add_task(MoveType::TURN_LEFT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 70);
        add_task(MoveType::STOP, 0, "C2");
        add_task(MoveType::FORWARD_CM, 73);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 65);
    }
     else if (point == "C3") {
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 75);
        add_task(MoveType::TURN_LEFT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 100);
        add_task(MoveType::STOP, 0, "C3");
        add_task(MoveType::FORWARD_CM, 33);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 65);
    }

    /*------------------ D区点位 ------------------*/
     else if (point == "D1") {
        add_task(MoveType::FORWARD_CM, 130);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 140);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 80);
        add_task(MoveType::STOP, 0, "D1");
        add_task(MoveType::TURN_LEFT_DEG, 180);
        add_task(MoveType::FORWARD_CM, 105);
    }
     else if (point == "D2") {
        add_task(MoveType::FORWARD_CM, 130);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 140);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 50);
        add_task(MoveType::STOP, 0, "D2");
        add_task(MoveType::TURN_LEFT_DEG, 180);
        add_task(MoveType::FORWARD_CM, 75);
    }
    else if (point == "D3") {
        add_task(MoveType::FORWARD_CM, 130);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 140);
        add_task(MoveType::TURN_RIGHT_DEG, 90);
        add_task(MoveType::FORWARD_CM, 25);
        add_task(MoveType::STOP, 0, "D3");
        add_task(MoveType::TURN_LEFT_DEG, 180);
        add_task(MoveType::FORWARD_CM, 45);
    }

    // 添加默认错误处理
    else {
        Serial.printf("[ERROR] Unknown point: %s\n", point.c_str());
    }
}

void Path_Init()
{   
    path_manager.clear_tasks();
    path_manager.stop_counter_ = 0;
    path_manager.add_task(MoveType::FORWARD_CM, 50);
    path_manager.add_task(MoveType::STOP, 0, "A1"); //
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "A2"); //
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "A3"); //
    path_manager.add_task(MoveType::FORWARD_CM, 20);
    path_manager.add_task(MoveType::TURN_RIGHT_DEG, 90);
    path_manager.add_task(MoveType::FORWARD_CM, 60);
    path_manager.add_task(MoveType::TURN_RIGHT_DEG, 90);
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "B3"); //
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "B2"); //
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "B1"); //
    path_manager.add_task(MoveType::TURN_LEFT_DEG, 180);
    path_manager.add_task(MoveType::STOP, 0, "C1"); //
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "C2"); //
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "C3"); //
    path_manager.add_task(MoveType::FORWARD_CM, 35);
    path_manager.add_task(MoveType::TURN_RIGHT_DEG, 90);
    path_manager.add_task(MoveType::FORWARD_CM, 60);
    path_manager.add_task(MoveType::TURN_RIGHT_DEG, 90);
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "D3"); //
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "D2"); //
    path_manager.add_task(MoveType::FORWARD_CM, 30);
    path_manager.add_task(MoveType::STOP, 0, "D1"); //
    path_manager.add_task(MoveType::TURN_LEFT_DEG, 180);
    path_manager.add_task(MoveType::FORWARD_CM, 100);
}
