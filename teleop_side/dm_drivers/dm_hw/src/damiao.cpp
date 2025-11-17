#include "dm_hw/damiao.h"
#include <signal.h>
#include <geometry_msgs/Twist.h>
namespace damiao
{

    Limit_param limit_param[Num_Of_Motor] =
        {
            {12.5, 30, 10},  // DM4310
            {12.5, 50, 10},  // DM4310_48V
            {12.5, 10, 28},  // DM4340
            {12.5, 10, 28},  // DM4340_48V
            {12.5, 45, 12},  // DM6006
            {12.5, 45, 20},  // DM8006
            {12.5, 45, 54},  // DM8009
            {12.5, 25, 200}, // DM10010L
            {12.5, 20, 200}, // DM10010
            {12.5, 280, 1},  // DMH3510
            {12.5, 45, 10},  // DMH6215
            {12.5, 45, 10}   // DMG6220
    };

    Motor::Motor(DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id)
        : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type)
    {
        this->limit_param = damiao::limit_param[Motor_Type];
    }

    void Motor::receive_data(float q, float dq, float tau)
    {
        this->state_q = q;
        this->state_dq = dq;
        this->state_tau = tau;
    }

    void Motor::set_param(int key, float value)
    {
        ValueType v{};
        v.value.floatValue = value;
        v.isFloat = true;
        param_map[key] = v;
    }

    void Motor::set_param(int key, uint32_t value)
    {
        ValueType v{};
        v.value.uint32Value = value;
        v.isFloat = false;
        param_map[key] = v;
    }

    float Motor::get_param_as_float(int key) const
    {
        auto it = param_map.find(key);
        if (it != param_map.end())
        {
            if (it->second.isFloat)
            {
                return it->second.value.floatValue;
            }
            else
            {
                return 0;
            }
        }
        return 0;
    }

    uint32_t Motor::get_param_as_uint32(int key) const
    {
        auto it = param_map.find(key);
        if (it != param_map.end())
        {
            if (!it->second.isFloat)
            {
                return it->second.value.uint32Value;
            }
            else
            {
                return 0;
            }
        }
        return 0;
    }

    bool Motor::is_have_param(int key) const
    {
        return param_map.find(key) != param_map.end();
    }

    /******一个port-ACM*，一个Motor_Control**********************/
    Motor_Control::Motor_Control(std::string serial_port, int seial_baud, std::unordered_map<int, DmActData> *data_ptr)
        : data_ptr_(data_ptr)
    {

        for (auto it = data_ptr_->begin(); it != data_ptr_->end(); ++it)
        {                                                                                         // 遍历该port下的所有电机
            Motor *motor = new Motor(it->second.motorType, it->second.can_id, it->second.mst_id); // 假设Motor的构造函数不需要参数
            addMotor(motor);
        }

        serial_.setPort(serial_port);
        serial_.setBaudrate(seial_baud);
        serial_.setFlowcontrol(serial::flowcontrol_none);
        serial_.setParity(serial::parity_none); // default is parity_none
        serial_.setStopbits(serial::stopbits_one);
        serial_.setBytesize(serial::eightbits);
        serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
        serial_.setTimeout(time_out);
        serial_.open();
        usleep(1000000); // 1s

        enable(); // 使能该接口下的所有电机
        // usleep(1000000);//1s
        rec_thread = std::thread(boost::bind(&Motor_Control::get_motor_data_thread, this));
        // rec_thread = std::make_unique<std::thread>(boost::bind(&Motor_Control::get_motor_data_thread, this));
        std::cerr << "Motor_Control init success!" << std::endl;
    }

    Motor_Control::~Motor_Control()
    {
        std::cerr << "enter ~Motor_Control()" << std::endl;

        for (const auto &pair : motors)
        {
            Motor_id id = pair.first;
            // std::cerr<<"id: "<<id<<std::endl;
            control_mit(*motors[id], 0, 0.3, 0, 0, 0);
        }
        stop_thread_ = true;
        // if (serial_.isOpen())
        //  {
        //    serial_.close();
        //  }
        if (rec_thread.joinable())
        {
            rec_thread.join();
        }
        if (serial_.isOpen())
        {
            serial_.close();
        }
    }

    void Motor_Control::enable()
    {
        for (auto &it : motors)
        {
            for (int j = 0; j < 20; j++)
            {
                control_cmd(it.second->GetSlaveId(), 0xFC);
                usleep(2000);
            }
        }
    }

    // 读电机反馈命令
    void Motor_Control::refresh_motor_status(const Motor &motor)
    {
        uint32_t id = 0x7FF;
        uint8_t can_low = motor.GetSlaveId() & 0xff;         // id low 8 bit
        uint8_t can_high = (motor.GetSlaveId() >> 8) & 0xff; // id high 8 bit
        std::array<uint8_t, 8> data_buf = {can_low, can_high, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00};
        send_data.modify(id, data_buf.data());
        serial_.write((uint8_t *)&send_data, sizeof(can_send_frame));
    }

    void Motor_Control::disable()
    {
        for (auto &it : motors)
        {
            for (int j = 0; j < 20; j++)
                control_cmd(it.second->GetSlaveId(), 0xFD);
            {
                usleep(2000);
            }
        }
    }

    void Motor_Control::set_zero_position()
    {
        for (auto &it : motors)
        {
            for (int j = 0; j < 20; j++)
                control_cmd(it.second->GetSlaveId(), 0xFE);
            {
                usleep(2000);
            }
        }
    }

    void Motor_Control::control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
    {
        // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
        static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t
        {
            float span = xmax - xmin;
            float data_norm = (x - xmin) / span;
            uint16_t data_uint = data_norm * ((1 << bits) - 1);
            return data_uint;
        };
        Motor_id id = DM_Motor.GetSlaveId();
        if (motors.find(id) == motors.end())
        {
            throw std::runtime_error("Motor_Control id not found");
        }
        auto &m = motors[id];
        uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
        uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
        Limit_param limit_param_cmd = m->get_limit_param();
        uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
        uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX, limit_param_cmd.DQ_MAX, 12);
        uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

        std::array<uint8_t, 8> data_buf{};
        data_buf[0] = (q_uint >> 8) & 0xff;
        data_buf[1] = q_uint & 0xff;
        data_buf[2] = dq_uint >> 4;
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
        data_buf[4] = kp_uint & 0xff;
        data_buf[5] = kd_uint >> 4;
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
        data_buf[7] = tau_uint & 0xff;

        send_data.modify(id, data_buf.data());
        serial_.write((uint8_t *)&send_data, sizeof(can_send_frame));
    }

    void Motor_Control::control_pos_vel(Motor &DM_Motor, float pos, float vel)
    {
        Motor_id id = DM_Motor.GetSlaveId();
        if (motors.find(id) == motors.end())
        {
            throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found");
        }
        std::array<uint8_t, 8> data_buf{};
        memcpy(data_buf.data(), &pos, sizeof(float));
        memcpy(data_buf.data() + 4, &vel, sizeof(float));
        id += POS_MODE;
        send_data.modify(id, data_buf.data());
        serial_.write(reinterpret_cast<uint8_t *>(&send_data), sizeof(can_send_frame));
    }

    void Motor_Control::control_vel(Motor &DM_Motor, float vel)
    {
        Motor_id id = DM_Motor.GetSlaveId();
        if (motors.find(id) == motors.end())
        {
            throw std::runtime_error("VEL ERROR : id not found");
        }
        std::array<uint8_t, 8> data_buf = {0};
        memcpy(data_buf.data(), &vel, sizeof(float));
        id = id + SPEED_MODE;
        send_data.modify(id, data_buf.data());
        serial_.write((uint8_t *)&send_data, sizeof(can_send_frame));
    }

    void Motor_Control::receive_param()
    {
        // serial_->recv((uint8_t*)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

        if (receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
        {
            auto &data = receive_data.canData;
            if (data[2] == 0x33 or data[2] == 0x55)
            {
                uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
                uint8_t RID = data[3];
                if (motors.find(slaveID) == motors.end())
                {
                    // can not found motor id
                    return;
                }
                if (is_in_ranges(RID))
                {
                    uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                    motors[slaveID]->set_param(RID, data_uint32);
                }
                else
                {
                    float data_float = uint8_to_float(data + 4);
                    motors[slaveID]->set_param(RID, data_float);
                }
            }
            return;
        }
    }

    /**
     * @brief add motor to class 添加电机
     * @param DM_Motor : motor object 电机对象
     */
    void Motor_Control::addMotor(Motor *DM_Motor)
    {
        motors.insert({DM_Motor->GetSlaveId(), DM_Motor});
        motors.insert({DM_Motor->GetMasterId(), DM_Motor});
    }

    /*
     * @description: read motor register param 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
     * @param DM_Motor: motor object 电机对象
     * @param RID: register id 寄存器ID  example: damiao::UV_Value
     * @return: motor param 电机参数 如果没查询到返回的参数为0
     */
    float Motor_Control::read_motor_param(Motor &DM_Motor, uint8_t RID)
    {
        uint32_t id = DM_Motor.GetSlaveId();
        uint8_t can_low = id & 0xff;
        uint8_t can_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00};
        send_data.modify(0x7FF, data_buf.data());
        serial_.write((uint8_t *)&send_data, sizeof(can_send_frame));
        for (uint8_t i = 0; i < max_retries; i++)
        {
            usleep(retry_interval);
            receive_param();
            if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
            {
                if (is_in_ranges(RID))
                {
                    return float(motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID));
                }
                else
                {
                    return motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID);
                }
            }
        }

        return 0;
    }

    /*
     * @description: switch control mode 切换电机控制模式
     * @param DM_Motor: motor object 电机对象
     * @param mode: control mode 控制模式 like:damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
     */
    bool Motor_Control::switchControlMode(Motor &DM_Motor, Control_Mode mode)
    {
        uint8_t write_data[4] = {(uint8_t)mode, 0x00, 0x00, 0x00};
        uint8_t RID = 10;
        write_motor_param(DM_Motor, RID, write_data);
        if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
        {
            return false;
        }
        for (uint8_t i = 0; i < max_retries; i++)
        {
            usleep(retry_interval);
            receive_param();
            if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
            {
                return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == mode;
            }
        }
        return false;
    }

    /*
     * @description: change motor param 修改电机内部寄存器参数 具体寄存器列表请参考达妙手册
     * @param DM_Motor: motor object 电机对象
     * @param RID: register id 寄存器ID
     * @param data: param data 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
     * @return: bool true or false  是否修改成功
     */
    bool Motor_Control::change_motor_param(Motor &DM_Motor, uint8_t RID, float data)
    {
        if (is_in_ranges(RID))
        {
            // 居然传进来的是整型的范围 救一下
            uint32_t data_uint32 = float_to_uint32(data);
            uint8_t *data_uint8;
            data_uint8 = (uint8_t *)&data_uint32;
            write_motor_param(DM_Motor, RID, data_uint8);
        }
        else
        {
            // is float
            uint8_t *data_uint8;
            data_uint8 = (uint8_t *)&data;
            write_motor_param(DM_Motor, RID, data_uint8);
        }
        if (motors.find(DM_Motor.GetSlaveId()) == motors.end())
        {
            return false;
        }
        for (uint8_t i = 0; i < max_retries; i++)
        {
            usleep(retry_interval);
            receive_param();
            if (motors[DM_Motor.GetSlaveId()]->is_have_param(RID))
            {
                if (is_in_ranges(RID))
                {
                    return motors[DM_Motor.GetSlaveId()]->get_param_as_uint32(RID) == float_to_uint32(data);
                }
                else
                {
                    return fabsf(motors[DM_Motor.GetSlaveId()]->get_param_as_float(RID) - data) < 0.1f;
                }
            }
        }
        return false;
    }

    /*
     * @description: save all param to motor flash 保存电机的所有参数到flash里面
     * @param DM_Motor: motor object 电机对象
     * 电机默认参数不会写到flash里面，需要进行写操作
     */
    void Motor_Control::save_motor_param(Motor &DM_Motor)
    {
        disable();
        uint32_t id = DM_Motor.GetSlaveId();
        uint8_t id_low = id & 0xff;
        uint8_t id_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
        send_data.modify(0x7FF, data_buf.data());
        serial_.write((uint8_t *)&send_data, sizeof(can_send_frame));
        usleep(100000); // 100ms wait for save
    }

    /*
     * @description: change motor limit param 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
     * @param DM_Motor: motor object 电机对象
     * @param P_MAX: position max 位置最大值
     * @param Q_MAX: velocity max 速度最大值
     * @param T_MAX: torque max 扭矩最大值
     */
    void Motor_Control::changeMotorLimit(Motor &DM_Motor, float P_MAX, float Q_MAX, float T_MAX)
    {
        limit_param[DM_Motor.GetMotorType()] = {P_MAX, Q_MAX, T_MAX};
    }

    void Motor_Control::control_cmd(Motor_id id, uint8_t cmd)
    {
        std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
        send_data.modify(id, data_buf.data());
        serial_.write((uint8_t *)&send_data, sizeof(can_send_frame));
    }

    void Motor_Control::write_motor_param(Motor &DM_Motor, uint8_t RID, const uint8_t data[4])
    {
        uint32_t id = DM_Motor.GetSlaveId();
        uint8_t can_low = id & 0xff;
        uint8_t can_high = (id >> 8) & 0xff;
        std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
        data_buf[4] = data[0];
        data_buf[5] = data[1];
        data_buf[6] = data[2];
        data_buf[7] = data[3];
        send_data.modify(0x7FF, data_buf.data());

        serial_.write((uint8_t *)&send_data, sizeof(can_send_frame));
    }

    void Motor_Control::write()
    {
        for (const auto &m : *data_ptr_)
        {
            int motor_id = m.first; // 这里指的是can_id
            if (motors.find(motor_id) == motors.end())
            {
                throw std::runtime_error("read ERROR : Motor_Control id not found");
            }
            auto &it = motors[motor_id];

            // std::cerr << "write" << std::endl;
            // std::cerr << "motor_id: " << motor_id << std::endl;
            // std::cerr << "pos: " << m.second.cmd_pos << " vel: " << m.second.cmd_vel << " effort: " << m.second.cmd_effort << std::endl;
            // std::cerr << "kp: " << m.second.kp << " kd: " << m.second.kd << std::endl;
            // std::cerr << "finishe write" << std::endl;
            control_mit(*it, m.second.kp, m.second.kd, m.second.cmd_pos, m.second.cmd_vel, m.second.cmd_effort); // control_mit(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
        }
    }

    void Motor_Control::read(ros::Publisher pub)
    {
        geometry_msgs::Twist motor_data_msg;
        std::array<double, 3> motor_positions;
        motor_positions.fill(0.0);

        for (auto &m : *data_ptr_)
        {
            int motor_id = m.first; // 这里指的是can_id
            if (motors.find(motor_id) == motors.end())
            {
                throw std::runtime_error("read ERROR : Motor_Control id not found");
            }
            auto &it = motors[motor_id];

            // m.second.pos = it->Get_Position();
            // m.second.vel = it->Get_Velocity();
            // m.second.effort = it->Get_tau();

            m.second.pos = it->Get_Position();
            // std::cerr << " position: " << position << std::endl;

            // std::cerr << "MotorType: " << motor_id << std::endl;
            // std::cerr << "pos: " << m.second.pos << " vel: " << m.second.vel << " effort: " << m.second.effort << std::endl;

            std::cerr << "====================" << std::endl;
            std::cerr << m.first << std::endl;
            std::cerr << "can_id " << m.second.can_id << std::endl;
            // std::cerr << "cmd_effort " << m.second.cmd_effort << std::endl;
            // std::cerr << "cmd_pos " << m.second.cmd_pos << std::endl;
            // std::cerr << "cmd_vel " << m.second.cmd_vel << std::endl;
            // std::cerr << "effort " << m.second.effort << std::endl;
            // std::cerr << "kd " << m.second.kd << std::endl;
            // std::cerr << "kp " << m.second.kp << std::endl;
            // std::cerr << "motorType " << m.second.motorType << std::endl;
            std::cerr << "mst_id " << m.second.mst_id << std::endl;
            // std::cerr << "name " << m.second.name << std::endl;
            std::cerr << "pos " << m.second.pos << std::endl;
            std::cerr << "vel " << m.second.vel << std::endl;

            motor_positions[motor_id - 1] = m.second.pos;
        }
        motor_data_msg.linear.x = motor_positions[1];
        motor_data_msg.linear.y = motor_positions[2];
        motor_data_msg.angular.z = motor_positions[0];
        pub.publish(motor_data_msg);
    }

    void Motor_Control::get_motor_data_thread()
    {
        while (!stop_thread_)
        {
            try
            {
                serial_.read((uint8_t *)&receive_data, sizeof(CAN_Receive_Frame));
                if (receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
                {
                    static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float
                    {
                        float span = xmax - xmin;
                        float data_norm = float(x) / ((1 << bits) - 1);
                        float data = data_norm * span + xmin;
                        return data;
                    };

                    auto &data = receive_data.canData;

                    uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
                    uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
                    uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];

                    if (motors.find(receive_data.canId) == motors.end())
                    {
                        return;
                    }
                    auto m = motors[receive_data.canId];
                    Limit_param limit_param_receive = m->get_limit_param();
                    float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                    float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                    float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                    m->receive_data(receive_q, receive_dq, receive_tau);
                }
                else
                {
                    // std::cerr<<"filed"<<std::endl;
                }
            }
            catch (const serial::SerialException &e)
            {
                std::cerr << "Serial exception: " << e.what() << std::endl;
            }
            catch (const serial::IOException &e)
            {
                std::cerr << "IO exception: " << e.what() << std::endl;
            }
        }
    }

}
