#include "robot_firmware/robot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
namespace robot_firmware
{
RobotInterface::RobotInterface()
{
        
}

RobotInterface::~RobotInterface()
{
    if(mcu_.IsOpen())
    {
        try
        {
            mcu_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), 
            "Something went wrong while closing the connection with seriel port");
        }
        
    }
}

// on_init()
// -----------------------------------------------------------------------------
// ros2_control tarafından donanım arayüzü oluşturulurken çağrılır.
//
// URDF/Xacro içerisindeki <ros2_control> bölümünde tanımlanan donanım ve joint
// bilgileri ros2_control tarafından HardwareInfo yapısına dönüştürülür ve
// bu fonksiyona hardware_info olarak gönderilir.
//
// Öncelikle base class'ın on_init() fonksiyonu çağrılarak bu bilgiler
// SystemInterface içerisinde işlenir ve info_ değişkenine aktarılır.
//
// Daha sonra info_ üzerinden:
//   - Seri port bilgisi       -> info_.hardware_parameters
//   - Joint bilgileri         -> info_.joints
//   alınarak RobotInterface içerisinde kullanılmak üzere hazırlanır.
//
// Bu aşamada henüz STM32 ile haberleşme başlatılmaz ve motora komut gönderilmez.
// Sadece donanım arayüzünün başlangıç yapılandırması yapılır.
CallbackReturn RobotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if(result != CallbackReturn::SUCCESS)
    {
        return result;
    }
    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range & e)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), "No serial port provided!");
        return CallbackReturn::FAILURE;
    }
    
    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    last_run_ = rclcpp::Clock().now();
    return CallbackReturn::SUCCESS;
}


// export_state_interfaces()
// -----------------------------------------------------------------------------
// Robotun durum (state) bilgilerini ros2_control'a tanıtır.
//
// Her joint için:
//   - Position state
//   - Velocity state
// oluşturulur.
//
// info_.joints içerisindeki bilgiler URDF/Xacro'daki <ros2_control>
// tanımından gelir.
//
// Her StateInterface doğrudan ilgili C++ değişkeninin adresine bağlanır.
// Böylece read() fonksiyonu içerisinde position_states_ veya
// velocity_states_ güncellendiğinde, ros2_control da güncel değeri görür.
//
// Örneğin:
//
// left_wheel_joint
//      ├── position  → position_states_[0]
//      └── velocity  → velocity_states_[0]
//
// right_wheel_joint
//      ├── position  → position_states_[1]
//      └── velocity  → velocity_states_[1]
//
// Bu fonksiyon veri okumaz veya STM32 ile haberleşmez.
// Sadece donanımda okunacak state değerleri ile ros2_control arasında
// bağlantı (interface) oluşturur.
// -----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>RobotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i< info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &position_states_[i]));
        
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }
    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> RobotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i = 0; i< info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }
    return command_interfaces;
}

CallbackReturn RobotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotInterface"), "Starting robot hardware...");
    velocity_commands_ = {0.0, 0.0};
    position_states_ = {0.0, 0.0};
    velocity_states_ = {0.0, 0.0};

    try
    {
        mcu_.Open(port_);
        mcu_.SetBaudRate(LibSerial::BaudRate::BAUD_1152000);
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), 
        "Someting went wrong while interacting with port"<< port_);
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotInterface"), "Hardware started, ready to take commands");
}

CallbackReturn RobotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("RobotInterface"), "Stopping robot hardware...");
    if(mcu_.IsOpen())
    {
        try
        {
            mcu_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotInterface"), 
            "Someting went wrong while closing the port"<< port_);
            return CallbackReturn::FAILURE;
        }
    }
}

hardware_interface::return_type RobotInterface::read(const rclcpp::Time & , const rclcpp::Duration &)
{
    if(mcu_.IsDataAvailable())
    {   
        auto dt = (rclcpp::Clock().now() - last_run_).seconds();
        std::string message;
        std::string res;
        mcu_.ReadLine(message);
        std::stringstream ss(message);
        int multiplier = 1;
        while(std::getline(ss, res, ','))
        {
            multiplier = res.at(1) == 'p' ? 1 : -1;
            if(res.at(0) == 'r')
            {
                velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
                position_states_.at(0) += velocity_states_.at(0) * dt;
            }
            else if (res.at(0)=='l')
            {
                velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
                position_states_.at(1) += velocity_states_.at(1) * dt;
            }
        }
        last_run_= rclcpp::Clock().now();
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotInterface::write(const rclcpp::Time & , const rclcpp::Duration &)
{
    std::stringstream message_stream;
    char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
    char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
    std::string compansate_zeros_right = "";
    std::string compansate_zeros_left = "";

    if(std::abs(velocity_commands_.at(0)) < 10.0)
    {
        compansate_zeros_right = "0";
    }
    else
    {
        compansate_zeros_right = "";
    }

    if(std::abs(velocity_commands_.at(1)) < 10.0)
    {
        compansate_zeros_left = "0";
    }
    else
    {
        compansate_zeros_left = "";
    }

    message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign
        << compansate_zeros_right << std::abs(velocity_commands_.at(0))
        << "l" << left_wheel_sign << compansate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";
    
    try
    {
        mcu_.Write(message_stream.str());
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("RobotInterface"), "Something went wrong while sending message " 
            << message_stream.str() << " on the port" << port_);
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_firmware::RobotInterface, hardware_interface::SystemInterface);