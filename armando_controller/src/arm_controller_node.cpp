#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ArmandoController : public rclcpp::Node
{
public:
  ArmandoController() : Node("arm_controller_node"), index_(0)
  {
  
    // Publisher node
    
    joint_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "position_controller/commands",
    10);
    
    // Trajectory controller
    
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "trajectory_controller/joint_trajectory",
    10);
    
    // Dichiarazione e lettura parametro controller_type
    this->declare_parameter<std::string>("controller_type", "position");
    controller_type_ = this->get_parameter("controller_type").as_string();
    
    
    
    
    auto client = this->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");

    // Aspetta che il servizio sia disponibile
    client->wait_for_service();

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    if (controller_type_ == "position") {
      request->activate_controllers  = {"position_controller"};
    } else if(controller_type_ == "trajectory"){
      request->deactivate_controllers  = {"position_controller"};
      request->activate_controllers  = {"trajectory_controller"};
    }else{
      RCLCPP_ERROR(this->get_logger(),
      "Valore del parametro 'controller_type' non valido: '%s'. "
      "Usa 'position' oppure 'trajectory'.",
      controller_type_.c_str());
  
      // Interrompe il costruttore ed evita di continuare col nodo
      rclcpp::shutdown();
      return;
    }
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->activate_asap = true;
    request->timeout.sec = 0;
    request->timeout.nanosec = 0;


    auto result_future = client->async_send_request(request);

    // Attendi e gestisci la risposta
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = result_future.get();
      if (result->ok) {
        RCLCPP_INFO(this->get_logger(), "✅ Controller activate successful.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Controller activate failed!");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "⚠️ Timeout waiting for activate_controller service response.");
    }
    
    
    
    // Timer
    timer_ = this->create_wall_timer(
        10000ms,
        std::bind(&ArmandoController::timer_callback, this));
    
    // Sottoscrizione al topic /joint_states
    
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(&ArmandoController::jointCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "arm_controller_node started and subscribed to /joint_states");

  }  


private:

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const 
  {
    RCLCPP_INFO(this->get_logger(), "Received joint positions:");

    // Scorriamo tutti i giunti presenti nel messaggio
	for (size_t i = 0; i < msg->position.size(); i++)
	{
	  std::string joint_name;

	  // Se il messaggio contiene il nome di questo giunto, usalo.
	  // Altrimenti, assegna un nome generico come "joint_0", "joint_1", ecc.
	  if (i < msg->name.size()) {
	    joint_name = msg->name[i];
	  } else {
	    joint_name = "joint_" + std::to_string(i);
	  }

	  // Stampa il nome e la posizione del giunto nel terminale
	  RCLCPP_INFO(
	    this->get_logger(),
	    "Joint %s has position %.3f",
	    joint_name.c_str(),
	    msg->position[i]
	  );
	}
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;
  
  
  void timer_callback()
  {
    if(controller_type_ == "position"){
    
        std::vector<std::vector<double>> commands = {
	    {0.9, 0.0, 0.0, 0.0},
	    {0.9, -0.6, 0.0, 0.0},
	    {0.9, -0.6, 0.3, 0.0},
	    {0.9, -0.6, 0.3, 0.1}
        };
    
        auto message = std_msgs::msg::Float64MultiArray();
	
        message.data = commands[index_];  // seleziona un set di valori
	
        RCLCPP_INFO(this->get_logger(),
            "Publishing positions: [%.2f, %.2f, %.2f, %.2f]",
            message.data[0], message.data[1], message.data[2], message.data[3]);
        joint_publisher_->publish(message);

        // passa al comando successivo oppure ferma il timer
        if (index_ < commands.size() - 1) {
          index_++;
        } else {
          RCLCPP_INFO(this->get_logger(), "Tutti i comandi pubblicati — stop del publisher.");
          timer_->cancel();  // Ferma il timer, subscriber continua
        }
        
    }
    else if(controller_type_ == "trajectory"){
        publish_trajectory_command();
        RCLCPP_INFO(this->get_logger(), "Traiettoria pubblicata — stop del timer.");
        timer_->cancel();  // Pubblica la traiettoria solo una volta
    }
    
  };
   
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  std::string controller_type_;
  size_t index_;
  
  
  void publish_trajectory_command()
  {
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();

    // Nomi dei giunti
    traj_msg.joint_names = {"j0", "j1", "j2", "j3"};
    
    // punti della traiettoria
    std::vector<std::vector<double>> trajectory_points = {
    {0.9, 0.0, 0.0, 0.0},
    {0.9, -0.6, 0.0, 0.0},
    {0.9, -0.6, 0.3, 0.0},
    {0.9, -0.6, 0.3, 0.1}
    };

    for(size_t i = 0; i < trajectory_points.size(); i++){
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = trajectory_points[i];
      point.velocities = {};     
      point.accelerations = {};  
      point.time_from_start = rclcpp::Duration::from_seconds((i + 1) * 1.0);
      traj_msg.points.push_back(point);
      
      RCLCPP_INFO(this->get_logger(),
        "Publishing trajectory positions: [%.2f, %.2f, %.2f, %.2f]",
        point.positions[0], point.positions[1],
        point.positions[2], point.positions[3]);
    }

    trajectory_publisher_->publish(traj_msg);
  }
  
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmandoController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

