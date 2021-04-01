#ifndef __robot_localization_demo__imu_data__
#define __robot_localization_demo__imu_data__

#include <random>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>


/*!
 *  \addtogroup robot_localization_demo_imu
 *  @{
 */

//! Interactions with turtlesim and imu errors declarations 
//namespace robot_localization_demo {
namespace robot_localization_demo {
/** 
 *  @brief Control de errores, frecuencia e interacciones con turtlesim
 */
  class TurtleImuSim {
    public:
      /** 
      *  @brief Recibe los parametros de configuración del IMU
      *  @param node_handle Inicia el nodo para el IMU
      *  @param frequency Frecuencia de publicación del nodo
      *  @param error_vx_systematic Error sistematico en la velocidad lineal
      *  @param error_vx_random Error aleatorio en la velocidad lineal
      *  @param error_wz_systematic Error sistematico en la velocidad angular
      *  @param error_wz_random Error aleatorio en la velocidad angular
      *  @param error_yaw_systematic Error sistematico en la orientación
      *  @param error_yaw_random Error aleatorio en la orientación
      *  @param visualize Bandera de visualización del turtlesim
      */
      TurtleImuSim(ros::NodeHandle node_handle, double frequency, double error_vx_systematic, double error_vx_random,
          double error_wz_systematic, double error_wz_random, double error_yaw_systematic,
          double error_yaw_random, bool visualize=false);
      ~TurtleImuSim();
      void spin();

    private:
      //! Node object
      ros::NodeHandle node_handle_;
      //! Register subscriber to get desired pose commands
      ros::Subscriber turtle_pose_subscriber_;
      //! Register publisher to send complete imu data
      ros::Publisher turtle_imudata_publisher_;
      //! Register publisher to send rpy imu data (without global orientation)
      ros::Publisher turtle_imurpy_publisher_;

      double frequency_;
      std::default_random_engine random_generator_;
      std::normal_distribution<double> random_distribution_vx_;
      std::normal_distribution<double> random_distribution_wz_;
      std::normal_distribution<double> random_distribution_yaw_;
      bool visualize_;
      std::string visualization_turtle_name_;
      unsigned frame_sequence_;
      ros::Time cached_pose_timestamp_;
      turtlesim::Pose cached_pose_;
      void turtlePoseCallback(const turtlesim::PoseConstPtr & message);
      inline bool isVisualizationRequested() { return visualize_; };
      inline bool isVisualizationTurtleAvailable() { return visualization_turtle_name_ != ""; };
      void spawnAndConfigureVisualizationTurtle(const turtlesim::Pose & initial_pose);
      void moveVisualizationTurtle(const turtlesim::Pose & measurement);
  };

} // End namespace robot_localization_demo_imu 
/*! @} End of Doxygen Groups*/
#endif
