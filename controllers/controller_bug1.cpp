#include "controller_bug1.hpp"

namespace argos {
   
   /****************************************/
   /****************************************/
   
   void ControllerBug1::Init(TConfigurationNode& t_tree) {
      /* Get the actuators and sensors */
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();
      
      // read target position from .argos file into m_cTargetPosition
      TConfigurationNode& tTargetNode = GetNode(t_tree, "target_position");
      Real targetX, targetY;
      GetNodeAttribute(tTargetNode, "x", targetX);
      GetNodeAttribute(tTargetNode, "y", targetY);
      m_cTargetPosition.Set(targetX, targetY, 0.0);

      /* your Init code here */
   }
   
   void ControllerBug1::ControlStep() {
      /* your ControlStep code here */
   }

   /****************************************/
   /****************************************/
   
   REGISTER_CONTROLLER(ControllerBug1, "controller_bug1");
   
}