#include "controller_bug1.hpp"

namespace argos {

   /****************************************/
   /****************************************/

   void ControllerBug1::Init(TConfigurationNode& t_tree) {
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();

      TConfigurationNode& tTargetNode = GetNode(t_tree, "target_position");
      Real targetX, targetY;
      GetNodeAttribute(tTargetNode, "x", targetX);
      GetNodeAttribute(tTargetNode, "y", targetY);
      m_cTargetPosition.Set(targetX, targetY, 0.0);

      // initialize variables
      m_fThresholdDistance = 0.05;
      m_fClosestDistanceToGoal = 1e9;
      m_nLeaveTimer = 0;
      m_fPrevDistToClosest = 1e9;
      m_bLeftHitPoint = false;
      m_eState = STATE_GO_TO_GOAL;

   }

   /****************************************/

   void ControllerBug1::ControlStep() {
      // update timer when switching states
      if(m_nLeaveTimer > 0) {
         m_nLeaveTimer--;
      }

      switch(m_eState) {
         case STATE_GO_TO_GOAL:
            GoToGoal();
            break;
         case STATE_FOLLOW_OBSTACLE:
            FollowObstacle();
            break;
         case STATE_GO_TO_CLOSEST_POINT:
            GoToClosestPoint();
            break;
         case STOP:
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            break;
      }
   }


// go to goal state
void ControllerBug1::GoToGoal() {
   // if target reached, stop
   if(IsTargetReached()) {
      m_eState = STOP;
      return;
   }

   if(IsObstacleDetected() && m_nLeaveTimer <= 0) {
      // initialize obstacle following variables for new obstacle
      m_bLeftHitPoint = false;               
      m_cObstacleHitPoint = m_pcPositioning->GetReading().Position;
      m_cClosestPointToGoal = m_cObstacleHitPoint;
      m_fClosestDistanceToGoal = DistanceToGoal(m_cObstacleHitPoint);
      
      // reset previous closest distance
      m_fPrevDistToClosest = 1e9; 

      m_eState = STATE_FOLLOW_OBSTACLE;
      return;
   }

   // move towards goal
   MoveTowardsTarget();
}

   // follow obstacle state
void ControllerBug1::FollowObstacle() {
   CVector3 currentPos = m_pcPositioning->GetReading().Position;
   Real distToGoal = DistanceToGoal(currentPos);

   if(distToGoal < m_fClosestDistanceToGoal) {
      m_fClosestDistanceToGoal = distToGoal;
      m_cClosestPointToGoal = currentPos;
   }

   Real distFromHit = (currentPos - m_cObstacleHitPoint).Length();

   if(!m_bLeftHitPoint) {
      if(distFromHit > 0.10) { 
         m_bLeftHitPoint = true;
         LOG << "Robot left hit point area of the current obstacle." << std::endl;
      }
   } 
   else {
      if(distFromHit < 0.1) { 
          LOG << "Full loop complete on current obstacle!" << std::endl;
          m_eState = STATE_GO_TO_CLOSEST_POINT;
          return;
      }
   }

   FollowWallOnly();
}

   // go to closest point state
   void ControllerBug1::GoToClosestPoint() {
      // set current position with positioning sensor
      CVector3 currentPos = m_pcPositioning->GetReading().Position;
      // variable for distance to closest point
      Real d = (currentPos - m_cClosestPointToGoal).Length();
   
      // if reached closest point to goal, switch to go to goal state
      if(d < 0.08) {
         m_nLeaveTimer = 150; // timer to avoid immediate re-detection of the obstacle
         m_eState = STATE_GO_TO_GOAL;
         return;
      }

      FollowWallOnly();
   }

   // follow wall helper function
   void ControllerBug1::FollowWallOnly() {
      if(ObstacleInFront()) {
         m_pcWheels->SetLinearVelocity(-0.05, 0.05); // turn left
      }
      else if(ObstacleOnLeft()) {
         m_pcWheels->SetLinearVelocity(0.1, 0.1);    // drive forward
      }
      else if(ObstacleOnRight()) {
         m_pcWheels->SetLinearVelocity(0.05, 0.02);  // turn right
      }
      else {
         m_pcWheels->SetLinearVelocity(0.03, 0.06);  // look for wall
      }
   }

   // obstacle detection helper function
   bool ControllerBug1::IsObstacleDetected() {
      return ObstacleInFront();
   }

   // obstacle position helper functions
   bool ControllerBug1::ObstacleInFront() {
      bool detected = false;
      // check front sensors (0 and 7)
      m_pcRangefinders->Visit([&detected](const auto& sensor) {
         if((sensor.Label == 0 || sensor.Label == 7) && 
            sensor.Proximity < std::get<3>(sensor.Configuration))
            detected = true;
      });
      return detected;
   }

   // obstacle position helper functions
   bool ControllerBug1::ObstacleOnLeft() {
      bool detected = false;
      m_pcRangefinders->Visit([&detected](const auto& sensor) {
         if((sensor.Label == 5 || sensor.Label == 6) && 
            sensor.Proximity < std::get<3>(sensor.Configuration))
            detected = true;
      });
      return detected;
   }

   // obstacle position helper functions
   bool ControllerBug1::ObstacleOnRight() {
      bool detected = false;
      m_pcRangefinders->Visit([&detected](const auto& sensor) {
         if((sensor.Label == 1 || sensor.Label == 2) && 
            sensor.Proximity < std::get<3>(sensor.Configuration))
            detected = true;
      });
      return detected;
   }

   // target reached helper function
   bool ControllerBug1::IsTargetReached() {
      const auto& blobs = m_pcCamera->GetReadings();
      for(const auto& blob : blobs.BlobList) {
         if(blob->Color == CColor::CYAN) return true;
      }
      return false;
   }

   // distance to goal helper function
   Real ControllerBug1::DistanceToGoal(const CVector3& pos) {
      return (pos - m_cTargetPosition).Length();
   }

   // move towards target helper function
   void ControllerBug1::MoveTowardsTarget() {
      MoveTowardsPoint(m_cTargetPosition);
   }

   // move towards point helper function
   void ControllerBug1::MoveTowardsPoint(const CVector3& point) {
      CRadians cZ, cY, cX;
      m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
      CVector3 pos = m_pcPositioning->GetReading().Position;
      Real angle = atan2(point.GetY() - pos.GetY(), point.GetX() - pos.GetX());
      CRadians targetAngle(angle);
      cZ.UnsignedNormalize();
      targetAngle.UnsignedNormalize();

      if(Abs(cZ.GetValue() - targetAngle.GetValue()) > m_fThresholdDistance) {
         m_pcWheels->SetLinearVelocity(0.05, -0.05);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.1, 0.1);
      }
   }

   REGISTER_CONTROLLER(ControllerBug1, "controller_bug1");
}