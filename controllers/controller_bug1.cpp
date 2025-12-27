#include "controller_bug1.hpp"

namespace argos {

   /****************************************/
   /****************************************/

   void ControllerBug1::Init(TConfigurationNode& t_tree) {
      /* אקטיואטורים וסנסורים */
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();

      /* קריאת מטרה */
      TConfigurationNode& tTargetNode = GetNode(t_tree, "target_position");
      Real targetX, targetY;
      GetNodeAttribute(tTargetNode, "x", targetX);
      GetNodeAttribute(tTargetNode, "y", targetY);
      m_cTargetPosition.Set(targetX, targetY, 0.0);

      /* אתחול משתנים */
      m_fThresholdDistance = 0.05;
      m_fClosestDistanceToGoal = 1e9;
      m_nLeaveTimer = 0;
      m_fPrevDistToClosest = 1e9;
      m_bLeftHitPoint = false;
      m_eState = STATE_GO_TO_GOAL;

      LOG << "INIT DONE - Bug1 Classic Ready" << std::endl;
   }

   /****************************************/

   void ControllerBug1::ControlStep() {
      /* עדכון טיימר בכל צעד */
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

   /****************************************/

   void ControllerBug1::GoToGoal() {
      if(IsTargetReached()) {
         m_eState = STOP;
         return;
      }

      /* אם יש מכשול והטיימר נגמר - כנס למצב עקיפה */
      if(IsObstacleDetected() && m_nLeaveTimer <= 0) {
         m_bLeftHitPoint = false;
         m_cObstacleHitPoint = m_pcPositioning->GetReading().Position;
         m_cClosestPointToGoal = m_cObstacleHitPoint;
         m_fClosestDistanceToGoal = DistanceToGoal(m_cObstacleHitPoint);

         LOG << "Hit Obstacle! Starting full loop..." << std::endl;
         m_eState = STATE_FOLLOW_OBSTACLE;
         return;
      }

      MoveTowardsTarget();
   }

   void ControllerBug1::FollowObstacle() {
      CVector3 currentPos = m_pcPositioning->GetReading().Position;
      Real distToGoal = DistanceToGoal(currentPos);

      /* שמירת הנקודה הכי קרובה */
      if(distToGoal < m_fClosestDistanceToGoal) {
         m_fClosestDistanceToGoal = distToGoal;
         m_cClosestPointToGoal = currentPos;
      }

      Real distFromHit = (currentPos - m_cObstacleHitPoint).Length();

      /* לוגיקת סיום הקפה מבוססת מרחק */
      if(!m_bLeftHitPoint) {
         if(distFromHit > 0.15) { // התרחקנו מההתחלה
            m_bLeftHitPoint = true;
         }
      } 
      else {
         if(distFromHit < 0.08) { // חזרנו להתחלה
            LOG << "Full loop complete! Going to closest point..." << std::endl;
            m_eState = STATE_GO_TO_CLOSEST_POINT;
            return;
         }
      }

      FollowWallOnly();
   }

   void ControllerBug1::GoToClosestPoint() {
      CVector3 currentPos = m_pcPositioning->GetReading().Position;
      Real d = (currentPos - m_cClosestPointToGoal).Length();

      /* אם הגענו לנקודת היציאה - צא למטרה */
      if(d < 0.08) {
         LOG << "At best exit point! Goal bound." << std::endl;
         m_nLeaveTimer = 150; // זמן חסד להתרחקות מהקיר
         m_eState = STATE_GO_TO_GOAL;
         return;
      }

      FollowWallOnly();
   }

   void ControllerBug1::FollowWallOnly() {
      if(ObstacleInFront()) {
         m_pcWheels->SetLinearVelocity(-0.05, 0.05); // פנייה שמאלה
      }
      else if(ObstacleOnLeft()) {
         m_pcWheels->SetLinearVelocity(0.1, 0.1);    // נסיעה ישר
      }
      else if(ObstacleOnRight()) {
         m_pcWheels->SetLinearVelocity(0.05, 0.02);  // תיקון ימינה
      }
      else {
         m_pcWheels->SetLinearVelocity(0.03, 0.06);  // חיפוש קיר בקשת
      }
   }

   /****************************************/
   /* Helpers */
   /****************************************/

   bool ControllerBug1::IsObstacleDetected() {
      return ObstacleInFront();
   }

   bool ControllerBug1::ObstacleInFront() {
      bool detected = false;
      m_pcRangefinders->Visit([&detected](const auto& sensor) {
         if((sensor.Label == 0 || sensor.Label == 7) && 
            sensor.Proximity < std::get<3>(sensor.Configuration))
            detected = true;
      });
      return detected;
   }

   bool ControllerBug1::ObstacleOnLeft() {
      bool detected = false;
      m_pcRangefinders->Visit([&detected](const auto& sensor) {
         if((sensor.Label == 5 || sensor.Label == 6) && 
            sensor.Proximity < std::get<3>(sensor.Configuration))
            detected = true;
      });
      return detected;
   }

   bool ControllerBug1::ObstacleOnRight() {
      bool detected = false;
      m_pcRangefinders->Visit([&detected](const auto& sensor) {
         if((sensor.Label == 1 || sensor.Label == 2) && 
            sensor.Proximity < std::get<3>(sensor.Configuration))
            detected = true;
      });
      return detected;
   }

   bool ControllerBug1::IsTargetReached() {
      const auto& blobs = m_pcCamera->GetReadings();
      for(const auto& blob : blobs.BlobList) {
         if(blob->Color == CColor::CYAN) return true;
      }
      return false;
   }

   Real ControllerBug1::DistanceToGoal(const CVector3& pos) {
      return (pos - m_cTargetPosition).Length();
   }

   void ControllerBug1::MoveTowardsTarget() {
      MoveTowardsPoint(m_cTargetPosition);
   }

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