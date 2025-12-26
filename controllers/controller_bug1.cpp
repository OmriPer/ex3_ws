#include "controller_bug1.hpp"
#include <limits>
#include <algorithm>

namespace argos {

   void ControllerBug1::Init(TConfigurationNode& t_tree) {
      /* Actuators and Sensors */
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();

      /* Target Position from XML */
      TConfigurationNode& tTargetNode = GetNode(t_tree, "target_position");
      Real targetX, targetY;
      GetNodeAttribute(tTargetNode, "x", targetX);
      GetNodeAttribute(tTargetNode, "y", targetY);
      m_cTargetPosition.Set(targetX, targetY, 0.0);

      /* Variables Initialization */
      m_eState = EBug1State::ALIGN_TO_TARGET;
      m_fBestDist = std::numeric_limits<Real>::max();
      m_bCompletedLoop = false;
      m_nStepsInFollow = 0;
      threshold_distance = 0.1; // סף זווית מעט יותר גמיש כדי להבטיח תחילת נסיעה
   }

   void ControllerBug1::ControlStep() {
      // הדפסת המצב הנוכחי ללוג כדי שנדע איפה הוא תקוע
      LOG << "Current State: " << (int)m_eState << std::endl;

      switch(m_eState) {
         case EBug1State::ALIGN_TO_TARGET:  AlignToTarget();  break;
         case EBug1State::MOVE_STRAIGHT:    MoveStraight();    break;
         case EBug1State::FOLLOW_OBSTACLE:  FollowObstacle();  break;
         case EBug1State::GO_TO_BEST_POINT: GoToBestPoint(); break;
         case EBug1State::FINISHED:         m_pcWheels->SetLinearVelocity(0,0); break;
      }
   }

   void ControllerBug1::AlignToTarget() {
      if(IsTargetReached()) { m_eState = EBug1State::FINISHED; return; }

      CVector2 cPos(m_pcPositioning->GetReading().Position.GetX(), m_pcPositioning->GetReading().Position.GetY());
      CVector2 cTarget(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
      CVector2 toTarget = cTarget - cPos;

      CRadians cZ, cY, cX;
      m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
      cZ.UnsignedNormalize();
      CRadians target_angle = ATan2(toTarget.GetY(), toTarget.GetX());
      target_angle.UnsignedNormalize();

      // חישוב שגיאת זווית בצורה חכמה
      Real fAngleError = (target_angle - cZ).SignedNormalize().GetValue();

      if(Abs(fAngleError) > threshold_distance) {
         // סיבוב על המקום כמו ב-Bug2
         if(fAngleError > 0) m_pcWheels->SetLinearVelocity(-0.05, 0.05);
         else m_pcWheels->SetLinearVelocity(0.05, -0.05);
      } else {
         LOG << "Aligned! Moving Straight..." << std::endl;
         m_eState = EBug1State::MOVE_STRAIGHT;
      }
   }

   void ControllerBug1::MoveStraight() {
      if(IsTargetReached()) { m_eState = EBug1State::FINISHED; return; }

      if(isObstacleDetected()) {
         LOG << "Obstacle Detected! Entering Follow State." << std::endl;
         CVector2 cPos(m_pcPositioning->GetReading().Position.GetX(), m_pcPositioning->GetReading().Position.GetY());
         CVector2 cTarget(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
         
         m_cHitPoint = cPos;
         m_cBestPoint = cPos;
         m_fBestDist = (cTarget - cPos).Length();
         m_nStepsInFollow = 0;
         m_bCompletedLoop = false;
         m_eState = EBug1State::FOLLOW_OBSTACLE;
      } else {
         m_pcWheels->SetLinearVelocity(0.1, 0.1); // מהירות נסיעה קבועה
      }
   }

   void ControllerBug1::FollowObstacle() {
      m_nStepsInFollow++;
      CVector2 cPos(m_pcPositioning->GetReading().Position.GetX(), m_pcPositioning->GetReading().Position.GetY());
      CVector2 cTarget(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());

      // עדכון הנקודה הטובה ביותר (הקרובה ביותר ליעד)
      Real d = (cTarget - cPos).Length();
      if(d < m_fBestDist) {
         m_fBestDist = d;
         m_cBestPoint = cPos;
      }

      // לוגיקת עקיבת קיר בדיוק כמו ב-Bug2 שלך
      if (isObstacleDetected()) {
         m_pcWheels->SetLinearVelocity(-0.05, 0.05); // פנייה ימינה (מכשול מלפנים)
      } else if (obstacleToMyLeft()) {
         m_pcWheels->SetLinearVelocity(0.1, 0.1);    // נסיעה קדימה (קיר משמאל)
      } else {
         m_pcWheels->SetLinearVelocity(0.05, 0.1);   // חיפוש קיר (פנייה שמאלה)
      }

      // בדיקת השלמת סיבוב מלא (חזרה ל-HitPoint)
      if(m_nStepsInFollow > 200 && (cPos - m_cHitPoint).Length() < 0.1) {
         LOG << "Loop Completed! Going to Best Point." << std::endl;
         m_eState = EBug1State::GO_TO_BEST_POINT;
      }
   }

   void ControllerBug1::GoToBestPoint() {
      CVector2 cPos(m_pcPositioning->GetReading().Position.GetX(), m_pcPositioning->GetReading().Position.GetY());
      
      // ממשיך לעקוב אחרי המכשול עד שמגיע לנקודה הכי טובה
      if((cPos - m_cBestPoint).Length() < 0.08) {
         LOG << "At Best Point! Re-aligning to Target." << std::endl;
         m_eState = EBug1State::ALIGN_TO_TARGET;
      } else {
         if (isObstacleDetected()) m_pcWheels->SetLinearVelocity(-0.05, 0.05);
         else if (obstacleToMyLeft()) m_pcWheels->SetLinearVelocity(0.1, 0.1);
         else m_pcWheels->SetLinearVelocity(0.05, 0.1);
      }
   }

   /* Sensing Logic - Using thresholds instead of raw range for stability */
   bool ControllerBug1::isObstacleDetected() {
      bool detected = false;
      m_pcRangefinders->Visit([&detected](const auto& sensor) {
         if (sensor.Label == 0 || sensor.Label == 7) {
            // אם הפרוקסימיטי מעל 0.1, יש מכשול מלפנים
            if (sensor.Proximity > 0.1) detected = true;
         }
      });
      return detected;
   }

   bool ControllerBug1::obstacleToMyLeft() {
      bool detected = false;
      m_pcRangefinders->Visit([&detected](const auto& sensor) {
         if (sensor.Label == 5 || sensor.Label == 6) {
            // אם הפרוקסימיטי מעל 0.1, יש קיר משמאל
            if (sensor.Proximity > 0.1) detected = true;
         }
      });
      return detected;
   }

   bool ControllerBug1::IsTargetReached() {
      // בדיקת יעד באמצעות המצלמה כמו ב-Bug2 שלך
      const auto& blobs = m_pcCamera->GetReadings();
      for (const auto& blob : blobs.BlobList) {
         if (blob->Color == CColor::CYAN) return true;
      }
      return false;
   }

   REGISTER_CONTROLLER(ControllerBug1, "controller_bug1");
}