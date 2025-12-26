

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>

namespace argos {

   class ControllerBug1 : public CCI_Controller {

   public:
      ControllerBug1() {}
      virtual ~ControllerBug1() {}

      virtual void Init(TConfigurationNode& t_tree) override;
      virtual void ControlStep() override;
      virtual void Reset() override {}
      virtual void Destroy() override {}

   private:
      /* Sensors and Actuators */
      CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
      CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;
      CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
      CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
      CCI_PiPuckSystemSensor* m_pcSystem = nullptr;
      CCI_PositioningSensor* m_pcPositioning = nullptr;

      /* Target information */
      CVector3 m_cTargetPosition;
      Real threshold_distance;

      /* Bug1 algorithm states */
      enum class EBug1State {
         ALIGN_TO_TARGET,
         MOVE_STRAIGHT,
         FOLLOW_OBSTACLE,
         GO_TO_BEST_POINT,
         FINISHED
      };
      
      EBug1State m_eState; 
      Real m_fSensorNoiseThreshold;
      bool m_bCalibrated;
      /* Bug1 specific tracking variables */
      CVector2 m_cHitPoint;    // הנקודה בה פגשנו את המכשול לראשונה
      CVector2 m_cBestPoint;   // הנקודה הכי קרובה ליעד שנמצאה במהלך ההקפה
      Real     m_fBestDist;    // המרחק המינימלי שנמדד מהיעד
      bool     m_bCompletedLoop;
      int      m_nStepsInFollow; // מונה צעדים למניעת עזיבה מוקדמת של ה-HitPoint

      /* Helper functions for navigation (Logic from your Bug2) */
      void AlignToTarget();
      void MoveStraight();
      void FollowObstacle();
      void GoToBestPoint();
      
      /* Sensing helpers (Mirroring your Bug2 functionality) */
      bool isObstacleDetected();
      bool obstacleToMyLeft();
      bool IsTargetReached();
   };
}
