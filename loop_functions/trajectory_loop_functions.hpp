#pragma once

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include <argos3/core/simulator/space/space.h>

namespace argos {
    
    typedef std::map<CPiPuckEntity*, std::vector<CVector3>> TPiPuckWaypoints;
    
    class CTrajectoryLoopFunctions : public CLoopFunctions {
        
        public:
        
        CTrajectoryLoopFunctions() {}
        
        virtual ~CTrajectoryLoopFunctions() {}
        
        virtual bool IsExperimentFinished() override;
        
        virtual void Init(TConfigurationNode& t_tree) override;
        
        virtual void PostStep() override;
        
        virtual void Reset() override;
        
        TPiPuckWaypoints& GetWaypoints() {
            return m_tWaypoints;
        }
        
        private:
        
        TPiPuckWaypoints m_tWaypoints = {};
        
        Real MIN_DISTANCE_SQUARED;
    };
}
