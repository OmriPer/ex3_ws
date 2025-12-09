#include "trajectory_loop_functions.hpp"

namespace argos {
	
	/****************************************/
	/****************************************/
	
	void CTrajectoryLoopFunctions::Init(TConfigurationNode& t_tree) {
		// parse the configuration file for min distance
		Real min_distance;
		GetNodeAttributeOrDefault(t_tree, "min_distance", min_distance, 0.1);
		this->MIN_DISTANCE_SQUARED = min_distance * min_distance;

		// create a waypoint vector for each Pi-Puck in the simulation
		// (with the initial position as first waypoint)
		CSpace::TMapPerType& pipucks = GetSpace().GetEntitiesByType("pipuck");
		for (auto& pipuck : pipucks) {
			CPiPuckEntity* pcPiPuck = any_cast<CPiPuckEntity*>(pipuck.second);
			m_tWaypoints[pcPiPuck] = std::vector<CVector3>();
			m_tWaypoints[pcPiPuck].push_back(pcPiPuck->GetEmbodiedEntity().GetOriginAnchor().Position);
		}
	}
	
	void CTrajectoryLoopFunctions::Reset() {
		// clear all waypoints and re-add the initial position
		CSpace::TMapPerType& pipucks = GetSpace().GetEntitiesByType("pipuck");
		for (auto& pipuck : pipucks) {
			CPiPuckEntity* pcPiPuck = any_cast<CPiPuckEntity*>(pipuck.second);
			m_tWaypoints[pcPiPuck].clear();
			m_tWaypoints[pcPiPuck].push_back(pcPiPuck->GetEmbodiedEntity().GetOriginAnchor().Position);
		}
	}
	
	bool CTrajectoryLoopFunctions::IsExperimentFinished() {
		return false;
	}
	
	void CTrajectoryLoopFunctions::PostStep() {
		/* Get the map of all pi-pucks from the space */
		CSpace::TMapPerType& pipucks = GetSpace().GetEntitiesByType("pipuck");
		/* Go through them */
		for (auto& pipuck : pipucks) {
			/* Create a pointer to the current pi-puck */
			CPiPuckEntity* pcPiPuck = any_cast<CPiPuckEntity*>(pipuck.second);
			/* Add the current position of the pi-puck if it's sufficiently far from the last */
			if(SquareDistance(pcPiPuck->GetEmbodiedEntity().GetOriginAnchor().Position,
			                  m_tWaypoints[pcPiPuck].back()) > MIN_DISTANCE_SQUARED) {
				m_tWaypoints[pcPiPuck].push_back(pcPiPuck->GetEmbodiedEntity().GetOriginAnchor().Position);
			}
		}
	}

		/****************************************/
		/****************************************/

	REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "trajectory_loop_functions");

}
