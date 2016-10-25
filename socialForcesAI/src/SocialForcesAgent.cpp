//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SocialForcesAgent.h"
#include "SocialForcesAIModule.h"
#include "SocialForces_Parameters.h"
// #include <math.h>

// #include "util/Geometry.h"

/// @file SocialForcesAgent.cpp
/// @brief Implements the SocialForcesAgent class.

#undef min
#undef max

#define AGENT_MASS 1.0f
#define PREFERED_SPEED 1.33f

using namespace Util;
using namespace SocialForcesGlobals;
using namespace SteerLib;

// #define _DEBUG_ENTROPY 1

SocialForcesAgent::SocialForcesAgent()
{
	_SocialForcesParams.sf_acceleration = sf_acceleration;
	_SocialForcesParams.sf_personal_space_threshold = sf_personal_space_threshold;
	_SocialForcesParams.sf_agent_repulsion_importance = sf_agent_repulsion_importance;
	_SocialForcesParams.sf_query_radius = sf_query_radius;
	_SocialForcesParams.sf_body_force = sf_body_force;
	_SocialForcesParams.sf_agent_body_force = sf_agent_body_force;
	_SocialForcesParams.sf_sliding_friction_force = sf_sliding_friction_force;
	_SocialForcesParams.sf_agent_b = sf_agent_b;
	_SocialForcesParams.sf_agent_a = sf_agent_a;
	_SocialForcesParams.sf_wall_b = sf_wall_b;
	_SocialForcesParams.sf_wall_a = sf_wall_a;
	_SocialForcesParams.sf_max_speed = sf_max_speed;

	_enabled = false;
}

SocialForcesAgent::~SocialForcesAgent()
{
	// std::cout << this << " is being deleted" << std::endl;
	/*
	if (this->enabled())
	{
	Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	// getSimulationEngine()->getSpatialDatabase()->removeObject( this, bounds);
	}*/
	// std::cout << "Someone is removing an agent " << std::endl;
}

SteerLib::EngineInterface * SocialForcesAgent::getSimulationEngine()
{
	return _gEngine;
}

void SocialForcesAgent::setParameters(Behaviour behave)
{
	this->_SocialForcesParams.setParameters(behave);
}

void SocialForcesAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// std::cout << "this agent is being disabled " << this << std::endl;
	assert(_enabled == true);


	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;


}

void SocialForcesAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	// std::cout << "resetting agent " << this << std::endl;
	_waypoints.clear();
	_midTermPath.clear();

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.5f, _position.z - _radius, _position.z + _radius);


	// initialize the agent based on the initial conditions
	/*
	position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
	radius_ = initialConditions.radius;
	velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
	velocity_ = velocity_ * initialConditions.speed;
	*/
	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;
	// std::cout << "inital colour of agent " << initialConditions.color << std::endl;
	if (initialConditions.colorSet == true)
	{
		this->_color = initialConditions.color;
	}
	else
	{
		this->_color = Util::gBlue;
	}

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.5f, _position.z - _radius, _position.z + _radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		getSimulationEngine()->getSpatialDatabase()->addObject(dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		// std::cout << "new position is " << _position << std::endl;
		// std::cout << "new bounds are " << newBounds << std::endl;
		// std::cout << "reset update " << this << std::endl;
		getSimulationEngine()->getSpatialDatabase()->updateObject(dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
		// engineInfo->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i = 0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
			initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
				_currentGoal.targetLocation = _goal.targetLocation;
			}
			else
			{
				_goalQueue.push(initialConditions.goals[i]);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; SocialForcesAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}
	}

	runLongTermPlanning(_goalQueue.front().targetLocation, dont_plan);

	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	* Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	* And that _waypoints is not empty
	*/
	Util::Vector goalDirection;
	if (!_midTermPath.empty())
	{
		this->updateLocalTarget();
		goalDirection = normalize(this->_currentLocalTarget - position());
	}
	else
	{
		goalDirection = normalize(_goalQueue.front().targetLocation - position());
	}

	_prefVelocity =
		(
		(
			(
				Util::Vector(goalDirection.x, 0.0f, goalDirection.z) *
				PERFERED_SPEED
				)
			- velocity()
			)
			/
			_SocialForcesParams.sf_acceleration
			)
		*
		MASS;

	// _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
		" and current velocity is: " << velocity_ << std::endl;
#endif


	// std::cout << "Parameter spec: " << _SocialForcesParams << std::endl;
	// _gEngine->addAgent(this, rvoModule);
	assert(_forward.length() != 0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);

	final_location = _goalQueue.front().targetLocation;
}


void SocialForcesAgent::calcNextStep(float dt)
{

}

std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
	// Return minimum distance between line segment vw and point p
	float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
	if (lSq == 0.0)
		return std::make_pair((p - l2).length(), l1);   // l1 == l2 case
														// Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
														// We find projection of point p onto the line.
														// It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
	const float t = dot(p - l1, l2 - l1) / lSq;
	if (t < 0.0)
	{
		return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
	}
	else if (t > 1.0)
	{
		return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
	}
	const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
	return std::make_pair((p - projection).length(), projection);
}


Util::Vector SocialForcesAgent::calcProximityForce(float dt)
{
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface * tmp_agent;
	SteerLib::ObstacleInterface * tmp_ob;
	Util::Vector away = Util::Vector(0, 0, 0);
	Util::Vector away_obs = Util::Vector(0, 0, 0);

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
		// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ((*neighbour)->isAgent())
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);

			// direction away from other agent
			Util::Vector away_tmp = normalize(position() - tmp_agent->position());
			// std::cout << "away_agent_tmp vec" << away_tmp << std::endl;
			// Scale force
			// std::cout << "the exp of agent distance is " << exp((radius() + tmp_agent->radius()) -
			//	(position() - tmp_agent->position()).length()) << std::endl;


			// away = away + (away_tmp * ( radius() / ((position() - tmp_agent->position()).length() * B) ));
			away = away +
				(
					away_tmp
					*
					(
						_SocialForcesParams.sf_agent_a
						*
						exp(
						(
							(
							(
								this->radius()
								+
								tmp_agent->radius()
								)
								-
								(
									this->position()
									-
									tmp_agent->position()
									).length()
								)
							/
							_SocialForcesParams.sf_agent_b
							)
						)


						)
					*
					dt
					);
			/*
			std::cout << "agent " << this->id() << " away this far " << away <<
			" distance " << exp(
			(
			(
			(
			radius()
			+
			tmp_agent->radius()
			)
			-
			(
			position()
			-
			tmp_agent->position()
			).length()
			)
			/
			_SocialForcesParams.sf_agent_b
			)
			) << std::endl;
			*/
		}
		else
		{
			// It is an obstacle
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
			CircleObstacle * obs_cir = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
			if (obs_cir != NULL && USE_CIRCLES)
			{
				// std::cout << "Found circle obstacle" << std::endl;
				Util::Vector away_tmp = normalize(position() - obs_cir->position());
				away = away +
					(
						away_tmp
						*
						(
							_SocialForcesParams.sf_wall_a
							*
							exp(
							(
								(
								(
									this->radius()
									+
									obs_cir->radius()
									)
									-
									(
										this->position()
										-
										obs_cir->position()
										).length()
									)
								/
								_SocialForcesParams.sf_wall_b
								)
							)


							)
						*
						dt
						);
			}
			else
			{
				Util::Vector wall_normal = calcWallNormal(tmp_ob);
				std::pair<Util::Point, Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
				// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				// 	(line.first.z+line.second.z)/2);
				std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
				// wall distance

				Util::Vector away_obs_tmp = normalize(position() - min_stuff.second);
				// std::cout << "away_obs_tmp vec" << away_obs_tmp << std::endl;
				// away_obs = away_obs + ( away_obs_tmp * ( radius() / ((position() - min_stuff.second).length() * B ) ) );
				away_obs = away_obs +
					(
						away_obs_tmp
						*
						(
							_SocialForcesParams.sf_wall_a
							*
							exp(
							(
								(
								(this->radius()) -
									(
										this->position()
										-
										min_stuff.second
										).length()
									)
								/
								_SocialForcesParams.sf_wall_b
								)
							)
							)
						*
						dt
						);
			}
		}

	}
	return away + away_obs;
}

Util::Vector SocialForcesAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
	std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
		(_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
	return calcWallRepulsionForce(dt) + (_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt));
}

Util::Vector SocialForcesAgent::calcAgentRepulsionForce(float dt)
{

	Util::Vector agent_repulsion_force = Util::Vector(0, 0, 0);

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		(this));

	SteerLib::AgentInterface * tmp_agent;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
		// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ((*neighbour)->isAgent())
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);


		}
		else
		{
			continue;
		}
		if ((id() != tmp_agent->id()) &&
			(tmp_agent->computePenetration(this->position(), this->radius()) > 0.000001)
			)
		{
			agent_repulsion_force = agent_repulsion_force +
				(tmp_agent->computePenetration(this->position(), this->radius()) * _SocialForcesParams.sf_agent_body_force * dt) *
				normalize(position() - tmp_agent->position());
			// normalized tangential force
			/*
			agent_repulsion_force = agent_repulsion_force +
			(
			(
			(-1*position()) - tmp_agent->position()
			)
			/
			(
			(-1*position()) - tmp_agent->position()
			).length()

			)*0.2;
			*/
			//TODO this can have some funny behaviour is velocity == 0
			Util::Vector tangent = cross(cross(tmp_agent->position() - position(), velocity()),
				tmp_agent->position() - position());
			tangent = tangent / tangent.length();
			float  tanget_v_diff = dot(tmp_agent->velocity() - velocity(), tangent);
			// std::cout << "Velocity diff is " << tanget_v_diff << " tangent is " << tangent <<
			//	" velocity is " << velocity() << std::endl;
			agent_repulsion_force = agent_repulsion_force +
				(_SocialForcesParams.sf_sliding_friction_force * dt *
				(
					tmp_agent->computePenetration(this->position(), this->radius())
					) * tangent * tanget_v_diff

					);
		}

	}
	return agent_repulsion_force;
}


// written by Bingchen
bool  SocialForcesAgent::isEllipticalCollisionDetected(SteerLib::ObstacleInterface * wall) {

	
		Util::AxisAlignedBox box = wall->getBounds();

		Util::Point mid_point_min = ((box.xmax - box.xmin) > (box.zmax - box.zmin)) ? Util::Point( box.xmin, 0, (box.zmin + box.zmax)/2 )  
																				    : Util::Point( (box.xmin + box.xmax)/2, 0, box.zmin );
		
		Util::Point mid_point_max = ((box.xmax - box.xmin) > (box.zmax - box.zmin)) ? Util::Point(box.xmax, 0, (box.zmin + box.zmax) / 2)
																					: Util::Point((box.xmin + box.xmax) / 2, 0, box.zmax);

		float standard_distance = ((box.xmax - box.xmin) > (box.zmax - box.zmin)) ? distanceBetween(mid_point_max, Util::Point(box.xmin, 0, box.zmin)) + (box.zmin + box.zmax) / 2
			: distanceBetween(mid_point_max, Util::Point(box.xmin, 0, box.zmin)) + (box.xmin + box.xmax) / 2;

		float self_distance = distanceBetween(mid_point_min, _position) + distanceBetween(mid_point_max, _position);

		return (self_distance < standard_distance);

}

Util::Vector SocialForcesAgent::calcWallRepulsionForce(float dt)
{

	Util::Vector wall_repulsion_force = Util::Vector(0, 0, 0);


	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.x + (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z - (this->_radius + _SocialForcesParams.sf_query_radius),
		_position.z + (this->_radius + _SocialForcesParams.sf_query_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::ObstacleInterface * tmp_ob;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++)
		// for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = _neighbors.begin();  tmp_o != _neighbors.end();  tmp_o++)
	{
		if (!(*neighbour)->isAgent())
		{
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
			
			// added by Bingchen
			//runLongTermPlanning(_goalQueue.front().targetLocation, false);
		}
		else
		{
			continue;
		}

		if (isEllipticalCollisionDetected(tmp_ob)) {
			std::cerr << "collision detected" << std::endl;
		}

		if (tmp_ob->computePenetration(this->position(), this->radius()) > 0.000001)
		{
			CircleObstacle * cir_obs = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
			if (cir_obs != NULL && USE_CIRCLES)
			{
				// std::cout << "Intersected circle obstacle" << std::endl;
				Util::Vector wall_normal = position() - cir_obs->position();
				// wall distance
				float distance = wall_normal.length() - cir_obs->radius();

				wall_normal = normalize(wall_normal);
				wall_repulsion_force = wall_repulsion_force +
					((
					(
						(
							wall_normal
							)
						*
						(
							radius() +
							_SocialForcesParams.sf_personal_space_threshold -
							(
								distance
								)
							)
						)
						/
						distance
						)* _SocialForcesParams.sf_body_force * dt);

				// tangential force
				// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
				// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
				// std::endl;
				wall_repulsion_force = wall_repulsion_force +
					(
						dot(forward(), rightSideInXZPlane(wall_normal))
						*
						rightSideInXZPlane(wall_normal)
						*
						cir_obs->computePenetration(this->position(), this->radius())
						)* _SocialForcesParams.sf_sliding_friction_force * dt;

			}
			else
			{
				Util::Vector wall_normal = calcWallNormal(tmp_ob);
				std::pair<Util::Point, Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
				// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				// 	(line.first.z+line.second.z)/2);
				std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
				// wall distance
				wall_repulsion_force = wall_repulsion_force +
					((
					(
						(
							wall_normal
							)
						*
						(
							radius() +
							_SocialForcesParams.sf_personal_space_threshold -
							(
								min_stuff.first
								)
							)
						)
						/
						min_stuff.first
						)* _SocialForcesParams.sf_body_force * dt);
				// tangential force
				// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
				// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
				// std::endl;
				wall_repulsion_force = wall_repulsion_force +
					(
						dot(forward(), rightSideInXZPlane(wall_normal))
						*
						rightSideInXZPlane(wall_normal)
						*
						tmp_ob->computePenetration(this->position(), this->radius())
						)* _SocialForcesParams.sf_sliding_friction_force * dt;
			}
		}

	}

	return wall_repulsion_force;
}

std::pair<Util::Point, Util::Point> SocialForcesAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if (normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin, 0, box.zmax), Util::Point(box.xmax, 0, box.zmax));
		// Ended here;
	}
	else if (normal.z == -1)
	{
		return std::make_pair(Util::Point(box.xmin, 0, box.zmin), Util::Point(box.xmax, 0, box.zmin));
	}
	else if (normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax, 0, box.zmin), Util::Point(box.xmax, 0, box.zmax));
	}
	else // normal.x == -1
	{
		return std::make_pair(Util::Point(box.xmin, 0, box.zmin), Util::Point(box.xmin, 0, box.zmax));
	}
}

/**
* Basically What side of the obstacle is the agent on use that as the normal
* DOES NOT SUPPORT non-axis-aligned boxes
*
*
* 			   \		   /
* 				\		  /
* 				 \	 a	 /
*				  \		/
* 					 _
* 			a		| |       a
* 					 -
* 				  /     \
* 				 /   a   \
* 				/	      \
* 			   /	       \
*
*
*/
Util::Vector SocialForcesAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if (position().x > box.xmax)
	{
		if (position().z > box.zmax)
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if (position().z < box.zmin)
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmax))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if (position().x < box.xmin)
	{
		if (position().z > box.zmax)
		{
			if (fabs(position().z - box.zmax) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if (position().z < box.zmin)
		{
			if (fabs(position().z - box.zmin) >
				fabs(position().x - box.xmin))
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if (position().z > box.zmax)
		{
			return Util::Vector(0, 0, 1);
		}
		else if (position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal(obs);
		}
	}

}

/**
* Treats Obstacles as a circle and calculates normal
*/
Util::Vector SocialForcesAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax + box.xmin) / 2, (box.ymax + box.ymin) / 2,
		(box.zmax + box.zmin) / 2);
	return normalize(position() - obs_centre);
}

bool hasSameSign(float a, float b) {
	return ((a > 0.0f && b > 0.0f) || (a <= 0.0f && b <= 0.0f));
}

bool isColorEqual(Util::Color c1, Util::Color c2) {
	return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b;
}

Util::Vector SocialForcesAgent::calcDeadlockForce()
{
	Util::Vector lock_force = { 0,0,0 };

	/*
	float search_radius = 5;
	std::vector<SteerLib::AgentInterface *> neighbor_agents = getNeighborAgents(search_radius);
	//std::cerr << neighbor_agents.size() << std::endl;


	float ahead_time = 2.0f;
	int i = 0;
	float distance = 3;
	SteerLib::AgentInterface *tmp_agent;

	for (std::vector<SteerLib::AgentInterface *>::iterator tmp_ag = neighbor_agents.begin(); tmp_ag != neighbor_agents.end(); tmp_ag++) {
		//only calculate agents that come from another direction
		//Util::Point future_position = _position.operator+(_velocity.operator*(ahead_time));
		//Util::Point tmp_ag_pos = (*tmp_ag)->position().operator+((*tmp_ag)->velocity().operator*(ahead_time));
		if (distanceBetween((*tmp_ag)->position(), _position) < distance) {
			//if (hasSameSign((*tmp_ag)->velocity().x, _velocity.x)) {
			//if( !isColorEqual( (*tmp_ag)->getColor() , _color ) ){
				//std::cerr << (*tmp_ag)->velocity() << std::endl;
				tmp_agent = (*tmp_ag);
				//if ((*tmp_ag)->getColor().b == 1) {
					lock_force += -total_force;
					//_velocity.operator*=(0);
					i++;
				//}
			//}
		//}
	}
	if (i != 0) {
		int randNum = rand() % (5) + 1;
		//lock_force = normalize(lock_force) * (float)randNum;
		lock_force = (lock_force) * (float)randNum;
	}
	*/

	if ( _position_last_frame.operator==(_position)) {
		std::cerr << "dead lock detected" << std::endl;
		
	}
	return lock_force;
}

/*
void SocialForcesAgent::computeNeighbors()
{
agentNeighbors_.clear();

if (_SocialForcesParams.rvo_max_neighbors > 0) {
// std::cout << "About to segfault" << std::endl;
dynamic_cast<SocialForcesAIModule *>(rvoModule)->kdTree_->computeAgentNeighbors(this, _SocialForcesParams.rvo_neighbor_distance * _SocialForcesParams.rvo_neighbor_distance);
// std::cout << "Made it past segfault" << std::endl;
}
}*/

// added by Bingchen, used to implement pursue and evade
Vector SocialForcesAgent::calcPursueAndEvadeForce(Vector _goalDirection, float _dt)
{	
	if (_color.r == 1.0f) {
		Util::Vector goal_force = (((_goalDirection * PERFERED_SPEED) - velocity()) / (_SocialForcesParams.sf_acceleration / _dt));
		return goal_force;

	}

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;

	float search_radius = 30;
	//void getItemsInRange(std::set<SpatialDatabaseItemPtr> & neighborList, float xmin, float xmax, float zmin, float zmax, SpatialDatabaseItemPtr exclude);
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + search_radius),
		_position.x + (this->_radius + search_radius),
		_position.z - (this->_radius + search_radius),
		_position.z + (this->_radius + search_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface *tmp_agent;
	Util::Vector neighbor_avg(0, 0, 0);
	std::vector<SteerLib::AgentInterface *> neighbor_agents;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++) {
		if ((*neighbour)->isAgent()) {
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);
		}
		else {
			continue;
		}
		if (id() != tmp_agent->id()) {
			neighbor_agents.push_back(tmp_agent);
		}
	}

	float pi = 0.3f;

	for (std::vector<SteerLib::AgentInterface *>::iterator tmp_ag = neighbor_agents.begin(); tmp_ag != neighbor_agents.end(); tmp_ag++) {
		//neighbor_avg += normalize((*tmp_ag)->currentGoal().targetLocation - (*tmp_ag)->position());
		
		//if the neighbot agent's color is red and the self agent's color is blue
		//it means the blue has to pursue the red
		if ( (*tmp_ag)->getColor().r == Util::gRed.r && _color.g == Util::gGreen.g ) {
			
			neighbor_avg -= normalize( (*tmp_ag)->position() - position() );
			Util::Vector _direction = normalize(pi*neighbor_avg);

			Util::Vector goal_force = _dt * AGENT_MASS * (_direction * PREFERED_SPEED * 2.0f - velocity()) / _dt;

			return goal_force;
		}

		//if the neighbot agent's color is red and the self agent's color is green
		//it means the green has to evade the red	
		if ((*tmp_ag)->getColor().r == Util::gRed.r && _color.b == Util::gBlue.b) {

			neighbor_avg += normalize((*tmp_ag)->position() - position());
			Util::Vector _direction = normalize(pi*neighbor_avg);

			Util::Vector goal_force = (((_goalDirection * PERFERED_SPEED) - velocity()) / (_SocialForcesParams.sf_acceleration / _dt))
				+ _dt * AGENT_MASS * (_direction * PREFERED_SPEED * 2.0f - velocity()) / _dt;

			return goal_force;
		}
	}
	return {0.0f, 0.0f, 0.0f};
}




// for the seek and flee written by Bingchen
Vector SocialForcesAgent::calcSeekAndFleeForce(Vector _goalDirection, float _dt)
{

	if (_color.b == 1.0f) {
		Util::Vector goal_force = (((_goalDirection * PERFERED_SPEED) - velocity()) / (_SocialForcesParams.sf_acceleration / _dt));
		return goal_force;
	}

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;

	float search_radius = 10;

	//void getItemsInRange(std::set<SpatialDatabaseItemPtr> & neighborList, float xmin, float xmax, float zmin, float zmax, SpatialDatabaseItemPtr exclude);
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + search_radius),
		_position.x + (this->_radius + search_radius),
		_position.z - (this->_radius + search_radius),
		_position.z + (this->_radius + search_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface *tmp_agent;
	Util::Vector neighbor_total(0, 0, 0);
	std::vector<SteerLib::AgentInterface *> neighbor_agents;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++) {
		if ((*neighbour)->isAgent()) {
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);
		}
		else {
			continue;
		}
		if (id() != tmp_agent->id()) {
			neighbor_agents.push_back(tmp_agent);
		}
	}

	size_t blue_number = 0;

	for (std::vector<SteerLib::AgentInterface *>::iterator tmp_ag = neighbor_agents.begin(); tmp_ag != neighbor_agents.end(); tmp_ag++) {
		// the red agents are flee agents
		//they will try to avoid blue, so here will calculate the avoid froce from blue agents
		if ((*tmp_ag)->getColor().b == Util::gBlue.b) {
			blue_number += 1;
			neighbor_total -= normalize((*tmp_ag)->currentGoal().targetLocation - (*tmp_ag)->position());
		}
	}

	float pi = 0.3f;
	if ( _color.r == 1.0f) {
		if ( blue_number != 0 ) {
			Util::Vector _direction = normalize(neighbor_total);
			Util::Vector goal_force =  _dt * AGENT_MASS * (_direction * PREFERED_SPEED * 1.5f - velocity()) / _dt;
			last_direction = goal_force;
			return goal_force;
		}	
		//return last_direction;
	}
	return { 0.0f, 0.0f, 0.0f };
}

// repath when detect potential colision written by Bingchen
Util::Vector SocialForcesAgent::calcUnalignedCollisionAvoidance() {

	float search_radius = 20;
	std::vector<SteerLib::AgentInterface *> neighbor_agents = getNeighborAgents(search_radius);
	//std::cerr << neighbor_agents.size() << std::endl;

	float ahead_time = 5.0f;
	for (std::vector<SteerLib::AgentInterface *>::iterator tmp_ag = neighbor_agents.begin(); tmp_ag != neighbor_agents.end(); tmp_ag++) {
		//only calculate agents that come from another direction
		if (distanceBetween((*tmp_ag)->position(), _position) > 5) {

			Util::Point future_position = { (*tmp_ag)->velocity().x * ahead_time + (*tmp_ag)->position().x,
				0,
				(*tmp_ag)->velocity().z * ahead_time + (*tmp_ag)->position().z };

			if (distanceBetween(future_position, _position) < 2) {

				Util::Vector current_direction = normalize(_currentLocalTarget - _position);
				Util::Vector new_direction = { -current_direction.z, 0, current_direction.x };

				return new_direction;

			}

		}
	}
	return{ 0.0f, 0.0f, 0.0f };
}

// return neighbor agents, by Bingchen
std::vector<SteerLib::AgentInterface*> SocialForcesAgent::getNeighborAgents(float search_radius)
{
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;

	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
		_position.x - (this->_radius + search_radius),
		_position.x + (this->_radius + search_radius),
		_position.z - (this->_radius + search_radius),
		_position.z + (this->_radius + search_radius),
		dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface *tmp_agent;
	Util::Vector neighbor_avg(0, 0, 0);
	std::vector<SteerLib::AgentInterface *> neighbor_agents;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin(); neighbour != _neighbors.end(); neighbour++) {
		if ((*neighbour)->isAgent()) {
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);
		}
		else {
			continue;
		}
		if (id() != tmp_agent->id()) {
			neighbor_agents.push_back(tmp_agent);
		}
	}


	return neighbor_agents;
}


bool SocialForcesAgent::hasFrontAgent() {
	return true;
}

Util::Vector SocialForcesAgent::calcSeparationForce( float separation_radius, float separation_space ) {
	std::vector<SteerLib::AgentInterface*> neighbor_agents = getNeighborAgents(separation_radius);

	Vector separate_force = { 0,0,0 };
	//std::cerr << _radius << std::endl;

	for (std::vector<SteerLib::AgentInterface *>::iterator tmp_ag = neighbor_agents.begin(); tmp_ag != neighbor_agents.end(); tmp_ag++) {

			separate_force.x += (*tmp_ag)->position().x - _position.x;
			separate_force.z += (*tmp_ag)->position().z - _position.z;
		
	}
	if (neighbor_agents.size() != 0) {
		separate_force.operator/=(-neighbor_agents.size());
		separate_force = normalize(separate_force);
		separate_force.operator*=(-1.0f);
	}

	separate_force.operator*=(separation_space);

	return separate_force;
}


Util::Vector SocialForcesAgent::Queue() {
	
	float search_radius = 2.0f;
	std::vector<SteerLib::AgentInterface*> neighbor_agents = getNeighborAgents(search_radius);

	float distance = 2;
	SteerLib::AgentInterface * tmp_agent;
	// get the first Agent ahead
	Vector queue_force = { 0,0,0};
	
	Vector brake_force = { 0,0,0 };
	Vector separate_force = { 0,0,0};

	float i = 0;
	float ahead_number = 0;
	//std::cerr << _radius << std::endl;

	if (neighbor_agents.size() != 0) {
		Util::Vector direction = normalize(_velocity);

		for (std::vector<SteerLib::AgentInterface *>::iterator tmp_ag = neighbor_agents.begin(); tmp_ag != neighbor_agents.end(); tmp_ag++) {
			Util::Point front_point = _position + 1.0f * direction;

			if (distanceBetween(front_point, (*tmp_ag)->position()) < 1.0f) {
				ahead_number += 1;
			}
			
			if(distanceBetween(_position, (*tmp_ag)->position()) <= 1.1f){
				separate_force.x += (*tmp_ag)->position().x - _position.x;
				separate_force.z += (*tmp_ag)->position().z - _position.z;
				i += 1;
			}
		}

		if (ahead_number > 0) {
			brake_force.operator-=(total_force * 0.8f);
			brake_force.operator-=(_velocity);
		}
		if (i > 0) {
			separate_force.operator/=(i);
			separate_force.operator*=(-1.0f);
			separate_force = normalize(separate_force);
			separate_force.operator*=(1.1f);
		}
		brake_force.operator+=(separate_force);
	}
	
	return brake_force;
}



void SocialForcesAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	// std::cout << "_SocialForcesParams.rvo_max_speed " << _SocialForcesParams._SocialForcesParams.rvo_max_speed << std::endl;
	Util::AutomaticFunctionProfiler profileThisFunction(&SocialForcesGlobals::gPhaseProfilers->aiProfiler);
	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	Util::Vector goalDirection;

	// std::cout << "midtermpath empty: " << _midTermPath.empty() << std::endl;
	if (!_midTermPath.empty() && (!this->hasLineOfSightTo(goalInfo.targetLocation)))
	{
		bool wandering = true;
		// added by Bingchen, used to realize wandering, a random target location to go
		if (wandering) {
			//if( _color.r == Util::gRed.r ){
			float x = ((float(rand()) / float(RAND_MAX)) * (95.0f - 5.5f)) + 5.5f;
			float z = ((float(rand()) / float(RAND_MAX)) * (88.0f)) - 44.0f;


			_currentLocalTarget = { x, 0, z };
		}

		if (reachedCurrentWaypoint())
		{
			this->updateMidTermPath();

		}
		this->updateLocalTarget();

		goalDirection = normalize(_currentLocalTarget - position());

	}
	else
	{

		goalDirection = normalize(goalInfo.targetLocation - position());
	}

	// all the force that are calculated 
	// for the seek or flee force    written by Bingchen
	///Util::Vector prefForce = calcSeekAndFleeForce(goalDirection, dt);
	Util::Vector prefForce = calcPursueAndEvadeForce(goalDirection, dt);
	///Util::Vector prefForce = (((goalDirection * PERFERED_SPEED) - velocity()) / (_SocialForcesParams.sf_acceleration / dt));
	///Util::Vector prefForce = goalDirection * PREFERED_SPEED;
	Util::Vector proximityForce = calcProximityForce(dt);
	Util::Vector repulsionForce = calcRepulsionForce(dt);
	///Util::Vector avoidForce = calcUnalignedCollisionAvoidance();

	if (repulsionForce.x != repulsionForce.x)
	{
		std::cout << "Found some nan" << std::endl;
		repulsionForce = velocity();
		// throw GenericException("SocialForces numerical issue");
	}
	

	// #define _DEBUG_ 1
#ifdef _DEBUG_
	std::cout << "agent" << id() << " repulsion force " << repulsionForce << std::endl;
	std::cout << "agent" << id() << " proximity force " << proximityForce << std::endl;
	std::cout << "agent" << id() << " pref force " << prefForce << std::endl;
#endif
	int alpha = 1;
	if (repulsionForce.length() > 0.0)
	{
		alpha = 0;
	}

	// modified by bingchen
	/// 1. seek and flee case
	///total_force = prefForce + repulsionForce  + proximityForce;

	/// 2. used for queue behavior, written by bingchen
	//total_force = prefForce + repulsionForce + proximityForce + 5.0f * avoidForce;
	total_force = prefForce + repulsionForce + proximityForce;

	//Util::Vector queueForce = Queue();
	//total_force += queueForce;

	//Util::Vector lockForce = calcDeadlockForce();
	//total_force += lockForce;
	/// 3. pursue and evade case
	///total_force = prefForce + repulsionForce  + proximityForce;
	
	/// 4. unaligned avoidence case
	///total_force = prefForce + repulsionForce  + proximityForce + 5.0f *avoidForce;
	Util::Vector acceleration = 20.0 * total_force / AGENT_MASS;
	_velocity = velocity() + acceleration * dt;
	//_velocity += total_force;
	_velocity = clamp(velocity(), _SocialForcesParams.sf_max_speed);
	_velocity.y = 0.0f;

#ifdef _DEBUG_
	std::cout << "agent" << id() << " speed is " << velocity().length() << std::endl;
#endif
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	* Or when the agent is removed for example its true location will not reflect its location in the grid database.
	* Not only that but this error will appear random depending on how well the agent lines up with the grid database
	* boundaries when removed.
	*/
	// std::cout << "Updating agent" << this->id() << " at " << this->position() << std::endl;
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	getSimulationEngine()->getSpatialDatabase()->updateObject(this, oldBounds, newBounds);

	/*
	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).length() < radius()*WAYPOINT_THRESHOLD_MULTIPLIER)
	{
	_waypoints.erase(_waypoints.begin());
	}
	*/
	/*
	* Now do the conversion from SocialForcesAgent into the SteerSuite coordinates
	*/
	// _velocity.y = 0.0f;

	if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
		(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
			Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
				goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
	{
		_goalQueue.pop();
		// std::cout << "Made it to a goal" << std::endl;
		if (_goalQueue.size() != 0)
		{
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else
		{
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if (velocity().lengthSquared() > 0.0)
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}
	// _position = _position + (_velocity * dt);

	_position_last_frame = position();
}


void SocialForcesAgent::draw()
{
#ifdef ENABLE_GUI
	AgentInterface::draw();
	// if the agent is selected, do some annotations just for demonstration

#ifdef DRAW_COLLISIONS
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors, _position.x - (this->_radius * 3), _position.x + (this->_radius * 3),
		_position.z - (this->_radius * 3), _position.z + (this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin(); neighbor != _neighbors.end(); neighbor++)
	{
		if ((*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
		{
			Util::DrawLib::drawStar(
				this->position()
				+
				(
				(
					dynamic_cast<AgentInterface*>(*neighbor)->position()
					-
					this->position()
					)
					/ 2), Util::Vector(1, 0, 0), 0.8f, gRed);
			// Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}
#endif
#ifdef DRAW_HISTORIES
	__oldPositions.push_back(position());
	int points = 0;
	float mostPoints = 100.0f;
	while (__oldPositions.size() > mostPoints)
	{
		__oldPositions.pop_front();
	}
	for (int q = __oldPositions.size() - 1; q > 0 && __oldPositions.size() > 1; q--)
	{
		DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q - 1), gBlack, q / (float)__oldPositions.size());
	}

#endif

#ifdef DRAW_ANNOTATIONS

	for (int i = 0; (_waypoints.size() > 1) && (i < (_waypoints.size() - 1)); i++)
	{
		if (_gEngine->isAgentSelected(this))
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i + 1), gYellow);
		}
		else
		{
			//DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
		}
	}

	for (int i = 0; i < (_waypoints.size()); i++)
	{
		DrawLib::drawStar(_waypoints.at(i), Util::Vector(1, 0, 0), 0.34f, gBlue);
	}

	for (int i = 0; (_midTermPath.size() > 1) && (i < (_midTermPath.size() - 1)); i++)
	{
		if (_gEngine->isAgentSelected(this))
		{
			DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i + 1), gMagenta);
		}
		else
		{
			// DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
		}
	}

	DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
	DrawLib::drawStar(this->_currentLocalTarget + Util::Vector(0, 0.001, 0), Util::Vector(1, 0, 0), 0.24f, gGray10);

	/*
	// draw normals and closest points on walls
	std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();

	for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
	{
	Util::Vector normal = calcWallNormal( *tmp_o );
	std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
	Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
	(line.first.z+line.second.z)/2);
	DrawLib::drawLine(midpoint, midpoint+normal, gGreen);

	// Draw the closes point as well
	std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
	DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
	}
	*/

#endif

#endif
}
