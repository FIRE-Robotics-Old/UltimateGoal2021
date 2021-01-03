package org.arcrobotics.ftclib.files.purepursuit;

import org.arcrobotics.ftclib.files.geometry.Pose2d;
import org.arcrobotics.ftclib.files.purepursuit.types.WaypointType;
import org.arcrobotics.ftclib.files.purepursuit.waypoints.GeneralWaypoint;

/**
 * A pure pursuit Waypoint is a point in which the robot traverses. Using Waypoints
 * one can construct a pure pursuit path for their robot to follow.
 * 
 * @see GeneralWaypoint , EndWaypoint, StartWaypoint, PointTurnWaypoint, InterruptWaypoint
 * @version 1.2
 * @author Michael Baljet, Team 14470
 *
 */
public interface Waypoint {
	
	/**
	 * Returns this WayPoint's type.
	 * @return this WayPoint's type.
	 */
	public WaypointType getType();
	
	/**
	 * Returns this Waypoint's position.
	 * @return this Waypoint's position.
	 */
	public Pose2d getPose();
	
	/**
	 * Returns the follow distance for this waypoint.
	 * @return the follow distance for this waypoint.
	 */
	public double getFollowDistance();
	
	/**
	 * Returns the timeout period of this waypoint.
	 * @return the timeout period of this waypoint.
	 */
	public long getTimeout();
	
}
