{ include("waypoints_calculation.asl") }

//initial beliefs
std_heading(0.0).
input_id(0).
use_heading(false).
fly_now(false).
stop_at_waypoints(false).
loop(false).
over_constraints(false).
over_max_velocity_hor(0.0).
over_max_acceleration_hor(0.0).
over_max_jerk_hor(0.0).
over_max_velocity_ver(0.0).
over_max_acceleration_ver(0.0).
over_max_jerk_ver(0.0).
relax_heading(false).

//rules
current_pos(CX, CY, CZ) :- uav1_odometry_odom_gps(header(seq(_),stamp(secs(_),nsecs(_)),frame_id(_)),child_frame_id(_),pose(pose(position(x(CX),y(CY),z(CZ)),orientation(x(_),y((_)),z((_)),w((_)))),covariance(_)),twist(twist(linear(x(_),y(_),z((_))),angular(x(_),y((_)),z((_)))),covariance(_))).
current_header(Seq, Secs, Nsecs) :- uav1_odometry_odom_gps(header(seq(Seq),stamp(secs(Secs),nsecs(Nsecs)),frame_id(_)),child_frame_id(_),pose(pose(position(x(_),y(_),z(_)),orientation(x(_),y((_)),z((_)),w((_)))),covariance(_)),twist(twist(linear(x(_),y(_),z((_))),angular(x(_),y((_)),z((_)))),covariance(_))).

//plans
+!generate_trajectory(WayList)
   :  my_frame_id(Frame_id)
      & input_id(II)
      & use_heading(UH)
      & fly_now(FN)
      & stop_at_waypoints(SAW)
      & loop(Loop)
      & over_constraints(OC)
      & over_max_velocity_hor(OMVH)
      & over_max_acceleration_hor(OMAH)
      & over_max_jerk_hor(OMJH)
      & over_max_velocity_ver(OMVV)
      & over_max_acceleration_ver(OMAV)
      & over_max_jerk_ver(OMJV)
      & relax_heading(RH)
      & std_heading(Heading)
      & my_number_string(N)
   <- .print("Generating trajectory");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","generate_trajectory", [N, 0, 0, 0, Frame_id, II, UH, FN, SAW, Loop, OC, OMVH, OMAH, OMJH, OMVV, OMAV, OMJV, RH, WayList, Heading]);
      .wait(1000);
      !goto_start_trajectory.

+!goto_start_trajectory
   :  my_number_string(N)
   <- .print("Going to the start of the trajectory");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","goto_trajectory_start", [N]);
      !start_trajectory_tracking.

+!start_trajectory_tracking
   :  my_number_string(N)
   <- .print("Started trajectory tracking");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","start_trajectory_tracking", [N]).

+found_fire(X, Y)[source(UAV)]
   <- .print("Fire found by ", UAV);
      +fire_extinguished(X, Y, false);
      !stop_trajectory_tracking;
      !goto_reference(X, Y).
   
+!stop_trajectory_tracking
   :  my_number_string(N)
   <- .print("Stoping trajectory tracking");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","stop_trajectory_tracking", [N]).

+!goto_reference(X, Y)
   :  current_pos(CX, CY, _)
      & diff(D)
      & abs(CX - X) >= D
      & abs(CY - Y) >= D
      & current_header(Seq, Secs, Nsecs)
      & my_frame_id(Frame_id)
      & std_altitude(Z)
      & std_heading(Heading)
      & my_number_string(N)
   <- .print("Going to the fire");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","goto_reference", [N, Seq, Secs, Nsecs, Frame_id, X, Y, Z, Heading]).
      !goto_reference(X, Y).

+!goto_reference(X, Y)
   :  current_pos(CX, CY, _)
      & diff(D)
      & abs(CX - X) < D
      & abs(CY - Y) < D
   <- .print("Arrived the fire");
      !extinguish_fire.

+!extinguish_fire
   : fire_extinguished(X, Y, false)
   <- .print("Extinguishing fire");
      //call service;
      !extinguish_fire.

+!extinguish_fire
   : fire_extinguished(X, Y, true)
   <- .print("Extinguishing fire");
      //call service;
      !fire_extinguished(X, Y).

+!fire_extinguished(X, Y)
   :  finished_trajectory(FT)
      & all_finished(AF)
      & land_position(LX, LY)
   <- .print("Fire extinguished");
      .print("Going to land position");
      !goto_landing_position(LX, LY).

+fire_extinguished(X, Y)
   :  finished_trajectory(FT)
   <- .print("Fire extinguished");
      .print("Going to land position");
      !wait_for_others.

+fire_extinguished(X, Y)
   <- .print("Fire extinguished");
      .print("Going to land position");
      !resume_trajectory_tracking(LX, LY).

+resume_trajectory_tracking
   :  my_number_string(N)
   <- .print("Resuming trajectory tracking");
      embedded.mas.bridges.jacamo.defaultEmbeddedInternalAction("roscore1","resume_trajectory_tracking", [N]).

+!wait_for_others
   :  fire_found(X, Y)
   <- .print("Fire found by ", UAV);
      +fire_extinguished(X, Y, false);
      !goto_reference(X, Y).

+!wait_for_others
   :  all_finished(AF)
   <- .print("All finished, going to land position");
      !goto_landing_position(LX, LY).

+!goto_landing_position(X, Y)
   :  current_pos(CX, CY, _)
      & diff(D)
      & abs(CX - X) >= D
      & abs(CY - Y) >= D
   <- .print("Going to landing position");
      !goto_landing_position(X, Y).

+!goto_landing_position(X, Y)
   :  current_pos(CX, CY, _)
      & diff(D)
      & abs(CX - X) < D
      & abs(CY - Y) < D
   <- .print("Arrived at landing position");
      !land.

+!land
   <- .print("Landing");
      //call service.
      .