function plan = runAtlasFootstepPlanning()
% Demonstration of footstep planning for the Atlas biped. We choose to define the footstep
% planning problem as one of finding a set of sequential foot positions which are
% kinematically reachable and which are *likely* to yield a dynamically stable walking
% motion. See also: runAtlasWalkingPlanning for the full dynamical plan.
% @retval plan a FootstepPlan describing the sequence of foot positions


checkDependency('lcmgl');
% Set up the model
load('data/atlas_fp.mat', 'xstar');
r = Atlas('urdf/atlas_minimal_contact.urdf');
r = r.setInitialState(xstar);

% Find the initial positions of the feet
kinsol = doKinematics(r, xstar(1:r.getNumPositions()));
start_pos = struct('right', forwardKin(r, kinsol, r.foot_frame_id.right, [0;0;0], 1), ...
                   'left', forwardKin(r, kinsol, r.foot_frame_id.left, [0;0;0], 1));

% Plan footsteps to the goal
goal_pos = struct('right', [1;-0.13;0;0;0;0], 'left', [1;0.13;0;0;0;0]);
%plan = r.planFootsteps(start_pos, goal_pos);

% ------------- START REPLACE LINE 22 TO GET DIFFERENT PLAN --------------------
lc = lcm.lcm.LCM.getSingleton();

% Find some values
coord_names = r.getStateFrame().getCoordinateNames();
num_positions = r.getNumPositions();
joint_names = coord_names(7:num_positions);
joint_pos = xstar(7:num_positions);
joint_vel = xstar(num_positions+7:end);
joint_eff = zeros(length(joint_names), 1);
ss_rot = rpy2quat(xstar(4:6));

% BUILD AND PUBLISH REQUEST
request = drake.footstep_plan_request_t();

goal = drake.position_3d_t();
goal.rotation = drake.quaternion_t();
goal.translation = drake.vector_3d_t();
goal.translation.x = (goal_pos.right(1)+goal_pos.left(1))/2;
goal.translation.y = (goal_pos.right(2)+goal_pos.left(2))/2;
request.goal_pos = goal;

startstate = drake.robot_state_t();
startstate.num_joints = length(joint_names);
startstate.joint_name = joint_names;
startstate.joint_position = joint_pos;
startstate.joint_velocity = joint_vel;
startstate.joint_effort = joint_eff;
startstate.twist = drake.twist_t();
startstate.twist.linear_velocity = drake.vector_3d_t();
startstate.twist.angular_velocity = drake.vector_3d_t();
startstate.force_torque = drake.force_torque_t();
startstate.pose = drake.position_3d_t();
startstate.pose.rotation = drake.quaternion_t();
startstate.pose.rotation.w = ss_rot(1);
startstate.pose.rotation.x = ss_rot(2);
startstate.pose.rotation.y = ss_rot(3);
startstate.pose.rotation.z = ss_rot(4);
startstate.pose.translation = drake.vector_3d_t();
startstate.pose.translation.x = xstar(1);
startstate.pose.translation.y = xstar(2);
startstate.pose.translation.z = xstar(3);
request.initial_state = startstate;

plan_params = drake.footstep_plan_params_t();
plan_params.max_num_steps = 6;
plan_params.min_num_steps = 1;
plan_params.min_step_width = 0.18;
plan_params.nom_step_width = 0.26;
plan_params.max_step_width = 0.38;
plan_params.nom_forward_step = 0.25;
plan_params.max_forward_step = 0.35;
plan_params.nom_upward_step = 0.25;
plan_params.nom_downward_step = 0.25;
plan_params.planning_mode = 0;
plan_params.behavior = 0;
plan_params.map_mode = 3;
plan_params.leading_foot = 1;
request.params = plan_params;

footstep_params = drake.footstep_params_t();
footstep_params.step_speed = 0.5;
footstep_params.drake_min_hold_time = 0.7;
footstep_params.step_height = 0.05;
footstep_params.constrain_full_foot_pose = true;
% Ignore BDI Parameters
% Ignore IHCM Parameters
footstep_params.mu = 1;
footstep_params.drake_instep_shift = 0;
footstep_params.support_contact_groups = 0;
footstep_params.prevent_swing_undershoot = false;
footstep_params.prevent_swing_overshoot = false;
request.default_step_params = footstep_params;

lc.publish('REQUEST', request);

% LISTEN FOR PLAN AND INFLATE FOOTSTEP FROM IT
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('FOOTSTEP_PLAN', aggregator);
while true
	disp waiting
	millis_to_wait = 500;
	msg = aggregator.getNextMessage(millis_to_wait);
	if length(msg) > 0
		break
	end
end

disp(sprintf('channel of received message: %s', char(msg.channel)));
disp(sprintf('raw bytes of received message: %d', msg.data'));
fplan = drake.footstep_plan_t(msg.data);
plan = FootstepPlan.from_footstep_plan_t(fplan, r);
% ------------- END REPLACE LINE 22 TO GET DIFFERENT PLAN --------------------

% Show the result
v = r.constructVisualizer();
v.draw(0, xstar);
if isa(v, 'BotVisualizer')
  lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'footstep_plan');
  plan.draw_lcmgl(lcmgl);
  lcmgl.switchBuffers();
else
  figure(25)
  plan.draw_2d();
end

end

