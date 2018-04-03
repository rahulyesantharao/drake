function runWalkingDemo(obj, walking_options)
% Run the new split QP controller, which consists of separate PlanEval
% and InstantaneousQPController objects. The controller will also
% automatically transition to standing when it reaches the end of its walking
% plan.

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin < 2; walking_options = struct(); end;

walking_options = applyDefaults(walking_options, struct('initial_pose', [],...
                                                        'navgoal', [1.5;0;0;0;0;0],...
                                                        'max_num_steps', 6,...
                                                        'rms_com_tolerance', 0.0051,...
                                                        'urdf_modifications_file', ''));
walking_options = applyDefaults(walking_options, obj.default_footstep_params);
walking_options = applyDefaults(walking_options, obj.default_walking_params);

% set initial state to fixed point
load(obj.fixed_point_file, 'xstar');
if ~isempty(walking_options.initial_pose), xstar(1:6) = walking_options.initial_pose; end
xstar = obj.resolveConstraints(xstar);
obj = obj.setInitialState(xstar);

v = obj.constructVisualizer;
v.display_dt = 0.01;

nq = getNumPositions(obj);

x0 = xstar;

% Find the initial positions of the feet
R=rotz(walking_options.navgoal(6));

rfoot_navgoal = walking_options.navgoal;
lfoot_navgoal = walking_options.navgoal;

rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan1 = obj.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', walking_options));

% ******* REPLACEMENT OF LINE 44 ******************
lc = lcm.lcm.LCM.getSingleton();

% Find some values
coord_names = obj.getStateFrame().getCoordinateNames();
num_positions = obj.getNumPositions();
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
plan_params.max_num_steps = walking_options.max_num_steps;
plan_params.min_num_steps = walking_options.min_num_steps;
plan_params.min_step_width = walking_options.min_step_width;
plan_params.nom_step_width = walking_options.nom_step_width;
plan_params.max_step_width = walking_options.max_step_width;
plan_params.nom_forward_step = walking_options.nom_forward_step;
plan_params.max_forward_step = walking_options.max_forward_step;
plan_params.nom_upward_step = walking_options.nom_upward_step;
plan_params.nom_downward_step = walking_options.nom_downward_step;
plan_params.planning_mode = 0;
plan_params.behavior = 0;
plan_params.map_mode = 3;
plan_params.leading_foot = walking_options.leading_foot;
request.params = plan_params;

footstep_params = drake.footstep_params_t();
footstep_params.step_speed = walking_options.step_speed;
footstep_params.drake_min_hold_time = walking_options.drake_min_hold_time;
footstep_params.step_height = walking_options.step_height;
footstep_params.constrain_full_foot_pose = walking_options.constrain_full_foot_pose;
% Ignore BDI Parameters
% Ignore IHCM Parameters
footstep_params.mu = walking_options.mu;
footstep_params.drake_instep_shift = walking_options.drake_instep_shift;
footstep_params.support_contact_groups = 0;
footstep_params.prevent_swing_undershoot = walking_options.prevent_swing_undershoot;
footstep_params.prevent_swing_overshoot = walking_options.prevent_swing_overshoot;
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
footstep_plan = FootstepPlan.from_footstep_plan_t(fplan, obj);
% for i = 2:3
%     footstep_plan.footsteps(i) = footstep_plan1.footsteps(i);
% end
for i=1:length(footstep_plan.footsteps)
    footstep_plan.footsteps(i).is_in_contact = 1;
end

% Visualize the footstep plan
% Show the result
v = obj.constructVisualizer();
v.draw(0, xstar);
if isa(v, 'BotVisualizer')
  lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'footstep_plan');
  footstep_plan.draw_lcmgl(lcmgl);
  lcmgl.switchBuffers();
else
  figure(25)
  footstep_plan.draw_2d();
end

% *************************************************

for j = 1:length(footstep_plan.footsteps)
  footstep_plan.footsteps(j).walking_params = walking_options;
end

% Generate a dynamic walking plan
walking_plan_data = obj.planWalkingZMP(x0(1:obj.getNumPositions()), footstep_plan);

[ytraj, com, rms_com] = obj.simulateWalking(walking_plan_data, walking_options);

v.playback(ytraj, struct('slider', true));

if ~rangecheck(rms_com, 0, walking_options.rms_com_tolerance);
  error('Drake:runWalkingDemo:BadCoMTracking', 'Center-of-mass during execution differs substantially from the plan.');
end


