%{
** Description of motion planning
* We created a robot with 4 DOF, and the Denavit - Hartenberg table is shown below;
* The first joint is a revolute base, which is used to declear the derection towards
* the chess found by the camera and opencv module. The second joint and the third
* joint could be planned together to strech the arm and make the end effector (which
* is an electromagnet in our project) over the chess found. The last joint is to make
* sure that the electromagnet is straight up and down to move the chess.
*

% arm:: 3 axis, RRR, stdDH, slowRNE
% +---+-----------+-----------+-----------+-----------+-----------+
% | j |     theta |         d |         a |     alpha |    offset |
% +---+-----------+-----------+-----------+-----------+-----------+
% |  1|         q1|         13|          0|     1.5708|          0|
% |  2|         q2|          0|         20|          0|          0|
% |  3|         q3|          0|         20|          0|    -1.5708|
% +---+-----------+-----------+-----------+-----------+-----------+
%}

% clear; clc;

% create links
L(1) = Link([0, 13, 0, pi / 2, 0]);
L(2) = Link([0, 0, 20, 0, 0]);
L(3) = Link([0, 0, 20, 0, 0]);

% add limitations of angle
L(1).qlim = [deg2rad(-90) deg2rad(90)];
L(2).qlim = [deg2rad(0) deg2rad(100)];
L(3).qlim = [deg2rad(-90) deg2rad(10)];
arm = SerialLink(L, 'name', 'arm');

% set an offset of the third link, this is the WORK position
% fig1 = figure(1);
% L(3).offset = -pi / 2;
% arm.plot([0 0 0])
% view(120, 20)
% print('./WORK_position.tiffn', fig1, '-r600', '-dtiffn')

% this is the REST position
% fig2 = figure(2);
start_postion = [deg2rad(0) deg2rad(90) deg2rad(-80)];
% view(120, 20)
% arm.plot(start_postion)
% arm.display()
% print('./REST_position.tiffn', fig2, '-r600', '-dtiffn')

% use "ctraj" function provided by RTB to plan for a straight line
% fig3 = figure(3);
% T1 = transl(30, 10, 0); % start point
% T2 = transl(30, 10, 20); % end point

% T = ctraj(T1, T2, 50);
% Tj = transl(T);
% plot3(Tj(:, 1), Tj(:, 2), Tj(:, 3)); % plot the end effector trajectory
% grid on;
% view(100, 20);

% % use inverse kinematics to solve the planning
% q = arm.ikine(T, 'mask', [1 1 1 0 0 0]);
% arm.plot(q);
% print('./pick_traj.tif', fig3, '-r600', '-dtiffn')

% PLAN-1: as shown, this uses all straight lines to plan
% fig4 = figure(4);
% pick = reshape([30 * ones(1, 100) 10 * ones(1, 100) linspace(0, 20, 100)], 100, 3);
% move = reshape([linspace(30, 5, 100) linspace(10, -10, 100) 20 * ones(1, 100)], 100, 3);
% place = reshape([5 * ones(1, 100) -10 * ones(1, 100) linspace(20, 0, 100)], 100, 3);
% traj = [pick; move; place];

% plot3(traj(:, 1), traj(:, 2), traj(:, 3), 'r');
% view(120, 20)
% zlim([-50, 60])

% T = transl(traj);
% q = arm.ikine(T, 'mask', [1 1 1 0 0 0]);
% hold on;

% for i = 1:size(q)
%     arm.plot(q(i, :));
%     gif_generator(fig4, './whole_traj_straightline.gif', i, 0.1)
% end
% print('./whole_traj_straightline.tif', fig4, '-r600', '-dtiffn')

% PLAN-2: as shown, simplify the motion planning [pick -> base rotate -> strech or withdraw -> place]
%{
** about atan2
* in the first domain the value is 0~pi / 2
* in the fourth domain the value is -pi / 2~0
%}
fig5 = figure(5);
height = 1;
pick_chess_location = [30 20 height];
place_chess_location = [15 -10 height];
pick_angle = atan2(pick_chess_location(2), pick_chess_location(1));
place_angle = atan2(place_chess_location(2), place_chess_location(1));

pick = reshape([pick_chess_location(1) * ones(1, 100) pick_chess_location(2) * ones(1, 100) linspace(height, height + 20, 100)], 100, 3);

if (pick_angle > 0 && place_angle < 0) % exclude the wrong solution(path too long)
    theta = flip(linspace(place_angle, pick_angle, 100));
else
    theta = linspace(pick_angle, place_angle, 100);
end

center = [0 0 20];
radius = sqrt(pick_chess_location(1)^2 + pick_chess_location(2)^2);
degs = zeros(100, 3);
degs(:, 1) = cos(theta)';
degs(:, 2) = sin(theta)';
degs(:, 3) = zeros(size(theta))';
move_1 = repmat(center, 100, 1) + radius * degs;
move_2 = reshape([linspace(cos(place_angle) * radius, place_chess_location(1), 100) linspace(sin(place_angle) * radius, place_chess_location(2), 100) 20 * ones(1, 100)], 100, 3);
move = [move_1; move_2];

place = reshape([place_chess_location(1) * ones(1, 100) place_chess_location(2) * ones(1, 100) linspace(height + 20, height, 100)], 100, 3);
traj = [pick; move; place];

plot3(traj(:, 1), traj(:, 2), traj(:, 3), 'r');
view(120, 120)
zlim([-50, 60])

T = transl(traj);
q = arm.ikine(T, 'mask', [1 1 1 0 0 0]);
hold on;
% arm.plot(q)

for i = 1:size(q)
    arm.plot(q(i, :));
    gif_generator(fig5, './whole_traj.gif', i, 0.1)
end
% print('./whole_traj_curve.tif', fig5, '-r600', '-dtiffn')

% some other useful maths
% J = arm.jacob0(start_postion);
%{
 J =

   -0.0000  -23.4730   -3.4730
   19.6962    0.0000    0.0000
   -0.0000   19.6962   19.6962
         0         0         0
   -0.0000   -1.0000   -1.0000
    1.0000    0.0000    0.0000 
%}

