clear all;
close all;
clc;

N=7;                        % Number of agents
max_iter = 1000;            % How many iterations to run the code for. Each iteration is about 0.033s.

videoFLag = 0;                          % Change to 1 to record video
circularInitialConditions = 1;          % Change to 0 for random initial condition (needed for Q2.d)

xybound = [-0.9, 0.9, -0.9, 0.9];
p_theta = (1:2:2*N)/(2*N)*2*pi;
p_circ = [xybound(2)*cos(p_theta) xybound(2)*cos(p_theta+pi); xybound(4)*sin(p_theta)  xybound(4)*sin(p_theta+pi)];

circularTargets = p_circ(:,1:N);

% Initialize robotarium
if circularInitialConditions
    r = Robotarium('NumberOfRobots',N, 'InitialConditions', [circularTargets;zeros(1,N)]);
else
    r = Robotarium('NumberOfRobots',N);
end
si_to_uni_dyn = create_si_to_uni_dynamics();                        % This is the nifty diffeomorphism that allows single integrator commands to be followed by our nonlinear robot.
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();  % This makes sure robots do not collide or leave the testbed.

A = [0 1 1 1 1 1 1;
     1 0 1 1 1 1 1;
     1 1 0 1 1 1 1;
     1 1 1 0 1 1 1;
     1 1 1 1 0 1 1;
     1 1 1 1 1 0 1;
     1 1 1 1 1 1 0];
D = (N - 1) * eye(N);
L = D - A;

% Get initial poses of robots
xuni = r.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                         % x-y positions only
r.step();                                                % Run robotarium step

% Initialize video
if videoFLag
    vid = VideoWriter('HW4_formationControl.mp4', 'MPEG-4'); % Must not be called when submitted to robotarium
    vid.Quality = 100;
    vid.FrameRate = 72;
    open(vid);
    writeVideo(vid, getframe(gcf));
end

for k = 1:max_iter

    % Get new data and initialize new null velocities
    x = r.get_poses();                                   % Get new robots' states
    xi = x(1:2,:);                                        % Extract single integrator states

    dxi=zeros(2,N);                                         % Initialize single integrator velocities to zero

%     % FILL THIS PART!!!
    range = 1:N;
    for i=range
        for j=range(A(i,:)==1)
            dxi(:,i) = dxi(:,i) + ... % FILL IN THIS PART
        end
    end

    dxu = si_to_uni_dyn(dxi, x);                            % Convert single integrator inputs into unicycle inputs
    dxu = uni_barrier_cert(dxu,x);                          % Avoid inter robot collisions and robots from escaping
    r.set_velocities(1:N, dxu);                             % Set new velocities to robots
    r.step();                                               % Send velocites and prep for next step.

    if videoFLag && mod(k,10)                               % Record a video frame every 10 iterations
            writeVideo(vid, getframe(gcf));
    end


end


if videoFLag
    close(vid);
end

%Debug information. This will display any errors that would cause a
%submission to be rejected on the robotarium.
r.debug();
