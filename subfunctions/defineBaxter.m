function [baxter_const, baxter_structure] = defineBaxter(varargin)
    % DEFINEBAXTER
    %
    % [baxter_const, baxter_structure] = defineBaxter()
    % [baxter_const, baxter_structure] = defineBaxter(...) 
    %                           allows additional optional parameters
    %       'Origin'        :   default [eye(3) [0;0;0]; [0 0 0] 1]
    %       'Pedestal'      :   'on'/'off' (default on)
    %       'LeftGripper' / :   structure with creation options accepted by
    %       'RightGripper'      baxterGripper as fields
    %                               -> param 
    %                               -> props
    %                           default: no gripper attached
    %
    % define-file for the Rethink Robotics Baxter.  Returns struct array 
    %   with the following form:
    %
    % root
    %     -> name                 : (1) 'baxter_left_arm'
    %                               (2) 'baxter_right_arm'
    %                               (3) 'baxter_head'
    %     -> kin
    %       -> H                : joint axes
    %       -> P                : inter-joint translation
    %       -> joint_type       : joint types
    %     -> limit
    %       -> upper_joint_limit :  upper joint limits [rad]
    %       -> lower_joint_limit :  lower joint limits [rad]
    %       -> velocity_limit    :  velocity limits    [rad/s]
    %       -> effort_limit      :  effort limits      [Nm/s]
    %     -> vis
    %       -> joints      :  struct array of joint definitions
    %       -> links       :  struct array of link definitions
    %       -> frame       :  3D frame dimensions
    %       -> peripherals :  variable length struct array for
    %                           (1) arm mount
    %                           (2) arm mount
    %                           (3) (sonar head, neck, [opt] pedestal)
    %
    % See also CREATEROBOT, CREATECOMBINEDROBOT, DEFINEBAXTERGRIPPER
    
    
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    flags = {'Origin', 'Pedestal', 'LeftGripper', 'RightGripper'};
    defaults = {[eye(3) zed; zed' 1], 'on', [], []};
    
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    origin = opt_values{1};
    cp = opt_values{2};
    left_gripper_options = opt_values{3};
    right_gripper_options = opt_values{4};
    
    n_grippers = ~isempty(left_gripper_options) + ...
                    ~isempty(right_gripper_options);

    R0 = origin(1:3,1:3);
    t0 = origin(1:3,4);
    
    joint_radius = [.075;.075;.075;.07;.07;.06;.06];
    joint_height = [.1;.19;.15;.18;.14;.15;.12];
    
    upper_joint_limit = [97.5; 60; 175; 150; 175; 120; 175]*pi/180;
    lower_joint_limit = [-97.5;-123; -175; -2.5; -175; -90; -175]*pi/180;
    velocity_limit = [2.5;2.5;2.5;2.5;5;5;5];
    torque_limit = [60;60;60;60;20;20;20];
    
    link_type1_props = {'FaceColor', [.9;0;0], ...
                            'EdgeAlpha', 0};
    link_type2_props = {'FaceColor', [.2;0.2;0.2], ...
                            'EdgeAlpha', 0};
    joint_type1_props = {'FaceColor', [0.2;0.2;0.2]};
    joint_type2_props = {'FaceColor', [0.9;0;0]};
    
    arm_mount_props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    arm_mount_param = struct('width',0.2,'length',0.15,'height',0.05);
    
    % Grab standard robot structure
    baxter_const = defineEmptyRobot(3 + 2*n_grippers);
    
    %%% Left Arm
    % Kinematic Constants
    baxter_const(1).name = 'baxter_left_arm';
    baxter_const(1).kin.H = R0*rot(z0,pi/4)*[z0 y0 x0 y0 x0 y0 x0];
    baxter_const(1).kin.P = R0*[[0.06375;.25888;0.119217], ...
                            rot(z0,pi/4)*[[0.069;0;0.27035], ...
                            zed, [0.36435;0;-0.069], ...
                            zed, [0.37429;0;-0.01], ...
                            zed, 0.229525*x0]];
    baxter_const(1).kin.P(:,1) = t0 + baxter_const(1).kin.P(:,1);
    baxter_const(1).kin.joint_type = zeros(1,7);
    
    % Dynamic Limits
    baxter_const(1).limit.upper_joint_limit = upper_joint_limit;
    baxter_const(1).limit.lower_joint_limit = lower_joint_limit;
    baxter_const(1).limit.velocity_limit = velocity_limit;
    baxter_const(1).limit.effort_limit = torque_limit;
    
    % Visualization definitions
    baxter_const(1).vis.joints = struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        baxter_const(1).vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        baxter_const(1).vis.joints(n).props = joint_type1_props;
    end
    baxter_const(1).vis.joints(2).props = joint_type2_props;
    baxter_const(1).vis.joints(3).props = joint_type2_props;
    baxter_const(1).vis.joints(5).props = joint_type2_props;
    
    baxter_const(1).vis.links = struct('handle',cell(1,8), ...
                            'R',cell(1,8),'t',cell(1,8), ...
                            'param',cell(1,8),'props',cell(1,8));

    baxter_const(1).vis.links(2).handle = @createCylinder;
    baxter_const(1).vis.links(2).R = eye(3);
    baxter_const(1).vis.links(2).t = [0;0;0.1777];
    baxter_const(1).vis.links(2).param = struct('radius',0.075,'height',0.2553);
    baxter_const(1).vis.links(2).props = link_type2_props;
    
    baxter_const(1).vis.links(4).handle = @createCylinder;
    baxter_const(1).vis.links(4).R = rot(z0,pi/4)*rot(y0,pi/2);
    baxter_const(1).vis.links(4).t = rot(z0,pi/4)*[.1847;0;0];
    baxter_const(1).vis.links(4).param = struct('radius',0.075,'height',.2193);
    baxter_const(1).vis.links(4).props = link_type1_props;
    
    baxter_const(1).vis.links(6).handle = @createCylinder;
    baxter_const(1).vis.links(6).R = rot(z0,pi/4)*rot(y0,pi/2);
    baxter_const(1).vis.links(6).t = rot(z0,pi/4)*[.1921;0;0];
    baxter_const(1).vis.links(6).param = struct('radius',0.07,'height',.2443);
    baxter_const(1).vis.links(6).props = link_type1_props;
        
    baxter_const(1).vis.links(8).handle = @createCylinder;
    baxter_const(1).vis.links(8).R = rot(z0,pi/4)*rot(y0,pi/2);
    baxter_const(1).vis.links(8).t = rot(z0,pi/4)*[.1448;0;0];
    baxter_const(1).vis.links(8).param = struct('radius',0.05,'height',.1695);
    baxter_const(1).vis.links(8).props = link_type2_props;
    
    baxter_const(1).vis.frame = struct('scale',0.2,'width',0.01);
    
    % Peripheral arm mount
    baxter_const(1).vis.peripherals.id = 'arm_mount';
    baxter_const(1).vis.peripherals.frame = 'base';
    baxter_const(1).vis.peripherals.handle = @createCuboid;
    baxter_const(1).vis.peripherals.R = R0*rot(z0,75*pi/180);
    baxter_const(1).vis.peripherals.t = ...
                            t0 + R0*([0.06375;.25888;0.119217] - ...
                                    rot(z0,75*pi/180)*[0.025;0;0.075]);
    baxter_const(1).vis.peripherals.param = arm_mount_param;
    baxter_const(1).vis.peripherals.props = arm_mount_props;
    
    %%% Right Arm
    % Kinematic Constants
    baxter_const(2).name = 'baxter_right_arm';
    baxter_const(2).kin.H = R0*rot(z0,-pi/4)*[z0 y0 x0 y0 x0 y0 x0];
    baxter_const(2).kin.P = R0*[[0.06375;-.25888;0.119217], ...
                            rot(z0,-pi/4)*[[0.069;0;0.27035], ...
                            zed, [0.36435;0;-0.069], ...
                            zed, [0.37429;0;-0.01], ...
                            zed, 0.229525*x0]];
    baxter_const(2).kin.P(:,1) = t0 + baxter_const(2).kin.P(:,1);
    baxter_const(2).kin.joint_type = zeros(1,7);
    
    
    % Dynamic Limits
    baxter_const(2).limit.upper_joint_limit = upper_joint_limit;
    baxter_const(2).limit.lower_joint_limit = lower_joint_limit;
    baxter_const(2).limit.velocity_limit = velocity_limit;
    baxter_const(2).limit.effort_limit = torque_limit;
    
    % Visualization definitions
    baxter_const(2).vis.joints = struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        baxter_const(2).vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        baxter_const(2).vis.joints(n).props = joint_type1_props;
    end
    baxter_const(2).vis.joints(2).props = joint_type2_props;
    baxter_const(2).vis.joints(3).props = joint_type2_props;
    baxter_const(2).vis.joints(5).props = joint_type2_props;
    
    baxter_const(2).vis.links = struct('handle',cell(1,8), ...
                            'R',cell(1,8),'t',cell(1,8), ...
                            'param',cell(1,8),'props',cell(1,8));

    baxter_const(2).vis.links(2).handle = @createCylinder;
    baxter_const(2).vis.links(2).R = eye(3);
    baxter_const(2).vis.links(2).t = [0;0;0.1777];
    baxter_const(2).vis.links(2).param = struct('radius',0.075,'height',0.2553);
    baxter_const(2).vis.links(2).props = link_type2_props;
    
    baxter_const(2).vis.links(4).handle = @createCylinder;
    baxter_const(2).vis.links(4).R = rot(z0,-pi/4)*rot(y0,pi/2);
    baxter_const(2).vis.links(4).t = rot(z0,-pi/4)*[.1847;0;0];
    baxter_const(2).vis.links(4).param = struct('radius',0.075,'height',.2193);
    baxter_const(2).vis.links(4).props = link_type1_props;
    
    baxter_const(2).vis.links(6).handle = @createCylinder;
    baxter_const(2).vis.links(6).R = rot(z0,-pi/4)*rot(y0,pi/2);
    baxter_const(2).vis.links(6).t = rot(z0,-pi/4)*[.1921;0;0];
    baxter_const(2).vis.links(6).param = struct('radius',0.07,'height',.2443);
    baxter_const(2).vis.links(6).props = link_type1_props;
        
    baxter_const(2).vis.links(8).handle = @createCylinder;
    baxter_const(2).vis.links(8).R = rot(z0,-pi/4)*rot(y0,pi/2);
    baxter_const(2).vis.links(8).t = rot(z0,-pi/4)*[.1448;0;0];
    baxter_const(2).vis.links(8).param = struct('radius',0.05,'height',.1695);
    baxter_const(2).vis.links(8).props = link_type2_props;
    
    baxter_const(2).vis.frame = struct('scale',0.2,'width',0.01);
    
    % Peripheral arm mount
    baxter_const(2).vis.peripherals.id = 'arm_mount';
    baxter_const(2).vis.peripherals.frame = 'base';
    baxter_const(2).vis.peripherals.handle = @createCuboid;
    baxter_const(2).vis.peripherals.R = R0*rot(z0,-75*pi/180);
    baxter_const(2).vis.peripherals.t = ...
                            t0 + R0*([0.06375;-.25888;0.119217] - ...
                                    rot(z0,-75*pi/180)*[0.025;0;0.075]);
    baxter_const(2).vis.peripherals.param = arm_mount_param;
    baxter_const(2).vis.peripherals.props = arm_mount_props;
    
    
    %%% Head
    % Kinematic Constants
    baxter_const(3).name = 'baxter_head';
    baxter_const(3).kin.H = R0*z0;
    baxter_const(3).kin.P = R0*[[.0599;0;.6955] zed];
    baxter_const(3).kin.P(:,1) = t0 + baxter_const(3).kin.P(:,1);
    baxter_const(3).kin.joint_type = 0;
    
    % Dynamic Limits
    baxter_const(3).limit.upper_joint_limit = pi/2;
    baxter_const(3).limit.lower_joint_limit = -pi/2;
    
    % Visualization definition
    baxter_const(3).vis.joints(1).param = struct('radius',0.06,'height',0.08);
    baxter_const(3).vis.joints(1).props = {'FaceColor',[0.2;0.2;0.2], ...
                                        'EdgeAlpha',0};
    
    baxter_const(3).vis.links(1).handle = @createCuboid;
    baxter_const(3).vis.links(1).R = eye(3);
    baxter_const(3).vis.links(1).t = t0 + [-.01;0;.26];
    baxter_const(3).vis.links(1).param = ...
                    struct('width',0.33, 'length',0.31,'height',.52);
    baxter_const(3).vis.links(1).props = {'FaceColor', [.2;0.2;0.2], ...
                                    'EdgeColor', [0;0;0], ...
                                    'EdgeAlpha', 1};

    baxter_const(3).vis.links(2).handle = @createCuboid;
    baxter_const(3).vis.links(2).R = rot(y0,pi/9)*rot(z0,pi/2);
    baxter_const(3).vis.links(2).t = [.1039;0;-0.0038];
    baxter_const(3).vis.links(2).param = ...
                    struct('width',0.3, 'length',0.02,'height',.2);
    baxter_const(3).vis.links(2).props = {'FaceColor', [0.9;0;0],...
                                    'EdgeColor',[0.5;0.5;0.5]};
    
    baxter_const(3).vis.frame = struct('scale',0.2,'width',0.01);
    
    % Peripheral head and neck
    baxter_const(3).vis.peripherals(1).id = 'sonar_head';
    baxter_const(3).vis.peripherals(1).frame = 'base';
    baxter_const(3).vis.peripherals(1).handle = @createCylinder;
    baxter_const(3).vis.peripherals(1).R = R0;
    baxter_const(3).vis.peripherals(1).t = t0 + R0*[.0599;0;.7755];
    baxter_const(3).vis.peripherals(1).param = struct('radius', 0.075, ...
                                              'radius2', 0.06, ...
                                              'height', 0.16);
    baxter_const(3).vis.peripherals(1).props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    
    baxter_const(3).vis.peripherals(2).id = 'neck';
    baxter_const(3).vis.peripherals(2).frame = 'base';
    baxter_const(3).vis.peripherals(2).handle = @createCylinder;
    baxter_const(3).vis.peripherals(2).R = R0*rot(y0,pi/8);
    baxter_const(3).vis.peripherals(2).t = R0*[-0.0796;0;0.6355];
    baxter_const(3).vis.peripherals(2).param = struct('radius', 0.04, 'height', 0.28);
    baxter_const(3).vis.peripherals(2).props = {'FaceColor',[0.9;0;0],'EdgeAlpha',0};
    
    % Pedestal (optional)
    if strcmp(cp,'on')
        baxter_const(3).vis.peripherals(3).id = 'pedestal_top';
        baxter_const(3).vis.peripherals(3).frame = 'base';
        baxter_const(3).vis.peripherals(3).handle = @createCylinder;
        baxter_const(3).vis.peripherals(3).R = R0;
        baxter_const(3).vis.peripherals(3).t = t0 - R0*.06*z0;
        baxter_const(3).vis.peripherals(3).param = struct('radius', 0.18, ...
                                                    'height', 0.12);
        baxter_const(3).vis.peripherals(3).props = ...
                            {'FaceColor', [0.4;0.4;0.4], 'EdgeAlpha', 0};
        
        baxter_const(3).vis.peripherals(4).id = 'pedestal_body';
        baxter_const(3).vis.peripherals(4).frame = 'base';
        baxter_const(3).vis.peripherals(4).handle = @createCylinder;
        baxter_const(3).vis.peripherals(4).R = R0;
        baxter_const(3).vis.peripherals(4).t = t0 - R0*.37*z0;
        baxter_const(3).vis.peripherals(4).param = struct('radius', 0.1, ...
                                                    'height', 0.5);
        baxter_const(3).vis.peripherals(4).props = ...
                            {'FaceColor', [0.2;0.2;0.2], 'EdgeAlpha', 0};
                        
        base_shape = [-.1 .4 .4 .15 .15 .4 .4 -.1 -.5 -.5 -.3 -.3 -.5 -.5; ...
                    .2 .4 .33 .1 -.1 -.33 -.4 -.2 -.4 -.27 -.1 .1 .27 .4];
        baxter_const(3).vis.peripherals(5).id = 'pedestal_base';
        baxter_const(3).vis.peripherals(5).frame = 'base';
        baxter_const(3).vis.peripherals(5).handle = @createPrism;
        baxter_const(3).vis.peripherals(5).R = R0;
        baxter_const(3).vis.peripherals(5).t = t0 - R0*.65*z0;
        baxter_const(3).vis.peripherals(5).param = struct('height', 0.06, ...
                                                    'polygon', base_shape);
        baxter_const(3).vis.peripherals(5).props = ...
                            {'FaceColor', [0.4;0.4;0.4], 'EdgeAlpha', 1};
    end
    
    % Attach grippers if requested
    if n_grippers > 0
        grippers_const = defineEmptyRobot(2*n_grippers);
        grippers_structure = defineEmptyRobotStructure(2*n_grippers);
        if ~isempty(left_gripper_options)
            Ogl = origin*[rot(z0,pi/4) zed; zed' 1];
            [grippers_const(1:2), ...
                grippers_structure(1:2)] = defineBaxterGripper(...
                                        left_gripper_options.param, ...
                                        left_gripper_options.props{:}, ...
                                        'Origin', Ogl, ...
                                        'Name', 'left_gripper');
            [grippers_structure(1:2).left] = deal(baxter_const(1).name);
        end
        if ~isempty(right_gripper_options)
            Ogr = origin*[rot(z0,-pi/4) zed; zed' 1];
            [grippers_const(end-1:end), ...
                grippers_structure(end-1:end)] = defineBaxterGripper(...
                                        right_gripper_options.param, ...
                                        right_gripper_options.props{:}, ...
                                        'Origin', Ogr, ...
                                        'Name', 'right_gripper');
            [grippers_structure(end-1:end).left] = deal(baxter_const(2).name);
        end
        baxter_const(4:3+2*n_grippers) = grippers_const;
    end
        
    
    %%% Define structure for combined robot.  All baxter robots attached 
    % to root, and, if defined, grippers are attached to arms
    baxter_structure = defineEmptyRobotStructure(3 + n_grippers);
    [baxter_structure.name] = baxter_const.name;
    if n_grippers > 0
        baxter_structure(4:3+2*n_grippers) = grippers_structure;
        if ~isempty(left_gripper_options)
            baxter_structure(1).right = {grippers_structure(1:2).name};
        end
        if ~isempty(right_gripper_options)
            baxter_structure(2).right = {grippers_structure(end-1:end).name};
        end
    end
end