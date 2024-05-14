



clear all
close all
clc

import org.opensim.modeling.*

%% general setting
folder_path = pwd;
Path_OpenSimToolbox = strcat(folder_path, '\..\OpenSimToolbox');
geometry_folder_path = strcat(Path_OpenSimToolbox,'\geometry_folder\Geometry_MoBL_ARMS\');
ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);

Visualization_on = 0;
%% load model

model = osim_model('MOBL_ARMS_fixed_41.osim');

% check body list, joint list, ...
model.BodySet_list;
model.JointSet_list;
model.CoordinateSet_list;
model.MuscleSet_list;

% delete markers
model.delete_all_markers;
%% define endefector/interest points as marker_point 
body_name = {'hand'}; % name of the attached body
pos_vec = {Vec3(0,-0.1,0)}; % relative position in corresponding body frame
model.add_marker_points('Hand_endeffector', body_name,pos_vec);
model.set_visualize;
%% set the list of the target coordinates/frames and muscles

muscle_list = {'ECRL','ECRB','ECU','FCR','FCU','EDCI'};
coord_list = {'elv_angle','shoulder_elv', 'shoulder_rot', 'elbow_flexion',...
    'pro_sup','deviation','flexion'};

n_muscle = length(muscle_list);
n_q = length(coord_list);
% coord_value = rand(7,1);%[0 0 0 1 1 0 1];
% model.set_coordinate_value(coord_list,coord_value);
% coord_q_value = model.get_coordinate_value(coord_list);

%% set Cartesian sampling parameters
% define Cartesian workspace 
workspace_cube_ori = [0;0;0];
workspace_cube_length = 0.6; % in meter
voxel_distance = 0.1; % sample distance in meter

[x_voxel,y_voxel,z_voxel] = ndgrid(...
        workspace_cube_ori(1)-workspace_cube_length:voxel_distance:workspace_cube_ori(1)+workspace_cube_length,...
        workspace_cube_ori(2)-workspace_cube_length:voxel_distance:workspace_cube_ori(2)+workspace_cube_length,...
        workspace_cube_ori(3)-workspace_cube_length:voxel_distance:workspace_cube_ori(3)+workspace_cube_length);
n_voxel = size(x_voxel,1)*size(y_voxel,2)*size(z_voxel,3);
% figure(101)
% for i = 1:size(x_voxel,1)
%     for j = 1: size(y_voxel,2)
%         for k = 1:size(z_voxel,3)
%             x_cube_i = [x_voxel(i,j,k);y_voxel(i,j,k);z_voxel(i,j,k)];
%             plot3(x_cube_i(1),x_cube_i(2),x_cube_i(3),'.')
%             hold on
%         end
%     end
% end

% generate a lookup table 
dataset_voxel_pos_table = zeros(n_voxel,3);
index = 1;
for i = 1:size(x_voxel,1)
    for j = 1: size(y_voxel,2)
        for k = 1:size(z_voxel,3)
            dataset_voxel_pos_table(index,:) = [x_voxel(i,j,k),y_voxel(i,j,k),z_voxel(i,j,k)];
            index = index+1;
        end
    end
end

%% Visualization
% model.model_visualize
% model.plot_all_body;
% model.plot_world_frame;
% model.plot_mp_frame;

%% run code for each voxel
% discretize the orientation
Orientation_set = {[0;0;0], [0;0;pi/2], [0;0;pi], [0;0;pi*3/2], [0;pi/2;0], [0;-pi/2;0]};
n_ori = length(Orientation_set);
% IK tolorance
tol_p = 1e-2;
tol_ori = 1/180*pi;

% dataset structure
% dimension 1: index of voxel
% dimension 2: index of orientation
% dimension 3: index of elbow configuration  
% dimension 4: 
%       data_struct

data_struct = struct(...
    'x', 0, ...
    'IK_status',0,...
    'q', 0, ...
    'J_rank', 0, ...
    'J', zeros(6,n_q),...
    'MomentArm',zeros(n_q,n_muscle),...
    'MomentArm_rank', 0,...
    'MassMatrix', zeros(n_q,n_q),...
    'Gravity', zeros(n_q,1),...
    'MaxIsometricForce', zeros(n_muscle,1),...
    'PennationAngle', zeros(n_muscle,1),...
    'MuscleLength', zeros(n_muscle,1));

dataset = {};

for i = 1:size(dataset_voxel_pos_table,1)
    % get position of the voxel
    p_i = dataset_voxel_pos_table(i,:);
    for j = 1:n_ori % 8 oriantation for each p_i
        % 6 dimensional x
        x_i = [p_i;Orientation_set{j}];

        for k = 1:n_elbow
            dataset{i,j,k} = data_struct;
            dataset{i,j,k}.x = x_i;

            % run Inverse Kinematic
            [q_i,x_res,phi_x,iter] = model.ik_numeric(coord_list, 1, p_i,...
                200, [1e-3*ones(3,1);5e-2*ones(3,1)],0.2);

            if max(abs(phi_x(1:3))) > tol_p | max(abs(phi_x(4:6))) > tol_ori
                %
                dataset{i,j,k}.IK_status = 0;
                break
            end

            dataset{i,j,k}.IK_status = 1;
            dataset{i,j,k}.q = q_i;
            % set q to model
            model.set_coordinate_value(coord_list,q_i);
            
            % visualization
            if Visualization_on
                % OpenSim API plot
                model.model_visualize; 
                % matlab plot
                model.plot_all_body; 
                model.plot_world_frame;
                model.plot_mp_frame;
            end

            % Jacobian
            Jacobian_sub = model.getJacobian_point_sub(1,coord_list );
            dataset{i,j,k}.J = Jacobian_sub; % 6xn_q
            dataset{i,j,k}.J_rank = rank(Jacobian_sub);
            
            % Moment arm matrix
            MA_matrix = model.get_MomentArmMatrix(coord_list, muscle_list);
            dataset{i,j,k}.MomentArm = MA_matrix;
            dataset{i,j,k}.MomentArm_rank = rank(MA_matrix);

            % Mass Matrix
            M_sub = model.getMassMatrix_sub(coord_list);
            dataset{i,j,k}.MassMatrix = M_sub;

            % get muscle properties
            dataset{i,j,k}.MaxIsometricForce = model.get_MaxIsometricForce(muscle_list);
            dataset{i,j,k}.PennationAngle = model.get_PennationAngle(muscle_list);
            dataset{i,j,k}.MuscleLength = model.get_muscleLength(muscle_list);


            % metric calculation
            

        end
    end
end




