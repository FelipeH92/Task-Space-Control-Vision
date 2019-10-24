clear all
close all
clc

bag = rosbag('BAGS - DUAL/2019-06-05-06-57-56.bag');
msgStructs = readMessages(bag,'DataFormat','struct');

xPointsCurrent = [];
yPointsCurrent = [];
zPointsCurrent = [];
xQuaternionCurrent = [];
yQuaternionCurrent = [];
zQuaternionCurrent = [];
wQuaternionCurrent = [];

i = 1;

aux = false;

while aux ~= true
    if strcmp(msgStructs{i}.Transforms.ChildFrameId,'initial_pose')
        initialPoseTranslation = [msgStructs{i}.Transforms.Transform.Translation.X, msgStructs{i}.Transforms.Transform.Translation.Y, msgStructs{i}.Transforms.Transform.Translation.Z];
        initialPoseQuaternion = [msgStructs{i}.Transforms.Transform.Rotation.X, msgStructs{i}.Transforms.Transform.Rotation.Y, msgStructs{i}.Transforms.Transform.Rotation.Z, msgStructs{i}.Transforms.Transform.Rotation.W];
        aux = true;
    end
    i = i+1;
end

i = 1;

while i < size(msgStructs,1)
    
    if strcmp(msgStructs{i}.Transforms.ChildFrameId,'current_pose')
        xPointsCurrent = [xPointsCurrent; msgStructs{i}.Transforms.Transform.Translation.X];
        yPointsCurrent = [yPointsCurrent; msgStructs{i}.Transforms.Transform.Translation.Y];
        zPointsCurrent = [zPointsCurrent; msgStructs{i}.Transforms.Transform.Translation.Z];
        xQuaternionCurrent = [xQuaternionCurrent; msgStructs{i}.Transforms.Transform.Rotation.X];
        yQuaternionCurrent = [yQuaternionCurrent; msgStructs{i}.Transforms.Transform.Rotation.Y];
        zQuaternionCurrent = [zQuaternionCurrent; msgStructs{i}.Transforms.Transform.Rotation.Z];
        wQuaternionCurrent = [wQuaternionCurrent; msgStructs{i}.Transforms.Transform.Rotation.W];
    end
    i = i + 1;
end

%plot3(xPointsCurrent,yPointsCurrent,zPointsCurrent)

filename = '/home/nascimento/Projects/VREP/Scenes/Matlab-VREP/BAGS - DUAL/Mat Files/';
mkdir(filename);
filename = strcat(filename,'/DualBox_Vision_9.mat');
save(filename,'initialPoseTranslation','initialPoseQuaternion','xPointsCurrent','yPointsCurrent','zPointsCurrent','xQuaternionCurrent','yQuaternionCurrent','zQuaternionCurrent','wQuaternionCurrent');
disp('Data Saved.');