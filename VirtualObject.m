function VirtualObject(block)

setup(block);
  
function setup(block)

  block.NumInputPorts  = 2;  % Force, Torque in global coordinate system
  block.NumOutputPorts = 2;  % Pose, Pose_dot in global coordinate system
  
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';

  %% mass, Inertia, dimensions, tsampling, pose0, pose_dot0
  block.NumDialogPrms     = 6;
  
  tsampling = block.DialogPrm(4).Data;
  block.SampleTimes = [tsampling 0];

  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  block.OutputPort(2).SamplingMode= tsampling;
  
  dimensions = block.DialogPrm(3).Data;
  block.InputPort(1).Dimensions = dimensions; % force is 2dof
  block.InputPort(2).Dimensions = dimensions-1; % torque is 1dof
  block.OutputPort(1).Dimensions= dimensions+1; % pose is 3dof  x,y,theta
  block.OutputPort(2).Dimensions= dimensions+1; % pose_dot is 3dof
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)

  block.NumDworks = 2;
  dimensions = block.DialogPrm(3).Data;
  
  block.Dwork(1).Name            = 'pose_n_1';
  block.Dwork(1).Dimensions      = dimensions+1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'pose_dot_n_1';
  block.Dwork(2).Dimensions      = dimensions+1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;
  

function Start(block)
  
  pose0 = block.DialogPrm(5).Data;
  pose_dot0 = block.DialogPrm(6).Data;

  block.Dwork(1).Data = pose0;
  block.Dwork(2).Data = pose_dot0; 
   

function Outputs(block)
  
  tsampling = block.DialogPrm(4).Data;
  inertia = block.DialogPrm(2).Data;
  mass = block.DialogPrm(1).Data;
  
  pose_n_1 = block.Dwork(1).Data;
  pose_dot_n_1 = block.Dwork(2).Data;
  
  Mass_Inertia = [mass 0 0; 0 mass 0; 0 0 inertia];
  force = block.InputPort(1).Data;
  torque = block.InputPort(2).Data;
  force_torque = [force(1); force(2); torque];
  pose_2dot = Mass_Inertia\force_torque;

  pose_dot_n = pose_dot_n_1 + pose_2dot*tsampling;
  pose_n = pose_n_1 + pose_dot_n_1*tsampling;
  
  block.OutputPort(1).Data = pose_n;
  block.OutputPort(2).Data = pose_dot_n;


function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(2).Data;
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);
