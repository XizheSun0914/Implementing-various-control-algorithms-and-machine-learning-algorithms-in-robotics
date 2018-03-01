function HapticController(block)

setup(block);
  

function setup(block)

  block.NumInputPorts  = 2; % velocity of the prone in global coordinate system, pose of virtual object
  block.NumOutputPorts = 3; %force, torque in global system
  
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';


  %% kWall, bWall, nWall, pWall, dimensions, tsampling, pBall0, vBall0
  block.NumDialogPrms     = 8;
  
  tsampling = block.DialogPrm(6).Data;
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode = tsampling;
  block.OutputPort(3).SamplingMode = tsampling;
  
  dimensions = block.DialogPrm(5).Data;
  block.InputPort(1).Dimensions = dimensions;
  block.InputPort(2).Dimensions = dimensions+1;
  block.OutputPort(1).Dimensions = dimensions;
  block.OutputPort(2).Dimensions = dimensions-1;
  block.OutputPort(3).Dimensions = dimensions;
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

   
function DoPostPropSetup(block)
  
  dimensions = block.DialogPrm(5).Data;
  block.NumDworks = 2;
  
  block.Dwork(1).Name            = 'pBall_n_1';
  block.Dwork(1).Dimensions      = dimensions;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'vBall_n_1';
  block.Dwork(2).Dimensions      = dimensions;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  pBall0 = block.DialogPrm(7).Data;
  vBall0 = block.DialogPrm(8).Data;

  block.Dwork(1).Data = pBall0;
  block.Dwork(2).Data = vBall0; 
   

function Outputs(block)
  
  kWall = block.DialogPrm(1).Data;
  bWall = block.DialogPrm(2).Data;
  nWall = block.DialogPrm(3).Data;
  pWall = block.DialogPrm(4).Data;
  dimensions = block.DialogPrm(5).Data;
  tsampling = block.DialogPrm(6).Data;
  vBall_global_n = block.InputPort(1).Data;
  pBall_global_n_1 = block.Dwork(1).Data;
  vBall_global_n_1 = block.Dwork(2).Data;
  pBall_global_n = pBall_global_n_1 + vBall_global_n_1*tsampling;  % compute pBall_n and vBall_n in global coordinate system
  
  pose = block.InputPort(2).Data;
  x = pose(1);
  y = pose(2);
  theta = pose(3);
  Trans = [cos(theta), -sin(theta), x; sin(theta), cos(theta), y; 0, 0, 1];  % compute transformation matrix
  
  pBall_global = [pBall_global_n; 1];  %% homogeneous to transform from global to local
  vBall_global = [vBall_global_n; 1];
  pBall_local = (Trans)\pBall_global;
  vBall_local = (Trans)\vBall_global; %% get position and velocity in local
  
  pBall = [pBall_local(1); pBall_local(2)]; %% extracting useful elements, collision detection and compute reaction force
  vBall = [vBall_local(1); vBall_local(2)];
  pWall_pBall_2 = pBall-pWall;
  if pWall_pBall_2'*nWall <= 0
      Fe_local = kWall*abs(nWall'*pWall_pBall_2)*nWall/(nWall'*nWall) - bWall*(nWall'*vBall)*nWall/(nWall'*nWall);
      if Fe_local'*nWall<0
          Fe_local = zeros(dimensions,1);
      end
  else
      Fe_local = zeros(dimensions,1);
  end
  
  Fe_global_3 = Trans*[Fe_local;0];
  Fe_global_2 = [Fe_global_3(1); Fe_global_3(2)];
  
  pWall_pBall_3 = [pWall_pBall_2; 0]; % homogeneous to use cross product and get torque
  Fe_3 = [Fe_local; 0];
  torque_3 = cross(pWall_pBall_3, -Fe_3);
  torque = torque_3(3);
  
  
  block.OutputPort(1).Data = Fe_global_2;
  block.OutputPort(2).Data = torque;
  block.OutputPort(3).Data = pBall_global_n;
  

function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(3).Data;
  block.Dwork(2).Data = block.InputPort(1).Data;
  

function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);
