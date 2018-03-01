function VirtualEnvironment5(block)

setup(block);

function setup(block)

  block.NumInputPorts  = 2; % pos,vel
  block.NumOutputPorts = 5; % force,energy
  
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

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
  block.OutputPort(4).DatatypeID  = 0; % double
  block.OutputPort(4).Complexity  = 'Real';

  %% kWall, bWall, nWall, xWall, dof, tsampling, qerror
  block.NumDialogPrms     = 7;

  dof = block.DialogPrm(5).Data;
  tsampling = block.DialogPrm(6).Data;
  
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions= dof;
  block.OutputPort(2).Dimensions= dof;
  block.OutputPort(3).Dimensions= 1;
  block.OutputPort(4).Dimensions= 1;
  block.OutputPort(5).Dimensions= 1;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  block.OutputPort(2).SamplingMode= tsampling;
  block.OutputPort(3).SamplingMode= tsampling;
  block.OutputPort(4).SamplingMode= tsampling;
  block.OutputPort(5).SamplingMode= tsampling;
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);
 
    
function DoPostPropSetup(block)
  block.NumDworks = 5;
  
  dof = block.DialogPrm(5).Data;
  
  block.Dwork(1).Name            = 'dpen_1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'Fe_1';
  block.Dwork(2).Dimensions      = dof;
  block.Dwork(2).DatatypeID      = 0;      % uint32
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'Energy_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % uint32
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'Eov_1';
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % uint32
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  block.Dwork(5).Name            = 'vel_1';
  block.Dwork(5).Dimensions      = dof;
  block.Dwork(5).DatatypeID      = 0;      % uint32
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  dof = block.DialogPrm(5).Data;
  
  block.Dwork(1).Data = 0;
  block.Dwork(2).Data = zeros(dof,1); 
  block.Dwork(3).Data = 0;
  block.Dwork(4).Data = 0;
  block.Dwork(5).Data = 0;
  

function Outputs(block)
  
  pos = block.InputPort(1).Data;
  vel = block.InputPort(2).Data;
  
  kWall = block.DialogPrm(1).Data;
  bWall = block.DialogPrm(2).Data;
  nWall = block.DialogPrm(3).Data;
  xWall = block.DialogPrm(4).Data;
  qerror = block.DialogPrm(7).Data;
  
  if (pos-xWall)*nWall<0
      dpen = abs(pos-xWall);
  else
      dpen = 0;
      vel=0;
  end
  
  dpen_1 = block.Dwork(1).Data;
  Fe_1 = block.Dwork(2).Data;
  Energy_1 = block.Dwork(3).Data;
  Eov_1 = block.Dwork(4).Data;
  vel_1 = block.Dwork(5).Data;
  Energy = Energy_1+Fe_1*(dpen-dpen_1);
  dpen_p = 2*dpen-dpen_1;
 
  if vel_1>=0 && vel<=0
      Eov_1=0;
  end
  
  if dpen_p>0
      [delta1,Eov] = OptimalEov(Fe_1,dpen_1,dpen,kWall,Eov_1,qerror);
      if dpen_p+delta1-dpen>10^(-19)
          fov = Eov/(dpen_p+delta1-dpen);
      else
          fov = 0;
      end
  else
      Eov = 0;
      delta1=0;
      fov = 0;
  end
  Fe = (kWall*(dpen_p+delta1)+fov)*nWall-bWall*vel;
  if Fe*nWall<0
      Fe = 0;
  end
  
  block.OutputPort(1).Data = dpen;
  block.OutputPort(2).Data = Fe;
  block.OutputPort(3).Data = Energy;
  block.OutputPort(4).Data = Eov;
  block.OutputPort(5).Data = dpen_p+delta1-dpen;
  
  

function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(2).Data;
  block.Dwork(3).Data = block.OutputPort(3).Data;
  block.Dwork(4).Data = block.OutputPort(4).Data;
  block.Dwork(5).Data = block.InputPort(2).Data;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);