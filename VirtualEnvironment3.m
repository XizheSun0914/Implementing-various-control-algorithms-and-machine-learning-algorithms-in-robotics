function VirtualEnvironment3(block)

setup(block);

function setup(block)

  block.NumInputPorts  = 2; % pos, vel
  block.NumOutputPorts = 3; % force, energy
  
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

  %% kBall, bBall, rBall, cBall, dof, tsampling, p0, qerror
  block.NumDialogPrms     = 8;
  
  dof = block.DialogPrm(5).Data;
  tsampling = block.DialogPrm(6).Data;
  
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions= dof;
  block.OutputPort(2).Dimensions= 1;
  block.OutputPort(3).Dimensions= 1;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  block.OutputPort(2).SamplingMode= tsampling;
  block.OutputPort(3).SamplingMode= tsampling;
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)
  block.NumDworks = 3;
  
  dof = block.DialogPrm(5).Data;
  
  block.Dwork(1).Name            = 'pos_1';
  block.Dwork(1).Dimensions      = dof;
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
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  dof = block.DialogPrm(5).Data;
  p0 = block.DialogPrm(7).Data;
  
  block.Dwork(1).Data = p0;
  block.Dwork(2).Data = zeros(dof,1);
  block.Dwork(3).Data = 0;
   

function Outputs(block)
  
  pos = block.InputPort(1).Data;
  vel = block.InputPort(2).Data;
  
  pos_1 = block.Dwork(1).Data;
  Fe_1 = block.Dwork(2).Data;
  Energy_1 = block.Dwork(3).Data;
  
  kBall = block.DialogPrm(1).Data;
  bBall = block.DialogPrm(2).Data;
  rBall = block.DialogPrm(3).Data;
  cBall = block.DialogPrm(4).Data;
  dof = block.DialogPrm(5).Data;
  tsampling = block.DialogPrm(6).Data;
  qerror = block.DialogPrm(8).Data;
  
  pos_p = pos+vel*tsampling;
  dpos = pos-pos_1;
  dpos_p = pos_p-pos;
  if norm(pos-cBall)<rBall
      dpen = rBall-norm(pos-cBall);
      dir = (pos-cBall)/norm(pos-cBall);
  else
      dpen = 0;
      dir = zeros(dof,1);
  end
  Energy = Energy_1 -Fe_1'*dpos;
  Fi = kBall*dpen*dir;
  
  if pos==pos_1
      delta = 0;
  else
      delta = rem(vel(3)*tsampling,qerror);
  end
  
  if norm(pos_p-cBall)<rBall
      dpen_p = rBall-norm(pos_p-cBall)-delta;
      if vel'*dir<0     %% come in
          if norm(Fi)>norm(Fe_1)
              Fe = (kBall*dpen_p+norm(Fi)-norm(Fe_1))*dir;
          else
              Fe = kBall*dpen_p*dir;
          end
      else              %% come out
          if norm(Fi)<norm(Fe_1)
              Fe = (kBall*dpen_p+norm(Fi)-norm(Fe_1))*dir;
          else
              Fe = kBall*dpen_p*dir;
          end
      end
  else
      Fe = zeros(dof,1);
  end

  block.OutputPort(1).Data = Fe;
  block.OutputPort(2).Data = Energy;
  block.OutputPort(3).Data = delta;
  

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(1).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
 
  
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);