function VirtualEnvironment0(block)

setup(block);
  

function setup(block)

  block.NumInputPorts  = 2; % pos, vel
  block.NumOutputPorts = 1;
  
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';

  %% kBall, bBall, nBall, cBall, rBall, dof, tsampling
  block.NumDialogPrms     = 7;
  
  dof = block.DialogPrm(6).Data;
  tsampling = block.DialogPrm(7).Data;
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions = dof;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  
  block.SampleTimes = [tsampling 0];
  
  block.SimStateCompliance = 'DefaultSimState';

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Terminate', @Terminate);


function Outputs(block)
  
  kBall = block.DialogPrm(1).Data;
  bBall = block.DialogPrm(2).Data;
  nBall = block.DialogPrm(3).Data;
  cBall = block.DialogPrm(4).Data;
  rBall = block.DialogPrm(5).Data;
  dof = block.DialogPrm(6).Data;
  tsampling = block.DialogPrm(7).Data;
  
  pos = block.InputPort(1).Data;
  vel = block.InputPort(2).Data;
  
  if norm(pos-cBall) <= rBall
      dpen = rBall - norm(pos-cBall);
      dir = (pos-cBall)/norm(pos-cBall);
      Fe = kBall*dpen*nBall*dir - bBall*(vel'*dir)*dir;
  else
      Fe = zeros(dof,1);
  end

  block.OutputPort(1).Data = Fe;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);