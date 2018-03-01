function virtual_environment(block)

setup(block);
  

function setup(block)

  block.NumInputPorts  = 1;  %% velocity
  block.NumOutputPorts = 2;  %% force

  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';

  %% kWall, bWall, nWall, xWall, v0, x0, dof, tsampling
  block.NumDialogPrms     = 8;
  
  tsampling = block.DialogPrm(8).Data;
  
  block.SampleTimes = [10*tsampling 0];
  
  dof = block.DialogPrm(7).Data;
  
  block.InputPort(1).Dimensions = dof;
  block.OutputPort(1).Dimensions= dof;
  block.OutputPort(2).Dimensions= dof;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  block.OutputPort(2).SamplingMode= tsampling;
  
  block.SimStateCompliance = 'DefaultSimState';

  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);
 
    
function DoPostPropSetup(block)

  dof = block.DialogPrm(7).Data;

  block.NumDworks = 2;
  
  block.Dwork(1).Name            = 'v_n_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'x_n_1';
  block.Dwork(2).Dimensions      = dof;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  v0 = block.DialogPrm(5).Data;
  p0 = block.DialogPrm(6).Data;

  block.Dwork(1).Data = v0;
  block.Dwork(2).Data = p0; 
   

function Outputs(block)
  
  v = block.Dwork(1).Data;
  tsampling = block.DialogPrm(8).Data;
  x_n_1 = block.Dwork(2).Data;
  x_n = x_n_1 + v*tsampling;
  kWall = block.DialogPrm(1).Data;
  bWall = block.DialogPrm(2).Data;
  nWall = block.DialogPrm(3).Data;
  xWall = block.DialogPrm(4).Data;
  if (x_n-xWall)*nWall <= 0
      fWall = kWall*abs(x_n-xWall)*nWall-bWall*v;
      if fWall*nWall < 0
          fWall = 0;
      end
  else
      fWall = 0;
  end
      
  block.OutputPort(1).Data = x_n;
  block.OutputPort(2).Data = fWall;
  

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(1).Data;


function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

