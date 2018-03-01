function popc(block)

setup(block);
  
function setup(block)

  block.NumInputPorts  = 2; %Fe, position
  block.NumOutputPorts = 3; % Fd, Energy, stiffness
  
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

  %% kWall, bWall, nWall, xWall, tsampling, x0
  block.NumDialogPrms     = 6;

  tsampling = block.DialogPrm(5).Data;
  
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode = tsampling;
  block.OutputPort(3).SamplingMode = tsampling;
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)
  block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'Fd_1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'pos_1';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % uint32
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'Energy_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  x0 = block.DialogPrm(6).Data;

  block.Dwork(1).Data = 0;
  block.Dwork(2).Data = x0;
  block.Dwork(3).Data = 0;
   

function Outputs(block)
  
  Fe = block.InputPort(1).Data;
  pos = block.InputPort(2).Data;
  
  Fd_1 = block.Dwork(1).Data;
  pos_1 = block.Dwork(2).Data;
  Energy_1 = block.Dwork(3).Data;
  
  kWall = block.DialogPrm(1).Data;
  bWall = block.DialogPrm(2).Data;
  nWall = block.DialogPrm(3).Data;
  xWall = block.DialogPrm(4).Data;
  
  if (pos-xWall)*nWall<0
      pen = abs(pos-xWall);
  else
      pen = 0;
  end
  
  Energy = Energy_1 - Fd_1*(pos-pos_1);
  Er = 0.5*kWall*pen^2;
  
  if (Energy-Er)<0
      if (pos-pos_1)~=0
          Fpc = (Energy-Er)/(pos-pos_1);
      else
          Fpc = 0;
      end
  else
      Fpc = 0;
  end
  
  Fd = Fe+Fpc;
  
  if pen~=0
      stiff = Fd/pen;
  else
      stiff = 0;
  end
  
  block.OutputPort(1).Data = Fd;
  block.OutputPort(2).Data = Energy;
  block.OutputPort(3).Data = stiff;
  

function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;
  block.Dwork(2).Data = block.InputPort(2).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);
