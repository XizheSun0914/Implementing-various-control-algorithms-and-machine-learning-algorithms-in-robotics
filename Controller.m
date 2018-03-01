function Controller(block)
  
setup(block);
  
function setup(block)

  block.NumInputPorts  = 3; % pos, Fen, Fet
  block.NumOutputPorts = 3; % Fd, E
  
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity  = 'Real';
  block.InputPort(3).DatatypeID  = 0;  % double
  block.InputPort(3).Complexity  = 'Real';
  
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';

  %% bDevice, dof, tsampling, p0
  block.NumDialogPrms     = 4;
  
  dof = block.DialogPrm(2).Data;
  tsampling = block.DialogPrm(3).Data;
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.InputPort(3).Dimensions = dof;
  block.OutputPort(1).Dimensions = dof;
  block.OutputPort(2).Dimensions = 1;
  block.OutputPort(3).Dimensions = 1;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.InputPort(3).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode = tsampling;
  block.OutputPort(2).SamplingMode = tsampling;
  block.OutputPort(3).SamplingMode = tsampling;
  
  block.SampleTimes = [tsampling 0];
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)

  dof = block.DialogPrm(2).Data;

  block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'pos_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'Fd_1';
  block.Dwork(2).Dimensions      = dof;
  block.Dwork(2).DatatypeID      = 0;      % uint32
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'E_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % uint32
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.AutoRegRuntimePrms;


function Start(block)

  dof = block.DialogPrm(2).Data;
  p0 = block.DialogPrm(4).Data;

  block.Dwork(1).Data = p0;
  block.Dwork(2).Data = zeros(dof,1); 
  block.Dwork(3).Data = 0;
   

function Outputs(block)
  
  pos = block.InputPort(1).Data;
  Fen = block.InputPort(2).Data;
  Fet = block.InputPort(3).Data;
  
  bDevice = block.DialogPrm(1).Data;
  dof = block.DialogPrm(2).Data;
  tsampling = block.DialogPrm(3).Data;
  
  pos_1 = block.Dwork(1).Data;
  Fd_1 = block.Dwork(2).Data;
  E_1 = block.Dwork(3).Data;
  
  dpos = pos-pos_1;
  B = bDevice*eye(dof)/tsampling;
  E = E_1 + dpos'*B*dpos - Fd_1'*dpos;
  
  if norm(Fen)~=0
      u = Fen/norm(Fen);
      alpha = u'*(B\u);
      fd_max = sqrt(4*E/alpha);
  else
      u = zeros(dof,1);
      alpha = 0;
      fd_max = 0;
  end
  
  if norm(Fen) > fd_max
      fdn = fd_max;
      check = 0;
  else
      fdn = 0;
      if fd_max ~= 0
          check = 1;
      else
          check = 0;
      end
  end
  
  Fdn = fdn*u;
  Fd = Fdn + Fet;
  
  block.OutputPort(1).Data = Fd;
  block.OutputPort(2).Data = E;
  block.OutputPort(3).Data = check;
  
  

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(1).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);