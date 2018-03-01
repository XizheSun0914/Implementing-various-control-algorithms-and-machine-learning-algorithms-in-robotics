function MoreConservative(block)

setup(block);
  

function setup(block)

  block.NumInputPorts  = 2; % Fe, position
  block.NumOutputPorts = 4; % Fd, xi, Energy, Stiffness
  
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

  %% bDevice, tsampling, x0, xWall, nWall
  block.NumDialogPrms     = 5;
  
  tsampling = block.DialogPrm(2).Data;
  
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  block.OutputPort(2).SamplingMode= tsampling;
  block.OutputPort(3).SamplingMode= tsampling;
  block.OutputPort(4).SamplingMode= tsampling;
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)
  block.NumDworks = 4;
  
  block.Dwork(1).Name            = 'pos_1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'Fd_1';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % uint32
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'xi_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'Energy_1';
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  x0 = block.DialogPrm(3).Data;

  block.Dwork(1).Data = x0;
  block.Dwork(2).Data = 0; 
  block.Dwork(3).Data = 1;
  block.Dwork(4).Data = 0; 
   

function Outputs(block)
  
  pos = block.InputPort(1).Data;
  Fe = block.InputPort(2).Data;
  
  pos_1 = block.Dwork(1).Data;
  Fd_1 = block.Dwork(2).Data;
  xi_1 = block.Dwork(3).Data;
  Energy_1 = block.Dwork(4).Data;
  
  bDevice = block.DialogPrm(1).Data;
  tsampling = block.DialogPrm(2).Data;
  xWall = block.DialogPrm(4).Data;
  nWall = block.DialogPrm(5).Data;
  
  B = bDevice/tsampling;
  C = chol(B);
  dpos = pos-pos_1;
  fe = norm(Fe);
  Phi = 2*xi_1*C'*dpos-inv(C)*Fd_1;
  if norm(Fe)~=0
      u = Fe/norm(Fe);
  else
      u = 0;
  end
  alpha = u'*inv(B)*u;
  if fe~=0
      xi = (xi_1*alpha*fe*fe)/(Phi'*Phi);
  else
      xi = xi_1;
  end
  if xi>1
      xi = 1;
  end
  fdmax = sqrt(xi*Phi'*Phi/(alpha*xi_1));
  fdmin = -fdmax;
  
  if fe>fdmax
      fd = fdmax;
  elseif fe<fdmin
      fd = fdmin;
  else
      fd = fe;
  end
  Fd = fd*u;
  Energy = Energy_1 +dpos'*B*dpos - Fd_1'*dpos;
  
  if (pos-xWall)*nWall<0
      pen = abs(pos-xWall);
  else
      pen = 0;
  end
  if pen~=0
      stiff = fd/pen;
  else
      stiff = 0;
  end

  block.OutputPort(1).Data = Fd;
  block.OutputPort(2).Data = xi;
  block.OutputPort(3).Data = Energy;
  block.OutputPort(4).Data = stiff;
  

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(1).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
  block.Dwork(4).Data = block.OutputPort(3).Data;
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);