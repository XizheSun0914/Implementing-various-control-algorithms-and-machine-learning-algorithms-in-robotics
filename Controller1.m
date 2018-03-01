function Controller1(block)
   
setup(block);
  
function setup(block)

  block.NumInputPorts  = 3; % pos, Fen, Fet
  block.NumOutputPorts = 2; % force, E
  
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
  %% bDevice, dof, tsampling, p0, pWall, nWall
  block.NumDialogPrms     = 6;

  dof = block.DialogPrm(2).Data;
  tsampling = block.DialogPrm(3).Data;
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.InputPort(3).Dimensions = dof;
  block.OutputPort(1).Dimensions= dof;
  block.OutputPort(2).Dimensions= 1;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.InputPort(3).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  block.OutputPort(2).SamplingMode= tsampling;
  
  block.SampleTimes = [tsampling 0];
  
  block.SimStateCompliance = 'DefaultSimState';
 
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

    
function DoPostPropSetup(block)

  dof = block.DialogPrm(2).Data;

  block.NumDworks = 4;
  
  block.Dwork(1).Name            = 'pos_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'force_1';
  block.Dwork(2).Dimensions      = dof;
  block.Dwork(2).DatatypeID      = 0;      % uint32
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'E_1';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % uint32
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'Fen_1';
  block.Dwork(4).Dimensions      = dof;
  block.Dwork(4).DatatypeID      = 0;      % uint32
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  p0 = block.DialogPrm(4).Data;
  dof = block.DialogPrm(2).Data;

  block.Dwork(1).Data = p0;
  block.Dwork(2).Data = zeros(dof,1);
  block.Dwork(3).Data = 0;
  block.Dwork(4).Data = zeros(dof,1);
   

function Outputs(block)
  
  pos = block.InputPort(1).Data;
  Fen = block.InputPort(2).Data;
  Fet = block.InputPort(3).Data;
  
  bDevice = block.DialogPrm(1).Data;
  dof = block.DialogPrm(2).Data;
  tsampling = block.DialogPrm(3).Data;
  pWall = block.DialogPrm(5).Data;
  nWall = block.DialogPrm(6).Data;
  
  pos_1 = block.Dwork(1).Data;
  force_1 = block.Dwork(2).Data;
  E_1 = block.Dwork(3).Data;
  Fen_1 = block.Dwork(4).Data;
  
  B = bDevice*eye(dof)/tsampling;
  dpos = pos-pos_1;
  dpen = (pos-pWall)'*nWall/norm(nWall);
  
  if norm(Fen_1)==0 && norm(Fen)~=0
      E = 0;
  else
      %{
      if dpen < 0 && dpen > -0.001
          E = E_1 + dpos'*B*dpos - force_1'*dpos;
      else
          E = E_1 + dpos'*B*dpos - force_1'*dpos;
      end
      %}
      E = E_1 + dpos'*B*dpos - force_1'*dpos;
  end
  
  if norm(Fen)~=0
      un = Fen/norm(Fen);
      ut = Fet/norm(Fet);
      alpha = un'*(B\un);
      mu = norm(Fet)/norm(Fen);
  else
      un = zeros(dof,1);
      ut = zeros(dof,1);
      alpha = 0;
      mu = 0;
  end

  if alpha~=0
      fn_max = sqrt(4*E/alpha);
  else
      fn_max = 0;
  end
  
  if norm(Fen)>fn_max
      fn = fn_max;
  else
      fn = norm(Fen);
  end
  ft = mu*fn;
  
  force = fn*un + ft*ut;
  
  block.OutputPort(1).Data = force;
  block.OutputPort(2).Data = E;
  

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(1).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
  block.Dwork(4).Data = block.InputPort(2).Data;
 
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);