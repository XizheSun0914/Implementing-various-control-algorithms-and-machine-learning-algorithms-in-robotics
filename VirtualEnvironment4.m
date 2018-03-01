function VirtualEnvironment4(block)

setup(block);

function setup(block)

  block.NumInputPorts  = 2; % pos, vel
  block.NumOutputPorts = 4; % force,energy
  
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

  %% kBall, bBall, rBall, cBall, dof, tsampling, p0, v0
  block.NumDialogPrms     = 8;

  dof = block.DialogPrm(5).Data;
  tsampling = block.DialogPrm(6).Data;
  
  block.SampleTimes = [tsampling 0];
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions= dof;
  block.OutputPort(2).Dimensions= dof;
  block.OutputPort(3).Dimensions= 1;
  block.OutputPort(4).Dimensions= 1;
  
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
  block.NumDworks = 5;
  
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
  
  block.Dwork(3).Name            = 'posp_1';
  block.Dwork(3).Dimensions      = dof;
  block.Dwork(3).DatatypeID      = 0;      % uint32
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'Eov_1';
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % uint32
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  block.Dwork(5).Name            = 'Energy_1';
  block.Dwork(5).Dimensions      = 1;
  block.Dwork(5).DatatypeID      = 0;      % uint32
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  dof = block.DialogPrm(5).Data;
  tsampling = block.DialogPrm(6).Data;
  p0 = block.DialogPrm(7).Data;
  v0 = block.DialogPrm(8).Data;
  
  block.Dwork(1).Data = p0;
  block.Dwork(2).Data = zeros(dof,1); 
  block.Dwork(3).Data = p0+v0*tsampling;
  block.Dwork(4).Data = 0;
  block.Dwork(5).Data = 0;
  

function Outputs(block)
  
  pos = block.InputPort(1).Data;
  vel = block.InputPort(2).Data;
  
  kBall = block.DialogPrm(1).Data;
  bBall = block.DialogPrm(2).Data;
  rBall = block.DialogPrm(3).Data;
  cBall = block.DialogPrm(4).Data;
  dof = block.DialogPrm(5).Data;
  tsampling = block.DialogPrm(6).Data;
  
  pos_1 = block.Dwork(1).Data;
  Fe_1 = block.Dwork(2).Data;
  posp_1 = block.Dwork(3).Data;
  Eov_1 = block.Dwork(4).Data;
  Energy_1 = block.Dwork(5).Data;
  
  posp = pos+vel*tsampling;
  dpenp = rBall-norm(posp-cBall);
  dpenp_1 = rBall-norm(posp_1-cBall);
  dpen_1 = rBall-norm(pos_1-cBall);
  dpos = pos-pos_1;
  dposp = posp-pos;
  dpen = rBall-norm(pos-cBall);
  if dpenp<0
      dpenp=0;
  end
  if dpenp_1<0
      dpenp_1=0;
  end
  if dpen_1<0
      dpen_1=0;
  end
  
  if dpen>=0
      dir = (pos-cBall)/norm(pos-cBall);
      dir_1 = (pos_1-cBall)/norm(pos_1-cBall);
      Fi = kBall*dpen*dir_1;
      
      if dpenp_1~=dpen_1
          Eov = kBall*(dpen-dpenp_1)*(dpen-dpen_1)+Eov_1*((dpenp_1-dpen)/(dpenp_1-dpen_1));
          if Eov < 0
              Eov=0;
          end
      else
          Eov = 0;
      end
      
      if dir'*dposp~=0
          fov = Eov/(dir'*dposp);
      else
          fov = 0;
      end
       Fe = (kBall*dpenp+fov)*dir;
  else
      Eov = 0;
      Fe = zeros(dof,1);
  end

  Energy = Energy_1 - Fe_1'*dpos;

  block.OutputPort(1).Data = Fe;
  block.OutputPort(2).Data = posp;
  block.OutputPort(3).Data = Eov;
  block.OutputPort(4).Data = Energy;
  

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(1).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
  block.Dwork(4).Data = block.OutputPort(3).Data;
  block.Dwork(5).Data = block.OutputPort(4).Data;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);