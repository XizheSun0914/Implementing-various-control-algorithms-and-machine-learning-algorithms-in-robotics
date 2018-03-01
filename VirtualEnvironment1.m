function VirtualEnvironment1(block)
   
setup(block);
  
function setup(block)

  block.NumInputPorts  = 2; % pos, vel
  block.NumOutputPorts = 3; % force, Energy
  
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

  %% kBall, bBall, nBall, cBall, rBall, dof, tsampling, quantilization error
  block.NumDialogPrms     = 8;
  
  dof = block.DialogPrm(6).Data;
  tsampling = block.DialogPrm(7).Data;
  
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
  
  block.SampleTimes = [tsampling 0];
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);

  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Terminate', @Terminate);

  
function DoPostPropSetup(block)
  block.NumDworks = 3;
  dof = block.DialogPrm(6).Data;
  
  block.Dwork(1).Name            = 'Fe_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'E_1';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'Pos_1';
  block.Dwork(3).Dimensions      = dof;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;


function Start(block)

  dof = block.DialogPrm(6).Data;
  qerror = block.DialogPrm(8).Data;

  block.Dwork(1).Data = zeros(dof,1);
  block.Dwork(2).Data = 0;
  block.Dwork(3).Data = [0;0;0.12];
   

function Outputs(block)
  
  pos = block.InputPort(1).Data;
  vel = block.InputPort(2).Data;
  
  kBall = block.DialogPrm(1).Data;
  bBall = block.DialogPrm(2).Data;
  nBall = block.DialogPrm(3).Data;
  cBall = block.DialogPrm(4).Data;
  rBall = block.DialogPrm(5).Data;
  dof = block.DialogPrm(6).Data;
  tsampling = block.DialogPrm(7).Data;
  qerror = block.DialogPrm(8).Data;
  
  Fe_1 = block.Dwork(1).Data;
  E_1 = block.Dwork(2).Data;
  pos_1 = block.Dwork(3).Data;  

  pos_p = pos+vel*tsampling;
  dpos = pos-pos_1;
  dpos_p = pos_p-pos;
  E = E_1 - Fe_1'*dpos;     
  
  if norm(pos_p-cBall)<= rBall
      if norm(pos-cBall)<=rBall
          dpen = rBall-norm(pos-cBall);
          dir = (pos-cBall)/norm(pos-cBall);
      else
          dpen = 0;
          dir = zeros(dof,1);
      end
      dir_p = (pos_p-cBall)/norm(pos_p-cBall);
      dpen_p = rBall-norm(pos_p-cBall);
      Fi = kBall*(dpen)*dir*nBall - bBall*vel'*dir*dir;
      if vel'*dir < 0 
          if norm(Fi)>norm(Fe_1)
              E_up = (Fi-Fe_1)'*dpos;
              if dir'*dpos_p~=0
                  f_op = E_up/(dir'*dpos_p);
              else
                  f_op = 0;
              end
              Fe = (kBall*dpen_p+f_op)*dir-bBall*vel'*dir*dir;
          else
              E_up = 0;
              Fe = (kBall*dpen_p)*dir-bBall*vel'*dir*dir;
          end
      else
          if norm(Fi)<norm(Fe_1)
              E_up = (Fi-Fe_1)'*dpos;
              if dir'*dpos_p~= 0
                  f_up = E_up/(dir'*dpos_p);
              else
                  f_up = 0;
              end
              if kBall*dpen_p+f_up<0
                  Fe = zeros(dof,1);
              else
                  Fe = (kBall*dpen_p+f_up)*dir-bBall*vel'*dir*dir;
              end
          else
              E_up = 0;
              Fe = (kBall*dpen_p)*dir-bBall*vel'*dir*dir;
          end
      end
  else 
      Fe = zeros(dof,1);
      E_up = 0;
  end

  block.OutputPort(1).Data = Fe;
  block.OutputPort(2).Data = E;
  block.OutputPort(3).Data = -E_up;
  

function Update(block)
  
  block.Dwork(1).Data = block.OutputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(2).Data;
  block.Dwork(3).Data = block.InputPort(1).Data;
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);