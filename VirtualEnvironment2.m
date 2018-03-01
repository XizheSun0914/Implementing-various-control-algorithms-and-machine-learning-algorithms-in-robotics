function VirtualEnvironment2(block)
   
setup(block);

function setup(block)

  block.NumInputPorts  = 2; % pos, pos_dot
  block.NumOutputPorts = 4; % Fe, energy
  
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
  block.OutputPort(4).Dimensions= dof;
  
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
  
  block.Dwork(4).Name            = 'vel_1';
  block.Dwork(4).Dimensions      = dof;
  block.Dwork(4).DatatypeID      = 0;      % uint32
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

function Start(block)

  dof = block.DialogPrm(5).Data;
  p0 = block.DialogPrm(7).Data;

  block.Dwork(1).Data = p0;
  block.Dwork(2).Data = zeros(dof,1);
  block.Dwork(3).Data = 0;
  block.Dwork(4).Data = zeros(dof,1);
   

function Outputs(block)

  pos = block.InputPort(1).Data;
  vel = block.InputPort(2).Data;
  
  pos_1 = block.Dwork(1).Data;
  Fe_1 = block.Dwork(2).Data;
  Energy_1 = block.Dwork(3).Data;
  vel_1 = block.Dwork(4).Data;
  
  kBall = block.DialogPrm(1).Data;
  bBall = block.DialogPrm(2).Data;
  rBall = block.DialogPrm(3).Data;
  cBall = block.DialogPrm(4).Data;
  dof = block.DialogPrm(5).Data;
  tsampling = block.DialogPrm(6).Data;
  qerror = block.DialogPrm(8).Data;
  
  pos_p = pos+vel*tsampling;
  dpen_p = rBall-norm(pos_p-cBall);
  if norm(pos-pos_1)<qerror
      delta = norm(vel)*tsampling;
  else
      delta = qerror;
  end
  
  if norm(pos_p-cBall)<=rBall
      if norm(pos-cBall)<=rBall
          dpen = rBall-norm(pos-cBall);
          dir = (pos-cBall)/norm(pos-cBall);
      else
          dpen = 0;
          dir = zeros(dof,1);
      end
      Fi = kBall*dpen*dir;
      if vel'*dir<0
          if norm(Fe_1)<norm(Fi)
              if vel'*vel_1<0
                  E_up = 0;
                  E_op = 0;
              else
                  E_up = -Fi'*(pos-pos_1+delta*vel/norm(vel))+Fe_1'*(pos-pos_1-delta*vel/norm(vel));
                  if E_up<0
                      E_up=0;
                  end
                  E_op = -Fi'*(pos-pos_1)+Fe_1'*(pos-pos_1);
              end
              if dir'*(pos_p-pos)~=0
                  f_cp = E_up/(-dir'*(pos_p-pos));
                  f_op = E_op/(-dir'*(pos_p-pos));
              else
                  f_cp = norm(Fi)-norm(Fe_1);
                  f_op = 0;
              end
              Fe = (kBall*dpen_p+f_cp)*dir;
              Fei = (kBall*dpen_p+f_op)*dir;
          else
              E_up = 0;
              f_cp = 0;
              Fe = kBall*dpen_p*dir;
              Fei = kBall*dpen_p*dir;
          end
      else
          if norm(Fi)<norm(Fe_1)
              if vel'*vel_1<0
                  E_up = 0;
                  E_op = 0;
              else
                  E_up = -Fi'*(pos-pos_1+delta*vel/norm(vel))+Fe_1'*(pos-pos_1-delta*vel/norm(vel));
                  if E_up<0
                      E_up = 0;
                  end
                  E_op = -Fi'*(pos-pos_1)+Fe_1'*(pos-pos_1);
              end
              if dir'*(pos_p-pos)~=0
                  f_cp = E_up/(-dir'*(pos_p-pos));
                  f_op = E_op/(-dir'*(pos_p-pos));
              else
                  f_cp = norm(Fi)-norm(Fe_1);
                  f_op = 0;
              end
              if (kBall*dpen_p+f_cp)<0
                  Fe = zeros(dof,1);
              else
                  Fe = (kBall*dpen_p+f_cp)*dir;
              end
              if kBall*dpen_p+f_op<0
                  Fei = zeros(dof,1);
              else
                  Fei = (kBall*dpen_p+f_op)*dir;
              end
          else
              E_up = 0;
              f_cp = 0;
              Fe = kBall*dpen_p*dir;
              Fei = kBall*dpen_p*dir;
          end
      end
  else
      E_up = 0;
      f_cp = 0;
      Fe = zeros(dof,1);
      Fei = zeros(dof,1);
  end
  
  Energy = Energy_1 - Fe_1'*(pos-pos_1);
  
  block.OutputPort(1).Data = Fe;
  block.OutputPort(2).Data = Energy;
  block.OutputPort(3).Data = E_up;
  block.OutputPort(4).Data = Fei;
  

function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(1).Data;
  block.Dwork(3).Data = block.OutputPort(2).Data;
  block.Dwork(4).Data = block.InputPort(2).Data;
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);
