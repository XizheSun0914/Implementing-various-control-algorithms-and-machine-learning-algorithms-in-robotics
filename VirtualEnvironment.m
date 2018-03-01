function VirtualEnvironment(block)
  
setup(block);
  

function setup(block)

  block.NumInputPorts  = 2; % pos, vel
  block.NumOutputPorts = 2; % force, num
  
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

  % sWall, nWall, xWall, pWall, dxWall, dof, tsampling, p0
  block.NumDialogPrms     = 8;
  
  dof = block.DialogPrm(6).Data;
  tsampling = block.DialogPrm(7).Data;
  
  block.InputPort(1).Dimensions = dof;
  block.InputPort(2).Dimensions = dof;
  block.OutputPort(1).Dimensions= dof;
  block.OutputPort(2).Dimensions= 1;
  
  block.InputPort(1).SamplingMode = tsampling;
  block.InputPort(2).SamplingMode = tsampling;
  block.OutputPort(1).SamplingMode= tsampling;
  block.OutputPort(2).SamplingMode= tsampling;
  
  block.SampleTimes = [tsampling 0];
  
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
  
  block.RegBlockMethod('Start', @Start);
  
  block.RegBlockMethod('Update', @Update);

  block.RegBlockMethod('Outputs', @Outputs);
  
  block.RegBlockMethod('Terminate', @Terminate);
  
  
  function DoPostPropSetup(block)
   
  dof = block.DialogPrm(6).Data;
   
  block.NumDworks = 2;
  
  block.Dwork(1).Name            = 'pos_1';
  block.Dwork(1).Dimensions      = dof;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'stiff_1';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

function Start(block)

  p0 = block.DialogPrm(8).Data;
    
  block.Dwork(1).Data = p0;
  block.Dwork(2).Data = 0; 
  
function Outputs(block)
  
  pos = block.InputPort(1).Data;
  vel = block.InputPort(2).Data;
  
  pos_1 = block.Dwork(1).Data;
  stiff_1 = block.Dwork(2).Data;
  
  sWall = block.DialogPrm(1).Data;
  nWall = block.DialogPrm(2).Data;
  xWall = block.DialogPrm(3).Data;
  pWall = block.DialogPrm(4).Data;
  dxWall = block.DialogPrm(5).Data;
  
  if (pos-pWall)*nWall < 0
      dpen = abs(pos-pWall);
  else
      dpen = 0;
  end
  
  if vel*nWall<0
  
      if dpen == 0
          f = 0;
      elseif dpen < xWall(1)
          f = sWall(1)*dpen;
      elseif dpen < xWall(2)
          f = sWall(1)*dxWall + sWall(2)*(dpen-xWall(1));
      elseif dpen < xWall(3)
          f = (sWall(1) + sWall(2))*dxWall + sWall(3)*(dpen-xWall(2));
      elseif dpen < xWall(4)
          f = (sWall(1)+sWall(2)+sWall(3))*dxWall + sWall(4)*(dpen-xWall(3));
      elseif dpen < xWall(5)
          f = (sWall(1)+sWall(2)+sWall(3)+sWall(4))*dxWall + sWall(5)*(dpen-xWall(4));
      elseif dpen < xWall(6)
          f = (sWall(1)+sWall(2)+sWall(3)+sWall(4)+sWall(5))*dxWall + sWall(6)*(dpen-xWall(5));
      elseif dpen < xWall(7)
          f = (sWall(1)+sWall(2)+sWall(3)+sWall(4)+sWall(5)+sWall(6))*dxWall + sWall(7)*(dpen-xWall(6));
      elseif dpen < xWall(8)
          f = (sWall(1)+sWall(2)+sWall(3)+sWall(4)+sWall(5)+sWall(6)+sWall(7))*dxWall + sWall(8)*(dpen-xWall(7));
      else
          f = (sWall(1)+sWall(2)+sWall(3)+sWall(4)+sWall(5)+sWall(6)+sWall(7)+sWall(8))*dxWall + sWall(9)*(dpen-xWall(8));
      end
      
  else
      
      f = sWall(15)*dpen;
      
  end
  
  if dpen~= 0 
      stiff = f/dpen;
  else
      stiff = stiff_1;
  end
  
  force = f*nWall;

  block.OutputPort(1).Data = force;
  block.OutputPort(2).Data = stiff;
  
  
function Update(block)
  
  block.Dwork(1).Data = block.InputPort(1).Data;
  block.Dwork(2).Data = block.OutputPort(2).Data;

    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);