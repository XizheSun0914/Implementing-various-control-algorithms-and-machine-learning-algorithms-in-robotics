function HumanOperator(block)

setup(block);
  
% Function: setup ===================================================
% Abstract:
%   Set up the S-function block's basic characteristics such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options

function setup(block)

  block.NumInputPorts  = 3;  % position, velocity, acceleration
  block.NumOutputPorts = 1;  % force of hand 
  
  block.InputPort(1).Dimensions=1;
  block.InputPort(2).Dimensions=1;
  block.InputPort(3).Dimensions=1;
  block.OutputPort(1).Dimensions=1;
  
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

  % Register the parameters.  Mo, Bo, Ko, Xeq
  block.NumDialogPrms     = 8;
  tinterval=block.DialogPrm(8).Data;
  
  block.SampleTimes = [tinterval 0];
  
  block.InputPort(1).SamplingMode = tinterval;
  block.InputPort(2).SamplingMode = tinterval;
  block.InputPort(3).SamplingMode = tinterval;
  block.OutputPort(1).SamplingMode = tinterval;
 
  block.SimStateCompliance = 'DefaultSimState';
  
  block.RegBlockMethod('Start', @Start);

  block.RegBlockMethod('Outputs', @Outputs);
  
  block.RegBlockMethod('Terminate', @Terminate);


function Start(block)

  x0=block.DialogPrm(5).Data;
  v0=block.DialogPrm(6).Data;
  a0=block.DialogPrm(7).Data;
  
  block.InputPort(1).Data=x0;
  block.InputPort(2).Data=v0;
  block.InputPort(3).Data=a0;
  

function Outputs(block)

  Mo=block.DialogPrm(1).Data;
  Bo=block.DialogPrm(2).Data;
  Ko=block.DialogPrm(3).Data;
  Xeq=block.DialogPrm(4).Data;
  
  x=block.InputPort(1).Data;
  v=block.InputPort(2).Data;
  a=block.InputPort(3).Data;
  
  Fh=-(Mo*a+Bo*v+Ko*(x-Xeq));
  
  block.OutputPort(1).Data = Fh;
  
    
function Terminate(block)


