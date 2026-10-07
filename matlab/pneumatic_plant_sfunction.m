function pneumatic_plant_sfunction(block)
% Continuous reference plant. Input: held valve command.
% Output: x,v,mA,mB,spool,signed supplied mass,signed exhausted mass.
block.NumDialogPrms=1;
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=1;
block.InputPort(1).DatatypeID=0; block.InputPort(1).Complexity='Real';
block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=8;
block.OutputPort(1).DatatypeID=0; block.OutputPort(1).Complexity='Real';
block.NumContStates=7;
block.SampleTimes=[0,0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('InitializeConditions',@initialize);
block.RegBlockMethod('Outputs',@output);
block.RegBlockMethod('Derivatives',@derivatives);
end

function initialize(block)
y=pneumatic_initial(block.DialogPrm(1).Data);
block.ContStates.Data=y([1,2,3,4,6,7,8]);
end

function output(block)
p=block.DialogPrm(1).Data;
block.OutputPort(1).Data=[block.ContStates.Data;p.supply];
end

function derivatives(block)
p=block.DialogPrm(1).Data; p.kp=0; p.ki=0; p.kd=0;
y=zeros(8,1); y([1,2,3,4,6,7,8])=block.ContStates.Data;
command=block.InputPort(1).Data;
dy=pneumatic_rhs(block.CurrentTime,y,p);
dy(6)=(command-y(6))/p.valve_tau;
block.Derivatives.Data=dy([1,2,3,4,6,7,8]);
end
