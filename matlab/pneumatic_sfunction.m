function pneumatic_sfunction(block)
% Continuous-time, simulation-only Level-2 MATLAB S-function for the RHS.
block.NumInputPorts=1; block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=9;
block.InputPort(1).DatatypeID=0;
block.InputPort(1).Complexity='Real';
block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=8;
block.OutputPort(1).DatatypeID=0;
block.OutputPort(1).Complexity='Real';
block.SampleTimes=[0,0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Outputs',@output);
end

function output(block)
block.OutputPort(1).Data=pneumatic_simulink_rhs(block.InputPort(1).Data);
end
