function pneumatic_controller_sfunction(block)
% 5 ms PI plus alpha-beta state estimate and supply interlock.
block.NumDialogPrms=1;
block.NumInputPorts=1; block.NumOutputPorts=2;
block.SetPreCompInpPortInfoToDynamic; block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=8;
block.InputPort(1).DatatypeID=0; block.InputPort(1).Complexity='Real';
block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=1;
block.OutputPort(1).DatatypeID=0; block.OutputPort(1).Complexity='Real';
block.OutputPort(2).Dimensions=3;
block.OutputPort(2).DatatypeID=0; block.OutputPort(2).Complexity='Real';
block.SampleTimes=[.005,0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Outputs',@output);
block.RegBlockMethod('PostPropagationSetup',@postprop);
block.RegBlockMethod('InitializeConditions',@initialize);
block.RegBlockMethod('Update',@update);
end

function postprop(block)
block.NumDworks=5;
block.Dwork(1).Name='integral'; block.Dwork(1).Dimensions=1;
block.Dwork(1).DatatypeID=0; block.Dwork(1).Complexity='Real';
block.Dwork(1).UsedAsDiscState=true;
names={'positionEstimate','velocityEstimate','lowPressureCount','faultLatched'};
for k=2:5
    block.Dwork(k).Name=names{k-1}; block.Dwork(k).Dimensions=1;
    block.Dwork(k).DatatypeID=0; block.Dwork(k).Complexity='Real';
    block.Dwork(k).UsedAsDiscState=true;
end
end

function initialize(block)
block.Dwork(1).Data=0;
block.Dwork(2).Data=.010;
block.Dwork(3).Data=0;
block.Dwork(4).Data=0;
block.Dwork(5).Data=0;
end

function [command,dint,xnew,vnew,countnew,faultnew]=control(block)
y=block.InputPort(1).Data; config=block.DialogPrm(1).Data;
t=block.CurrentTime; target=.010;
if t>=.5, target=.035; end; if t>=3, target=.015; end
predicted=block.Dwork(2).Data+.005*block.Dwork(3).Data;
residual=y(1)-predicted;
xnew=predicted+config(4)*residual;
vnew=block.Dwork(3).Data+config(5)/.005*residual;
countnew=block.Dwork(4).Data;
if y(8)<config(6), countnew=countnew+1; else, countnew=0; end
faultnew=double(block.Dwork(5).Data~=0 || countnew>=config(7));
error=target-xnew;
raw=config(1)*error+config(2)*block.Dwork(1).Data-config(3)*vnew;
command=max(-1,min(1,raw)); dint=error;
if (raw>1 && error>0)||(raw< -1 && error<0), dint=0; end
if faultnew, command=0; dint=0; end
end

function output(block)
[command,~,xnew,vnew,~,faultnew]=control(block);
block.OutputPort(1).Data=command;
block.OutputPort(2).Data=[xnew;vnew;faultnew];
end

function update(block)
[~,dint,xnew,vnew,countnew,faultnew]=control(block);
block.Dwork(1).Data=block.Dwork(1).Data+.005*dint;
block.Dwork(2).Data=xnew;
block.Dwork(3).Data=vnew;
block.Dwork(4).Data=countnew;
block.Dwork(5).Data=faultnew;
end
