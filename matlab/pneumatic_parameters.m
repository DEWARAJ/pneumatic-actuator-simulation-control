function p=pneumatic_parameters(caseName)
if nargin==0, caseName='tuned'; end
root=fileparts(fileparts(mfilename('fullpath')));
cases=jsondecode(fileread(fullfile(root,'results','parameters.json')));
p=cases.(caseName);
end
