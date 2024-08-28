function uOpt = optCtrl(obj, ~, x, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing

if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

uOpt = cell(obj.nu, 1);

%% Optimal control
if strcmp(uMode, 'max')
  uOpt{1} = (deriv{obj.dims==1}.*cos(x{obj.dims==3})+deriv{obj.dims==2}.*sin(x{obj.dims==3}) >=0)*obj.vRange(2) +...
      (deriv{obj.dims==1}.*cos(x{3})+deriv{obj.dims==2}.*sin(x{3}) <=0)*obj.vRange(1);
  uOpt{2} = (deriv{obj.dims==3}>=0)*obj.wRange(2) + (deriv{obj.dims==3}<0)*(obj.wRange(1));
elseif strcmp(uMode, 'min')
  uOpt{1} = (deriv{obj.dims==1}.*cos(x{obj.dims==3})+deriv{obj.dims==2}.*sin(x{obj.dims==3}) >=0)*obj.vRange(1) +...
      (deriv{obj.dims==1}.*cos(x{3})+deriv{obj.dims==2}.*sin(x{3}) <=0)*obj.vRange(2);
  uOpt{2} = (deriv{obj.dims==3}>=0)*(obj.wRange(1)) + (deriv{obj.dims==3}<0)*obj.wRange(2);
else
  error('Unknown uMode!')
end

end