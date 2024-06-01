function [obs,dst] = nearest_obs(Obs,reference,l)
if ~iscolumn(reference)
    reference = reference';
end
relative = Obs - reference;
rel_norm = vecnorm(relative); 
[dst,ind] =  min(rel_norm);
obs = Obs(:,ind);

if dst > (l+0.2)
    obs = [];
end

end