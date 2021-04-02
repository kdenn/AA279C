function M = EcToM(Ec, e)
% Convert Eccentric anomaly to Mean anomaly

M = Ec-e*sin(Ec);

end