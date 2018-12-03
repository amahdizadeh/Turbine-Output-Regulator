function exo = buildExo(u)

order = length(u);
canonical= circshift(eye(order),-1,2);
canonical(1,:) = u;

exo=canonical;