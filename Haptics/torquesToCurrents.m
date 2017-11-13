function currents = torquesToCurrents(Tau)

% A = [1/100,1/250,1/175]';
A = [1,2,1]';
B = [0,0,0]';

currents = A .* Tau + B;

end