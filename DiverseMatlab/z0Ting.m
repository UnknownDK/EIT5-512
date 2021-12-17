er = 4.9; %relativ permativitet
W = 0.00265; %bredde 
h = 0.0015; %hoejde af print
t = 0.000034; %bane hoejde
f = 2.4e9; %freq
c = 3e8; %lys speed

a = 88.75 / (sqrt(er + 1.47));
b = (5.97 * h) / (0.8 * W + t);

z0 = a*log(b)


eEff = ((er+1)/2) +  ((er-1)/2) * ((1 + 12*h/W)^-0.5)

L = (z0/c) * sqrt(eEff);

v = z0/L
lambda = v/f;

Zl = 33 - (73i); %aflaest datablad 
Zn = Zl / 50

kvart = lambda/4
halv = lambda/2


%for 50 ohm z0
Yn = 0.25 + 0.55i; %
kortL = (( 0.1843 - 0.084) * lambda)% -  ((7*0.000127)+0.000905+0.005029)
aabenS = 0.326 * lambda
lukketS = (0.326-0.25)*lambda


%Ting vi ikke bruger
%slut = 50 / z0
%kvartZ = sqrt(z0*50)
%syms Wt
%ligning = kvartZ == (88.75 / (sqrt(er + 1.47))) * log((5.97 * h) / (0.8 * Wt + t));
%kvartW = floor(solve(ligning ,Wt) * 1e6)