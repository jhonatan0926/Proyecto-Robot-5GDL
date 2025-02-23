    %% CINEMATICA DIRECTA DEL ROBOT
% FUNCION DE DAVID - HARTENBERG

function T = denavit(theta, d, a, alpha)
    T = TRz(theta)*Txyz([0; 0; d])*Txyz([a; 0; 0])*TRx(alpha);
end
