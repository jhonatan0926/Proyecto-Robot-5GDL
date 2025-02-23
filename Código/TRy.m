%% Funcion de rotación en Y en matriz de traslación homogenena

function T=TRy(psi)

    T=[cos(psi) 0 sin(psi) 0;
        0 1 0 0;
        -sin(psi) 0 cos(psi) 0;
        0 0 0 1];

end