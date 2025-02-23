%% Funcion de traslacion en matriz de transformación homogenena

function T=Txyz(P)

    T=[1 0 0 P(1);
        0 1 0 P(2);
        0 0 1 P(3);
        0 0 0 1];

end