%% Funcion de rotación en Z en matriz de traslación homogenena

function T=TRz(theta)

    T=[cos(theta) -sin(theta) 0 0;
       sin(theta) cos(theta) 0  0
       0 0 1 0;
       0 0 0 1];

end