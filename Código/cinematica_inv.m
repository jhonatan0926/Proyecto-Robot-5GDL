function angulos = cinematica_inv(L1,L2,L3,L4,L5,p_x,p_y,p_z,phi,theta,psi)

    %Definimos el estado inicial
    estado = 0;

    %Calculo de R n_z o_z a_z
    Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
    Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    R = Rz*Ry*Rx;

    n_z = R(3,1); o_z = R(3,2); a_z = R(3,3);

    %Calculo de los angulos q_1 q_5 q_234
    q_1 = atan(p_y/p_x);
    q_5 = atan(-o_z/n_z);
    q_234 = asin(a_z);

    %Correccion de los valores de q_1 q_5 q_234
    if (q_1<0)
        q_1 = q_1 + pi;
    end
    if (q_5<0)
        q_5 = q_5 + pi;
    end
    if (q_234<0)
        q_234 = q_234 + 3*pi/2;
    end

    %Buscamos angulos adecuados para q_2 q_3 y q_4
    for ang = 0:0.1:180
        q_2 = ang*pi/180;
        syms x;
        eq = L3*sin(q_2+x) + L2*sin(q_2) == p_z - L1 - L4*cos(q_234) - L5*sin(q_234);
        solu = solve(eq,x,"Real",true);
    
        if ~isempty(solu) % Verificar si el valor es real
            for n = 1:1:length(solu)
                q_3 = solu(n); % Agregar a la raíz
                q_4 = q_234 - q_2 - q_3;
                if (double(q_4)>0 && double(q_3)>0)
                    estado = 1;
                    break
                end
            end
            if estado==1, break; end
        end
    end

    if estado == 1
        angulos_n = [q_1 q_2 q_3 q_4 q_5];
        angulos = double(angulos_n*180/3.14159);
    else
        disp('No se puede ubicar la posicion')
        angulos = [0, 0, 0, 0, 0];
    end
    
    return

end