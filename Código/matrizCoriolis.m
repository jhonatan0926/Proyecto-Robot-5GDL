function C = matrizCoriolis(D, q, dq)

    GDL = length(q); % Número de grados de libertad
    C = sym(zeros(GDL, GDL)); 

    for i = 1:GDL

        for j = 1:GDL
            C_ij = 0;

            for k = 1:GDL
                C_ij = C_ij +(diff(D(i, j), q(k)) + diff(D(i, k), q(j)) - diff(D(j, k), q(i))) / 2 *dq(k);

            end
            C(i,j) = C_ij;

        end
    end
end
