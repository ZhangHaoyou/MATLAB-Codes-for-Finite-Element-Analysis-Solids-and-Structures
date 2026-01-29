% ex3.2.5(2).m


function ex3_2_5_2()
    E = 2.95e11;
    A = 0.0001;
    x1 = 0;
    y1 = 0;
    x2 = 0.4;
    y2 = 0;
    x3 = 0.4;
    y3 = 0.3;
    x4 = 0;
    y4 = 0.3;
    alpha1 = 0;
    alpha2 = 90;
    alpha3 = atan(0.75)*180/pi;
    k1 = Bar2D2Node_Stiffness(E, A, x1, y1, x2, y2, alpha1);
    fprintf("k1 =\n");
    disp(k1);
    k2 = Bar2D2Node_Stiffness(E, A, x2, y2, x3, y3, alpha2);
    fprintf("k2 =\n");
    disp(k2);
    k3 = Bar2D2Node_Stiffness(E, A, x1, y1, x3, y3, alpha3);
    fprintf("k3 =\n");
    disp(k3);
    k4 = Bar2D2Node_Stiffness(E, A, x4, y4, x3, y3, alpha1);
    fprintf("k4 =\n");
    disp(k4);
    KK = zeros(8,8);
    KK = Bar2D2Node_Assembly(KK, k1, 1, 2);
    KK = Bar2D2Node_Assembly(KK, k2, 2, 3);
    KK = Bar2D2Node_Assembly(KK, k3, 1, 3);
    KK = Bar2D2Node_Assembly(KK, k4, 4, 3);
    fprintf("KK =\n");
    disp(KK);

    k = KK([3, 5, 6], [3, 5, 6]);
    fprintf("k =\n");
    disp(k);

    p = [20000.0; 0.0; -25000];
    u = k\p;
    fprintf("u =\n");
    disp(u);

    q = zeros(8,1);
    q([3 5 6]) = u;
    fprintf("q =\n");
    disp(q);

    P = KK*q;
    fprintf("P =\n");
    disp(P);

    u1 = [q(1); q(2); q(3); q(4)];
    stress1 = Bar2D2Node_Stress(E, x1, y1, x2, y2, alpha1, u1);
    fprintf("stress1 =\n");
    disp(stress1);
    u2 = [q(3); q(4); q(5); q(6)];
    stress2 = Bar2D2Node_Stress(E, x2, y2, x3, y3, alpha2, u2);
    fprintf("stress2 =\n");
    disp(stress2);
    u3 = [q(1); q(2); q(5); q(6)];
    stress3 = Bar2D2Node_Stress(E, x1, y1, x3, y3, alpha3, u3);
    fprintf("stress3 =\n");
    disp(stress3);
    u4 = [q(7); q(8); q(5); q(6)];
    stress4 = Bar2D2Node_Stress(E, x4, y4, x3, y3, alpha1, u4);
    fprintf("stress4 =\n");
    disp(stress4);



function k = Bar2D2Node_Stiffness(E, A, x1, y1, x2, y2, alpha)
    % Calculate 2-node element stiffness matrix
    % 
    % Args:
    %       E: elastic modulus
    %       A: cross-sectional area
    %       x1, y1, x2, y2: first and second node coordinates
    %       alpha: element angle (degree)
    % 
    % Returns:
    %       element stiffness k(4x4)
    L = sqrt((x2-x1)^2 + (y2-y1)^2);
    x = alpha*pi/180;
    C = cos(x);
    S = sin(x);
    k = E*A/L*[ C*C  C*S -C*C -C*S;
                C*S  S*S -C*S -S*S;
               -C*C -C*S  C*C  C*S;
               -C*S -S*S  C*S  S*S];


function z = Bar2D2Node_Assembly(KK, k, i, j)
    % Assemble global stiffness using local element stiffness
    % 
    % Args:
    %       k: element stiffness
    %       i, j: node numbering of the element
    % 
    % Returns:
    %       global stiffness matrix KK
    DOF(1) = 2*i-1;
    DOF(2) = 2*i;
    DOF(3) = 2*j-1;
    DOF(4) = 2*j;
    for n1 = 1:4
        for n2 = 1:4
            KK(DOF(n1), DOF(n2)) = KK(DOF(n1), DOF(n2)) + k(n1, n2);
        end
    end
    z = KK;


function stress = Bar2D2Node_Stress(E, x1, y1, x2, y2, alpha, u)
    % Calculate element stress
    % 
    % Args:
    %       E: elastic modulus
    %       x1, y1, x2, y2: first and second node coordinates
    %       alpha: element angle (degree)
    %       u: node displace (2x1)
    % 
    % Returns:
    %       stress of element
    L = sqrt((x2-x1)^2 + (y2-y1)^2);
    x = alpha*pi/180;
    C = cos(x);
    S = sin(x);
    stress = E/L*[-C -S C S] * u;


function forces = Bar2D2Node_Forces(E, x1, y1, x2, y2, alpha, u)
    % Calculate element stress
    % 
    % Args:
    %       E: elastic modulus
    %       x1, y1, x2, y2: first and second node coordinates
    %       alpha: element angle (degree)
    %       u: node displace (2x1)
    % 
    % Returns:
    %       forces of element
    L = sqrt((x2-x1)^2 + (y2-y1)^2);
    x = alpha*pi/180;
    C = cos(x);
    S = sin(x);
    forces = E*A/L*[-C -S C S] * u;



