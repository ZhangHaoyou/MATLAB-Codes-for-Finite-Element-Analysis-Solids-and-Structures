% prog3.2.4(2) on page 43
% 2D 2-node bar


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