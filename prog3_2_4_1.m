% prog3.2.4(1) on page 41
% 1D 2-node bar


function k = Bar1D2Node_Stiffness(E, A, L)
    % Calculate 2-node element stiffness matrix
    % 
    % Args:
    %       E: elastic modulus
    %       A: cross-sectional area
    %       L: element length
    % 
    % Returns:
    %       element stiffness k(2x2)
    k = [E*A/L -E*A/L; -E*A/L E*A/L];


function z = Bar1D2Node_Assembly(KK, k, i, j)
    % Assemble global stiffness using local element stiffness
    % 
    % Args:
    %       k: element stiffness
    %       i, j: node numbering of the element
    % 
    % Returns:
    %       global stiffness matrix KK
    DOF(1) = i;
    DOF(2) = j;
    for n1 = 1:2
        for n2 = 1:2
            KK(DOF(n1), DOF(n2)) = KK(DOF(n1), DOF(n2)) + k(n1, n2);
        end
    end
    z = KK;


function stress = Bar1D2Node_Stress(k, u, A)
    % Calculate element stress
    % 
    % Args:
    %       k: element stiffness
    %       u: node displace (2x1)
    %       A: cross-section area
    % 
    % Returns:
    %       stress of element
    stress = k * u / A