% Chapter 2.5, Problem 1


function ch2_5()
    
    elementNodes = [1 2; 2 3; 2 4]; % element connections
    numberElements = size(elementNodes, 1);
    numberNodes = 4;

    displacements = zeros(numberNodes, 1);
    force = zeros(numberNodes, 1);
    stiffness = zeros(numberNodes);
    
    force(2) = 10.0;
    for e = 1:numberElements
        elementDof = elementNodes(e,:);
        stiffness(elementDof, elementDof) = ...
            stiffness(elementDof, elementDof) + [1 -1; -1 1];
    end

    prescribedDof = [1; 3; 4];
    activeDof = setdiff((1:numberNodes)', prescribedDof);
    displacements(activeDof) = ...
        stiffness(activeDof, activeDof) \ force(activeDof);
    outputDisplacementsReactions(displacements, stiffness, ...
        numberNodes, prescribedDof)
end

function outputDisplacementsReactions...
    (displacements, stiffness, GDof, prescribedDof)
    disp('Displacements')
    jj = 1:GDof; format("default")
    [jj' displacements]

    F = stiffness * displacements;
    reactions = F(prescribedDof);
    disp('reactions')
    [prescribedDof reactions]
end