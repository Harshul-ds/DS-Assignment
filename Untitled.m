clc; 
nProducts = 2;
nMachines = 2;
nPeriods = 3;
Demand = [4000 1000;8000 5000; 3000 5000];
AvailMachineTime = [700 1500;300 400; 1000 300];
ProdRate = [0.15 0.16;0.12 0.14];
OperatingCost = 5; 
InventoryCost = 0.1;

nVar = nProducts*nMachines*nPeriods + (nPeriods-1)*nProducts; % Dimension of the problem is determined

% // Linear equality constraints
% // Demand constraints
nEqConstraints = nProducts*nPeriods;
Aeq = zeros(nEqConstraints,nVar);
% // Demand constraints for period 1
Aeq1 = zeros(nProducts,nVar);
for i = 1:nProducts
    index1 = (i-1)*nProducts+1:i*nProducts
    Aeq1(i,index1) = 1;
    index2 = nMachines*nProducts+i
    Aeq1(i,index2) = -1;
    beq1(i,1) = Demand(1,i);
end
% // Demand constraints for period 2 to (nPeriods-1)th period
Aeq2 = zeros(nProducts,nVar);
for i = 2:nPeriods-1
    for j = 1:nProducts
        index3 = (i-1)*(nProducts*nMachines+nProducts)+(j-1)*nMachines+1:(i-1)*(nProducts*nMachines+nProducts)+(j-1)*nMachines+nMachines;
        Aeq2(j,index3) = 1; 
        index4 = (i-1)*(nProducts*nMachines+nProducts) - nProducts+j;
        Aeq2(j,index4) = 1;
        index5 = i*(nProducts*nMachines+nProducts)- nProducts+j;
        Aeq2(j,index5) = -1;
        beq2(j,1) = Demand(i,j);
    end
end

% // Demand constraints for last period
Aeq3 = zeros(nProducts,nVar);
for i = 1:nProducts
    index6 = (nProducts*nMachines+nProducts)*(nPeriods-1)+(i-1)*nProducts+1:(nProducts*nMachines+nProducts)*(nPeriods-1)+i*nProducts
    Aeq3(i,index6) = 1;
    index7 = (nProducts*nMachines+nProducts)*(nPeriods-1) - nProducts+i;
    Aeq3(i,index7) = 1;
    beq3(i,1) = Demand(nPeriods,i);
end
    Aeq = [Aeq1;Aeq2;Aeq3];
    beq = [beq1;beq2;beq3];

% // Linear inequality constraints
% // Machine time constraints
for i = 1:nPeriods
    for j = 1:nProducts
        Cindex = (i-1)*(nProducts*nMachines+nProducts)+j:nProducts:(i-1)*(nProducts*nMachines+nProducts)+nMachines+j
        Rindex = (i-1)*nProducts+j;
        A(Rindex,Cindex) = ProdRate(:,j)';
        b(Rindex,1) = AvailMachineTime(i,j);
    end
end

% // Objective function
TotalProductionCost = [];
for j = 1:nProducts
    TotalProductionCost = [TotalProductionCost ProdRate(j,:)*OperatingCost];
end

for i = 1:nPeriods
    index = (i-1)*(nProducts*nMachines+nProducts)+1:(i-1)*(nProducts*nMachines+nProducts)+nProducts*nMachines;
    nindex = length(index);
    cost(index,1) = TotalProductionCost';
    cost(index(nindex)+1:index(nindex)+nProducts,1) = InventoryCost;
end
cost(nVar+1:nVar+nProducts) = [];
lb = zeros(1,nVar);

[xopt,fopt,exitflag,output,lambda]=linprog(cost, A, b, Aeq, beq, lb,[]);

% //Result representation

if exitflag == 0 then
    disp(" Optimal Solution Found")
    M = ["   "];
    for m = 1:nMachines
        M = [M strcat(["Machine",string(m)])];
    end
    P = [];
    for p = 1:nProducts
        P = [P;strcat(["Product ",string(p)])];
    end

    for i = 1:nPeriods
        Sol = [];
        for j = 1:nProducts
            Ind1 = (i-1)*(nProducts*nMachines + nProducts)+(j-1)*nProducts+1:(i-1)*(nProducts*nMachines + nProducts)+j*nProducts;
            Sol = [Sol;xopt(Ind1)']; 
        end

        disp(strcat(["Production schedule for the Period ", string(i)]));
        disp([M; [P string(Sol)]]);
    end

    for i = 1:nPeriods-1
        ind = i*(nProducts*nMachines+1:nProducts*nMachines+nProducts);
        inventory = xopt(ind);
        disp(strcat(["Inventory at the Period ", string(i)]));
        disp([P string(inventory)]);
    end

    disp(["The optimal cost is ", string(fopt)])
elseif exitflag == 1 
    disp("Primal Infeasible")
else
    disp("Error encountered")
end