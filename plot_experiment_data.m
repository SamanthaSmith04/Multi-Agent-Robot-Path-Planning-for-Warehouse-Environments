clc;
clear;
close all;

fileNameFAR = "Experiment_Results/FAR_exp1map1-2.xml";
fileNameRRT = "Experiment_Results/RRT_exp1map1-2.xml";

expStructFAR = read_exp_struct(fileNameFAR);
expStructRRT = read_exp_struct(fileNameRRT);


allDist = table;
%%Plot Box Plot For Distances/Proximity
shapeMap = size(expStructRRT.distances)
if (shapeMap(2) > 1)
    %Assume 3 Robots 
    distances1 = expStructRRT.distances(:,1);
    robotGroup1 = repmat("1->2",[1 length(expStructRRT.distances(:,1))])';

    distances2 = expStructRRT.distances(:,2);
    robotGroup2 = repmat("1->3",[1 length(expStructRRT.distances(:,2))])';
    
    distances3 = expStructRRT.distances(:,3);
    robotGroup3 = repmat("2->3",[1 length(expStructRRT.distances(:,3))])';
    
    rrtDistances = [distances1;distances2;distances3];
    rrtGroups = [robotGroup1;robotGroup2;robotGroup3];
    rrtMethod = repmat("RRT",[1 length(rrtDistances)])';
    
    distances1 = expStructFAR.distances(:,1);
    robotGroup1 = repmat("1->2",[1 length(expStructFAR.distances(:,1))])';

    distances2 = expStructFAR.distances(:,2);
    robotGroup2 = repmat("1->3",[1 length(expStructFAR.distances(:,2))])';
    
    distances3 = expStructFAR.distances(:,3);
    robotGroup3 = repmat("2->3",[1 length(expStructFAR.distances(:,3))])';
    
    farDistances = [distances1;distances2;distances3];
    farGroups = [robotGroup1;robotGroup2;robotGroup3];
    farMethod =  repmat("FAR",[1 length(farDistances)])';
    
    allDist.method = [rrtMethod;farMethod];
    allDist.group = [rrtGroups;farGroups];
    allDist.distance = [rrtDistances;farDistances];
    
else
    %Assume 2 Robots 
    rrtDistance = expStructRRT.distances(:,1);
    rrtrobotGroup1 = repmat("1->2",[1 length(rrtDistance)])';
    rrtMethod = repmat("RRT",[1 length(rrtDistance)])';
    
    farDistance = expStructFAR.distances(:,1);
    farrobotGroup1 = repmat("1->2",[1 length(farDistance)])';
    farMethod = repmat("FAR",[1 length(farDistance)])';
    
    allDist.method = [rrtMethod;farMethod];
    allDist.group = [rrtrobotGroup1;farrobotGroup1];
    allDist.distance = [rrtDistance;farDistance];
end

allDist.group = categorical(allDist.group);
allDist.method = categorical(allDist.method);

boxchart(allDist.group,allDist.distance,'GroupByColor',allDist.method)
ylabel('Robot Proximity (m)')
xlabel('Pairwise Selection of Robots');
legend