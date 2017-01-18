function [] = matlabknn2(filename)
warning('off','all')
warning
f = dir('files');
Num = length(f(not([f.isdir])));
colors = {'red';'green';'blue';'yellow';'brown';'white';'pink';'orange';'purple'};
sizes = {'small';'medium';'big'};
shapes = {'cube';'sphere';'cylinder'};
for i=1:Num
    name = f(i+2).name;
    fileID = fopen(sprintf('files/%s',name),'r');
    b=fscanf(fileID,'%f');
    fclose(fileID);
    colorTag(i)= b(336);
    shapeTag(i)=b(337);
    sizeTag(i)=b(338);
    a = readFromFileEA(sprintf('files/%s',name));
    colorTrain(i,:) = transpose(b(15:17));
    s = b(19);
    sizeTrain(i,:) = s;
    shapeTrain(i,:)=a(27:71);
end
tA = readFromFileEA(filename);


shapeTest = tA(27:71);
fileID = fopen(sprintf(filename),'r');
t=fscanf(fileID,'%f');
fclose(fileID);
colorTest=transpose(t(15:17));
sT = t(19);
sizeTest = sT;



class1 = knnclassify(colorTest,colorTrain,colorTag,3);
class2 = knnclassify(sizeTest,sizeTrain,sizeTag,3);
class3 = knnclassify(shapeTest,shapeTrain,shapeTag,3);

col = char(colors(class1+1));
si = char(sizes(class2+1));
sh = char(shapes(class3+1));
fprintf('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n\n');
fprintf('This is a %s %s %s',si,col,sh);
fprintf('\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
